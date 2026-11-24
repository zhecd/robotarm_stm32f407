param([ValidateRange(1,3)][int]$Axis=2,[ValidateSet('D0','D1')][string]$Direction='D0',
      [ValidateSet(-1,0,1)][int]$ExpectedSign=0,[switch]$ExecuteAuthorized,[switch]$SelfTest,
      [string]$AnalyzeDirectionFile='')
# 公共单轴首次小点动：20微步/50每秒/500ms。默认拒绝，不能自动HOME或重试。
$ErrorActionPreference='Stop'
function ExpectedImage([int]$id){
 $prefix=if($id -eq 1){'M115 FIRMWARE:M1HomingPrepare PROTOCOL:M1HOME8 '}else{"M115 FIRMWARE:AxisHomingPrepare PROTOCOL:AXISHOME1 AXIS:$id "}
 return $prefix+'UART:115200 MOTION_COMPILED:1 AUTO_MOTION:0 JOG_STEPS:20 JOG_DEADLINE_MS:500 SEEK_STEPS:200 BACKOFF_STEPS:40 HOME_DEADLINE_MS:5500 STEP_PERIOD_MS:20'
}
function CheckImage([string]$reply,[int]$id){if($reply -ne (ExpectedImage $id)){throw 'Wrong axis, image or motion bounds'}}
function CheckState([string]$reply,[string]$mode){
 if(!$reply.StartsWith('M408 ')){throw 'Expected M408'}
 $f=@{};foreach($m in [regex]::Matches($reply,'([A-Z_]+):([^ ]+)')){$f[$m.Groups[1].Value]=$m.Groups[2].Value}
 foreach($key in @('STATE','REASON','USED','STEP','JOG','SEEK','BACKOFF','DRIVER','DIR_CONFIRMED','HOMED','ALLOWED','RXERR','RXOVF')){if(!$f.ContainsKey($key) -or $f[$key] -notmatch '^\d+$'){throw "Invalid $key"}}
 foreach($key in @('REASON','SEEK','BACKOFF','DIR_CONFIRMED','HOMED','ALLOWED','RXERR','RXOVF')){if($f[$key] -ne '0'){throw "Unexpected $key : $reply"}}
 if($f['EN'] -notin @('ENABLED','DISABLED') -or $f['STEP'] -notin @('0','1') -or [int]$f['JOG'] -gt 20){throw 'Invalid jog outputs or count'}
 if($mode -eq 'idle'){
  if($f['STATE'] -ne '0' -or $f['USED'] -ne '0' -or $f['JOG'] -ne '0' -or $f['EN'] -ne 'DISABLED' -or $f['STEP'] -ne '0' -or $f['DRIVER'] -notin @('0','1')){throw 'Not unused disabled idle'}
 }else{
  if($f['STATE'] -notin @('1','5') -or $f['USED'] -ne '1' -or $f['DRIVER'] -ne '1'){throw "Not bounded JOG: $reply"}
  if($f['STATE'] -eq '5' -and ($f['EN'] -ne 'DISABLED' -or $f['STEP'] -ne '0' -or $f['JOG'] -ne '20')){throw 'Incomplete or enabled DONE'}
  if($mode -eq 'done' -and $f['STATE'] -ne '5'){throw 'Expected DONE'}
 }
 return $f
}
function CheckMovement([double]$before,[double[]]$angles){
 # 大于约4个AS5600原始码的变化才作为有运动反馈的证据；不是角度精度验收。
 if(!$angles -or [double]::IsNaN($before) -or [double]::IsInfinity($before)){throw 'Missing finite encoder baseline'}
 $peak=0.0
 foreach($angle in $angles){
  if([double]::IsNaN($angle) -or [double]::IsInfinity($angle)){throw 'Non-finite encoder trace'}
  $delta=$angle-$before;if([Math]::Abs($delta) -gt [Math]::Abs($peak)){$peak=$delta}
 }
 if([Math]::Abs($peak) -lt 0.35){throw 'No clear encoder movement; stopped, no retry'}
 if([Math]::Abs($peak) -gt 5.0){throw 'Encoder change outside small-jog envelope; stopped, no retry'}
 return $peak
}
function Median([double[]]$values){
 $sorted=@($values|Sort-Object);if(!$sorted.Count){throw 'Empty median'}
 $middle=[int][Math]::Floor($sorted.Count/2)
 if(($sorted.Count%2) -eq 0){return ($sorted[$middle-1]+$sorted[$middle])/2};return $sorted[$middle]
}
function StepDirection([object[]]$Samples,[object[]]$Records){
 # 方向只从EN已使能、JOG计数4..18的中段判断；不将使能对齐/禁用释放混入。
 # M114与随后M408异步，限制两者间隔100ms、反馈年龄25ms，并要求跨至少8步。
 $points=[Collections.Generic.List[object]]::new();$last=-1
 foreach($sample in $Samples){
  if($sample.phase -ne 'active' -or [int]$sample.age_ms -gt 25){continue}
  $status=$Records|Where-Object {$_.type -eq 'RX' -and $_.text.StartsWith('M408 ') -and $_.ms -ge $sample.ms}|Select-Object -First 1
  if(!$status -or $status.ms-$sample.ms -gt 100 -or $status.text -notmatch '^M408 STATE:1 REASON:0 USED:1 EN:ENABLED STEP:[01] JOG:([0-9]+) SEEK:0 BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0$'){continue}
  $count=[int]$Matches[1]
  if($count -lt 4 -or $count -gt 18){continue}
  if($count -eq $last){continue};if($count -lt $last){throw 'Direction STEP counter decreased'}
  $angle=[double]$sample.motor_deg
  if([double]::IsNaN($angle) -or [double]::IsInfinity($angle)){throw 'Non-finite active feedback'}
  $points.Add([pscustomobject]@{count=$count;angle=$angle});$last=$count
 }
 if($points.Count -lt 4 -or $points[-1].count-$points[0].count -lt 8){throw 'Insufficient STEP-active evidence; direction unconfirmed'}
 $half=[int][Math]::Floor($points.Count/2)
 $early=Median @($points[0..($half-1)]|ForEach-Object angle)
 $late=Median @($points[($points.Count-$half)..($points.Count-1)]|ForEach-Object angle)
 $delta=$late-$early;if([Math]::Abs($delta) -lt 0.35){throw 'STEP-active displacement too small; direction unconfirmed'}
 $sign=if($delta -gt 0){1}else{-1};$opposing=0
 for($i=1;$i -lt $points.Count;$i++){if(($points[$i].angle-$points[$i-1].angle)*$sign -lt -0.1){$opposing++}}
 if($opposing -gt [Math]::Floor(($points.Count-1)*0.2)){throw 'STEP-active trend is contradictory; direction unconfirmed'}
 return [pscustomobject]@{encoder_sign=$sign;active_delta_motor_deg=$delta;points=$points.Count;first_count=$points[0].count;last_count=$points[-1].count;opposing_intervals=$opposing;method='ENABLED STEP counts4..18 half-window medians'}
}
if($AnalyzeDirectionFile){
 if($ExecuteAuthorized){throw 'Offline analysis cannot execute hardware'}
 $data=Get-Content -LiteralPath $AnalyzeDirectionFile -Raw|ConvertFrom-Json
 StepDirection @($data.samples) @($data.records)|ConvertTo-Json;return
}
if($SelfTest){
 if($ExecuteAuthorized){throw 'SelfTest never executes hardware'}
 foreach($id in 1..3){CheckImage (ExpectedImage $id) $id}
 $idle='M408 STATE:0 REASON:0 USED:0 EN:DISABLED STEP:0 JOG:0 SEEK:0 BACKOFF:0 DRIVER:0 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0'
 $done=$idle.Replace('STATE:0','STATE:5').Replace('USED:0','USED:1').Replace('JOG:0','JOG:20').Replace('DRIVER:0','DRIVER:1')
 $null=CheckState $idle 'idle';$null=CheckState $done 'done'
 $bad=@($done.Replace('JOG:20','JOG:21'),$done.Replace('JOG:20','JOG:19'),$done.Replace('SEEK:0','SEEK:1'),$done.Replace('BACKOFF:0','BACKOFF:1'),$done.Replace('STEP:0','STEP:1'),$done.Replace('EN:DISABLED','EN:ENABLED'),$done.Replace('HOMED:0','HOMED:1'),$done.Replace('ALLOWED:0','ALLOWED:1'),$done.Replace('DRIVER:1','DRIVER:0'),$done.Replace('DIR_CONFIRMED:0','DIR_CONFIRMED:1'))
 foreach($s in $bad){$failed=$false;try{$null=CheckState $s 'done'}catch{$failed=$true};if(!$failed){throw 'Accepted invalid jog state'}}
 foreach($reason in 1..11){$failed=$false;try{$null=CheckState ($done.Replace('REASON:0','REASON:'+$reason)) 'done'}catch{$failed=$true};if(!$failed){throw 'Accepted motion failure'}}
 $failed=$false;try{CheckImage (ExpectedImage 1) 2}catch{$failed=$true};if(!$failed){throw 'Wrong-axis image accepted'}
 if((CheckMovement 720 @(720,719,718)) -ne -2){throw 'Lost signed/multiturn feedback delta'}
 foreach($angles in @(@(720.1),@(720.0),@(726.0),@([double]::NaN),@([double]::PositiveInfinity))){
  $failed=$false;try{$null=CheckMovement 720 $angles}catch{$failed=$true};if(!$failed){throw 'Accepted missing/invalid movement evidence'}
 }
 function SyntheticTrend([double[]]$angles){
  $ss=@();$rr=@();$counts=@(4,7,10,13,16,18)
  for($i=0;$i -lt $angles.Count;$i++){
   $ss+=[pscustomobject]@{phase='active';ms=$i*30;motor_deg=$angles[$i];age_ms=10}
   $rr+=[pscustomobject]@{type='RX';ms=$i*30+1;text=('M408 STATE:1 REASON:0 USED:1 EN:ENABLED STEP:0 JOG:'+$counts[$i]+' SEEK:0 BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0')}
  };return [pscustomobject]@{samples=$ss;records=$rr}
 }
 foreach($sign in @(-1,1)){
  $d=SyntheticTrend @(10,10.2,10.4,10.6,10.8,11)
  foreach($s in $d.samples){$s.motor_deg*= $sign}
  # 大的静置/释放偏移不应反转STEP中段趋势。
  $d.samples=@([pscustomobject]@{phase='pre';ms=-10;motor_deg=900;age_ms=0})+$d.samples+@([pscustomobject]@{phase='post';ms=999;motor_deg=-900;age_ms=0})
  if((StepDirection $d.samples $d.records).encoder_sign -ne $sign){throw 'Active direction sign lost to static offset'}
 }
 foreach($angles in @(@(10,10,10,10,10,10),@(10,11,9,12,10,13),@([double]::NaN,10,11,12,13,14),@([double]::PositiveInfinity,10,11,12,13,14))){
  $d=SyntheticTrend $angles;$failed=$false;try{$null=StepDirection $d.samples $d.records}catch{$failed=$true};if(!$failed){throw 'Accepted unclear active direction'}
 }
 $d=SyntheticTrend @(10,11,12,13,14,15);foreach($s in $d.samples){$s.age_ms=26}
 $failed=$false;try{$null=StepDirection $d.samples $d.records}catch{$failed=$true};if(!$failed){throw 'Accepted stale direction feedback'}
 Write-Output 'AXIS_JOG_CONTRACT_PASS; 3 identities, idle/DONE, 21 invalid states, wrong-axis and encoder-evidence checks; no serial access';return
}
if(!$ExecuteAuthorized){throw 'Explicit single-axis jog approval required; no serial opened'}
if($Direction -eq 'D1' -and $ExpectedSign -eq 0){throw 'D1 direction probe requires explicit expected encoder sign; no serial opened'}
$claimPath=Join-Path $PSScriptRoot 'jog_once.claim';$resultPath=Join-Path $PSScriptRoot 'jog_result.json'
if((Test-Path -LiteralPath $claimPath) -or (Test-Path -LiteralPath $resultPath)){throw 'Prior attempt exists; no replay'}
$records=[Collections.Generic.List[object]]::new();$samples=[Collections.Generic.List[object]]::new()
$clock=[Diagnostics.Stopwatch]::StartNew();$phase='pre';$issued=$false;$completed=$false;$failure=$null;$final=$null
$p=[IO.Ports.SerialPort]::new('COM4',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$p.DtrEnable=$false;$p.RtsEnable=$false;$p.ReadTimeout=50;$p.WriteTimeout=300;$p.NewLine="`n"
function Send([string]$cmd){
 if($cmd -notin @('M115','M408','M114','M119','M120','M121','M122','CONFIG',('JOG '+$Direction),'KEEP','M112')){throw 'Command outside jog scope'}
 $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$clock.ElapsedMilliseconds;phase=$phase;type='TX';text=$cmd});$p.Write($cmd+"`n")
}
function Reply([string]$pattern,[int]$timeout=300){
 $watch=[Diagnostics.Stopwatch]::StartNew()
 while($watch.ElapsedMilliseconds -lt $timeout){
  try{$s=$p.ReadLine().Trim()}catch [TimeoutException]{continue}
  $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$clock.ElapsedMilliseconds;phase=$phase;type='RX';text=$s})
  if($s -match $pattern){return $s};if($s.StartsWith('error:')){throw $s}
 };throw "Reply timeout $pattern"
}
function Q([string]$cmd){Send $cmd;return Reply ('^'+[regex]::Escape($cmd)+' ')}
function Sample {
 $s=Q 'M114';$pattern='^M114 RAW_M'+$Axis+':([+-]?[0-9.]+) VALID:1 AGE_MS:(\d+)$'
 if($s -notmatch $pattern -or [int]$Matches[2] -ge 100){throw "Invalid feedback $s"}
 $angle=[double]::Parse($Matches[1],[Globalization.CultureInfo]::InvariantCulture)
 $samples.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$clock.ElapsedMilliseconds;phase=$phase;motor_deg=$angle;age_ms=[int]$Matches[2]})
}
try{
 $p.Open();Start-Sleep -Milliseconds 100;$p.DiscardInBuffer();CheckImage (Q 'M115') $Axis
 $null=CheckState (Q 'M408') 'idle'
 if((Q 'M119') -ne 'M119 M1:OPEN M2:OPEN M3:OPEN'){throw 'Initial limit active'}
 $null=Q 'M120';$null=Q 'M121';$null=Q 'M122'
 $cfg=Q 'CONFIG'
 if($cfg -notmatch '^CONFIG VALID:1 RESULT:0 REG:FF READ_ERROR:0 CURRENT_REF:INTERNAL IRUN_CODE:8 IHOLD_CODE:4 GCONF:000001C0 CHOPCONF:14000053 IFCNT:(\d+),(\d+)$' -or (([int]$Matches[2]-[int]$Matches[1]+256)%256) -ne 4){throw 'Config not confirmed'}
 for($i=0;$i -lt 10;$i++){Sample;Start-Sleep -Milliseconds 30}
 if((Q 'M119') -ne 'M119 M1:OPEN M2:OPEN M3:OPEN'){throw 'Limit changed before jog'}
 $claim=[IO.File]::Open($claimPath,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None);$claim.Dispose()
 $phase='active';$issued=$true;$start=$clock.ElapsedMilliseconds;Send ('JOG '+$Direction)
 $accepted=Reply '^(accepted:|error:)' 700
 $expected=if($Axis -eq 1){'accepted: one-shot M1 test; lease required'}else{'accepted: one-shot axis test; lease required'}
 if($accepted -ne $expected){throw 'Jog not accepted'}
 $keep=$clock.ElapsedMilliseconds+60;$lastCount=0
 while($true){
  if($clock.ElapsedMilliseconds-$start -gt 650){throw 'Host 650ms bound; no retry'}
  # 续约发送时动作可能恰好结束；允许该回复，随后仍须用M408确认DONE或故障。
  if($clock.ElapsedMilliseconds -ge $keep){Send 'KEEP';$null=Reply '^(KEEP OK|error: no live session)$' 200;$keep=$clock.ElapsedMilliseconds+80}
  Sample;$s=Q 'M408';$f=CheckState $s 'active'
  if([int]$f['JOG'] -lt $lastCount){throw 'Counter reset'};$lastCount=[int]$f['JOG']
  if($f['STATE'] -eq '5'){$final=$s;$completed=$true;break};Start-Sleep -Milliseconds 15
 }
 $phase='post';$end=$clock.ElapsedMilliseconds+3000
 while($clock.ElapsedMilliseconds -lt $end){Sample;Start-Sleep -Milliseconds 30}
 $final=Q 'M408';$null=CheckState $final 'done'
 if((Q 'M119') -ne 'M119 M1:OPEN M2:OPEN M3:OPEN'){throw 'Limit active after jog'}
 $null=Q 'M120';$null=Q 'M121'
 $local=Q 'M122'
 if($local -notmatch '^M122 LOCAL_REF:0 VALID:0 REASON:1 .* WHOLE_HOMED:0 ALLOWED:0$'){throw 'JOG must not establish HOME reference'}
 $before=($samples|Where-Object phase -eq 'pre'|Select-Object -Last 1).motor_deg
 $observed=@($samples|Where-Object phase -ne 'pre'|ForEach-Object motor_deg)
 $peakMotorDelta=CheckMovement $before $observed
 if($ExpectedSign -ne 0){
  $postAngles=@($samples|Where-Object phase -eq 'post'|Select-Object -Last 10|ForEach-Object motor_deg)
  $settledMotorDelta=(Median $postAngles)-$before # 仅记录释放后的总差，不作为方向依据。
  $directionAnalysis=StepDirection @($samples) @($records)
  if($directionAnalysis.encoder_sign -ne $ExpectedSign){throw 'STEP-active direction contradicts expected sign; no retry'}
 }
}catch{
 $failure=$_.Exception.Message;$phase='stop'
 if($issued -and $p.IsOpen){try{Send 'M112';$null=Reply '^M112OK$' 500;$final=Q 'M408'}catch{}}
}finally{
 if($p.IsOpen){$p.Close()};$p.Dispose()
 [pscustomobject]@{pass=($completed -and !$failure);axis=$Axis;jog_direction=$Direction;expected_encoder_sign=$ExpectedSign;settled_motor_delta_deg=$settledMotorDelta;direction_analysis=$directionAnalysis;encoder_trend_matched=($completed -and !$failure -and $ExpectedSign -ne 0);firmware_direction_recorded=$false;action_issued=$issued;error=$failure;final_reply=$final;peak_motor_delta_deg=$peakMotorDelta;feedback_movement_threshold_deg=0.35;feedback_movement_max_deg=5.0;samples=$samples;records=$records}|ConvertTo-Json -Depth 7|Set-Content -LiteralPath $resultPath -Encoding utf8
}
if($failure){throw $failure}
Write-Output "ONE_AXIS_JOG_DONE_DISABLED; axis=$Axis; direction=$Direction; 20 microsteps; no HOME/automatic reverse/retry"
