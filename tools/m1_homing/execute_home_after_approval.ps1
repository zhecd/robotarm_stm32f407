param([switch]$ExecuteAuthorized,[switch]$SelfTest)
# 单次HOME主机监督。默认拒绝运行；-SelfTest仅验证解析，不创建串口。
# 获用户对D0寻限位及D1退回的明确授权后，将本脚本复制到新的硬件记录目录执行。
$ErrorActionPreference='Stop'
$expectedImage='M115 FIRMWARE:M1HomingPrepare PROTOCOL:M1HOME8 UART:115200 MOTION_COMPILED:1 AUTO_MOTION:0 JOG_STEPS:20 JOG_DEADLINE_MS:500 SEEK_STEPS:200 BACKOFF_STEPS:40 HOME_DEADLINE_MS:5500 STEP_PERIOD_MS:20'
function AssertImage([string]$reply){if($reply -ne $expectedImage){throw 'Unexpected HOME image or bounds'}}
function ReferenceFields([string]$reply,[bool]$ready){
 # M122的VALID仅表示局部参考有效，整机HOMED和运动许可必须仍为0。
 $number='[+-]?(?:\d+(?:\.\d*)?|\.\d+)'
 $pattern='^M122 LOCAL_REF:(\d+) VALID:([01]) REASON:(\d+) ORIGIN_MOTOR:('+ $number +') REL_MOTOR:('+ $number +') REL_JOINT:('+ $number +') GEAR_RATIO:4\.500 AGE_MS:(\d+) WHOLE_HOMED:0 ALLOWED:0$'
 if($reply -notmatch $pattern){throw 'Invalid local-reference reply'}
 foreach($field in @(4,5,6)){
  $value=[double]::Parse($Matches[$field],[Globalization.CultureInfo]::InvariantCulture)
  if([double]::IsNaN($value) -or [double]::IsInfinity($value)){throw 'Non-finite reference value'}
 }
 if($ready){
  if($Matches[1] -ne '2' -or $Matches[2] -ne '1' -or $Matches[3] -ne '0' -or [int]$Matches[7] -ge 100){throw 'Local reference not valid/fresh'}
  $motor=[double]::Parse($Matches[5],[Globalization.CultureInfo]::InvariantCulture)
  $joint=[double]::Parse($Matches[6],[Globalization.CultureInfo]::InvariantCulture)
  if([Math]::Abs($motor-4.5*$joint) -gt 0.01){throw 'Reference units disagree'}
 }elseif($Matches[1] -ne '0' -or $Matches[2] -ne '0' -or $Matches[3] -ne '1'){
  throw 'Reference already used or invalid before HOME'
 }
}
function HomeFields([string]$s,[string]$mode){
 if(!$s.StartsWith('M408 ')){throw 'Expected M408'}
 $f=@{};foreach($m in [regex]::Matches($s,'([A-Z_]+):([^ ]+)')){$f[$m.Groups[1].Value]=$m.Groups[2].Value}
 foreach($key in @('STATE','REASON','USED','EN','STEP','JOG','SEEK','BACKOFF','DRIVER','DIR_CONFIRMED','HOMED','ALLOWED','RXERR','RXOVF')){if(!$f.ContainsKey($key)){throw "Missing field $key"}}
 foreach($key in @('STATE','REASON','USED','STEP','JOG','SEEK','BACKOFF','DRIVER','DIR_CONFIRMED','HOMED','ALLOWED','RXERR','RXOVF')){
  if($f[$key] -notmatch '^\d+$'){throw "Non-numeric $key"}
 }
 if($f['EN'] -notin @('ENABLED','DISABLED') -or $f['STEP'] -notin @('0','1')){throw 'Invalid outputs'}
 if($f['JOG'] -ne '0' -or [int]$f['SEEK'] -gt 200 -or [int]$f['BACKOFF'] -gt 40){throw "Unexpected motion counters $s"}
 foreach($key in @('HOMED','ALLOWED','RXERR','RXOVF')){if($f[$key] -ne '0'){throw "Unexpected $key"}}
 if($f['REASON'] -ne '0'){throw "Stopped/faulted: $s"}
 if($mode -in @('IdleBefore','IdleDirection')){
  $dir=if($mode -eq 'IdleDirection'){'1'}else{'0'}
  if($f['STATE'] -ne '0' -or $f['USED'] -ne '0' -or $f['EN'] -ne 'DISABLED' -or $f['STEP'] -ne '0' -or $f['SEEK'] -ne '0' -or $f['BACKOFF'] -ne '0' -or $f['DIR_CONFIRMED'] -ne $dir){throw 'Not expected unused disabled idle'}
  if($mode -eq 'IdleDirection' -and $f['DRIVER'] -ne '1'){throw 'Driver not ready'}
 }elseif($mode -in @('ActiveOrDone','Done')){
  if($f['USED'] -ne '1' -or $f['DRIVER'] -ne '1' -or $f['DIR_CONFIRMED'] -ne '1' -or $f['STATE'] -notin @('2','3','4','5')){throw "Invalid HOME state: $s"}
  if($f['STATE'] -eq '5'){
   if($f['EN'] -ne 'DISABLED' -or $f['STEP'] -ne '0' -or $f['BACKOFF'] -ne '40'){throw 'DONE without full retreat or disabled output'}
  }elseif($mode -eq 'Done'){throw 'HOME not complete'}
 }else{throw 'Unknown validation mode'}
 return $f
}
if($SelfTest){
 if($ExecuteAuthorized){throw 'Choose self-test or execution'}
 $idle='M408 STATE:0 REASON:0 USED:0 EN:DISABLED STEP:0 JOG:0 SEEK:0 BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0'
 $directed=$idle.Replace('DIR_CONFIRMED:0','DIR_CONFIRMED:1')
 $seek=$directed.Replace('STATE:0','STATE:2').Replace('USED:0','USED:1').Replace('EN:DISABLED','EN:ENABLED').Replace('SEEK:0','SEEK:12')
 $done=$seek.Replace('STATE:2','STATE:5').Replace('EN:ENABLED','EN:DISABLED').Replace('BACKOFF:0','BACKOFF:40')
 AssertImage $expectedImage
 $null=HomeFields $idle 'IdleBefore';$null=HomeFields $directed 'IdleDirection'
 foreach($state in @('2','3','4')){$null=HomeFields ($seek.Replace('STATE:2','STATE:'+$state)) 'ActiveOrDone'}
 $null=HomeFields $done 'Done'
 $rejected=0
 foreach($bad in @($done.Replace('SEEK:12','SEEK:201'),$done.Replace('BACKOFF:40','BACKOFF:41'),$done.Replace('BACKOFF:40','BACKOFF:39'),$done.Replace('JOG:0','JOG:1'),$done.Replace('DIR_CONFIRMED:1','DIR_CONFIRMED:0'),$done.Replace('EN:DISABLED','EN:ENABLED'),$done.Replace('STEP:0','STEP:1'),$done.Replace('HOMED:0','HOMED:1'),$done.Replace('ALLOWED:0','ALLOWED:1'),$done.Replace('RXERR:0','RXERR:1'),$done.Replace('RXOVF:0','RXOVF:1'),$done.Replace('DRIVER:1','DRIVER:0'),$done.Replace('SEEK:12','SEEK:NaN'),$seek)){
  $failed=$false;try{$null=HomeFields $bad 'Done'}catch{$failed=$true};if(!$failed){throw "Self-test accepted invalid state $bad"};$rejected++
 }
 for($reason=1;$reason -le 11;$reason++){
  $failed=$false;try{$null=HomeFields ($done.Replace('STATE:5','STATE:7').Replace('REASON:0','REASON:'+$reason)) 'ActiveOrDone'}catch{$failed=$true};if(!$failed){throw 'Accepted fault state'};$rejected++
 }
 $failed=$false;try{AssertImage ($expectedImage.Replace('HOME_DEADLINE_MS:5500','HOME_DEADLINE_MS:9000'))}catch{$failed=$true};if(!$failed){throw 'Accepted widened HOME deadline'}
 $ref='M122 LOCAL_REF:2 VALID:1 REASON:0 ORIGIN_MOTOR:720.000 REL_MOTOR:4.500 REL_JOINT:1.000 GEAR_RATIO:4.500 AGE_MS:20 WHOLE_HOMED:0 ALLOWED:0'
 ReferenceFields $ref $true
 ReferenceFields ($ref.Replace('LOCAL_REF:2 VALID:1 REASON:0','LOCAL_REF:0 VALID:0 REASON:1')) $false
 $refRejected=0
 foreach($bad in @($ref.Replace('VALID:1','VALID:0'),$ref.Replace('REASON:0','REASON:3'),$ref.Replace('LOCAL_REF:2','LOCAL_REF:1'),$ref.Replace('AGE_MS:20','AGE_MS:100'),$ref.Replace('WHOLE_HOMED:0','WHOLE_HOMED:1'),$ref.Replace('ALLOWED:0','ALLOWED:1'),$ref.Replace('GEAR_RATIO:4.500','GEAR_RATIO:1.000'),$ref.Replace('ORIGIN_MOTOR:720.000','ORIGIN_MOTOR:NaN'),$ref.Replace('REL_MOTOR:4.500','REL_MOTOR:Infinity'),$ref.Replace('REL_JOINT:1.000','REL_JOINT:2.000'))){
  $failed=$false;try{ReferenceFields $bad $true}catch{$failed=$true};if(!$failed){throw "Accepted invalid reference $bad"};$refRejected++
 }
 $failed=$false;try{ReferenceFields $ref $false}catch{$failed=$true};if(!$failed){throw 'Accepted pre-existing reference'};$refRejected++
 $failed=$false;try{AssertImage ($expectedImage.Replace('M1HOME8','M1HOME7'))}catch{$failed=$true};if(!$failed){throw 'Accepted old firmware without reference support'}
 Write-Output "HOME_CONTRACT_SELFTEST_PASS; 7 motion valid, $($rejected+1) motion invalid; 2 reference valid, $refRejected reference invalid; old image rejected; no serial or hardware access"
 return
}
if(!$ExecuteAuthorized){throw 'Explicit HOME motion approval required; candidate does not run by default'}
$claimPath=Join-Path $PSScriptRoot 'home_once.claim';$resultPath=Join-Path $PSScriptRoot 'home_result.json'
if((Test-Path -LiteralPath $claimPath) -or (Test-Path -LiteralPath $resultPath)){throw 'Prior attempt exists; no replay or overwrite'}
$records=[Collections.Generic.List[object]]::new();$samples=[Collections.Generic.List[object]]::new()
$timeline=[Diagnostics.Stopwatch]::StartNew();$phase='pre';$issued=$false;$completed=$false;$failure=$null;$homeStartMs=$null;$final=$null
$p=[IO.Ports.SerialPort]::new('COM4',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$p.DtrEnable=$false;$p.RtsEnable=$false;$p.Handshake=[IO.Ports.Handshake]::None;$p.ReadTimeout=50;$p.WriteTimeout=300;$p.NewLine="`n"
function Send([string]$cmd){
 if($cmd -notin @('M115','M408','M114','M119','M120','M121','M122','CONFIG','DIR D0','HOME','KEEP','M112')){throw 'Command outside single HOME scope'}
 $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$timeline.ElapsedMilliseconds;phase=$phase;type='TX';text=$cmd});$p.Write($cmd+"`n")
}
function Reply([string]$pattern,[int]$timeout=300){
 $watch=[Diagnostics.Stopwatch]::StartNew()
 while($watch.ElapsedMilliseconds -lt $timeout){
  try{$s=$p.ReadLine().Trim()}catch [TimeoutException]{continue}
  $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$timeline.ElapsedMilliseconds;phase=$phase;type='RX';text=$s})
  if($s -match $pattern){return $s};if($s.StartsWith('error:')){throw "Firmware rejected: $s"}
 };throw "Reply timeout $pattern"
}
function Q([string]$cmd){Send $cmd;return Reply ('^'+[regex]::Escape($cmd)+' ')}
function SensorAngle {
 $s=Q 'M114'
 if($s -notmatch '^M114 RAW_M1:([+-]?[0-9.]+) VALID:1 AGE_MS:(\d+)$' -or [int]$Matches[2] -ge 100){throw "Invalid feedback $s"}
 $angle=[double]::Parse($Matches[1],[Globalization.CultureInfo]::InvariantCulture)
 $samples.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$timeline.ElapsedMilliseconds;phase=$phase;raw_motor_deg=$angle;age_ms=[int]$Matches[2]})
 return $angle
}
try{
 $p.Open();Start-Sleep -Milliseconds 100;$p.DiscardInBuffer();AssertImage (Q 'M115')
 $null=HomeFields (Q 'M408') 'IdleBefore'
 $null=Q 'M120';$null=Q 'M121'
 ReferenceFields (Q 'M122') $false
 if((Q 'M119') -ne 'M119 M1:OPEN M2:OPEN M3:OPEN'){throw 'Limit initially active'}
 $cfg=Q 'CONFIG'
 if($cfg -notmatch '^CONFIG VALID:1 RESULT:0 REG:FF READ_ERROR:0 CURRENT_REF:INTERNAL IRUN_CODE:8 IHOLD_CODE:4 GCONF:000001C0 CHOPCONF:14000053 IFCNT:(\d+),(\d+)$'){throw 'Driver config confirmation failed'}
 if((([int]$Matches[2]-[int]$Matches[1]+256)%256) -ne 4){throw 'IFCNT not +4'}
 # D0的真实趋向由阶段39人工确认；这里只记录方向，不发STEP。
 Send 'DIR D0';if((Reply '^DIR RECORDED: operator verification required$') -ne 'DIR RECORDED: operator verification required'){throw 'Direction not recorded'}
 $null=HomeFields (Q 'M408') 'IdleDirection'
 $baselineEnd=$timeline.ElapsedMilliseconds+1000
 while($timeline.ElapsedMilliseconds -lt $baselineEnd){$null=SensorAngle;Start-Sleep -Milliseconds 35}
 if((Q 'M119') -ne 'M119 M1:OPEN M2:OPEN M3:OPEN'){throw 'Limit changed before HOME'}
 $null=SensorAngle
 $claim=[IO.File]::Open($claimPath,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None);$claim.Dispose()
 $phase='active';$homeStartMs=$timeline.ElapsedMilliseconds;$issued=$true
 Send 'HOME';if((Reply '^(accepted:|error:)' 700) -ne 'accepted: one-shot M1 test; lease required'){throw 'HOME not accepted'}
 $nextKeep=$timeline.ElapsedMilliseconds+60;$nextState=$timeline.ElapsedMilliseconds;$lastSeek=0;$lastBackoff=0
 while($true){
  if($timeline.ElapsedMilliseconds-$homeStartMs -gt 5700){throw 'Host 5.7s supervision deadline; stop, no retry'}
  if($timeline.ElapsedMilliseconds -ge $nextKeep){Send 'KEEP';Reply '^(KEEP OK|error: no live session)$' 200|Out-Null;$nextKeep=$timeline.ElapsedMilliseconds+80}
  $null=SensorAngle
  if($timeline.ElapsedMilliseconds -ge $nextState){
   $s=Q 'M408';$f=HomeFields $s 'ActiveOrDone'
   if([int]$f['SEEK'] -lt $lastSeek -or [int]$f['BACKOFF'] -lt $lastBackoff){throw 'Motion counters decreased/reset'}
   $lastSeek=[int]$f['SEEK'];$lastBackoff=[int]$f['BACKOFF']
   if($f['STATE'] -eq '5'){$null=HomeFields $s 'Done';$completed=$true;break}
   # M119仅观察，不以主机采样替代固件1ms停步/10ms触发确认。
   $limit=Q 'M119';if($limit -notmatch '^M119 M1:(OPEN|TRIGGERED) M2:OPEN M3:OPEN$'){throw 'Unexpected other limit'}
   $null=Q 'M122' # 连续记录参考生命周期；此处允许尚未完成HOME的UNSET/WAIT。
   $nextState=$timeline.ElapsedMilliseconds+70
  }
  Start-Sleep -Milliseconds 25
 }
 $phase='post';$postEnd=$timeline.ElapsedMilliseconds+3000;$nextState=$timeline.ElapsedMilliseconds
 while($timeline.ElapsedMilliseconds -lt $postEnd){
  $null=SensorAngle
  if($timeline.ElapsedMilliseconds -ge $nextState){$final=Q 'M408';$null=HomeFields $final 'Done';$null=Q 'M122';$nextState=$timeline.ElapsedMilliseconds+150}
  Start-Sleep -Milliseconds 35
 }
 $limits=Q 'M119';if($limits -ne 'M119 M1:OPEN M2:OPEN M3:OPEN'){throw 'Limit not released after retreat'}
 $final=Q 'M408';$null=HomeFields $final 'Done'
 $null=Q 'M120';$null=Q 'M121'
 ReferenceFields (Q 'M122') $true
}catch{
 $failure=$_.Exception.Message;$phase='stop'
 # 失败只停机取证，绝不追加HOME或扩展寻限位行程。
 if($p.IsOpen){try{Send 'M112';Reply '^M112OK$' 500|Out-Null;$final=Q 'M408';$null=Q 'M120';$null=Q 'M121';$null=Q 'M122'}catch{$records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$timeline.ElapsedMilliseconds;phase=$phase;type='ERROR';text=$_.Exception.Message})}}
}finally{
 if($p.IsOpen){$p.Close()};$p.Dispose()
 [pscustomobject]@{pass=($completed -and !$failure);action_issued=$issued;completed=$completed;error=$failure;port='COM4';baud=115200;home_start_ms=$homeStartMs;final_reply=$final;limits_after=$limits;samples=$samples;records=$records}|ConvertTo-Json -Depth 7|Set-Content -LiteralPath $resultPath -Encoding utf8
}
if($failure){throw $failure}
Write-Output "ONE_HOME_DONE_DISABLED; continuous_samples=$($samples.Count); no retry or whole-robot homed claim"
