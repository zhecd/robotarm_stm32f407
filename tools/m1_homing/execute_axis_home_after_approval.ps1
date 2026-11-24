param([int]$Axis=0,[string]$SeekDirection='',[switch]$DirectionVerified,[string]$DirectionEvidence='',
      [switch]$ExecuteAuthorized,[switch]$SelfTest)
# 通用单轴有限HOME：与板端轴绑定分离。默认拒绝；方向必须有机械依据。
# 实机前复制本文件及axis_home_contract.ps1到新的验证目录，不复用历史claim。
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'axis_home_contract.ps1')
if($SelfTest){
 if($ExecuteAuthorized){throw 'SelfTest cannot execute hardware'}
 $script:rejected=0
 function Reject([scriptblock]$Block){$failed=$false;try{& $Block|Out-Null}catch{$failed=$true};if(!$failed){throw 'Contract accepted an invalid case'};$script:rejected++}
 foreach($id in 1..3){
  foreach($direction in @('D0','D1')){Assert-AxisHomeRequest $true $id $direction $true 'synthetic unit-test evidence, not real authorization'}
  foreach($motion in @(0,1)){if((Get-AxisHomeImage $id $motion) -notmatch "MOTION_COMPILED:$motion AUTO_MOTION:0"){throw 'Identity mismatch'}}
  $null=Read-AxisHomeFeedback "M114 RAW_M${id}:720.125 VALID:1 AGE_MS:99" $id
  foreach($mask in 0..7){
   $reply='M119';foreach($n in 1..3){$state=if($mask -band (1 -shl ($n-1))){'TRIGGERED'}else{'OPEN'};$reply+=" M${n}:$state"}
   if((Read-AxisHomeLimits $reply) -ne $mask){throw 'Limit mask mismatch'}
   if(($mask -band (7 -bxor (1 -shl ($id-1)))) -eq 0){Assert-AxisHomeLimits $reply $id $true}else{Reject {Assert-AxisHomeLimits $reply $id $true}}
   if($mask -eq 0){Assert-AxisHomeLimits $reply $id $false}else{Reject {Assert-AxisHomeLimits $reply $id $false}}
  }
 }
 Reject {Assert-AxisHomeRequest $false 2 'D0' $true 'source'}
 Reject {Assert-AxisHomeRequest $true 0 'D0' $true 'source'}
 Reject {Assert-AxisHomeRequest $true 4 'D0' $true 'source'}
 Reject {Assert-AxisHomeRequest $true 2 'D2' $true 'source'}
 Reject {Assert-AxisHomeRequest $true 2 'D0' $false 'source'}
 Reject {Assert-AxisHomeRequest $true 2 'D0' $true '  '}
 $idle='M408 STATE:0 REASON:0 USED:0 EN:DISABLED STEP:0 JOG:0 SEEK:0 BACKOFF:0 DRIVER:1 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0'
 $directed=$idle.Replace('DIR_CONFIRMED:0','DIR_CONFIRMED:1')
 $seek=$directed.Replace('STATE:0','STATE:2').Replace('USED:0','USED:1').Replace('EN:DISABLED','EN:ENABLED').Replace('SEEK:0','SEEK:12')
 $back=$seek.Replace('STATE:2','STATE:4').Replace('BACKOFF:0','BACKOFF:20')
 $done=$seek.Replace('STATE:2','STATE:5').Replace('EN:ENABLED','EN:DISABLED').Replace('BACKOFF:0','BACKOFF:40')
 $null=Assert-AxisHomeState $idle 'IdleBefore';$null=Assert-AxisHomeState $directed 'IdleDirection'
 foreach($s in @($seek,$seek.Replace('STATE:2','STATE:3'),$back,$done)){$null=Assert-AxisHomeState $s 'ActiveOrDone'}
 $null=Assert-AxisHomeState $done 'Done'
 foreach($bad in @($done+' RXERR:0',$done.Replace('JOG:0','EXTRA:0'),$done.Replace('JOG:0','JOG:-1'),$done.Replace('SEEK:12','SEEK:4294967296'),$done.Replace('SEEK:12','SEEK:201'),$done.Replace('BACKOFF:40','BACKOFF:39'),$done.Replace('BACKOFF:40','BACKOFF:41'),$done.Replace('STEP:0','STEP:1'),$done.Replace('EN:DISABLED','EN:ENABLED'),$done.Replace('HOMED:0','HOMED:1'),$done.Replace('ALLOWED:0','ALLOWED:1'),$done.Replace('RXERR:0','RXERR:1'),$done.Replace('DRIVER:1','DRIVER:2'),$done.Replace('DIR_CONFIRMED:1','DIR_CONFIRMED:0'))){Reject {$null=Assert-AxisHomeState $bad 'Done'}}
 foreach($reason in 1..11){Reject {$null=Assert-AxisHomeState ($done.Replace('STATE:5','STATE:7').Replace('REASON:0','REASON:'+$reason)) 'ActiveOrDone'}}
 Reject {$null=Assert-AxisHomeState ($seek.Replace('BACKOFF:0','BACKOFF:1')) 'ActiveOrDone'}
 Reject {$null=Assert-AxisHomeState ($seek.Replace('STATE:2','STATE:3').Replace('STEP:0','STEP:1')) 'ActiveOrDone'}
 foreach($bad in @('M114 RAW_M1:720.000 VALID:1 AGE_MS:1','M114 RAW_M2:NaN VALID:1 AGE_MS:1','M114 RAW_M2:720.000 VALID:0 AGE_MS:1','M114 RAW_M2:720.000 VALID:1 AGE_MS:100','M114 RAW_M2:720.000 VALID:1 AGE_MS:4294967296')){Reject {$null=Read-AxisHomeFeedback $bad 2}}
 $initial='M122 LOCAL_REF:0 VALID:0 REASON:1 ORIGIN_MOTOR:0.000 REL_MOTOR:0.000 REL_JOINT:0.000 GEAR_RATIO:4.500 AGE_MS:0 WHOLE_HOMED:0 ALLOWED:0'
 $ready='M122 LOCAL_REF:2 VALID:1 REASON:0 ORIGIN_MOTOR:720.000 REL_MOTOR:4.500 REL_JOINT:1.000 GEAR_RATIO:4.500 AGE_MS:99 WHOLE_HOMED:0 ALLOWED:0'
 Assert-AxisHomeReference $initial $false;Assert-AxisHomeReference $ready $true
 foreach($bad in @($ready.Replace('AGE_MS:99','AGE_MS:100'),$ready.Replace('VALID:1','VALID:0'),$ready.Replace('GEAR_RATIO:4.500','GEAR_RATIO:1.000'),$ready.Replace('REL_JOINT:1.000','REL_JOINT:2.000'),$ready.Replace('WHOLE_HOMED:0','WHOLE_HOMED:1'),$ready.Replace('ORIGIN_MOTOR:720.000','ORIGIN_MOTOR:NaN'))){Reject {Assert-AxisHomeReference $bad $true}}
 Reject {Assert-AxisHomeReference ($initial.Replace('ORIGIN_MOTOR:0.000','ORIGIN_MOTOR:1.000')) $false}
 Write-Output "AXIS_HOME_CONTRACT_PASS; 3 axes, 2 directions, all limit masks, 7 valid HOME states, $script:rejected invalid cases; no serial access"
 return
}
Assert-AxisHomeRequest $ExecuteAuthorized.IsPresent $Axis $SeekDirection $DirectionVerified.IsPresent $DirectionEvidence
$expectedImage=Get-AxisHomeImage $Axis 1
$claimPath=Join-Path $PSScriptRoot 'home_once.claim';$resultPath=Join-Path $PSScriptRoot 'home_result.json'
foreach($name in @('home_once.claim','home_result.json','jog_once.claim')){if(Test-Path -LiteralPath (Join-Path $PSScriptRoot $name)){throw 'Prior attempt exists; no replay or mixed record directory'}}
$records=[Collections.Generic.List[object]]::new();$samples=[Collections.Generic.List[object]]::new()
$clock=[Diagnostics.Stopwatch]::StartNew();$phase='pre';$issued=$false;$completed=$false;$failure=$null;$homeStartMs=$null;$final=$null;$limits=$null;$imageVerified=$false
$p=[IO.Ports.SerialPort]::new('COM4',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$p.DtrEnable=$false;$p.RtsEnable=$false;$p.Handshake=[IO.Ports.Handshake]::None;$p.ReadTimeout=50;$p.WriteTimeout=300;$p.NewLine="`n"
function Send([string]$Command){
 if($Command -notin @('M115','M408','M114','M119','M120','M121','M122','CONFIG',('DIR '+$SeekDirection),'HOME','KEEP','M112')){throw 'Command outside single HOME scope'}
 $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$clock.ElapsedMilliseconds;phase=$phase;type='TX';text=$Command});$p.Write($Command+"`n")
}
function Reply([string]$Pattern,[int]$Timeout=300){
 $watch=[Diagnostics.Stopwatch]::StartNew()
 while($watch.ElapsedMilliseconds -lt $Timeout){
  try{$s=$p.ReadLine().Trim()}catch [TimeoutException]{continue}
  $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$clock.ElapsedMilliseconds;phase=$phase;type='RX';text=$s})
  if($s -match $Pattern){return $s};if($s.StartsWith('error:')){throw "Firmware rejected: $s"}
 };throw "Reply timeout $Pattern"
}
function Q([string]$Command){Send $Command;return Reply ('^'+[regex]::Escape($Command)+' ')}
function Sample {
 $feedback=Read-AxisHomeFeedback (Q 'M114') $Axis
 $samples.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$clock.ElapsedMilliseconds;phase=$phase;motor_deg=$feedback.motor_deg;age_ms=$feedback.age_ms})
}
try{
 $p.Open();Start-Sleep -Milliseconds 100;$p.DiscardInBuffer()
 if((Q 'M115') -cne $expectedImage){throw 'Wrong axis, image or HOME bounds'};$imageVerified=$true
 $null=Assert-AxisHomeState (Q 'M408') 'IdleBefore'
 Assert-AxisHomeLimits (Q 'M119') $Axis $false
 Assert-AxisHomeReference (Q 'M122') $false
 $null=Q 'M120';$null=Q 'M121'
 $config=Q 'CONFIG'
 if($config -notmatch '^CONFIG VALID:1 RESULT:0 REG:FF READ_ERROR:0 CURRENT_REF:INTERNAL IRUN_CODE:8 IHOLD_CODE:4 GCONF:000001C0 CHOPCONF:14000053 IFCNT:([0-9]+),([0-9]+)$' -or [int]$Matches[1] -gt 255 -or [int]$Matches[2] -gt 255 -or (([int]$Matches[2]-[int]$Matches[1]+256)%256) -ne 4){throw 'Driver configuration not confirmed'}
 # 方向依据来自显式参数并归档；编码器正负号、旧版默认值都不能代替机械确认。
 Send ('DIR '+$SeekDirection);$null=Reply '^DIR RECORDED: operator verification required$'
 $null=Assert-AxisHomeState (Q 'M408') 'IdleDirection'
 $end=$clock.ElapsedMilliseconds+1000;while($clock.ElapsedMilliseconds -lt $end){Sample;Start-Sleep -Milliseconds 35}
 Assert-AxisHomeLimits (Q 'M119') $Axis $false;Sample
 $claim=[IO.File]::Open($claimPath,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None);$claim.Dispose()
 $phase='active';$homeStartMs=$clock.ElapsedMilliseconds;$issued=$true
 Send 'HOME';$accepted=Reply '^(accepted:|error:)' 700
 $expected=if($Axis -eq 1){'accepted: one-shot M1 test; lease required'}else{'accepted: one-shot axis test; lease required'}
 if($accepted -cne $expected){throw 'HOME not accepted'}
 $keep=$clock.ElapsedMilliseconds+60;$nextState=$clock.ElapsedMilliseconds;$lastSeek=0;$lastBackoff=0
 while($true){
  if($clock.ElapsedMilliseconds-$homeStartMs -gt 5700){throw 'Host 5700ms deadline; stop without retry'}
  if($clock.ElapsedMilliseconds -ge $keep){Send 'KEEP';$null=Reply '^(KEEP OK|error: no live session)$' 200;$keep=$clock.ElapsedMilliseconds+80}
  Sample
  if($clock.ElapsedMilliseconds -ge $nextState){
   $status=Q 'M408';$f=Assert-AxisHomeState $status 'ActiveOrDone'
   if($f['SEEK'] -lt $lastSeek -or $f['BACKOFF'] -lt $lastBackoff){throw 'HOME counters decreased or reset'}
   $lastSeek=$f['SEEK'];$lastBackoff=$f['BACKOFF']
   if($f['STATE'] -eq 5){$final=$status;$completed=$true;break}
   Assert-AxisHomeLimits (Q 'M119') $Axis $true
   $null=Q 'M122';$nextState=$clock.ElapsedMilliseconds+70
  };Start-Sleep -Milliseconds 25
 }
 $phase='post';$end=$clock.ElapsedMilliseconds+3000;$nextState=$clock.ElapsedMilliseconds
 while($clock.ElapsedMilliseconds -lt $end){
  Sample
  if($clock.ElapsedMilliseconds -ge $nextState){$final=Q 'M408';$null=Assert-AxisHomeState $final 'Done';$null=Q 'M122';$nextState=$clock.ElapsedMilliseconds+150}
  Start-Sleep -Milliseconds 35
 }
 $limits=Q 'M119';Assert-AxisHomeLimits $limits $Axis $false
 $final=Q 'M408';$null=Assert-AxisHomeState $final 'Done'
 $null=Q 'M120';$null=Q 'M121';Assert-AxisHomeReference (Q 'M122') $true
}catch{
 $failure=$_.Exception.Message;$phase='stop'
 # 原始故障回复保留在records；M112后的HOST_STOP不能覆盖最初失败原因。
 $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');ms=$clock.ElapsedMilliseconds;phase=$phase;type='ERROR';text=$failure})
 if($imageVerified -and $p.IsOpen){try{Send 'M112';$null=Reply '^M112OK$' 500;$final=Q 'M408';$null=Q 'M120';$null=Q 'M121';$null=Q 'M122'}catch{}}
}finally{
 if($p.IsOpen){$p.Close()};$p.Dispose()
 [pscustomobject]@{pass=($completed -and !$failure);axis=$Axis;seek_direction=$SeekDirection;direction_evidence=$DirectionEvidence;direction_verified=$DirectionVerified.IsPresent;action_issued=$issued;completed=$completed;error=$failure;port='COM4';baud=115200;home_start_ms=$homeStartMs;final_reply=$final;limits_after=$limits;samples=$samples;records=$records;automatic_retry=$false;whole_robot_homed=$false}|ConvertTo-Json -Depth 7|Set-Content -LiteralPath $resultPath -Encoding utf8
}
if($failure){throw $failure}
Write-Output "ONE_AXIS_HOME_DONE_DISABLED; axis=$Axis; seek=$SeekDirection; no retry or whole-robot homed claim"
