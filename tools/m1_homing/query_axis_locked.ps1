param([ValidateRange(1,3)][int]$Axis=2,[ValidateRange(1,10000)][int]$Rounds=20,
      [ValidatePattern('^[A-Za-z0-9_-]+$')][string]$Name='locked_queries',[switch]$Connect,[switch]$SelfTest)
# 复制到新的验证目录后显式-Connect；本脚本只查询，不配置驱动或请求动作。
$ErrorActionPreference='Stop'
function ExpectedImage([int]$id){
 $prefix=if($id -eq 1){'M115 FIRMWARE:M1HomingPrepare PROTOCOL:M1HOME8 '}else{"M115 FIRMWARE:AxisHomingPrepare PROTOCOL:AXISHOME1 AXIS:$id "}
 return $prefix+'UART:115200 MOTION_COMPILED:0 AUTO_MOTION:0 JOG_STEPS:20 JOG_DEADLINE_MS:500 SEEK_STEPS:200 BACKOFF_STEPS:40 HOME_DEADLINE_MS:5500 STEP_PERIOD_MS:20'
}
if($SelfTest){
 if($Connect){throw 'Choose self-test or hardware connection'}
 foreach($id in 1..3){if((ExpectedImage $id) -notmatch 'MOTION_COMPILED:0 AUTO_MOTION:0'){throw 'Not locked'}}
 if((ExpectedImage 1) -eq (ExpectedImage 2)){throw 'Axis identity collision'}
 Write-Output 'AXIS_QUERY_CONTRACT_PASS; three distinct locked identities; no serial access';return
}
if(!$Connect){throw 'Explicit -Connect required; no serial opened'}
if((Test-Path -LiteralPath (Join-Path $PSScriptRoot ($Name+'.json'))) -or (Test-Path -LiteralPath (Join-Path $PSScriptRoot ($Name+'_error.json')))){throw 'Prior collection exists; no overwrite or serial opened'}
$records=[Collections.Generic.List[object]]::new()
$p=[IO.Ports.SerialPort]::new('COM4',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$p.DtrEnable=$false;$p.RtsEnable=$false;$p.ReadTimeout=200;$p.WriteTimeout=500;$p.NewLine="`n"
function Q([string]$cmd){
 if($cmd -notin @('M115','M408','M114','M119','M120','M121','M122')){throw 'Read-only locked scope'}
 $p.Write($cmd+"`n");$sw=[Diagnostics.Stopwatch]::StartNew()
 while($sw.ElapsedMilliseconds -lt 1500){
  try{$s=$p.ReadLine().Trim()}catch [TimeoutException]{continue}
  $records.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');command=$cmd;reply=$s})
  if($s.StartsWith($cmd+' ')){return $s};if($s.StartsWith('error:')){throw $s}
 };throw "Timeout $cmd"
}
try{
 $p.Open();Start-Sleep -Milliseconds 100;$p.DiscardInBuffer()
 if((Q 'M115') -ne (ExpectedImage $Axis)){throw 'Wrong axis or not the compiled-lock image'}
 $lastSamples=-1L;$completedRounds=0
 for($i=0;$i -lt $Rounds;$i++){
  if(Test-Path -LiteralPath (Join-Path $PSScriptRoot ($Name+'.stop'))){break}
  $s=Q 'M408'
  if($s -ne 'M408 STATE:0 REASON:0 USED:0 EN:DISABLED STEP:0 JOG:0 SEEK:0 BACKOFF:0 DRIVER:0 DIR_CONFIRMED:0 HOMED:0 ALLOWED:0 RXERR:0 RXOVF:0'){throw "State changed $s"}
  $enc=Q 'M114';if($enc -notmatch ('^M114 RAW_M'+$Axis+':[+-]?[0-9.]+ VALID:1 AGE_MS:(\d+)$') -or [int]$Matches[1] -ge 100){throw 'Feedback invalid'}
  $null=Q 'M119'
  $raw=Q 'M120'
  if($raw -notmatch 'ACTIVE_MASK:([0-9A-F]{2}) CONFIG_OK:1 LED_ON_PINS:([0-9A-F]{2})$'){throw "Invalid raw input config $raw"}
  if([Convert]::ToInt32($Matches[2],16) -ne ([Convert]::ToInt32($Matches[1],16)-shl 1)){throw 'LED map mismatch'}
  $trace=Q 'M121'
  if($trace -notmatch '^M121 SAMPLES:(\d+) LOW:\d+,\d+,\d+ CHANGES:\d+,\d+,\d+ LOW_SEEN_MASK:[0-9A-F]{2}$'){throw "Trace invalid $trace"}
  if([long]$Matches[1] -le $lastSamples){throw 'Sampling clock did not advance'}
  $lastSamples=[long]$Matches[1]
  $local=Q 'M122'
  if($local -ne 'M122 LOCAL_REF:0 VALID:0 REASON:1 ORIGIN_MOTOR:0.000 REL_MOTOR:0.000 REL_JOINT:0.000 GEAR_RATIO:4.500 AGE_MS:0 WHOLE_HOMED:0 ALLOWED:0'){throw ('Unexpected initial reference: '+$local)}
  $completedRounds++
  Start-Sleep -Milliseconds 20
 }
 [pscustomobject]@{pass=$true;rounds=$completedRounds;records=$records}|ConvertTo-Json -Depth 6|Set-Content (Join-Path $PSScriptRoot ($Name+'.json')) -Encoding utf8
 $records|Select-Object -Last 5|Format-Table -Wrap -AutoSize
}catch{
 [pscustomobject]@{pass=$false;error=$_.Exception.Message;records=$records}|ConvertTo-Json -Depth 6|Set-Content (Join-Path $PSScriptRoot ($Name+'_error.json')) -Encoding utf8
 throw
}finally{if($p.IsOpen){$p.Close()};$p.Dispose()}
