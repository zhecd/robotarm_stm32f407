$ErrorActionPreference='Stop'
$log=[Collections.Generic.List[object]]::new()
function Open-Port {
 $script:p=[IO.Ports.SerialPort]::new('COM4',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
 $p.DtrEnable=$false;$p.RtsEnable=$false;$p.ReadTimeout=15;$p.WriteTimeout=100;$p.NewLine="`n";$p.Open();$p.DiscardInBuffer()
}
function Send-Line([string]$c){
 $p.Write("$c`n");$log.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');command=$c;reply=$null})
}
function Q([string]$c){
 Send-Line $c;$w=[Diagnostics.Stopwatch]::StartNew()
 while($w.ElapsedMilliseconds -lt 180){
  try{$x=$p.ReadLine().TrimEnd([char]13)}catch [TimeoutException]{continue}
  $log.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');command=$c;reply=$x})
  if($x.StartsWith('error:')){throw $x}
  if($x.StartsWith("$c ")){return $x}
 }
 throw "timeout $c"
}
function Wait-BootReady {
 $watch=[Diagnostics.Stopwatch]::StartNew()
 while($watch.ElapsedMilliseconds -lt 1000){
  $h=Q 'M126'
  if($h -notmatch '^M126 PHASE:0 REASON:0 AXIS:0 CONTACTS:0 COMPLETE:0 OWNED:[0-7] REARMED:0 SEEK:0,0,0 BACK:0,0,0 HOLD:0 HOLDS:0 COMMITTED:0 BOOT_READY:([01])$'){throw $h}
  if($Matches[1] -eq '1'){return}
  Start-Sleep -Milliseconds 10
 }
 throw 'Startup inputs remain unqualified; no START permitted'
}
function Check-Wait([int]$compiled){
 $id=Q 'M115'
 if($id -cne "M115 FIRMWARE:RobotArmStartupTrial PROTOCOL:STARTUP8 UART:115200 MOTION_COMPILED:$compiled AUTO_MOTION:0 SEEK_TIMEOUT_MS:0 STEP_US:2000 BACKOFF_STEP_US:2000 DEADLINE_MS:220000 BACKOFF:4000,1240,400 SEQUENCE:0,1,2 DIR:0,0,1 FEATURES:IRQ_EVIDENCE,SEQUENTIAL_HOME,M126,LIMIT_STATE_1MS,HOME_INPUT_CONFIRM,M127,CONTACT_CANCEL_10MS"){throw "Wrong identity: $id"}
 for($i=0;$i -lt 5;$i++){
  $s=Q 'M123';if($s -cne 'M123 STATE:0 REASON:0 USED:0 READY_HOMED:0 READY_ALLOWED:0 EN:0 SEEK:0,0,0 BACK:0,0,0 ELAPSED_MS:0'){throw $s}
  $f=Q 'M124';if($f -notmatch 'AGE_MS:(\d+) VALID:7 ' -or [int]$Matches[1] -ge 150){throw $f}
  Wait-BootReady
  $s=Q 'M408';if($s -cne 'M408 STATE:CHECKING HOMED:0 ALLOWED:0 MODE:GCODE FAULT:0 MOTION_FAULT:0 QUEUE:0 PENDING:0 RECOVERY:0 IRQ_LATCH_MASK:0'){throw $s}
  $null=Q 'M119'
  $g=Q 'M410';if($g -notmatch '^M410 BOUND:0 TARGET:NONE ANGLE_DEG:0 PWM_REQUESTED:0 '){throw $g}
 }
 Wait-BootReady
}
