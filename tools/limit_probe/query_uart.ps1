param([Parameter(Mandatory=$true)][string]$OutputDir)
$ErrorActionPreference='Stop'
$taskPath=Join-Path $OutputDir 'uart_capture.json'
if(Test-Path -LiteralPath $taskPath){throw 'Preserve previous capture'}
$taskRecord=[ordered]@{started_at_utc=[DateTime]::UtcNow.ToString('o');port='COM4';
 messages=[Collections.Generic.List[object]]::new();result='pending'}
$taskPort=[IO.Ports.SerialPort]::new('COM4',115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
$taskPort.DtrEnable=$false;$taskPort.RtsEnable=$false;$taskPort.Handshake=[IO.Ports.Handshake]::None
$taskPort.ReadTimeout=100;$taskPort.WriteTimeout=2000;$taskPort.NewLine="`n";$taskPort.Encoding=[Text.Encoding]::ASCII
function Request([string]$command){
 if($command -notin @('M115','M408','M114','M119')){throw 'Whitelist violation'}
 $taskRecord.messages.Add([ordered]@{time_utc=[DateTime]::UtcNow.ToString('o');direction='TX';text=$command})
 $taskPort.Write($command+"`r`n");$w=[Diagnostics.Stopwatch]::StartNew()
 while($w.Elapsed.TotalSeconds -lt 3){
  try{$line=$taskPort.ReadLine().Trim()}catch [TimeoutException]{continue}
  if(-not $line){continue}
  $taskRecord.messages.Add([ordered]@{time_utc=[DateTime]::UtcNow.ToString('o');direction='RX';text=$line})
  if($line.StartsWith($command+' ')){return $line}
  throw "Unexpected reply: $line"
 };throw "Timeout; do not resend: $command"
}
try {
 $taskPort.Open();$identity=Request 'M115'
 if($identity -ne 'M115 FIRMWARE:LimitProbe PROTOCOL:LPUART1 UART:115200 MOTOR_ENABLE:LOCKED FEATURES:GPIO,EXTI,POLL_UART'){throw 'Wrong firmware identity'}
 $taskRecord.identity=$identity;$watch=[Diagnostics.Stopwatch]::StartNew();$lastProgress=0
 while($watch.Elapsed.TotalSeconds -lt 75){
  $status=Request 'M408';$limits=Request 'M119';$encoder=Request 'M114';$id=Request 'M115'
  if($status -notmatch '^M408 STATE:LIMIT_PROBE EN:DISABLED PHASE:[0-4] MS:\d+ PENDING:\d+,\d+,\d+$'){throw 'Invalid probe status'}
  if($limits -notmatch '^M119 M1:(OPEN|TRIGGERED) M2:(OPEN|TRIGGERED) M3:(OPEN|TRIGGERED)$'){throw 'Invalid limits'}
  if($encoder -ne 'M114 RAWENC:0,0,0 VALID:0,0,0 ENCODER_POLL:0'){throw 'Unexpected encoder mode'}
  if($watch.Elapsed.TotalSeconds-$lastProgress -ge 20){Write-Output $status;$lastProgress=$watch.Elapsed.TotalSeconds}
  Start-Sleep -Milliseconds 250
 }
 $taskRecord.elapsed_seconds=$watch.Elapsed.TotalSeconds;$taskRecord.result='query_completed'
}catch{$taskRecord.result='failed';$taskRecord.error=$_.Exception.Message}
finally {
 if($taskPort.IsOpen){$taskPort.Close()};$taskPort.Dispose();$taskRecord.serial_closed=$true
 $taskRecord.completed_at_utc=[DateTime]::UtcNow.ToString('o')
 $taskRecord | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath $taskPath -Encoding utf8
}
[pscustomobject]$taskRecord | Select-Object result,error,serial_closed,elapsed_seconds | ConvertTo-Json -Compress
if($taskRecord.result -eq 'failed'){exit 1}
