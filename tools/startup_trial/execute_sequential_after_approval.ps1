# 逐轴GPIO试验执行器；必须在新证据目录记录当次明确许可。
# 部署、独立回读和锁定检查由外层先完成；本脚本不烧录、不复位。
param([Parameter(Mandatory)][string]$EvidenceDirectory)
$ErrorActionPreference='Stop'
$dir=(Resolve-Path -LiteralPath $EvidenceDirectory).Path
$approval=Get-Content -Raw -LiteralPath (Join-Path $dir 'authorization.json')|ConvertFrom-Json
if(!$approval.approved -or !$approval.single_START_only -or $approval.UART -ne 115200 -or
   ($approval.backoff_steps -join ',') -ne '4000,1240,400' -or $approval.total_timeout_ms -ne 220000){
 throw 'Missing approved single-trial contract; no port opened'
}
if(Test-Path -LiteralPath (Join-Path $dir 'start_once.claim')){throw 'Trial already claimed; no port opened'}
# serial_common.ps1来自本轮冻结材料，Check-Wait必须核对其完整固件身份。
. (Join-Path $dir 'serial_common.ps1')
. "$PSScriptRoot/query_status.ps1"
$started=$false;$finished=$false;$terminal=$null;$failure=$null
$holdMs=0
if($approval.PSObject.Properties.Name -contains 'ready_hold_ms'){$holdMs=[int]$approval.ready_hold_ms}
if($holdMs -notin @(0,10000)){throw 'Unexpected READY hold contract; no port opened'}
$script:trialQueryWatch=[Diagnostics.Stopwatch]::StartNew()
function Active-Query([string]$c){
 $send={param($text) Send-Line $text}
 $receive={
  try{$x=$p.ReadLine().TrimEnd([char]13)}catch [TimeoutException]{return $null}
  $log.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');command=$c;reply=$x})
  return $x
 }
 # Clock是被调函数参数，未限定的同名$clock会被其遮蔽；显式脚本作用域。
 Invoke-TrialStatusQuery $c $send $receive {$script:trialQueryWatch.ElapsedMilliseconds}
}
try{
 Open-Port;Check-Wait 1
 $limits=Q 'M119'
 # 本参数允许已确认压下的M3从当前位置固定退10°；M1/M2要求松开。
 if($limits -cnotmatch '^M119 M1:OPEN M2:OPEN M3:(OPEN|TRIGGERED)$'){throw $limits}
 $claim=[IO.File]::Open((Join-Path $dir 'start_once.claim'),[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
 $claim.Dispose();$started=$true
 # START恰好发送一次。应答丢失也不重发，进入停止清理。
 if((Q 'START') -cne 'START ACCEPTED'){throw 'START not acknowledged'}
 $watch=[Diagnostics.Stopwatch]::StartNew()
 while($watch.ElapsedMilliseconds -lt 222000){
  $s=Active-Query 'M123'
  if($s -match '^M123 STATE:([23]) '){$terminal=$s;$finished=$true;break}
  foreach($c in @('M124','M126','M127')){$null=Active-Query $c}
  if($holdMs){$null=Active-Query 'M128'}
  Start-Sleep -Milliseconds 10
 }
 if(!$finished){throw 'No terminal state by222s host deadline'}
 if($holdMs -and $terminal -match '^M123 STATE:2 '){
  $hold=Q 'M128'
  if($hold -notmatch '^M128 HOLD_STATE:2 DURATION_MS:10000 ELAPSED_MS:(\d+)$' -or [long]$Matches[1] -lt 10000){
   throw "READY hold completion unverified: $hold"
  }
 }
 for($i=0;$i -lt 5;$i++){
  foreach($c in @('M123','M126','M127','M124','M119','M408','M410')){$null=Q $c}
 }
}catch{
 $failure=$_.Exception.Message
 if($started -and !$finished -and $p -and $p.IsOpen){try{Send-Line 'M112'}catch{}}
}finally{
 if($p){$p.Close();$p.Dispose()}
 [pscustomobject]@{started=$started;terminal_observed=$finished;terminal=$terminal;failure=$failure;
  automatic_retry=$false;read_only_query_retries=$true;trace=@($log.ToArray())}|
 ConvertTo-Json -Depth 7|Set-Content -Encoding utf8 -LiteralPath (Join-Path $dir 'action.json')
}
if($failure){throw $failure};$terminal
