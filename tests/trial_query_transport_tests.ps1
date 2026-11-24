# 虚拟时钟/传输，无串口、无复位、无板卡动作。
$ErrorActionPreference='Stop'
. "$PSScriptRoot/../tools/startup_trial/query_status.ps1"
function Check([bool]$ok,[string]$message){if(!$ok){throw $message}}
foreach($scenario in @('normal','rx_discard','late','silent','error','stale')){
 $script:ms=0;$script:sent=[Collections.Generic.List[object]]::new()
 $script:attempts=0;$script:received=$false;$script:mode=$scenario
 $send={param($s) $script:sent.Add(@($script:ms,$s));if($s -eq 'M124'){$script:attempts++}}
 $receive={
  $script:ms+=15
  if($script:mode -eq 'error'){return 'error: stopped'}
  if($script:mode -eq 'stale' -and $script:ms -eq 15){return 'M123 STATE:1'}
  $ready=switch($script:mode){normal{15} rx_discard{120} late{255} stale{30} default{999}}
  if($script:ms -ge $ready -and !$script:received){$script:received=$true;return 'M124 VALID:7'}
  return $null
 }
 $failed=$false;$answer=$null
 try{$answer=Invoke-TrialStatusQuery M124 $send $receive {$script:ms}}catch{$failed=$true}
 Check ($failed -eq ($scenario -in @('silent','error'))) "Wrong outcome $scenario"
 if(!$failed){Check ($answer -ceq 'M124 VALID:7') "Wrong response $scenario"}
 Check ($script:ms -le 300) "Unbounded wait $scenario"
 $keeps=@($script:sent | Where-Object {$_[1] -eq 'KEEP'})
 for($i=1;$i -lt $keeps.Count;$i++){Check (($keeps[$i][0]-$keeps[$i-1][0]) -le 85) 'Heartbeat lost'}
 if($scenario -eq 'rx_discard'){Check ($script:attempts -eq 2) 'Dropped query was not retried'}
 if($scenario -eq 'silent'){Check ($script:attempts -eq 3) 'Retry budget changed'}
}
foreach($command in @('START','M999','M112','G28','KEEP')){
 $rejected=$false
 try{Invoke-TrialStatusQuery $command {throw 'Transport must not run'} {} {0}}catch{$rejected=$_.Exception.Message -eq 'Only read-only trial queries may be retried'}
 Check $rejected "Action accepted by retry function: $command"
}
# 从真实执行器AST提取Active-Query函数，验证调用层而非只测注入的假时钟。
$tokens=$null;$errors=$null
$ast=[Management.Automation.Language.Parser]::ParseFile((Resolve-Path "$PSScriptRoot/../tools/startup_trial/execute_sequential_after_approval.ps1"),[ref]$tokens,[ref]$errors)
Check (!$errors) 'Executor syntax error'
$fn=$ast.Find({param($n) $n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq 'Active-Query'},$true)
Invoke-Expression $fn.Extent.Text
Add-Type -TypeDefinition @'
public class TrialQueryFakePort {
 public int QueryAttempts; public int ReadCount; public bool Reply;
 public string ReadLine() {
  System.Threading.Thread.Sleep(15); ReadCount++;
  if(ReadCount>50) throw new System.InvalidOperationException("Clock did not progress");
  if(Reply && QueryAttempts>=2) return "M124 VALID:7";
  throw new System.TimeoutException();
 }
}
'@
foreach($scenario in @('actual_retry','actual_timeout')){
 $script:trialQueryWatch=[Diagnostics.Stopwatch]::StartNew()
 $script:clock=$script:trialQueryWatch # 保留原错误的变量名，确保新代码不会被Clock参数遮蔽。
 $script:attempts=0;$script:keeps=0;$script:readCount=0;$script:mode=$scenario
 $script:log=[Collections.Generic.List[object]]::new()
 function Send-Line([string]$c){if($c -eq 'M124'){$script:attempts++;$script:p.QueryAttempts++};if($c -eq 'KEEP'){$script:keeps++}}
 $script:p=[TrialQueryFakePort]::new();$p.Reply=$scenario -eq 'actual_retry'
 $failed=$null;$answer=$null
 try{$answer=Active-Query 'M124'}catch{$failed=$_.Exception.Message}
 if($scenario -eq 'actual_retry'){
  Check ($answer -eq 'M124 VALID:7' -and $attempts -ge 2 -and $keeps -ge 2) 'Actual executor did not retry/renew'
 }else{
  Check ($failed -like 'timeout M124 after bounded query retries') 'Actual executor timeout not bounded'
  Check ($keeps -ge 3 -and $p.ReadCount -lt 50) 'Actual executor heartbeat or clock failed'
 }
}
'13 offline transport scenarios passed; includes actual executor clock; no device access'
