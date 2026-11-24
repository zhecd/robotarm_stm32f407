# 回零完成会丢弃尚未消费的RX，状态查询可能因此少一次应答。
# 只重发白名单只读查询；START/M999/动作指令绝不进入本函数。
# 等待期间继续KEEP，避免主机等查询时停止续租；板上截止/看门狗不变。
function Invoke-TrialStatusQuery {
 param(
  [Parameter(Mandatory)][string]$Command,
  [Parameter(Mandatory)][scriptblock]$Send,
  [Parameter(Mandatory)][scriptblock]$Receive,
  [Parameter(Mandatory)][scriptblock]$Clock
 )
 if($Command -cnotin @('M115','M123','M124','M126','M127','M128','M408','M409','M410','M114','M119')){
  throw 'Only read-only trial queries may be retried'
 }
 [long]$began=& $Clock
 [long]$lastQuery=-100;[long]$lastKeep=-70
 while($true){
  [long]$age=(& $Clock)-$began
  if($age -ge 300){throw "timeout $Command after bounded query retries"}
  if($age-$lastKeep -ge 70){& $Send 'KEEP';$lastKeep=$age}
  if($age-$lastQuery -ge 100){& $Send $Command;$lastQuery=$age}
  # 实际串口Receive须使用短ReadTimeout(如15ms)，不得无限阻塞。
  $line=& $Receive
  if($null -eq $line){continue}
  if($line.StartsWith('error:')){throw $line}
  if($line.StartsWith("$Command ")){return $line}
 }
}
