# 仅由显式连接/动作入口加载；115200、8N1，不使用DTR/RTS复位。
function New-GripPort([string]$Name) {
    $s=[IO.Ports.SerialPort]::new($Name,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
    $s.DtrEnable=$false;$s.RtsEnable=$false;$s.ReadTimeout=50;$s.WriteTimeout=100;$s.NewLine="`n"
    $s.Open();$s.DiscardInBuffer();return $s
}
function Invoke-GripQuery($Serial,[string]$Command,$Log) {
    $Serial.Write("$Command`n")
    $clock=[Diagnostics.Stopwatch]::StartNew()
    while($clock.ElapsedMilliseconds -lt 150){
        try{$line=$Serial.ReadLine().TrimEnd([char]13)}catch [TimeoutException]{continue}
        if($line.Length){$Log.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');command=$Command;reply=$line});return $line}
    }
    throw "Reply timeout: $Command"
}
