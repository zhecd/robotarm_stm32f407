param([switch]$ExecuteAuthorized,[string]$RecordDirectory,[string]$Port='COM4',[switch]$RequireIrqEvidence,[switch]$SelfTest)
$ErrorActionPreference='Stop'
. "$PSScriptRoot/contract.ps1"
if($SelfTest){Test-TrialContract;return}
# 在构造串口前拒绝。正式许可仅适用于一次本合同，不允许M999/重启重放。
if(!$ExecuteAuthorized -or !$RecordDirectory){throw 'Explicit whole-startup motion approval and fresh RecordDirectory required; serial not opened'}
$dir=[IO.Path]::GetFullPath($RecordDirectory)
if(!(Test-Path -LiteralPath $dir -PathType Container)){throw 'RecordDirectory must exist'}
foreach($f in Get-ChildItem -LiteralPath $dir -File){if($f.Name -match '(claim|result|trace)'){throw 'Existing execution evidence; replay forbidden'}}
$serial=$null;$identified=$false;$complete=$false;$claimed=$false;$failure=$null;$outputsStopped=$false
$log=[Collections.Generic.List[object]]::new();$script:active=$false;$script:clock=[Diagnostics.Stopwatch]::new();$script:lastKeep=-100L
function Write-Trial([string]$Command){
    $serial.Write("$Command`n");$log.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');tx=$Command})
}
function Keep-Trial {
    if($script:active -and $script:clock.ElapsedMilliseconds-$script:lastKeep -ge 70){Write-Trial KEEP;$script:lastKeep=$script:clock.ElapsedMilliseconds}
}
function Query-Trial([string]$Command,[string]$Prefix){
    Keep-Trial;Write-Trial $Command;$wait=[Diagnostics.Stopwatch]::StartNew()
    while($wait.ElapsedMilliseconds -lt 160){
        Keep-Trial
        try{$line=$serial.ReadLine().TrimEnd([char]13)}catch [TimeoutException]{continue}
        $log.Add([pscustomobject]@{utc=[DateTime]::UtcNow.ToString('o');rx=$line})
        if($line.StartsWith('error:')){throw $line}
        if($line.StartsWith($Prefix)){return $line}
    }
    throw "Reply timeout: $Command"
}
try{
    $serial=[IO.Ports.SerialPort]::new($Port,115200,[IO.Ports.Parity]::None,8,[IO.Ports.StopBits]::One)
    $serial.DtrEnable=$false;$serial.RtsEnable=$false;$serial.ReadTimeout=20;$serial.WriteTimeout=100;$serial.NewLine="`n"
    $serial.Open();$serial.DiscardInBuffer()
    Assert-TrialIdentity (Query-Trial M115 'M115 ') 1 ([bool]$RequireIrqEvidence);$identified=$true
    for($i=0;$i -lt 5;$i++){
        $s=Convert-TrialStatus (Query-Trial M123 'M123 ')
        if($s.State -ne 0 -or $s.Used -ne 0 -or $s.Enable -ne 0 -or $s.Reason -ne 0 -or $s.Homed -ne 0 -or $s.Allowed -ne 0 -or ($s.Seek -join ',') -ne '0,0,0' -or ($s.Back -join ',') -ne '0,0,0'){throw 'Trial not freshly waiting'}
        $null=Convert-TrialFeedback (Query-Trial M124 'M124 ')
        if((Query-Trial M119 'M119 ') -cne 'M119 M1:OPEN M2:OPEN M3:OPEN'){throw 'Preflight requires three released limits'}
        if((Query-Trial M408 'M408 ') -notmatch 'STATE:CHECKING HOMED:0 ALLOWED:0 .*FAULT:0 MOTION_FAULT:0 .*IRQ_LATCH_MASK:0$'){throw 'Startup preflight not healthy'}
    }
    $file=[IO.File]::Open((Join-Path $dir 'startup_once.claim'),[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    try{$bytes=[Text.Encoding]::UTF8.GetBytes("STARTUP1,115200,200/4000/1240/200,20000us,90000ms,$([DateTime]::UtcNow.ToString('o'))");$file.Write($bytes,0,$bytes.Length)}finally{$file.Dispose()}
    $claimed=$true;$script:clock.Start();$script:active=$true
    if((Query-Trial START 'START ') -cne 'START ACCEPTED'){throw 'START not accepted'}
    $lastSample=0L
    while($script:clock.ElapsedMilliseconds -lt 90500){
        Keep-Trial
        if($script:clock.ElapsedMilliseconds-$lastSample -ge 100){
            $lastSample=$script:clock.ElapsedMilliseconds
            $s=Convert-TrialStatus (Query-Trial M123 'M123 ')
            $null=Convert-TrialFeedback (Query-Trial M124 'M124 ')
            $grip=Query-Trial M410 'M410 '
            if($grip -notmatch ' ERROR:0 POSITION:UNMEASURED$'){throw 'Gripper driver error'}
            if($s.State -eq 2){Assert-TrialGripperDone $grip;$complete=$true;break}
            if($s.State -ne 1 -or $s.Used -ne 1){throw 'Trial lost active state'}
        }
        Start-Sleep -Milliseconds 5
    }
    $script:active=$false
    if(!$complete){throw 'Host deadline reached'}
    Start-Sleep -Milliseconds 30
    $post=Query-Trial M408 'M408 '
    if($post -notmatch 'STATE:OUTPUT_LOCKED HOMED:0 ALLOWED:0 .*FAULT:7 '){throw 'Final interlock not confirmed'}
    $null=Convert-TrialStatus (Query-Trial M123 'M123 ')
    $outputsStopped=$true
}catch{
    $failure=$_.Exception.Message;$complete=$false;$script:active=$false
    if($identified -and $null -ne $serial){
        try{$null=Query-Trial M112 'M112OK'}catch{}
        try{
            $stopTrial=Query-Trial M123 'M123 '
            $stopRobot=Query-Trial M408 'M408 '
            $stopGrip=Query-Trial M410 'M410 '
            $outputsStopped=Test-TrialStopEvidence $stopTrial $stopRobot $stopGrip
        }catch{}
    }
}finally{
    if($null -ne $serial){$serial.Close();$serial.Dispose()}
    [pscustomobject]@{complete=$complete;failure=$failure;claimed=$claimed;baud=115200;normal_ready_boundary_recorded=$complete;final_outputs_locked=$outputsStopped;physical_accuracy_verified=$false;trace=@($log.ToArray())}|ConvertTo-Json -Depth 7|Set-Content -Encoding utf8 -LiteralPath (Join-Path $dir 'result.json')
}
if(!$complete){throw $failure}
'Single startup trial completed and outputs relocked; no subsequent action allowed.'
