# 纯合同解析，无串口、烧录、复位和机械动作。
function Assert-TrialIdentity([string]$Line,[int]$Compiled=1,[bool]$Evidence=$false) {
    $expected="M115 FIRMWARE:RobotArmStartupTrial PROTOCOL:STARTUP1 UART:115200 MOTION_COMPILED:$Compiled AUTO_MOTION:0 SEEK_MAX:200 STEP_US:20000 DEADLINE_MS:90000 BACKOFF:4000,1240,200"
    if($Evidence){$expected+=' FEATURES:IRQ_EVIDENCE'}
    if($Line -cne $expected){throw 'Wrong trial identity or profile'}
}
function Convert-TrialStatus([string]$Line) {
    if($Line -cnotmatch '^M123 STATE:([0-3]) REASON:([0-9]+) USED:([01]) READY_HOMED:([01]) READY_ALLOWED:([01]) EN:([01]) SEEK:([0-9]+),([0-9]+),([0-9]+) BACK:([0-9]+),([0-9]+),([0-9]+) ELAPSED_MS:([0-9]+)$'){throw 'Malformed trial status'}
    $m=$Matches.Clone()
    $r=[pscustomobject]@{State=[int]$m[1];Reason=[int]$m[2];Used=[int]$m[3];Homed=[int]$m[4];Allowed=[int]$m[5];Enable=[int]$m[6];Seek=@([uint32]$m[7],[uint32]$m[8],[uint32]$m[9]);Back=@([uint32]$m[10],[uint32]$m[11],[uint32]$m[12]);Elapsed=[uint32]$m[13]}
    for($a=0;$a -lt 3;$a++){
        if($r.Seek[$a] -gt 200 -or $r.Back[$a] -gt @(4000,1240,200)[$a]){throw 'Pulse budget exceeded'}
    }
    if($r.State -eq 3){throw "Trial failed, first reason $($r.Reason)"}
    if($r.State -eq 2 -and ($r.Reason -ne 1 -or $r.Used -ne 1 -or $r.Homed -ne 1 -or $r.Allowed -ne 1 -or $r.Enable -ne 0 -or ($r.Back -join ',') -ne '4000,1240,200')){throw 'False completion'}
    return $r
}
function Convert-TrialFeedback([string]$Line) {
    if($Line -cnotmatch '^M124 FRAME:MOTOR ZERO_EPOCH:([01]) SEQ:([0-9]+) AGE_MS:([0-9]+) VALID:7 MOTOR_DEG:(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+),(-?[0-9]+\.[0-9]+) LIMIT_MASK:([0-7])$'){throw 'Invalid or unavailable motor feedback'}
    $m=$Matches.Clone()
    if([uint32]$m[2] -eq 0 -or [uint32]$m[3] -ge 150){throw 'Stale feedback'}
    $motor=@([double]::Parse($m[4],[Globalization.CultureInfo]::InvariantCulture),[double]::Parse($m[5],[Globalization.CultureInfo]::InvariantCulture),[double]::Parse($m[6],[Globalization.CultureInfo]::InvariantCulture))
    foreach($angle in $motor){if([double]::IsInfinity($angle) -or [double]::IsNaN($angle)){throw 'Nonfinite feedback'}}
    return [pscustomobject]@{Epoch=[int]$m[1];Seq=[uint32]$m[2];Age=[uint32]$m[3];LimitMask=[int]$m[7];Motor=$motor}
}
function Assert-TrialGripperDone([string]$Line) {
    if($Line -cne 'M410 BOUND:1 TARGET:CLOSE ANGLE_DEG:72 PWM_REQUESTED:0 HOLD_MS:600 ELAPSED_MS:600 ERROR:0 POSITION:UNMEASURED'){throw 'Startup gripper stop not confirmed'}
}
function Test-TrialStopEvidence([string]$Trial,[string]$Robot,[string]$Gripper) {
    # “试验没完成”与“停止状态已确认”必须分别报告；查询不清首故障。
    return $Trial -cmatch '^M123 .* EN:0 ' -and
        $Robot -cmatch '^M408 STATE:(OUTPUT_LOCKED|FAILED) HOMED:0 ALLOWED:0 ' -and
        $Gripper -cmatch '^M410 .* PWM_REQUESTED:0 .* ERROR:0 POSITION:UNMEASURED$'
}
function Test-TrialContract {
    $id='M115 FIRMWARE:RobotArmStartupTrial PROTOCOL:STARTUP1 UART:115200 MOTION_COMPILED:1 AUTO_MOTION:0 SEEK_MAX:200 STEP_US:20000 DEADLINE_MS:90000 BACKOFF:4000,1240,200'
    Assert-TrialIdentity $id
    Assert-TrialIdentity ($id+' FEATURES:IRQ_EVIDENCE') 1 $true
    $caught=$false;try{Assert-TrialIdentity $id 1 $true}catch{$caught=$true};if(!$caught){throw 'Missing IRQ evidence capability accepted'}
    $done='M123 STATE:2 REASON:1 USED:1 READY_HOMED:1 READY_ALLOWED:1 EN:0 SEEK:30,60,90 BACK:4000,1240,200 ELAPSED_MS:82000'
    $null=Convert-TrialStatus $done
    $raw='M124 FRAME:MOTOR ZERO_EPOCH:0 SEQ:1 AGE_MS:12 VALID:7 MOTOR_DEG:1.000,-2.000,3.000 LIMIT_MASK:0'
    $null=Convert-TrialFeedback $raw
    $grip='M410 BOUND:1 TARGET:CLOSE ANGLE_DEG:72 PWM_REQUESTED:0 HOLD_MS:600 ELAPSED_MS:600 ERROR:0 POSITION:UNMEASURED'
    Assert-TrialGripperDone $grip
    $failed='M123 STATE:3 REASON:7 USED:1 READY_HOMED:0 READY_ALLOWED:0 EN:0 SEEK:0,0,0 BACK:0,0,0 ELAPSED_MS:100'
    $robot='M408 STATE:OUTPUT_LOCKED HOMED:0 ALLOWED:0 MODE:GCODE FAULT:8 MOTION_FAULT:7'
    if(!(Test-TrialStopEvidence $failed $robot $grip)){throw 'Failed but stopped status lost'}
    if((Test-TrialStopEvidence ($failed.Replace('EN:0','EN:1')) $robot $grip) -or
       (Test-TrialStopEvidence $failed $robot ($grip.Replace('REQUESTED:0','REQUESTED:1')))){throw 'Active output accepted as stopped'}
    $bad=@($done.Replace('STATE:2','STATE:3'),$done.Replace('EN:0','EN:1'),$done.Replace('HOMED:1','HOMED:0'),$done.Replace('4000,1240','4001,1240'),$done.Replace('4000,1240','3999,1240'),$done.Replace('30,60,90','201,60,90'))
    foreach($x in $bad){$caught=$false;try{$null=Convert-TrialStatus $x}catch{$caught=$true};if(!$caught){throw 'Negative status accepted'}}
    foreach($x in @($raw.Replace('AGE_MS:12','AGE_MS:150'),$raw.Replace('VALID:7','VALID:3'),$raw.Replace('1.000','nan'),$raw.Replace('SEQ:1','SEQ:0'))){$caught=$false;try{$null=Convert-TrialFeedback $x}catch{$caught=$true};if(!$caught){throw 'Negative feedback accepted'}}
    $caught=$false;try{Assert-TrialIdentity ($id.Replace('COMPILED:1','COMPILED:0'))}catch{$caught=$true};if(!$caught){throw 'Locked candidate accepted as motion'}
    foreach($x in @($grip.Replace('REQUESTED:0','REQUESTED:1'),$grip.Replace('ERROR:0','ERROR:2'))){$caught=$false;try{Assert-TrialGripperDone $x}catch{$caught=$true};if(!$caught){throw 'Gripper stop failure accepted'}}
    'Trial contract positive and 16 negative cases passed; no serial opened'
}
