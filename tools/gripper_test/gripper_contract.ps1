# 纯协议校验：不创建串口，不写板卡，不代表舵机实际到位。
Set-StrictMode -Version Latest
function Assert-GripIdentity([string]$Line,[int]$Motion) {
    $expected="M115 FIRMWARE:GripperTest PROTOCOL:GRIP1 UART:115200 MOTION_COMPILED:$Motion AUTO_MOTION:0 OPEN_DEG:100 CLOSE_DEG:72 WINDOW_MS:600 JOINTS:LOCKED"
    if($Motion -notin 0,1 -or $Line -cne $expected){throw 'Unexpected gripper identity'}
}
function Convert-GripStatus([string]$Line) {
    $pattern='^M408 STATE:([0-4]) REASON:([0-7]) USED:([01]) PWM:([01]) TARGET:([0-2]) TARGET_DEG:(0\.000|100\.000|72\.000) PORT_READY:([01]) LIMIT_MASK:([0-7]) RXERR:(0|[1-9][0-9]*) RXOVF:(0|[1-9][0-9]*) EN:DISABLED STEP:0 HOMED:0 ALLOWED:0$'
    if($Line -cnotmatch $pattern){throw 'Malformed or unsafe gripper status'}
    $s=[pscustomobject]@{State=[int]$Matches[1];Reason=[int]$Matches[2];Used=[int]$Matches[3];PWM=[int]$Matches[4];Target=[int]$Matches[5];Angle=[double]::Parse($Matches[6],[Globalization.CultureInfo]::InvariantCulture);Ready=[int]$Matches[7];Mask=[int]$Matches[8];RxError=[uint32]$Matches[9];RxOverflow=[uint32]$Matches[10]}
    $angles=@(0,100,72)
    if($s.Angle -ne $angles[$s.Target] -or $s.RxError -ne 0 -or $s.RxOverflow -ne 0){throw 'Target or receive integrity mismatch'}
    if($s.State -eq 0 -and ($s.Reason -ne 0 -or $s.Used -ne 0 -or $s.PWM -ne 0 -or $s.Target -ne 0)){throw 'Invalid idle state'}
    if($s.State -eq 1 -and ($s.Reason -ne 0 -or $s.Used -ne 1 -or $s.Target -eq 0 -or $s.Ready -ne 1)){throw 'Invalid active state'}
    if($s.State -eq 2 -and ($s.Reason -ne 0 -or $s.Used -ne 1 -or $s.PWM -ne 0 -or $s.Target -eq 0 -or $s.Ready -ne 1)){throw 'Invalid completed state'}
    if($s.State -ge 3 -and ($s.Used -ne 1 -or $s.PWM -ne 0 -or $s.Reason -eq 0)){throw 'Invalid stopped state'}
    return $s
}
function Assert-GripIdle($Status,[int]$Ready) {
    if($Status.State -ne 0 -or $Status.Ready -ne $Ready){throw 'Gripper is not unused and idle'}
}
function Assert-GripProgress($Status,[int]$Target,[int]$Mask) {
    if($Status.State -notin 1,2 -or $Status.Target -ne $Target -or $Status.Mask -ne $Mask){throw 'Action failed, changed target or changed limit mask'}
}
function Assert-GripLimits([string]$Line,[int]$Mask) {
    $labels=for($i=0;$i -lt 3;$i++){if($Mask -band (1 -shl $i)){'TRIGGERED'}else{'OPEN'}}
    if($Line -cne "M119 M1:$($labels[0]) M2:$($labels[1]) M3:$($labels[2])"){throw 'Limit snapshot mismatch'}
}
function Test-GripContract {
    $count=0
    foreach($motion in 0,1){Assert-GripIdentity "M115 FIRMWARE:GripperTest PROTOCOL:GRIP1 UART:115200 MOTION_COMPILED:$motion AUTO_MOTION:0 OPEN_DEG:100 CLOSE_DEG:72 WINDOW_MS:600 JOINTS:LOCKED" $motion;$count++}
    $idle='M408 STATE:0 REASON:0 USED:0 PWM:0 TARGET:0 TARGET_DEG:0.000 PORT_READY:0 LIMIT_MASK:2 RXERR:0 RXOVF:0 EN:DISABLED STEP:0 HOMED:0 ALLOWED:0'
    Assert-GripIdle (Convert-GripStatus $idle) 0;$count++
    foreach($mask in 0..7){foreach($target in 1,2){foreach($state in 1,2){
        $angle=if($target -eq 1){'100.000'}else{'72.000'}
        $line="M408 STATE:$state REASON:0 USED:1 PWM:0 TARGET:$target TARGET_DEG:$angle PORT_READY:1 LIMIT_MASK:$mask RXERR:0 RXOVF:0 EN:DISABLED STEP:0 HOMED:0 ALLOWED:0"
        Assert-GripProgress (Convert-GripStatus $line) $target $mask;$count++
    }}}
    foreach($change in @(@('EN:DISABLED','EN:ENABLED'),@('STEP:0','STEP:1'),@('HOMED:0','HOMED:1'),@('ALLOWED:0','ALLOWED:1'),@('PWM:0','PWM:1'),@('USED:0','USED:1'),@('TARGET_DEG:0.000','TARGET_DEG:NaN'),@('RXERR:0','RXERR:1'),@('RXOVF:0','RXOVF:1'),@('LIMIT_MASK:2','LIMIT_MASK:8'),@('STATE:0','STATE:5'),@('REASON:0','REASON:1'))){
        $rejected=$false;try{$null=Convert-GripStatus ($idle.Replace($change[0],$change[1]))}catch{$rejected=$true};if(!$rejected){throw "Missed mutation: $change"};$count++
    }
    $rejected=$false;try{Assert-GripIdentity 'other firmware' 1}catch{$rejected=$true};if(!$rejected){throw 'Identity gate failed'};$count++
    $rejected=$false;try{Assert-GripIdle (Convert-GripStatus $idle) 1}catch{$rejected=$true};if(!$rejected){throw 'Ready gate failed'};$count++
    Assert-GripLimits 'M119 M1:OPEN M2:TRIGGERED M3:OPEN' 2;$count++
    "gripper host contract: $count cases passed; no serial access"
}
