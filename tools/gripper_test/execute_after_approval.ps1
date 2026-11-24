param([ValidateSet('OPEN','CLOSE')][string]$Action,[switch]$ExecuteAuthorized,[string]$RecordDirectory,[string]$Port='COM4',[switch]$SelfTest)
$ErrorActionPreference='Stop'
. "$PSScriptRoot/gripper_contract.ps1"
if($SelfTest){Test-GripContract;return}
# 缺少本次明确许可、目标和独立记录目录时，在构造串口前退出。
if(!$ExecuteAuthorized -or !$Action -or !$RecordDirectory){throw 'Explicit one-action approval, OPEN/CLOSE and fresh RecordDirectory required; serial not opened'}
$dir=[IO.Path]::GetFullPath($RecordDirectory)
if(!(Test-Path -LiteralPath $dir -PathType Container)){throw 'RecordDirectory must already exist'}
foreach($file in Get-ChildItem -LiteralPath $dir -File){if($file.Name -match '(claim|result|trace)'){throw 'Existing execution evidence: do not replay'}}
. "$PSScriptRoot/serial_session.ps1"
$serial=$null;$identified=$false;$claimed=$false;$complete=$false;$failure=$null
$log=[Collections.Generic.List[object]]::new()
try {
    $serial=New-GripPort $Port
    Assert-GripIdentity (Invoke-GripQuery $serial M115 $log) 1;$identified=$true
    $mask=-1
    for($i=0;$i -lt 5;$i++){
        $status=Convert-GripStatus (Invoke-GripQuery $serial M408 $log);Assert-GripIdle $status 1
        if($mask -eq -1){$mask=$status.Mask};if($status.Mask -ne $mask){throw 'Unstable preflight limits'}
        Assert-GripLimits (Invoke-GripQuery $serial M119 $log) $mask
    }
    $claim=[IO.File]::Open((Join-Path $dir 'action_once.claim'),[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
    try{$bytes=[Text.Encoding]::UTF8.GetBytes("$Action,115200,600ms,$([DateTime]::UtcNow.ToString('o'))");$claim.Write($bytes,0,$bytes.Length)}finally{$claim.Dispose()}
    $claimed=$true;$clock=[Diagnostics.Stopwatch]::StartNew();$lastKeep=-100L
    if((Invoke-GripQuery $serial $Action $log) -cne 'accepted: one-shot gripper; lease required'){throw 'Action was not accepted'}
    while($clock.ElapsedMilliseconds -lt 800){
        if($clock.ElapsedMilliseconds-$lastKeep -ge 70){
            $reply=Invoke-GripQuery $serial KEEP $log
            if($reply -cnotin 'KEEP OK','KEEP IDLE'){throw 'Invalid keep reply'}
            $lastKeep=$clock.ElapsedMilliseconds
        }
        $status=Convert-GripStatus (Invoke-GripQuery $serial M408 $log)
        Assert-GripProgress $status $(if($Action -eq 'OPEN'){1}else{2}) $mask
        if($status.State -eq 2){$complete=$true;break}
        Start-Sleep -Milliseconds 20
    }
    if(!$complete){throw 'Host deadline reached'}
    $post=[Diagnostics.Stopwatch]::StartNew()
    while($post.ElapsedMilliseconds -lt 250){
        $status=Convert-GripStatus (Invoke-GripQuery $serial M408 $log)
        Assert-GripProgress $status $(if($Action -eq 'OPEN'){1}else{2}) $mask
        if($status.State -ne 2){throw 'Completed state changed'}
        Start-Sleep -Milliseconds 25
    }
} catch {
    $failure=$_.Exception.Message;$complete=$false
    # 只对已核对身份的测试固件停止；保留第一次失败，不复位、不重试。
    if($identified -and $null -ne $serial){try{$null=Invoke-GripQuery $serial M112 $log}catch{}}
} finally {
    if($null -ne $serial){$serial.Close();$serial.Dispose()}
    [pscustomobject]@{complete=$complete;failure=$failure;action=$Action;claimed=$claimed;baud=115200;window_ms=600;host_deadline_ms=800;physical_arrival_verified=$false;trace=@($log.ToArray())}|ConvertTo-Json -Depth 7|Set-Content -Encoding utf8 -LiteralPath (Join-Path $dir 'result.json')
}
if(!$complete){throw $failure}
'One PWM command window completed; physical gripper position is not measured.'
