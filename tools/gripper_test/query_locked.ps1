param([switch]$Connect,[string]$Port='COM4',[int]$Rounds=20,[string]$OutputPath,[switch]$SelfTest)
$ErrorActionPreference='Stop'
. "$PSScriptRoot/gripper_contract.ps1"
if($SelfTest){Test-GripContract;return}
if(!$Connect -or !$OutputPath -or $Rounds -lt 1 -or $Rounds -gt 200){throw 'Explicit Connect, OutputPath and 1..200 rounds required; serial not opened'}
if(Test-Path -LiteralPath $OutputPath){throw 'Output already exists'}
. "$PSScriptRoot/serial_session.ps1"
$serial=$null;$passed=$false;$failure=$null;$log=[Collections.Generic.List[object]]::new()
try {
    $serial=New-GripPort $Port;Assert-GripIdentity (Invoke-GripQuery $serial M115 $log) 0
    for($i=0;$i -lt $Rounds;$i++){
        $s=Convert-GripStatus (Invoke-GripQuery $serial M408 $log);Assert-GripIdle $s 0
        Assert-GripLimits (Invoke-GripQuery $serial M119 $log) $s.Mask
        Start-Sleep -Milliseconds 30
    };$passed=$true
}catch{$failure=$_.Exception.Message}finally{
    if($null -ne $serial){$serial.Close();$serial.Dispose()}
    [pscustomobject]@{passed=$passed;failure=$failure;baud=115200;motion_commands=0;trace=@($log.ToArray())}|ConvertTo-Json -Depth 7|Set-Content -Encoding utf8 -LiteralPath $OutputPath
}
if(!$passed){throw $failure}
