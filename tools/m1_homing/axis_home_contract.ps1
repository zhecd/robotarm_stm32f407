# 通用单轴HOME的纯合同。仅解释字符串/数值，不创建串口，不操作文件或硬件。
# 轴号是板端编号1..3；D0/D1是GPIO方向，不自动等同于机械正负或趋近限位。
function Get-AxisHomeImage([int]$Id,[int]$Motion=1){
 if($Id -notin 1..3 -or $Motion -notin @(0,1)){throw 'Invalid axis/image selection'}
 $prefix=if($Id -eq 1){'M115 FIRMWARE:M1HomingPrepare PROTOCOL:M1HOME8 '}else{"M115 FIRMWARE:AxisHomingPrepare PROTOCOL:AXISHOME1 AXIS:$Id "}
 return $prefix+"UART:115200 MOTION_COMPILED:$Motion AUTO_MOTION:0 JOG_STEPS:20 JOG_DEADLINE_MS:500 SEEK_STEPS:200 BACKOFF_STEPS:40 HOME_DEADLINE_MS:5500 STEP_PERIOD_MS:20"
}
function Assert-AxisHomeRequest([bool]$Authorized,[int]$Id,[string]$Direction,[bool]$Verified,[string]$Evidence){
 # 默认拒绝和缺少方向依据的分支必须在创建SerialPort之前执行。
 if(!$Authorized){throw 'Explicit single-axis HOME approval required; no serial opened'}
 if($Id -notin 1..3){throw 'Explicit axis 1..3 required; no serial opened'}
 if($Direction -notin @('D0','D1') -or !$Verified -or [string]::IsNullOrWhiteSpace($Evidence)){
  throw 'Verified mechanical seek direction and its evidence required; no serial opened'
 }
}
function Read-AxisHomeStatus([string]$Reply){
 if(!$Reply.StartsWith('M408 ')){throw 'Expected M408'}
 $keys=@('STATE','REASON','USED','EN','STEP','JOG','SEEK','BACKOFF','DRIVER','DIR_CONFIRMED','HOMED','ALLOWED','RXERR','RXOVF')
 $tokens=$Reply.Substring(5).Split(' ');$fields=@{}
 if($tokens.Count -ne $keys.Count){throw 'Unexpected M408 field count'}
 foreach($token in $tokens){
  $pair=$token.Split(':')
  if($pair.Count -ne 2 -or $pair[0] -cnotin $keys -or $fields.ContainsKey($pair[0])){throw 'Unknown or duplicate M408 field'}
  if($pair[0] -eq 'EN'){
   if($pair[1] -cnotin @('ENABLED','DISABLED')){throw 'Invalid EN value'}
   $fields[$pair[0]]=$pair[1]
  }else{
   [uint32]$value=0
   if($pair[1] -notmatch '^(0|[1-9][0-9]*)$' -or ![uint32]::TryParse($pair[1],[ref]$value)){throw 'Invalid uint32 field'}
   $fields[$pair[0]]=$value
  }
 }
 foreach($key in @('USED','STEP','DRIVER','DIR_CONFIRMED','HOMED','ALLOWED')){if($fields[$key] -gt 1){throw 'Invalid flag'}}
 if($fields['STATE'] -gt 7 -or $fields['REASON'] -gt 11){throw 'Unknown state or reason'}
 return $fields
}
function Assert-AxisHomeState([string]$Reply,[ValidateSet('IdleBefore','IdleDirection','ActiveOrDone','Done')][string]$Mode){
 $f=Read-AxisHomeStatus $Reply
 if($f['REASON'] -ne 0){throw "HOME fault; preserve first reason: $Reply"}
 foreach($key in @('JOG','HOMED','ALLOWED','RXERR','RXOVF')){if($f[$key] -ne 0){throw "Unexpected $key : $Reply"}}
 if($f['SEEK'] -gt 200 -or $f['BACKOFF'] -gt 40){throw 'HOME counter outside configured range'}
 if($Mode -in @('IdleBefore','IdleDirection')){
  $direction=if($Mode -eq 'IdleDirection'){1}else{0}
  if($f['STATE'] -ne 0 -or $f['USED'] -ne 0 -or $f['EN'] -ne 'DISABLED' -or $f['STEP'] -ne 0 -or $f['SEEK'] -ne 0 -or $f['BACKOFF'] -ne 0 -or $f['DIR_CONFIRMED'] -ne $direction){throw 'Not expected unused disabled idle'}
  if($Mode -eq 'IdleDirection' -and $f['DRIVER'] -ne 1){throw 'Unconfirmed driver'}
 }else{
  if($f['STATE'] -notin @(2,3,4,5) -or $f['USED'] -ne 1 -or $f['DRIVER'] -ne 1 -or $f['DIR_CONFIRMED'] -ne 1){throw 'Not a confirmed HOME session'}
  if($f['STATE'] -eq 5){
   if($f['EN'] -ne 'DISABLED' -or $f['STEP'] -ne 0 -or $f['BACKOFF'] -ne 40){throw 'DONE requires disabled output and 40-step retreat'}
  }else{
   if($Mode -eq 'Done' -or $f['EN'] -ne 'ENABLED'){throw 'Not active or not DONE'}
   if($f['STATE'] -in @(2,3) -and $f['BACKOFF'] -ne 0){throw 'Retreat count before BACKOFF'}
   if($f['STATE'] -eq 3 -and $f['STEP'] -ne 0){throw 'STEP high while qualifying limit'}
  }
 }
 return $f
}
function Read-AxisHomeFeedback([string]$Reply,[int]$Id){
 if($Id -notin 1..3){throw 'Invalid feedback axis'}
 if($Reply -cnotmatch ('^M114 RAW_M'+$Id+':([+-]?[0-9]+(?:\.[0-9]+)?) VALID:1 AGE_MS:([0-9]+)$')){throw 'Wrong axis or invalid feedback'}
 $angle=[double]::Parse($Matches[1],[Globalization.CultureInfo]::InvariantCulture)
 [uint32]$age=0
 if([double]::IsNaN($angle) -or [double]::IsInfinity($angle) -or ![uint32]::TryParse($Matches[2],[ref]$age) -or $age -ge 100){throw 'Non-finite or stale feedback'}
 return [pscustomobject]@{motor_deg=$angle;age_ms=$age}
}
function Read-AxisHomeLimits([string]$Reply){
 if($Reply -cnotmatch '^M119 M1:(OPEN|TRIGGERED) M2:(OPEN|TRIGGERED) M3:(OPEN|TRIGGERED)$'){throw 'Invalid limits'}
 $mask=0;foreach($id in 1..3){if($Matches[$id] -eq 'TRIGGERED'){$mask=$mask -bor (1 -shl ($id-1))}}
 return $mask
}
function Assert-AxisHomeLimits([string]$Reply,[int]$Id,[bool]$Active){
 if($Id -notin 1..3){throw 'Invalid limit axis'}
 $mask=Read-AxisHomeLimits $Reply
 $allowed=if($Active){1 -shl ($Id-1)}else{0}
 if(($mask -band (7 -bxor $allowed)) -ne 0){throw 'Unexpected limit on selected or other axis'}
}
function Assert-AxisHomeReference([string]$Reply,[bool]$Ready){
 $number='[+-]?[0-9]+(?:\.[0-9]+)?'
 $pattern='^M122 LOCAL_REF:([0-3]) VALID:([01]) REASON:([0-9]+) ORIGIN_MOTOR:('+$number+') REL_MOTOR:('+$number+') REL_JOINT:('+$number+') GEAR_RATIO:4\.500 AGE_MS:([0-9]+) WHOLE_HOMED:0 ALLOWED:0$'
 if($Reply -cnotmatch $pattern){throw 'Invalid local reference'}
 $values=@();foreach($key in 4..6){
  $v=[double]::Parse($Matches[$key],[Globalization.CultureInfo]::InvariantCulture)
  if([double]::IsNaN($v) -or [double]::IsInfinity($v)){throw 'Non-finite local reference'}
  $values+=$v
 }
 [uint32]$age=0;if(![uint32]::TryParse($Matches[7],[ref]$age)){throw 'Invalid reference age'}
 if($Ready){
  if($Matches[1] -ne '2' -or $Matches[2] -ne '1' -or $Matches[3] -ne '0' -or $age -ge 100 -or [Math]::Abs($values[1]-4.5*$values[2]) -gt 0.01){throw 'Local reference is invalid, stale or inconsistent'}
 }elseif($Matches[1] -ne '0' -or $Matches[2] -ne '0' -or $Matches[3] -ne '1' -or $age -ne 0 -or ($values|Where-Object {$_ -ne 0}).Count -ne 0){
  throw 'Reference is not the initial UNSET state'
 }
}
