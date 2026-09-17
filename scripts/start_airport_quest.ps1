param([string]$Serial='2G97C5ZH5D006C',[string]$Server='http://100.104.0.108:8767')
$ErrorActionPreference='Stop'
$airportRoot=Split-Path -Parent $PSScriptRoot
$airportPython=Join-Path $airportRoot '.venv-r1pro\Scripts\python.exe'
$airportAdb=Join-Path $airportRoot 'deliverables\quest3\adb\platform-tools\adb.exe'
$airportSession=Join-Path $airportRoot 'runs\airport_quest_session.json'
$airportState=& $airportAdb -s $Serial get-state 2>&1
if ($LASTEXITCODE -ne 0 -or "$airportState".Trim() -ne 'device') {throw 'Quest USB is not authorized. Unlock headset and allow USB debugging, then retry.'}
$airportHealthy=$false
try {$airportHealthy=(Invoke-RestMethod 'http://127.0.0.1:8768/health' -TimeoutSec 2).ready} catch {}
if (!$airportHealthy) {
    & $airportPython -c "import secrets,json,pathlib;pathlib.Path(r'$airportSession').write_text(json.dumps({'key':secrets.token_urlsafe(24)}))"
    $airportProcess=Start-Process -FilePath $airportPython -ArgumentList @('-m','r1pro_teleop.quest_relay','--airport','--port','8768','--server',$Server,'--session-file',('"'+$airportSession+'"'),'--output','runs/airport_quest_relay') -WorkingDirectory $airportRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $airportRoot 'runs\airport_quest_stdout.log') -RedirectStandardError (Join-Path $airportRoot 'runs\airport_quest_stderr.log')
    @{pid=$airportProcess.Id;port=8768;server=$Server} | ConvertTo-Json | Set-Content (Join-Path $airportRoot 'runs\airport_quest_process.json')
    for($airportTry=0;$airportTry -lt 30;$airportTry++) {
        Start-Sleep -Milliseconds 300
        try {$airportHealthy=(Invoke-RestMethod 'http://127.0.0.1:8768/health' -TimeoutSec 2).ready;if($airportHealthy){break}} catch {}
    }
    if(!$airportHealthy){throw 'Airport relay failed; see runs/airport_quest_stderr.log'}
}
$airportKey=(Get-Content -LiteralPath $airportSession -Raw | ConvertFrom-Json).key
& $airportAdb -s $Serial reverse tcp:8768 tcp:8768
if($LASTEXITCODE -ne 0){throw 'ADB USB reverse failed'}
$airportLaunch=& $airportAdb -s $Serial shell am start -W -a android.intent.action.VIEW -d "http://localhost:8768/session/$airportKey" -p com.oculus.browser
if($LASTEXITCODE -ne 0){throw 'Could not open Quest browser'}
Write-Host 'Airport Quest page opened. Enter VR, ENABLE, then RECORD. FINISH requests success validation; ABORT ends without success.'
