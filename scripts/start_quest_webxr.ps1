param([string]$Serial='2G97C5ZH5D006C')
$ErrorActionPreference='Stop'
$questRoot=Split-Path -Parent $PSScriptRoot
$questPython=Join-Path $questRoot '.venv-r1pro\Scripts\python.exe'
$questAdb=Join-Path $questRoot 'deliverables\quest3\adb\platform-tools\adb.exe'
$questSessionFile=Join-Path $questRoot 'runs\quest_webxr_session.json'
$questHealthy=$false
try {$questHealthy=(Invoke-RestMethod 'http://127.0.0.1:8766/health' -TimeoutSec 2).service -eq 'humanvideo-quest-usb-relay'} catch {}
if (!$questHealthy) {
    & $questPython -c "import secrets,json,pathlib;pathlib.Path(r'$questSessionFile').write_text(json.dumps({'key':secrets.token_urlsafe(24)}))"
    if ($LASTEXITCODE -ne 0) {throw 'Cannot prepare relay session'}
    $questProcess=Start-Process -FilePath $questPython -ArgumentList @('-m','r1pro_teleop.quest_relay') -WorkingDirectory $questRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $questRoot 'runs\quest_webxr_stdout.log') -RedirectStandardError (Join-Path $questRoot 'runs\quest_webxr_stderr.log')
    @{pid=$questProcess.Id;port=8766} | ConvertTo-Json | Set-Content (Join-Path $questRoot 'runs\quest_webxr_process.json')
    for($questTry=0;$questTry -lt 30;$questTry++) {
        Start-Sleep -Milliseconds 300
        try {$questHealthy=(Invoke-RestMethod 'http://127.0.0.1:8766/health' -TimeoutSec 2).ready; if($questHealthy){break}} catch {}
    }
    if (!$questHealthy) {throw 'Relay did not start; inspect runs/quest_webxr_stderr.log'}
}
$questKey=(Get-Content -LiteralPath $questSessionFile -Raw | ConvertFrom-Json).key
& $questAdb -s $Serial reverse tcp:8766 tcp:8766
if ($LASTEXITCODE -ne 0) {throw 'USB reverse connection failed'}
& $questAdb -s $Serial shell am force-stop org.humanvideo.questreader
& $questAdb -s $Serial shell input keyevent KEYCODE_WAKEUP
# Keep the session key out of console/log output. It is not the server token.
$questLaunch=& $questAdb -s $Serial shell am start -W -a android.intent.action.VIEW -d "http://localhost:8766/session/$questKey" -p com.oculus.browser
if ($LASTEXITCODE -ne 0) {throw 'Could not open Quest Browser'}
Write-Host 'Quest Browser opened. Enter VR, then press Y or ENABLE to enable simulation control; B pauses.'
