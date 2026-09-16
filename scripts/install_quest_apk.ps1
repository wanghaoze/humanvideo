param([string]$Serial, [string]$Adb = 'adb', [switch]$InspectOnly,
      [ValidateSet('fixed','upstream')][string]$Variant='fixed')
$ErrorActionPreference = 'Stop'
$TaskRoot = Split-Path -Parent $PSScriptRoot
if (-not (Get-Command $Adb -ErrorAction SilentlyContinue)) {
    throw 'ADB not found. Install Android Platform Tools from https://developer.android.com/tools/releases/platform-tools and provide -Adb <absolute path to adb.exe>.'
}
if (-not $Serial) {
    $Devices = @(& $Adb devices | Select-String '^([^\s]+)\s+device$' | ForEach-Object { $_.Matches[0].Groups[1].Value })
    if ($Devices.Count -ne 1) { throw 'Connect and authorize one headset over USB, or specify -Serial.' }
    $Serial = $Devices[0]
}
$DeviceModel = (& $Adb -s $Serial shell getprop ro.product.model).Trim()
$Manufacturer = (& $Adb -s $Serial shell getprop ro.product.manufacturer).Trim()
Write-Host "Detected headset: $Manufacturer / $DeviceModel ($Serial)"
if ($InspectOnly) { exit 0 }
if ($DeviceModel -notmatch 'Quest\s*3') { throw 'This bundle targets Quest 3. S3A/M3 Pro and other models require identification before installing a compatible APK.' }
$ManifestName = if ($Variant -eq 'fixed') {'fixed-provenance.json'} else {'provenance.json'}
$Manifest = Get-Content (Join-Path $TaskRoot "deliverables/quest3/$ManifestName") -Raw | ConvertFrom-Json
$ApkPath = Join-Path $TaskRoot ("deliverables/quest3/" + $Manifest.apk)
$ActualHash = (Get-FileHash -LiteralPath $ApkPath -Algorithm SHA256).Hash.ToLowerInvariant()
if ($ActualHash -ne $Manifest.sha256) { throw 'APK checksum mismatch.' }
& $Adb -s $Serial install -r -t $ApkPath
if ($LASTEXITCODE -ne 0) { throw 'APK installation failed. Preserve the adb error; do not uninstall existing apps automatically.' }
Write-Host "Installed $Variant APK. Use check_quest_reader.py for a bounded diagnostic; installation does not validate tracking."
