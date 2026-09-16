param([string]$BuildEnv = "$env:LOCALAPPDATA\miniconda3\envs\quest-build")
$ErrorActionPreference='Stop'
$questRoot=Split-Path -Parent $PSScriptRoot
$questApp=Join-Path $questRoot 'third_party\oculus_reader_fixed\app'
$env:JAVA_HOME=Join-Path $BuildEnv 'Library'
$env:ANDROID_HOME=Join-Path $BuildEnv 'android-sdk'
$env:ANDROID_SDK_ROOT=$env:ANDROID_HOME
$env:GRADLE_USER_HOME=Join-Path $BuildEnv 'gradle-cache'
if (!(Test-Path "$env:JAVA_HOME\bin\java.exe")) { throw 'Activate/provide the quest-build Conda environment with OpenJDK 17 first.' }
foreach ($questComponent in @('ndk\27.0.12077973\source.properties','platforms\android-32\android.jar','cmake\3.22.1\bin\cmake.exe')) {
    if (!(Test-Path (Join-Path $env:ANDROID_HOME $questComponent))) {throw "Missing Android build component: $questComponent"}
}
$questSdkRevision=& git -C (Join-Path $questRoot 'third_party\Meta-OpenXR-SDK-v85') rev-parse HEAD
if ($questSdkRevision -ne 'bbed2f20e38a5df7113630771c83cb8279e4fc26') {throw 'Unexpected Meta SDK revision.'}
Push-Location $questApp
try {
    & .\gradlew.bat --no-daemon --console=plain --max-workers=4 assembleDebug
    if ($LASTEXITCODE -ne 0) {throw 'Quest APK build failed.'}
} finally {Pop-Location}
$questOutput=Join-Path $questRoot 'deliverables\quest3\humanvideo-quest-reader.apk'
Copy-Item (Join-Path $questApp 'build\outputs\apk\debug\OculusTeleop-debug.apk') $questOutput -Force
Get-FileHash -LiteralPath $questOutput -Algorithm SHA256
