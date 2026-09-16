$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
& "$PSScriptRoot/.venv-mujoco/Scripts/python.exe" "$PSScriptRoot/scripts/view_r1pro_scene.py"
