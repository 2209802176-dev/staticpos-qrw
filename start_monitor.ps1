param([switch]$SmokeTest, [switch]$SelfTest)
$ErrorActionPreference = 'Stop'
$taskMonitor = Join-Path $PSScriptRoot 'monitor\static_monitor.py'
$taskMonitorArgs = @()
if ($SmokeTest) { $taskMonitorArgs += '--smoke-gui' }
if ($SelfTest) { $taskMonitorArgs += '--self-test' }
# Prefer a repository-local virtual environment; otherwise use Python Launcher.
$taskVenvPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
if (Test-Path -LiteralPath $taskVenvPython) {
    & $taskVenvPython $taskMonitor @taskMonitorArgs
} elseif (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 $taskMonitor @taskMonitorArgs
} else {
    & python $taskMonitor @taskMonitorArgs
}
if ($LASTEXITCODE -ne 0) { throw 'Monitor failed. See the error above.' }
