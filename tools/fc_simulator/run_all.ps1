param(
  [switch]$SkipTraces
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$python = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$platformio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
$output = Join-Path $PSScriptRoot "output"

if(-not (Test-Path -LiteralPath $python))
{
  throw "PlatformIO Python was not found at $python"
}
if(-not (Test-Path -LiteralPath $platformio))
{
  throw "PlatformIO was not found at $platformio"
}

Push-Location $repo
try
{
  $env:PYTHONDONTWRITEBYTECODE = "1"

  Write-Host "[1/3] Running flight-simulator assertions"
  & $python -m unittest discover -s "tools\fc_simulator\tests" -v
  if($LASTEXITCODE -ne 0) { throw "Flight-simulator assertions failed" }

  Write-Host "[2/3] Running production safety-policy and source-router tests"
  & $platformio test -e native_drone_proto
  if($LASTEXITCODE -ne 0) { throw "Native FC safety tests failed" }

  if(-not $SkipTraces)
  {
    Write-Host "[3/3] Generating deterministic CSV traces"
    & $python "tools\fc_simulator\simulator.py" --scenario all --output $output
    if($LASTEXITCODE -ne 0) { throw "Simulation trace generation failed" }
    Write-Host "Simulation traces: $output"
  }
  else
  {
    Write-Host "[3/3] Trace generation skipped"
  }

  Write-Host "All Drone Prototype software checks passed."
}
finally
{
  Pop-Location
}
