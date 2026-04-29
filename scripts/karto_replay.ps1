# karto_replay.ps1 — deploy, build, run, and fetch results for the Phase-0
# karto_sdk replay smoke test.
#
# Usage:
#   .\scripts\karto_replay.ps1 -Action Deploy
#   .\scripts\karto_replay.ps1 -Action Build
#   .\scripts\karto_replay.ps1 -Action Run -LogPath '~/scan_logs/scan_20251115T030000.jsonl' -Out 'replay1'
#   .\scripts\karto_replay.ps1 -Action Fetch -Out 'replay1'
#   .\scripts\karto_replay.ps1 -Action All   -LogPath '~/scan_logs/...' -Out 'replay1'
#
# Pi assumptions (set by Phase 0):
#   - libkartoSdk.so at ~/karto_port/slam_toolbox/lib/karto_sdk/build_standalone/
#   - SSH passwordless-ish; falls back on prompting

[CmdletBinding()]
param(
  [ValidateSet('Deploy','Build','Run','Fetch','All')]
  [string]$Action = 'All',

  [string]$PiHost = 'rosie@192.168.x.x',
  [string]$RemoteDir = '~/karto_port/replay',
  [string]$LogPath,           # remote path to the .jsonl
  [string]$Out = 'replay',    # output prefix; produces <Out>.pgm/.yaml/.csv
  [string]$LocalOutDir = 'replay_out'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
$srcDir   = Join-Path $repoRoot 'tools\karto_replay'

function Invoke-Pi([string]$cmd) {
  Write-Host "[pi] $cmd" -ForegroundColor DarkCyan
  ssh $PiHost $cmd
  if ($LASTEXITCODE -ne 0) { throw "ssh exit $LASTEXITCODE" }
}

function Do-Deploy {
  Write-Host "==> Deploying sources to $PiHost`:$RemoteDir" -ForegroundColor Cyan
  Invoke-Pi "mkdir -p $RemoteDir"
  scp (Join-Path $srcDir 'replay.cpp')      "${PiHost}:${RemoteDir}/"
  scp (Join-Path $srcDir 'null_solver.h')   "${PiHost}:${RemoteDir}/"
  scp (Join-Path $srcDir 'CMakeLists.txt')  "${PiHost}:${RemoteDir}/"
}

function Do-Build {
  Write-Host "==> Building karto_replay on Pi (single-threaded, low-RAM)" -ForegroundColor Cyan
  Invoke-Pi "cd $RemoteDir && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j1"
}

function Do-Run {
  if (-not $LogPath) { throw "-LogPath required for Run" }
  Write-Host "==> Running karto_replay on $LogPath -> $Out" -ForegroundColor Cyan
  $exe = "$RemoteDir/build/karto_replay"
  Invoke-Pi "cd $RemoteDir && $exe '$LogPath' '$Out'"
}

function Do-Fetch {
  Write-Host "==> Fetching results back to $LocalOutDir" -ForegroundColor Cyan
  $localDir = Join-Path $repoRoot $LocalOutDir
  New-Item -ItemType Directory -Force -Path $localDir | Out-Null
  foreach ($ext in 'pgm','yaml','csv') {
    scp "${PiHost}:${RemoteDir}/${Out}.${ext}" $localDir
  }
  Write-Host "Wrote: $localDir" -ForegroundColor Green
}

switch ($Action) {
  'Deploy' { Do-Deploy }
  'Build'  { Do-Build  }
  'Run'    { Do-Run    }
  'Fetch'  { Do-Fetch  }
  'All'    { Do-Deploy; Do-Build; Do-Run; Do-Fetch }
}
