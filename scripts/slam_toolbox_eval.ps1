# slam_toolbox_eval.ps1 — deploy/build/run/fetch helper for the slam_toolbox
# offline evaluation. Mirrors scripts/karto_replay.ps1.
#
# Usage:
#   .\scripts\slam_toolbox_eval.ps1 -Action Deploy
#   .\scripts\slam_toolbox_eval.ps1 -Action Build           # ~10 min first time (apt install)
#   .\scripts\slam_toolbox_eval.ps1 -Action Run -LogPath '/home/rosie/scan_logs/scan_*.jsonl' -Out 'slam1'
#   .\scripts\slam_toolbox_eval.ps1 -Action Fetch -Out 'slam1'
#   .\scripts\slam_toolbox_eval.ps1 -Action All -LogPath '...' -Out 'slam1'

[CmdletBinding()]
param(
  [ValidateSet('Deploy','Build','Run','Fetch','All','Clean')]
  [string]$Action = 'All',

  [string]$PiHost = 'rosie@192.168.x.x',
  [string]$RemoteDir = '~/slam_eval',
  [string]$ImageTag = 'rosie-slam-eval:latest',
  [string]$LogPath,
  [string]$Out = 'slam1',
  [string]$LocalOutDir = 'replay_out',
  [int]$Speed = 5,
  [int]$Settle = 15
)

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')
$srcDir   = Join-Path $repoRoot 'tools\slam_toolbox_eval'

function Invoke-Pi([string]$cmd) {
  Write-Host "[pi] $cmd" -ForegroundColor DarkCyan
  ssh $PiHost $cmd
  if ($LASTEXITCODE -ne 0) { throw "ssh exit $LASTEXITCODE" }
}

function Do-Deploy {
  Write-Host "==> Deploying sources to $PiHost`:$RemoteDir" -ForegroundColor Cyan
  Invoke-Pi "mkdir -p $RemoteDir"
  scp (Join-Path $srcDir 'Dockerfile')        "${PiHost}:${RemoteDir}/"
  scp (Join-Path $srcDir 'replay_node.py')    "${PiHost}:${RemoteDir}/"
  scp (Join-Path $srcDir 'slam_params.yaml')  "${PiHost}:${RemoteDir}/"
  scp (Join-Path $srcDir 'run_eval.sh')       "${PiHost}:${RemoteDir}/"
}

function Do-Build {
  Write-Host "==> Building $ImageTag (slow first time: pulls ros:jazzy ~600 MB + apt install)" -ForegroundColor Cyan
  Invoke-Pi "cd $RemoteDir && docker build -t $ImageTag ."
}

function Do-Run {
  if (-not $LogPath) { throw "-LogPath required for Run" }
  $outRemote = "$RemoteDir/out_$Out"
  Write-Host "==> Running eval: log=$LogPath out=$outRemote" -ForegroundColor Cyan
  Invoke-Pi "mkdir -p $outRemote && rm -f $outRemote/*"
  $cmd = @(
    "docker run --rm",
    "-v $LogPath`:/data/scan.jsonl:ro",
    "-v $outRemote`:/out",
    "-e ROSIE_REPLAY_SPEED=$Speed",
    "-e ROSIE_REPLAY_SETTLE=$Settle",
    "--name rosie_slam_eval",
    $ImageTag
  ) -join ' '
  Invoke-Pi $cmd
}

function Do-Fetch {
  Write-Host "==> Fetching from $RemoteDir/out_$Out -> $LocalOutDir" -ForegroundColor Cyan
  $localDir = Join-Path $repoRoot $LocalOutDir
  New-Item -ItemType Directory -Force -Path $localDir | Out-Null
  scp -r "${PiHost}:${RemoteDir}/out_${Out}/" $localDir
  Write-Host "Wrote: $localDir\out_$Out" -ForegroundColor Green
}

function Do-Clean {
  Write-Host "==> Removing image $ImageTag" -ForegroundColor Cyan
  Invoke-Pi "docker rmi $ImageTag 2>/dev/null || true"
}

switch ($Action) {
  'Deploy' { Do-Deploy }
  'Build'  { Do-Build  }
  'Run'    { Do-Run    }
  'Fetch'  { Do-Fetch  }
  'Clean'  { Do-Clean  }
  'All'    { Do-Deploy; Do-Build; Do-Run; Do-Fetch }
}
