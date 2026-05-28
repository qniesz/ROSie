#Requires -Version 5.1
<#
.SYNOPSIS
    Deploy ROSie custom Lovelace cards to Home Assistant.

.DESCRIPTION
    Copies all ha/www/*.js files to the HA config/www/ directory via Samba,
    then registers them as Lovelace JS Module resources via the HA WebSocket API.

    Requires in .env:
        HA_SAMBA_HOST   IP or hostname of HA (e.g. 192.168.2.99)
        HA_SAMBA_USER   Samba username
        HA_SAMBA_PASS   Samba password
        HA_URL          HA base URL (e.g. http://192.168.2.99:8123)
        HA_TOKEN        HA long-lived access token

    Safe to run multiple times (idempotent).

.EXAMPLE
    .\scripts\ha_deploy.ps1
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).ProviderPath
$EnvPath  = Join-Path $RepoRoot ".env"

# ---------------------------------------------------------------------------
# Parse .env
# ---------------------------------------------------------------------------
function Read-DotEnv {
    param([string]$Path)
    $values = @{}
    if (-not (Test-Path -LiteralPath $Path)) { return $values }
    foreach ($line in Get-Content -LiteralPath $Path) {
        $t = $line.Trim()
        if ($t.Length -eq 0 -or $t.StartsWith("#")) { continue }
        $idx = $t.IndexOf("=")
        if ($idx -le 0) { continue }
        $k = $t.Substring(0, $idx).Trim()
        $v = $t.Substring($idx + 1).Trim().Trim('"').Trim("'")
        $values[$k] = $v
    }
    return $values
}

$env = Read-DotEnv $EnvPath

$sambaHost = $env["HA_SAMBA_HOST"]
$sambaUser = $env["HA_SAMBA_USER"]
$sambaPass = $env["HA_SAMBA_PASS"]

foreach ($required in @("HA_SAMBA_HOST","HA_SAMBA_USER","HA_SAMBA_PASS","HA_URL","HA_TOKEN")) {
    if (-not $env.ContainsKey($required) -or [string]::IsNullOrWhiteSpace($env[$required])) {
        Write-Error "$required is not set in .env"
        exit 1
    }
}

# ---------------------------------------------------------------------------
# Copy JS files via Samba
# ---------------------------------------------------------------------------
$wwwSrc = Join-Path $RepoRoot "ha\www"
$jsFiles = Get-ChildItem $wwwSrc -Filter "*.js"
if (-not $jsFiles) {
    Write-Warning "No *.js files found in $wwwSrc"
    exit 0
}

$sharePath = "\\$sambaHost\config"
$cred = New-Object PSCredential(
    $sambaUser,
    (ConvertTo-SecureString $sambaPass -AsPlainText -Force)
)

Write-Host "==> Connecting to $sharePath ..." -ForegroundColor Cyan
$drive = $null
try {
    $drive = New-PSDrive -Name "HADeploy" -PSProvider FileSystem -Root $sharePath -Credential $cred -ErrorAction Stop
} catch {
    Write-Error "Cannot mount $sharePath — check HA_SAMBA_HOST, HA_SAMBA_USER, HA_SAMBA_PASS in .env`n$_"
    exit 1
}

try {
    $wwwDst = "HADeploy:\www"
    if (-not (Test-Path $wwwDst)) {
        New-Item $wwwDst -ItemType Directory | Out-Null
        Write-Host "    created config/www/" -ForegroundColor DarkGray
    }

    foreach ($f in $jsFiles) {
        Copy-Item $f.FullName "$wwwDst\$($f.Name)" -Force
        Write-Host "    copied  $($f.Name) -> HA config/www/" -ForegroundColor Green
    }
} finally {
    Remove-PSDrive -Name "HADeploy" -Force -ErrorAction SilentlyContinue
}

# ---------------------------------------------------------------------------
# Register Lovelace resources
# ---------------------------------------------------------------------------
Write-Host "==> Registering Lovelace resources ..." -ForegroundColor Cyan
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) {
    Write-Warning "Python not found — skipping resource registration."
    Write-Warning "Run manually: python scripts\ha_add_resource.py"
    exit 0
}

$scriptPath = Join-Path $PSScriptRoot "ha_add_resource.py"
& python $scriptPath
if ($LASTEXITCODE -ne 0) {
    Write-Error "Resource registration failed (see above)."
    exit 1
}

Write-Host ""
Write-Host "Done! Hard-refresh HA (Ctrl+Shift+R) to pick up the new cards." -ForegroundColor Green
