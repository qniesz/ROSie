#Requires -Version 5.1
<#
.SYNOPSIS
    Pi-first development helper for ROSie.

.DESCRIPTION
    Keeps normal development pointed at the real Raspberry Pi runtime. Use this
    script to sync selected files, validate them on the Pi, restart the Pi
    service, inspect logs, and protect a development Pi from public Git updates.

    This is not the first-time installer. For first-time setup, use
    scripts/deploy_pi.ps1.
#>

[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet("help", "dev-mode", "setup-ssh", "sync", "sync-runtime", "check", "restart", "status", "logs", "collect-logs", "release-check")]
    [string]$Command,

    [Parameter(Position = 1, ValueFromRemainingArguments = $true)]
    [string[]]$Paths = @(),

    [Alias("Host")]
    [string]$PiHost,
    [string]$User,
    [int]$Port,
    [int]$Lines = 160,
    [switch]$Follow,
    [switch]$Sidecar
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$script:RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).ProviderPath
$script:EnvPath = Join-Path $script:RepoRoot ".env"

function Read-DotEnv {
    param([string]$Path)

    $values = @{}
    if (-not (Test-Path -LiteralPath $Path)) {
        return $values
    }

    foreach ($line in Get-Content -LiteralPath $Path) {
        $trimmed = $line.Trim()
        if ($trimmed.Length -eq 0 -or $trimmed.StartsWith("#")) {
            continue
        }
        $idx = $trimmed.IndexOf("=")
        if ($idx -le 0) {
            continue
        }
        $key = $trimmed.Substring(0, $idx).Trim()
        $value = $trimmed.Substring($idx + 1).Trim()
        if (($value.StartsWith('"') -and $value.EndsWith('"')) -or ($value.StartsWith("'") -and $value.EndsWith("'"))) {
            $value = $value.Substring(1, $value.Length - 2)
        }
        $values[$key] = $value
    }
    return $values
}

function Write-Step {
    param([string]$Message)
    Write-Host "==> $Message" -ForegroundColor Cyan
}

function Write-Warn {
    param([string]$Message)
    Write-Host "WARN: $Message" -ForegroundColor Yellow
}

function Show-Usage {
        Write-Host @"
ROSie Pi development helper

Usage:
    .\scripts\dev_pi.ps1 setup-ssh
    .\scripts\dev_pi.ps1 dev-mode on|off
    .\scripts\dev_pi.ps1 sync <relative-path> [more paths]
    .\scripts\dev_pi.ps1 sync-runtime
    .\scripts\dev_pi.ps1 check [relative-path ...]
    .\scripts\dev_pi.ps1 restart [-Sidecar]
    .\scripts\dev_pi.ps1 status
    .\scripts\dev_pi.ps1 logs [-Lines 160] [-Follow]
    .\scripts\dev_pi.ps1 collect-logs
    .\scripts\dev_pi.ps1 release-check

Examples:
    .\scripts\dev_pi.ps1 dev-mode on
    .\scripts\dev_pi.ps1 sync pi\rosie_driver\map_pipeline.py
    .\scripts\dev_pi.ps1 check pi\rosie_driver\map_pipeline.py
    .\scripts\dev_pi.ps1 restart
    .\scripts\dev_pi.ps1 logs
"@
}

if ([string]::IsNullOrWhiteSpace($Command) -or $Command -eq "help") {
        Show-Usage
        return
}

function ConvertTo-BashSingleQuoted {
    param([Parameter(Mandatory)][string]$Value)
    $singleQuote = [string][char]39
    $backslash = [string][char]92
    $escaped = $Value.Replace($singleQuote, "$singleQuote$backslash$singleQuote$singleQuote")
    return "$singleQuote$escaped$singleQuote"
}

function ConvertTo-RemotePath {
    param([Parameter(Mandatory)][string]$RelativePath)

    $clean = $RelativePath.Replace("\", "/").Trim()
    while ($clean.StartsWith("./")) {
        $clean = $clean.Substring(2)
    }
    if ([System.IO.Path]::IsPathRooted($clean) -or $clean -match "^[A-Za-z]:" -or $clean -match "(^|/)\.\.(/|$)") {
        throw "Only workspace-relative paths can be synced: $RelativePath"
    }
    return "$script:RemoteRepo/$clean"
}

function Get-LocalPath {
    param([Parameter(Mandatory)][string]$RelativePath)
    return Join-Path $script:RepoRoot $RelativePath
}

$envValues = Read-DotEnv -Path $script:EnvPath
if (-not $PiHost) { $PiHost = $envValues["PI_HOST"] }
if (-not $User) { $User = $envValues["PI_USER"] }
if (-not $Port -and $envValues.ContainsKey("PI_PORT")) { $Port = [int]$envValues["PI_PORT"] }
if (-not $Port) { $Port = 22 }

if (-not $PiHost -or -not $User) {
    throw "Pi connection is missing. Set PI_HOST/PI_USER in .env or pass -Host and -User."
}

$script:Target = "$User@$PiHost"
$script:RemoteRepo = "/home/$User/rosie"
$script:RemoteVenv = "/home/$User/rosie-venv"
$script:SshDir = Join-Path $env:USERPROFILE ".ssh"
$script:KnownHostsFile = Join-Path $script:SshDir "rosie_known_hosts"
$script:SshKeyFile = Join-Path $script:SshDir "rosie_id"
if (-not (Test-Path $script:SshDir)) {
    New-Item -ItemType Directory -Path $script:SshDir -Force | Out-Null
}
if (-not (Test-Path $script:KnownHostsFile)) {
    New-Item -ItemType File -Path $script:KnownHostsFile -Force | Out-Null
}

$script:SshBaseOpts = @(
    "-p", "$Port",
    "-o", "KexAlgorithms=+diffie-hellman-group14-sha256",
    "-o", "HostKeyAlgorithms=+ssh-rsa",
    "-o", "PubkeyAcceptedAlgorithms=+ssh-rsa",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "LogLevel=ERROR",
    "-o", "UserKnownHostsFile=$script:KnownHostsFile",
    "-o", "ServerAliveInterval=15",
    "-o", "ServerAliveCountMax=40",
    "-o", "ConnectTimeout=15"
)
if (Test-Path $script:SshKeyFile) {
    $script:SshBaseOpts += @("-i", $script:SshKeyFile, "-o", "PasswordAuthentication=no")
}

$script:ScpBaseOpts = @(
    "-P", "$Port",
    "-o", "KexAlgorithms=+diffie-hellman-group14-sha256",
    "-o", "HostKeyAlgorithms=+ssh-rsa",
    "-o", "PubkeyAcceptedAlgorithms=+ssh-rsa",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "LogLevel=ERROR",
    "-o", "UserKnownHostsFile=$script:KnownHostsFile"
)
if (Test-Path $script:SshKeyFile) {
    $script:ScpBaseOpts += @("-i", $script:SshKeyFile, "-o", "PasswordAuthentication=no")
}

function Invoke-Pi {
    param(
        [Parameter(Mandatory)][string]$RemoteCommand,
        [switch]$AllowFail,
        [switch]$PassThru
    )

    $sshArgs = @() + $script:SshBaseOpts
    $oldEap = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $output = & ssh.exe @sshArgs $script:Target $RemoteCommand 2>&1
        $exitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $oldEap
    }
    if ($PassThru) {
        return [pscustomobject]@{ ExitCode = $exitCode; Output = ($output | Out-String).TrimEnd() }
    }
    if ($output) { $output }
    if ($exitCode -ne 0 -and -not $AllowFail) {
        throw "Pi command failed with exit code ${exitCode}: $RemoteCommand"
    }
}

function Copy-FromPi {
    param(
        [Parameter(Mandatory)][string]$RemotePath,
        [Parameter(Mandatory)][string]$LocalPath,
        [switch]$AllowFail
    )

    $scpArgs = @() + $script:ScpBaseOpts
    $sourcePath = "${script:Target}:$RemotePath"
    $oldEap = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $output = & scp.exe @scpArgs $sourcePath $LocalPath 2>&1
        $exitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $oldEap
    }
    if ($output) { $output }
    if ($exitCode -ne 0 -and -not $AllowFail) {
        throw "SCP failed with exit code ${exitCode}: $RemotePath -> $LocalPath"
    }
}

function Copy-ToPi {
    param(
        [Parameter(Mandatory)][string]$LocalPath,
        [Parameter(Mandatory)][string]$RemotePath,
        [switch]$Recurse,
        [switch]$AllowFail
    )

    $scpArgs = @() + $script:ScpBaseOpts
    if ($Recurse) { $scpArgs += "-r" }
    $targetPath = "${script:Target}:$RemotePath"
    $oldEap = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $output = & scp.exe @scpArgs $LocalPath $targetPath 2>&1
        $exitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $oldEap
    }
    if ($output) { $output }
    if ($exitCode -ne 0 -and -not $AllowFail) {
        throw "SCP failed with exit code ${exitCode}: $LocalPath -> $RemotePath"
    }
}

function Sync-Path {
    param([Parameter(Mandatory)][string]$RelativePath)

    $localPath = Get-LocalPath -RelativePath $RelativePath
    if (-not (Test-Path -LiteralPath $localPath)) {
        throw "Local path not found: $RelativePath"
    }

    $remotePath = ConvertTo-RemotePath -RelativePath $RelativePath
    $remoteParent = $remotePath.Substring(0, $remotePath.LastIndexOf("/"))

    if ((Get-Item -LiteralPath $localPath).PSIsContainer) {
        $remoteParent = $remotePath.Substring(0, $remotePath.LastIndexOf("/"))
        Invoke-Pi "mkdir -p $(ConvertTo-BashSingleQuoted $remoteParent)"
        Copy-ToPi -LocalPath $localPath -RemotePath ($remoteParent + "/") -Recurse
        Invoke-Pi "find $(ConvertTo-BashSingleQuoted $remotePath) -name '*.sh' -exec chmod +x {} +" -AllowFail
    } else {
        Invoke-Pi "mkdir -p $(ConvertTo-BashSingleQuoted $remoteParent)"
        Copy-ToPi -LocalPath $localPath -RemotePath $remotePath
        if ($RelativePath -like "*.sh") {
            Invoke-Pi "chmod +x $(ConvertTo-BashSingleQuoted $remotePath)" -AllowFail
        }
    }
}

function Get-DefaultCheckPaths {
    return @(
        "pi/rosie_driver/main.py",
        "pi/rosie_driver/map_pipeline.py",
        "pi/rosie_driver/mqtt_bridge.py",
        "tools/slam_toolbox_eval/run_online.sh",
        "tools/slam_toolbox_eval/mqtt_ros_bridge.py"
    )
}

function Invoke-Check {
    param([string[]]$CheckPaths)

    if (-not $CheckPaths -or $CheckPaths.Count -eq 0) {
        $CheckPaths = Get-DefaultCheckPaths
    }

    $pythonFiles = @()
    $shellFiles = @()
    foreach ($path in $CheckPaths) {
        $localPath = Get-LocalPath -RelativePath $path
        if (Test-Path -LiteralPath $localPath -PathType Container) {
            $remoteDir = ConvertTo-RemotePath -RelativePath $path
            Write-Step "Checking Python files under $path on Pi"
            Invoke-Pi "if [ -d $(ConvertTo-BashSingleQuoted $remoteDir) ]; then find $(ConvertTo-BashSingleQuoted $remoteDir) -name '*.py' -print0 | xargs -0 -r $script:RemoteVenv/bin/python3 -m py_compile; fi"
            Write-Step "Checking shell files under $path on Pi"
            Invoke-Pi "if [ -d $(ConvertTo-BashSingleQuoted $remoteDir) ]; then find $(ConvertTo-BashSingleQuoted $remoteDir) -name '*.sh' -print0 | xargs -0 -r bash -n; fi"
            continue
        }
        if ($path -like "*.py") { $pythonFiles += (ConvertTo-RemotePath -RelativePath $path) }
        if ($path -like "*.sh") { $shellFiles += (ConvertTo-RemotePath -RelativePath $path) }
    }

    if ($pythonFiles.Count -gt 0) {
        $quoted = ($pythonFiles | ForEach-Object { ConvertTo-BashSingleQuoted $_ }) -join " "
        Write-Step "Checking Python syntax on Pi"
        Invoke-Pi "$script:RemoteVenv/bin/python3 -m py_compile $quoted"
    }
    if ($shellFiles.Count -gt 0) {
        $quoted = ($shellFiles | ForEach-Object { ConvertTo-BashSingleQuoted $_ }) -join " "
        Write-Step "Checking shell syntax on Pi"
        Invoke-Pi "bash -n $quoted"
    }
}

function Set-DevMode {
    param([Parameter(Mandatory)][ValidateSet("on", "off")][string]$State)

    if ($State -eq "on") {
        Write-Step "Enabling Pi dev mode"
        Invoke-Pi "grep -q '^ROSIE_MANUAL_UPDATES=' ~/rosie-driver.env && sed -i 's/^ROSIE_MANUAL_UPDATES=.*/ROSIE_MANUAL_UPDATES=true/' ~/rosie-driver.env || printf '\nROSIE_MANUAL_UPDATES=true\n' >> ~/rosie-driver.env"
        Invoke-Pi 'mkdir -p ~/rosie && printf "ROSie dev mode enabled on %s\n" "$(date -Iseconds)" > ~/rosie/.rosie-dev-mode'
        $timerResult = Invoke-Pi "sudo -n systemctl disable --now rosie-update.timer rosie-check-updates.timer" -AllowFail -PassThru
        if ($timerResult.ExitCode -ne 0) {
            Write-Warn "Could not disable update timers without sudo password. Dev marker is still active; deploy the guarded pi/update.sh before relying on forced-update blocking."
        }
        Invoke-Pi "grep '^ROSIE_MANUAL_UPDATES=' ~/rosie-driver.env; ls -l ~/rosie/.rosie-dev-mode; systemctl list-timers --all | grep rosie || true"
        return
    }

    Write-Step "Disabling Pi dev mode"
    Invoke-Pi "rm -f ~/rosie/.rosie-dev-mode"
    Invoke-Pi "grep -q '^ROSIE_MANUAL_UPDATES=' ~/rosie-driver.env && sed -i 's/^ROSIE_MANUAL_UPDATES=.*/ROSIE_MANUAL_UPDATES=false/' ~/rosie-driver.env || printf '\nROSIE_MANUAL_UPDATES=false\n' >> ~/rosie-driver.env"
    $timerResult = Invoke-Pi "sudo -n systemctl enable --now rosie-update.timer rosie-check-updates.timer" -AllowFail -PassThru
    if ($timerResult.ExitCode -ne 0) {
        Write-Warn "Could not re-enable update timers without sudo password. Re-enable them manually on the Pi when ready."
    }
    Invoke-Pi "grep '^ROSIE_MANUAL_UPDATES=' ~/rosie-driver.env; systemctl list-timers --all | grep rosie || true"
}

function Collect-Logs {
    $stamp = Get-Date -Format "yyyyMMdd-HHmmss"
    $dest = Join-Path $script:RepoRoot "logs\pi\$stamp"
    New-Item -ItemType Directory -Path $dest -Force | Out-Null
    Write-Step "Collecting Pi logs into $dest"

    $captures = @(
        @{ Name = "rosie-service.log"; Command = "journalctl -u rosie -n 800 --no-pager" },
        @{ Name = "rosie-update.log"; Command = "journalctl -u rosie-update -n 300 --no-pager" },
        @{ Name = "rosie-check-updates.log"; Command = "journalctl -u rosie-check-updates -n 300 --no-pager" },
        @{ Name = "system-status.txt"; Command = "systemctl status rosie --no-pager -l; echo; systemctl list-timers --all | grep rosie || true; echo; ls -l ~/rosie/.rosie-dev-mode 2>/dev/null || true" },
        @{ Name = "last-update.txt"; Command = "cat ~/last-update.txt 2>/dev/null || true" },
        @{ Name = "update-history.log"; Command = "cat ~/update-history.log 2>/dev/null || true" }
    )

    foreach ($capture in $captures) {
        $result = Invoke-Pi -RemoteCommand $capture.Command -PassThru -AllowFail
        Set-Content -LiteralPath (Join-Path $dest $capture.Name) -Value $result.Output -Encoding UTF8
    }

    Copy-FromPi -RemotePath "/tmp/rosie_online_out/*.log" -LocalPath $dest -AllowFail
    Copy-FromPi -RemotePath "/tmp/rosie_online_out/mode.json" -LocalPath $dest -AllowFail
    Write-Host "Logs collected: $dest"
}

function Invoke-ReleaseCheck {
    Write-Step "Git status"
    & git.exe -C $script:RepoRoot status --short
    if ($LASTEXITCODE -ne 0) { throw "git status failed" }

    Write-Step "Diff summary"
    & git.exe -C $script:RepoRoot diff --stat
    if ($LASTEXITCODE -ne 0) { throw "git diff failed" }

    $trackedEnv = & git.exe -C $script:RepoRoot ls-files --error-unmatch .env 2>$null
    if ($LASTEXITCODE -eq 0 -or $trackedEnv) {
        Write-Warn ".env is tracked by Git. Do not release secrets."
    }

    $staged = & git.exe -C $script:RepoRoot diff --cached --name-only
    if ($LASTEXITCODE -eq 0 -and ($staged -contains ".env")) {
        Write-Warn ".env is staged. Unstage it before committing."
    }

    Write-Host "Release rule: commit and push only after the Pi-tested behavior is ready for everyone."
}

switch ($Command) {
    "dev-mode" {
        if ($Paths.Count -ne 1 -or @("on", "off") -notcontains $Paths[0]) {
            Show-Usage
            Write-Warn "dev-mode requires either 'on' or 'off'."
            return
        }
        Set-DevMode -State $Paths[0]
    }
    "sync" {
        if ($Paths.Count -eq 0) {
            Show-Usage
            Write-Warn "sync requires at least one workspace-relative path."
            return
        }
        foreach ($path in $Paths) {
            Write-Step "Syncing $path"
            Sync-Path -RelativePath $path
        }
    }
    "sync-runtime" {
        $runtimePaths = @(
            "pi/rosie_driver",
            "tools/slam_toolbox_eval"
        )
        foreach ($path in $runtimePaths) {
            Write-Step "Syncing $path"
            Sync-Path -RelativePath $path
        }
    }
    "check" {
        Invoke-Check -CheckPaths $Paths
    }
    "restart" {
        if ($Sidecar) {
            Write-Step "Removing online SLAM sidecar before restart"
            Invoke-Pi "docker rm -f rosie_slam_online 2>/dev/null || true"
        }
        Write-Step "Restarting rosie.service on Pi"
        Invoke-Pi "sudo systemctl restart rosie"
        Invoke-Pi "systemctl is-active rosie"
    }
    "status" {
        Invoke-Pi "systemctl status rosie --no-pager -l; echo; systemctl list-timers --all | grep rosie || true; echo; ls -l ~/rosie/.rosie-dev-mode 2>/dev/null || true" -AllowFail
    }
    "logs" {
        if ($Follow) {
            $sshArgs = @() + $script:SshBaseOpts
            & ssh.exe @sshArgs $script:Target "journalctl -u rosie -f"
            if ($LASTEXITCODE -ne 0) {
                Write-Warn "Live log session ended with SSH exit code $LASTEXITCODE."
            }
            return
        }
        Invoke-Pi "journalctl -u rosie -n $Lines --no-pager" -AllowFail
    }
    "setup-ssh" {
        if (Test-Path $script:SshKeyFile) {
            Write-Host "SSH key already exists at $script:SshKeyFile" -ForegroundColor Green
        } else {
            Write-Step "Generating SSH key at $script:SshKeyFile"
            $kfEsc = $script:SshKeyFile -replace '"', '\"'
            $proc = Start-Process -FilePath "ssh-keygen.exe" `
                -ArgumentList "-t ed25519 -f `"$kfEsc`" -N `"`" -C rosie-dev" `
                -NoNewWindow -Wait -PassThru
            if ($proc.ExitCode -ne 0) { throw "ssh-keygen failed" }
        }
        $pubKey = Get-Content "$script:SshKeyFile.pub" -Raw
        $pubKey = $pubKey.Trim()
        Write-Step "Copying public key to $script:Target (you will be prompted for your password once)"
        $remoteCmd = "mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo '$pubKey' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
        $sshNoKey = @(
            "-p", "$Port",
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", "UserKnownHostsFile=$script:KnownHostsFile",
            "-o", "LogLevel=ERROR"
        )
        & ssh.exe @sshNoKey $script:Target $remoteCmd
        if ($LASTEXITCODE -ne 0) { throw "Failed to install public key on Pi" }
        Write-Host "SSH key installed. Password prompts are gone." -ForegroundColor Green
    }
    "collect-logs" {
        Collect-Logs
    }
    "release-check" {
        Invoke-ReleaseCheck
    }
}