#Requires -Version 5.1
<#
.SYNOPSIS
    One-time deployment script for ROSie on a Raspberry Pi Zero 2 W.

.DESCRIPTION
    Interactively prompts for Pi connection details and MQTT credentials,
    then performs a full first-time setup over SSH:
      - Pushes your SSH public key (one interactive password entry)
      - Verifies hardware and 64-bit OS
      - Installs OS packages
      - Clones the ROSie GitHub repository
      - Creates the Python venv and installs packages
      - Writes ~/rosie-driver.env with MQTT credentials
      - Installs the sudoers entry for passwordless service restart
      - Installs and enables all systemd units
      - Configures unattended OS security updates
      - Verifies the service started successfully

.NOTES
    Requires: Windows 10+ with the built-in OpenSSH client (ssh.exe / scp.exe).
    No external PowerShell modules are used.

    You will be prompted for the Pi password ONCE while the SSH key is pushed.
    Everything after that is non-interactive via key auth.

    The Pi user must have sudo access (default on Raspberry Pi OS).
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# --- Step output helpers -------------------------------------------------------
$script:StepNum = 0

function Write-Step {
    param([string]$Title)
    $script:StepNum++
    Write-Host ""
    Write-Host ("  [{0:D2}] {1}" -f $script:StepNum, $Title) -ForegroundColor Cyan
}

function Write-OK   { Write-Host "       OK" -ForegroundColor Green }
function Write-Warn { param([string]$Msg) Write-Host "       WARN: $Msg" -ForegroundColor Yellow }
function Write-Fail { param([string]$Msg) Write-Host "       FAIL: $Msg" -ForegroundColor Red; throw "Deployment step failed: $Msg" }

# --- SSH options --------------------------------------------------------------
# The Pi OS version on this robot uses modern defaults that OpenSSH on Windows
# sometimes can't negotiate without help. These options mirror what worked
# interactively: allow the legacy KEX/host-key algorithms the Pi currently
# prefers, and disable strict host key checking so the first run doesn't get
# stuck on a yes/no prompt. We store the host key in a dedicated file so we
# don't pollute the user's personal known_hosts.
$script:KnownHostsFile = Join-Path $env:USERPROFILE ".ssh\rosie_known_hosts"
if (-not (Test-Path (Split-Path $script:KnownHostsFile -Parent))) {
    New-Item -ItemType Directory -Path (Split-Path $script:KnownHostsFile -Parent) -Force | Out-Null
}
if (-not (Test-Path $script:KnownHostsFile)) {
    New-Item -ItemType File -Path $script:KnownHostsFile -Force | Out-Null
}

$script:SshBaseOpts = @(
    "-o", "KexAlgorithms=+diffie-hellman-group14-sha256",
    "-o", "HostKeyAlgorithms=+ssh-rsa",
    "-o", "PubkeyAcceptedAlgorithms=+ssh-rsa",
    "-o", "StrictHostKeyChecking=accept-new",
    "-o", "LogLevel=ERROR",
    "-o", "UserKnownHostsFile=$script:KnownHostsFile",
    "-o", "ServerAliveInterval=30",
    "-o", "ConnectTimeout=15"
)

# --- SSH command helpers ------------------------------------------------------
# Capture native-command stdout/stderr without letting benign stderr text become
# a terminating PowerShell error when $ErrorActionPreference is Stop.
function Invoke-NativeCapture {
    param(
        [Parameter(Mandatory)][string]$Exe,
        [string[]]$Arguments = @(),
        [string]$StdInText
    )
    $oldEap = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        if ($PSBoundParameters.ContainsKey("StdInText")) {
            $raw = $StdInText | & $Exe @Arguments 2>&1
        } else {
            $raw = & $Exe @Arguments 2>&1
        }
        $exit = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $oldEap
    }
    return [pscustomobject]@{
        ExitCode = $exit
        Output   = ($raw | Out-String).TrimEnd()
    }
}

# --- SSH command helpers ------------------------------------------------------
# All Pi-side commands are shipped via ssh.exe. We route stdout + stderr through
# pipes and capture them into PowerShell variables so we can make decisions
# based on the output, not just the exit code.
function Invoke-Pi {
    param(
        [Parameter(Mandatory)][string]$Command,
        [switch]$AllowFail,
        [switch]$UseKey   # if $true, forces pubkey-only (no password prompt)
    )
    $target = "$script:PiUser@$script:PiHost"
    $sshArgs = @() + $script:SshBaseOpts
    if ($UseKey) {
        $sshArgs += @(
            "-o", "BatchMode=yes",
            "-o", "PasswordAuthentication=no",
            "-o", "PreferredAuthentications=publickey"
        )
        if ($script:KeyPath -and (Test-Path $script:KeyPath)) {
            $sshArgs += @("-i", $script:KeyPath, "-o", "IdentitiesOnly=yes")
        }
    }
    $sshArgs += @($target, $Command)

    # Run ssh.exe directly (no cmd /c wrapper) so argument quoting is sane.
    $native = Invoke-NativeCapture -Exe "ssh.exe" -Arguments $sshArgs
    $code = $native.ExitCode
    $outStr = $native.Output
    if (-not $AllowFail -and $code -ne 0) {
        Write-Fail "Remote command failed (exit $code)`n  cmd : $Command`n  out : $outStr"
    }
    return [pscustomobject]@{
        ExitCode = $code
        Output   = $outStr
    }
}

# Run a remote command as root. Requires pubkey + passwordless sudo (set up in
# the SSH key / sudoers steps below). Use -AllowFail to tolerate non-zero.
function Invoke-PiSudo {
    param(
        [Parameter(Mandatory)][string]$Command,
        [switch]$AllowFail
    )
    $wrapped = "sudo -n $Command"
    if ($AllowFail) {
        return Invoke-Pi $wrapped -UseKey -AllowFail
    } else {
        return Invoke-Pi $wrapped -UseKey
    }
}

# Copy a file to the Pi using scp.exe with the same option set.
function Copy-ToPi {
    param(
        [Parameter(Mandatory)][string]$LocalPath,
        [Parameter(Mandatory)][string]$RemotePath,
        [switch]$UseKey
    )
    $target = "$script:PiUser@${script:PiHost}:$RemotePath"
    $scpArgs = @() + $script:SshBaseOpts
    if ($UseKey) {
        $scpArgs += @(
            "-o", "BatchMode=yes",
            "-o", "PasswordAuthentication=no",
            "-o", "PreferredAuthentications=publickey"
        )
        if ($script:KeyPath -and (Test-Path $script:KeyPath)) {
            $scpArgs += @("-i", $script:KeyPath, "-o", "IdentitiesOnly=yes")
        }
    }
    $scpArgs += @($LocalPath, $target)
    $native = Invoke-NativeCapture -Exe "scp.exe" -Arguments $scpArgs
    if ($native.ExitCode -ne 0) {
        Write-Fail "scp failed: $($native.Output)"
    }
}

# --- Config file loader -------------------------------------------------------
# Looks for ROSie.conf in the same directory as this script.
# Format is one "Key:Value" per line. Recognised keys:
#   Pi_IP_address, Pi_username,
#   MQTT_broker, MQTT_port, MQTT_username, MQTT_password,
#   GitHub_PAT  (use "public" to skip)
# Note: Pi_password is intentionally NOT read from the config anymore - you
# type it once when ssh.exe prompts during the SSH key push.
function Find-RosieConfig {
    $local = Join-Path $PSScriptRoot "ROSie.conf"
    if (Test-Path $local) { return $local }
    return $null
}

function Read-RosieConfig {
    param([string]$Path)
    $cfg = @{}
    foreach ($line in (Get-Content -LiteralPath $Path)) {
        $trim = $line.Trim()
        if ($trim -eq "" -or $trim.StartsWith("#")) { continue }
        $idx = $trim.IndexOf(":")
        if ($idx -lt 1) { continue }
        $key = $trim.Substring(0, $idx).Trim()
        $val = $trim.Substring($idx + 1).Trim()
        $cfg[$key] = $val
    }
    return $cfg
}

function Get-Setting {
    param(
        [hashtable]$Cfg,
        [string]$Key,
        [string]$Prompt,
        [string]$Default = "",
        [switch]$Secure
    )
    if ($Cfg.ContainsKey($Key) -and $Cfg[$Key] -ne "") {
        $val = $Cfg[$Key]
        Write-Host ("  {0,-30} {1}" -f $Prompt, $(if ($Secure) { "********" } else { $val })) -ForegroundColor DarkGray
        if ($Secure) {
            return (ConvertTo-SecureString -String $val -AsPlainText -Force)
        }
        return $val
    }
    if ($Secure) {
        return Read-Host "  $Prompt" -AsSecureString
    }
    $entered = Read-Host "  $Prompt"
    if ([string]::IsNullOrWhiteSpace($entered) -and $Default -ne "") { return $Default }
    return $entered
}

# --- Interactive prompts -------------------------------------------------------
Write-Host ""
Write-Host "ROSie Pi Deployment" -ForegroundColor White
Write-Host "===================" -ForegroundColor White
Write-Host ""

$cfgPath = Find-RosieConfig
$cfg     = @{}
if ($cfgPath) {
    Write-Host "  Loaded settings from $cfgPath" -ForegroundColor Green
    Write-Host ""
    $cfg = Read-RosieConfig -Path $cfgPath
} else {
    Write-Host "  (No ROSie.conf found - prompting for each setting)" -ForegroundColor DarkGray
    Write-Host ""
}

$script:PiHost = Get-Setting -Cfg $cfg -Key "Pi_IP_address" -Prompt "Pi IP address      (e.g. 192.168.x.x)"
$script:PiUser = Get-Setting -Cfg $cfg -Key "Pi_username"   -Prompt "Pi username        (e.g. rosie)"
$MqttHost      = Get-Setting -Cfg $cfg -Key "MQTT_broker"   -Prompt "MQTT broker / HA IP (e.g. 192.168.x.x)"
$MqttPort      = Get-Setting -Cfg $cfg -Key "MQTT_port"     -Prompt "MQTT port          (press Enter for 1883)" -Default "1883"
$MqttUser      = Get-Setting -Cfg $cfg -Key "MQTT_username" -Prompt "MQTT username"
$MqttPassSec   = Get-Setting -Cfg $cfg -Key "MQTT_password" -Prompt "MQTT password" -Secure
$GitTokenSec   = Get-Setting -Cfg $cfg -Key "GitHub_PAT"    -Prompt "GitHub PAT          (press Enter - repo is public)" -Secure

if ([string]::IsNullOrWhiteSpace($MqttPort)) { $MqttPort = "1883" }

$ManualUpdates = Get-Setting -Cfg $cfg -Key "manual_updates" -Prompt "Disable scheduled auto-updates (true/false)" -Default "false"
if ([string]::IsNullOrWhiteSpace($ManualUpdates)) { $ManualUpdates = "false" }

$MqttPass = [System.Net.NetworkCredential]::new("", $MqttPassSec).Password
$GitToken = [System.Net.NetworkCredential]::new("", $GitTokenSec).Password
if ($GitToken -eq "public") { $GitToken = "" }

Write-Host ""

# --- Verify ssh.exe + scp.exe are available ----------------------------------
Write-Step "Checking OpenSSH client"
if (-not (Get-Command ssh.exe -ErrorAction SilentlyContinue)) {
    Write-Fail "ssh.exe not found. Install the OpenSSH Client Windows optional feature and re-run."
}
if (-not (Get-Command scp.exe -ErrorAction SilentlyContinue)) {
    Write-Fail "scp.exe not found. Install the OpenSSH Client Windows optional feature and re-run."
}
Write-OK

# --- Clear any stale host key in the personal known_hosts --------------------
# If the Pi was reflashed, the user's main known_hosts file may still have an
# old entry. We clean both our dedicated file and the default one so ssh.exe
# doesn't bail out on "REMOTE HOST IDENTIFICATION HAS CHANGED".
Write-Step "Clearing any stale host key for $script:PiHost"
$defaultKnown = Join-Path $env:USERPROFILE ".ssh\known_hosts"
foreach ($kh in @($script:KnownHostsFile, $defaultKnown)) {
    if (Test-Path $kh) {
        # ssh-keygen prints "Host not found" to stderr when the host isn't in
        # the file - that's fine, swallow stdout+stderr and ignore exit code.
        & cmd /c "ssh-keygen.exe -R `"$script:PiHost`" -f `"$kh`" >NUL 2>&1"
    }
}
Write-OK

# --- SSH key setup (the ONE interactive step) --------------------------------
Write-Step "Setting up SSH key authentication"
$script:KeyPath = Join-Path $env:USERPROFILE ".ssh\rosie_deploy_ed25519"
$pubPath = "$script:KeyPath.pub"
if (-not (Test-Path $pubPath)) {
    Write-Host "       Generating dedicated ed25519 deploy key at $script:KeyPath..." -ForegroundColor Gray
    # -N '""' sets an empty passphrase on Windows ssh-keygen
    & ssh-keygen.exe -t ed25519 -a 64 -N '""' -f $script:KeyPath -q
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $pubPath)) {
        Write-Fail "ssh-keygen failed to create $pubPath"
    }
}
$pubKey = (Get-Content $pubPath -Raw).Trim()
$pubKeyEsc = $pubKey.Replace("'", "'""'""'")

Write-Host "       You will be prompted for the Pi password ONCE now." -ForegroundColor Yellow
Write-Host "       After this, every step runs non-interactively over SSH key auth." -ForegroundColor Gray

# Install the public key + normalise permissions as a one-line remote command.
# This avoids CRLF issues that can appear when sending multiline scripts from
# Windows to ssh stdin.
$installKeyCmd = "set -e; umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys; if ! grep -qxF '$pubKeyEsc' ~/.ssh/authorized_keys; then printf '%s\\n' '$pubKeyEsc' >> ~/.ssh/authorized_keys; fi; echo KEY_INSTALLED_OK"

$target = "$script:PiUser@$script:PiHost"
# NOTE: no BatchMode here - we WANT the interactive password prompt this one time.
$sshArgs = @() + $script:SshBaseOpts + @($target, $installKeyCmd)
$keyInstall = Invoke-NativeCapture -Exe "ssh.exe" -Arguments $sshArgs
if ($keyInstall.ExitCode -ne 0 -or $keyInstall.Output -notmatch "KEY_INSTALLED_OK") {
    Write-Fail "SSH key install failed. Output:`n$($keyInstall.Output)"
}
Write-OK

# --- Verify key auth works (no more password prompts after this) -------------
Write-Step "Verifying key-based auth"
$whoami = Invoke-Pi "whoami" -UseKey -AllowFail
if ($whoami.ExitCode -ne 0 -or $whoami.Output.Trim() -ne $script:PiUser) {
    Write-Fail "Key-based SSH not working. Got: $($whoami.Output)"
}
Write-OK

try {

    # --- Hardware verification -------------------------------------------------
    Write-Step "Verifying hardware"
    $modelRaw = (Invoke-Pi "cat /proc/device-tree/model 2>/dev/null || grep Model /proc/cpuinfo 2>/dev/null | head -1 || echo unknown" -UseKey -AllowFail).Output
    if ($modelRaw -notmatch "Pi Zero 2") {
        Write-Warn "Expected Pi Zero 2 W, detected: $($modelRaw.Trim())"
        $cont = Read-Host "  Continue anyway? (y/N)"
        if ($cont -ne "y") { throw "Aborted by user" }
    } else {
        Write-OK
    }
    $arch = (Invoke-Pi "uname -m" -UseKey).Output.Trim()
    if ($arch -ne "aarch64") {
        Write-Warn "Expected aarch64 (64-bit), got: $arch - performance may be reduced"
    }

    # --- Sudo check -----------------------------------------------------------
    # We need passwordless sudo to install system files. On a fresh Raspberry
    # Pi OS install, the default user (rosie) is in the sudo group with
    # NOPASSWD via /etc/sudoers.d/010_pi-nopasswd. If that isn't the case,
    # stop with a clear message - there is no clean way to drive `sudo -S`
    # non-interactively from ssh.exe without storing the password.
    Write-Step "Checking passwordless sudo"
    $sudoCheck = Invoke-Pi "sudo -n true" -UseKey -AllowFail
    if ($sudoCheck.ExitCode -ne 0) {
        Write-Fail @"
Passwordless sudo is required for this installer.
On the Pi, run (once) as the rosie user:
  sudo visudo -f /etc/sudoers.d/010_rosie-nopasswd
and add this line:
  $script:PiUser ALL=(ALL) NOPASSWD:ALL
Then re-run this script.
"@
    }
    Write-OK

    # --- OS packages ----------------------------------------------------------
    Write-Step "Updating package lists and installing OS packages"
    Write-Host "       (apt-get update - up to a couple of minutes)" -ForegroundColor Gray
    $upd = Invoke-PiSudo "apt-get update" -AllowFail
    if ($upd.ExitCode -ne 0) {
        Write-Warn "apt-get update returned exit $($upd.ExitCode) - continuing"
    } else {
        Write-Host "       package lists refreshed" -ForegroundColor Gray
    }

    $pkgGroups = [ordered]@{
        "build tools (gcc, python3-dev, python3-venv, git)"            = "gcc python3-dev python3-venv git"
        "system services (unattended-upgrades, mosquitto-clients)"     = "unattended-upgrades mosquitto-clients fonts-dejavu-core"
        "python libs (numpy, scipy, pillow, serial, RPi.GPIO, psutil)" = "python3-numpy python3-scipy python3-pil python3-serial python3-rpi.gpio python3-psutil"
    }
    foreach ($label in $pkgGroups.Keys) {
        Write-Host ("       installing {0}..." -f $label) -ForegroundColor Gray
        $grp = $pkgGroups[$label]
        Invoke-PiSudo "DEBIAN_FRONTEND=noninteractive apt-get install -y -qq $grp" | Out-Null
        Write-Host "           done" -ForegroundColor DarkGray
    }
    Write-OK

    # --- Clone repository -----------------------------------------------------
    Write-Step "Cloning ROSie repository"
    if ([string]::IsNullOrWhiteSpace($GitToken)) {
        $repoUrl = "https://github.com/qniesz/ROSie.git"
    } else {
        $repoUrl = "https://${GitToken}@github.com/qniesz/ROSie.git"
    }
    $clone = Invoke-Pi "if [ -d ~/rosie/.git ]; then echo ALREADY_CLONED; else GIT_TERMINAL_PROMPT=0 git clone $repoUrl ~/rosie; fi" -UseKey -AllowFail
    if ($clone.ExitCode -ne 0) {
        if ($clone.Output -match "could not read Username|Authentication failed") {
            Write-Fail "Repo requires authentication. Add GitHub_PAT to ROSie.conf and re-run.`n  Create a token at: https://github.com/settings/tokens (scope: repo)"
        }
        Write-Fail "git clone failed:`n$($clone.Output)"
    }
    if ($clone.Output -match "ALREADY_CLONED") {
        Write-Warn "~/rosie already exists - skipping clone (run 'git pull' manually if needed)"
    } else {
        Write-OK
    }

    # --- Python venv ----------------------------------------------------------
    Write-Step "Creating Python venv and installing packages"
    Write-Host "       (breezyslam compiles C - allow ~3 min on Pi Zero)" -ForegroundColor Gray
    Invoke-Pi "python3 -m venv --system-site-packages ~/rosie-venv" -UseKey | Out-Null
    Write-Host "       installing paho-mqtt..." -ForegroundColor Gray
    Invoke-Pi "~/rosie-venv/bin/pip install -q paho-mqtt==1.6.1" -UseKey | Out-Null
    Write-Host "       building BreezySLAM from source (this is the slow part)..." -ForegroundColor Gray
    Invoke-Pi "~/rosie-venv/bin/pip install -q 'git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python'" -UseKey | Out-Null
    Write-OK

    # --- Maps directory -------------------------------------------------------
    Write-Step "Creating maps directory"
    Invoke-Pi "mkdir -p ~/maps" -UseKey | Out-Null
    Write-OK

    # --- Environment file -----------------------------------------------------
    Write-Step "Writing ~/rosie-driver.env"
    # Build the file locally, scp it over - this way the MQTT password never
    # appears on any remote command line.
    $tmpEnv = [System.IO.Path]::GetTempFileName()
    $envBody = @(
        "MQTT_HOST=$MqttHost",
        "MQTT_PORT=$MqttPort",
        "MQTT_USER=$MqttUser",
        "MQTT_PASS=$MqttPass",
        "MQTT_PREFIX=rosie",
        "ROSIE_SERIAL_PORT=/dev/ttyACM0",
        "ROSIE_MANUAL_UPDATES=$ManualUpdates"
    ) -join "`n"
    # Write with LF line endings and no BOM
    [System.IO.File]::WriteAllText($tmpEnv, $envBody + "`n", [System.Text.UTF8Encoding]::new($false))
    try {
        Copy-ToPi -LocalPath $tmpEnv -RemotePath "~/rosie-driver.env" -UseKey
        Invoke-Pi "chmod 600 ~/rosie-driver.env" -UseKey | Out-Null
    } finally {
        Remove-Item -LiteralPath $tmpEnv -ErrorAction SilentlyContinue
    }
    Write-OK

    # --- Sudoers entry --------------------------------------------------------
    Write-Step "Installing sudoers entry"
    Invoke-PiSudo "cp /home/$script:PiUser/rosie/pi/sudoers.d/rosie /etc/sudoers.d/rosie" | Out-Null
    Invoke-PiSudo "chmod 440 /etc/sudoers.d/rosie" | Out-Null
    $visudoCheck = Invoke-PiSudo "visudo -c" -AllowFail
    if ($visudoCheck.ExitCode -ne 0) {
        Write-Warn "visudo -c reported an issue: $($visudoCheck.Output)"
    } else {
        Write-OK
    }

    # --- Make scripts executable ----------------------------------------------
    Write-Step "Making update scripts executable"
    Invoke-Pi "chmod +x ~/rosie/pi/update.sh ~/rosie/pi/check_updates.sh" -UseKey | Out-Null
    Write-OK

    # --- Systemd units --------------------------------------------------------
    Write-Step "Installing systemd units"
    $units = @(
        "rosie.service",
        "rosie-update.service",
        "rosie-update.timer",
        "rosie-check-updates.service",
        "rosie-check-updates.timer"
    )
    foreach ($unit in $units) {
        Invoke-PiSudo "cp /home/$script:PiUser/rosie/pi/systemd/$unit /etc/systemd/system/$unit" | Out-Null
    }
    Invoke-PiSudo "systemctl daemon-reload" | Out-Null
    Invoke-PiSudo "systemctl enable rosie.service rosie-update.timer rosie-check-updates.timer" | Out-Null
    Invoke-PiSudo "systemctl start rosie.service" -AllowFail | Out-Null
    Write-OK

    # --- Unattended OS security updates ---------------------------------------
    Write-Step "Configuring automatic OS security patches"
    $autoUpg = @"
APT::Periodic::Update-Package-Lists `"1`";
APT::Periodic::Unattended-Upgrade `"1`";
APT::Periodic::AutocleanInterval `"7`";
"@
    $tmpAuto = [System.IO.Path]::GetTempFileName()
    [System.IO.File]::WriteAllText($tmpAuto, $autoUpg, [System.Text.UTF8Encoding]::new($false))
    try {
        Copy-ToPi -LocalPath $tmpAuto -RemotePath "/tmp/20auto-upgrades" -UseKey
        Invoke-PiSudo "mv /tmp/20auto-upgrades /etc/apt/apt.conf.d/20auto-upgrades" | Out-Null
    } finally {
        Remove-Item -LiteralPath $tmpAuto -ErrorAction SilentlyContinue
    }
    Invoke-PiSudo "bash -c 'grep -q Automatic-Reboot /etc/apt/apt.conf.d/50unattended-upgrades || echo ''Unattended-Upgrade::Automatic-Reboot \""false\"";'' >> /etc/apt/apt.conf.d/50unattended-upgrades'" | Out-Null
    Write-OK

    # --- Verify service -------------------------------------------------------
    Write-Step "Waiting for service to stabilise (5 s)"
    Start-Sleep -Seconds 5
    $svc = (Invoke-Pi "systemctl is-active rosie" -UseKey -AllowFail).Output.Trim()
    if ($svc -eq "active") {
        Write-OK
        Write-Host ""
        Write-Host "  ROSie is running!" -ForegroundColor Green
        Write-Host "  Check Home Assistant entities in ~30 seconds." -ForegroundColor Green
    } else {
        Write-Warn "Service status: $svc"
        Write-Host "  Run this to see why:" -ForegroundColor Yellow
        Write-Host "    ssh $script:PiUser@$script:PiHost 'journalctl -u rosie -n 50 --no-pager'" -ForegroundColor Gray
        Write-Host "  Common cause: Neato not plugged in yet (driver retries automatically)" -ForegroundColor Gray
    }

    # --- Summary --------------------------------------------------------------
    Write-Host ""
    Write-Host "  Deployment complete. Useful commands:" -ForegroundColor White
    Write-Host ""
    Write-Host "    ssh $script:PiUser@$script:PiHost" -ForegroundColor Gray
    Write-Host "    journalctl -u rosie -f                      # live logs" -ForegroundColor Gray
    Write-Host "    systemctl list-timers                        # scheduled timers" -ForegroundColor Gray
    Write-Host "    ~/rosie/pi/update.sh --force                 # manual update" -ForegroundColor Gray
    Write-Host ""
    Write-Host "  Serial port is set to /dev/ttyACM0 in ~/rosie-driver.env." -ForegroundColor Gray
    Write-Host "  Change ROSIE_SERIAL_PORT if your Neato appears on a different port." -ForegroundColor Gray
    Write-Host ""

} catch {
    throw
}
