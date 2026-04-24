#Requires -Version 5.1
<#
.SYNOPSIS
    One-time deployment script for ROSie on a Raspberry Pi Zero 2 W.

.DESCRIPTION
    Interactively prompts for Pi connection details and MQTT credentials,
    then performs a full first-time setup over SSH:
      - Verifies hardware and 64-bit OS
      - Pushes your SSH public key (no password required after this)
      - Installs OS packages
      - Clones the ROSie GitHub repository
      - Creates the Python venv and installs packages
      - Writes ~/rosie-driver.env with MQTT credentials
      - Installs the sudoers entry for passwordless service restart
      - Installs and enables all systemd units
      - Configures unattended OS security updates
      - Verifies the service started successfully

.NOTES
    Requires: Windows 10+ with OpenSSH client (built-in) and
              Posh-SSH module (auto-installed from PSGallery if missing).

    The Pi user must have sudo access (default on Raspberry Pi OS).
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# --- Posh-SSH module -----------------------------------------------------------
if (-not (Get-Module -ListAvailable -Name Posh-SSH)) {
    Write-Host "Installing Posh-SSH module from PSGallery..." -ForegroundColor Yellow
    try {
        Install-Module -Name Posh-SSH -Scope CurrentUser -Force -AllowClobber
    } catch {
        Write-Error "Failed to install Posh-SSH: $_`nInstall manually: Install-Module Posh-SSH -Scope CurrentUser"
        exit 1
    }
}
Import-Module Posh-SSH -DisableNameChecking

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

# --- SSH command helper --------------------------------------------------------
function Invoke-Pi {
    param(
        [Parameter(Mandatory)][string]$Command,
        [int]$TimeoutSec = 300,
        [switch]$AllowFail
    )
    $result = Invoke-SSHCommand -SessionId $script:Session.SessionId `
                                -Command $Command -TimeOut $TimeoutSec
    if (-not $AllowFail -and $result.ExitStatus -ne 0) {
        $errPart = if ($result.Error) { "`n  err : $($result.Error -join ' | ')" } else { "" }
        Write-Fail "Command failed (exit $($result.ExitStatus))`n  cmd : $Command`n  out : $($result.Output -join ' | ')$errPart"
    }
    return $result
}

# --- Sudo helper: feeds password via stdin when needed ------------------------
function Sudo-Pi {
    param(
        [Parameter(Mandatory)][string]$Command,
        [int]$TimeoutSec = 300,
        [switch]$AllowFail
    )
    if ($script:SudoNeedsPassword) {
        # Escape single quotes in password for shell single-quote wrapping
        $escPass = $PiPass -replace "'", "'\''"
        $wrapped = "echo '$escPass' | sudo -S -p '' $Command"
    } else {
        $wrapped = "sudo $Command"
    }
    if ($AllowFail) {
        return Invoke-Pi $wrapped -TimeoutSec $TimeoutSec -AllowFail
    } else {
        return Invoke-Pi $wrapped -TimeoutSec $TimeoutSec
    }
}

# --- Interactive prompts -------------------------------------------------------
Write-Host ""
Write-Host "ROSie Pi Deployment" -ForegroundColor White
Write-Host "===================" -ForegroundColor White
Write-Host ""

$PiHost      = Read-Host "  Pi IP address      (e.g. 192.168.x.x)"
$PiUser      = Read-Host "  Pi username        (e.g. rosie)"
$PiPassSec   = Read-Host "  Pi password" -AsSecureString
$MqttHost    = Read-Host "  MQTT broker / HA IP (e.g. 192.168.x.x)"
$MqttPort    = Read-Host "  MQTT port          (press Enter for 1883)"
$MqttUser    = Read-Host "  MQTT username"
$MqttPassSec = Read-Host "  MQTT password" -AsSecureString
$GitTokenSec = Read-Host "  GitHub PAT          (press Enter - repo is public)" -AsSecureString

if ([string]::IsNullOrWhiteSpace($MqttPort)) { $MqttPort = "1883" }

# Plain-text versions needed inside SSH command strings
$MqttPass = [System.Net.NetworkCredential]::new("", $MqttPassSec).Password
$PiPass   = [System.Net.NetworkCredential]::new("", $PiPassSec).Password
$GitToken = [System.Net.NetworkCredential]::new("", $GitTokenSec).Password
$script:SudoNeedsPassword = $false  # set after probe

Write-Host ""

# --- Pre-flight: clear stale host key + handle KEX algorithm mismatch ---------
# When a Pi is reflashed, its SSH host key changes; any previous entry in
# ~/.ssh/known_hosts will block all SSH attempts. We silently clear it.
# Also, modern Pi OS (Trixie+) defaults to KEX algorithms that the SSH.NET
# library bundled with Posh-SSH does not support, so we make sure the Pi
# accepts an older KEX before letting Posh-SSH connect.
Write-Step "SSH pre-flight (clearing stale host key, checking KEX support)"
$knownHosts = Join-Path $env:USERPROFILE ".ssh\known_hosts"
if (Test-Path $knownHosts) {
    & ssh-keygen -R $PiHost 2>&1 | Out-Null
}

# Probe Posh-SSH connectivity. If KEX negotiation fails, run a one-time
# bootstrap via native ssh.exe (which supports modern KEX) to enable
# legacy KEX algorithms on the Pi side.
$Cred  = New-Object System.Management.Automation.PSCredential($PiUser, $PiPassSec)
$probe = $null
try {
    $probe = New-SSHSession -ComputerName $PiHost -Credential $Cred -AcceptKey -ErrorAction Stop
} catch {
    if ($_.Exception.Message -match "Key exchange|kex|negotiation failed") {
        Write-Warn "Posh-SSH cannot negotiate a key exchange with this Pi."
        Write-Host "       Applying a one-time SSH compatibility fix on the Pi..." -ForegroundColor Gray
        Write-Host "       You will be prompted for the Pi password by native ssh.exe:" -ForegroundColor Gray
        $kexFix = "echo 'KexAlgorithms +diffie-hellman-group14-sha1,diffie-hellman-group-exchange-sha256,diffie-hellman-group14-sha256' | sudo tee /etc/ssh/sshd_config.d/99-legacy-kex.conf >/dev/null && sudo systemctl restart ssh && echo OK"
        & ssh -o StrictHostKeyChecking=accept-new "$PiUser@$PiHost" $kexFix
        if ($LASTEXITCODE -ne 0) {
            Write-Fail "Could not apply SSH KEX fix via native ssh.exe (exit $LASTEXITCODE)."
        }
        Write-Host "       SSH compatibility fix applied." -ForegroundColor Gray
    } else {
        Write-Fail "Cannot reach ${PiHost}: $($_.Exception.Message)"
    }
}
# Close probe session if it opened (we re-create it below for the real run)
if ($probe) { Remove-SSHSession -SessionId $probe.SessionId | Out-Null }
Write-OK

# --- Connect ------------------------------------------------------------------
Write-Step "Connecting to $PiHost"
try {
    $script:Session = New-SSHSession -ComputerName $PiHost -Credential $Cred -AcceptKey
    Write-OK
} catch {
    Write-Fail "Cannot connect to ${PiHost}: $_"
}

try {

    # --- Hardware verification -------------------------------------------------
    Write-Step "Verifying hardware"
    $modelRaw = (Invoke-Pi "cat /proc/device-tree/model 2>/dev/null || grep Model /proc/cpuinfo 2>/dev/null | head -1 || echo unknown" -AllowFail).Output
    if ($modelRaw -notmatch "Pi Zero 2") {
        Write-Warn "Expected Pi Zero 2 W, detected: $($modelRaw.Trim())"
        $cont = Read-Host "  Continue anyway? (y/N)"
        if ($cont -ne "y") { throw "Aborted by user" }
    } else {
        Write-OK
    }
    $arch = (Invoke-Pi "uname -m").Output.Trim()
    if ($arch -ne "aarch64") {
        Write-Warn "Expected aarch64 (64-bit), got: $arch - performance may be reduced"
    }

    # --- Verify sudo works ----------------------------------------------------
    Write-Step "Checking sudo access"
    $sudoCheck = Invoke-Pi "sudo -n true" -AllowFail
    if ($sudoCheck.ExitStatus -eq 0) {
        Write-OK
        Write-Host "       (passwordless sudo)" -ForegroundColor Gray
    } else {
        $script:SudoNeedsPassword = $true
        # Verify the supplied password actually works for sudo
        $sudoTest = Sudo-Pi "true" -AllowFail
        if ($sudoTest.ExitStatus -ne 0) {
            Write-Fail "Pi password does not grant sudo access. err: $($sudoTest.Error -join ' | ')"
        }
        Write-OK
        Write-Host "       (sudo password will be supplied via stdin)" -ForegroundColor Gray
    }

    # --- SSH key setup --------------------------------------------------------
    Write-Step "Setting up SSH key authentication"
    $keyPath = Join-Path $env:USERPROFILE ".ssh\id_rsa"
    $pubPath = "$keyPath.pub"
    if (-not (Test-Path $pubPath)) {
        Write-Host "       Generating RSA 4096 key pair at $keyPath..." -ForegroundColor Gray
        $null = & ssh-keygen -t rsa -b 4096 -N '""' -f $keyPath -q 2>&1
        if ($LASTEXITCODE -ne 0) {
            Write-Warn "ssh-keygen failed - continuing without key setup"
        }
    }
    if (Test-Path $pubPath) {
        $pubKey = (Get-Content $pubPath -Raw).Trim()
        # Append key, de-duplicate, fix permissions
        $keyCmd = "mkdir -p ~/.ssh && echo '$pubKey' >> ~/.ssh/authorized_keys && sort -u ~/.ssh/authorized_keys -o ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
        Invoke-Pi $keyCmd | Out-Null
        Write-OK
    } else {
        Write-Warn "No public key found - skipping key setup (password auth only)"
    }

    # --- OS packages ----------------------------------------------------------
    Write-Step "Updating package lists and installing OS packages"
    Write-Host "       (apt-get update - up to a couple of minutes)" -ForegroundColor Gray
    $updateResult = Sudo-Pi "apt-get update 2>&1" -TimeoutSec 300 -AllowFail
    if ($updateResult.ExitStatus -ne 0) {
        Write-Warn "apt-get update returned exit $($updateResult.ExitStatus) - continuing"
        Write-Host "       $($updateResult.Output -join ' | ')" -ForegroundColor Gray
    } else {
        Write-Host "       package lists refreshed" -ForegroundColor Gray
    }

    # Install in groups so the user sees progress between SSH calls
    $pkgGroups = [ordered]@{
        "build tools (gcc, python3-dev, python3-venv, git)"           = "gcc python3-dev python3-venv git"
        "system services (unattended-upgrades, mosquitto-clients)"    = "unattended-upgrades mosquitto-clients fonts-dejavu-core"
        "python libs (numpy, scipy, pillow, serial, RPi.GPIO, psutil)" = "python3-numpy python3-scipy python3-pil python3-serial python3-rpi.gpio python3-psutil"
    }
    foreach ($label in $pkgGroups.Keys) {
        Write-Host ("       installing {0}..." -f $label) -ForegroundColor Gray
        $grp = $pkgGroups[$label]
        $r = Sudo-Pi "DEBIAN_FRONTEND=noninteractive apt-get install -y -qq $grp" -TimeoutSec 600
        Write-Host "           done" -ForegroundColor DarkGray
    }
    Write-OK

    # --- Clone repository -----------------------------------------------------
    Write-Step "Cloning ROSie repository"
    if ([string]::IsNullOrWhiteSpace($GitToken)) {
        $repoUrl = "https://github.com/qniesz/ROSie.git"
    } else {
        # PAT-authenticated URL (token used as username, blank password works)
        $repoUrl = "https://${GitToken}@github.com/qniesz/ROSie.git"
    }
    # GIT_TERMINAL_PROMPT=0 makes git fail fast instead of hanging on a tty prompt
    $cloneOut = (Invoke-Pi "if [ -d ~/rosie/.git ]; then echo ALREADY_CLONED; else GIT_TERMINAL_PROMPT=0 git clone $repoUrl ~/rosie; fi" -TimeoutSec 300 -AllowFail)
    if ($cloneOut.ExitStatus -ne 0) {
        if ($cloneOut.Error -match "could not read Username|Authentication failed") {
            Write-Fail "Repo requires authentication. Re-run and provide a GitHub Personal Access Token at the prompt.`n        Create one at: https://github.com/settings/tokens (scope: repo)"
        } else {
            Write-Fail "git clone failed: $($cloneOut.Error -join ' | ')"
        }
    }
    if ($cloneOut.Output -match "ALREADY_CLONED") {
        Write-Warn "~/rosie already exists - skipping clone (run 'git pull' manually if needed)"
    } else {
        Write-OK
    }

    # --- Python venv ----------------------------------------------------------
    Write-Step "Creating Python venv and installing packages"
    Write-Host "       (breezyslam compiles C - allow ~3 min on Pi Zero)" -ForegroundColor Gray
    # --system-site-packages reuses distro numpy/scipy/Pillow/pyserial/RPi.GPIO
    # to avoid long compile times; pip adds paho-mqtt (pinned <2.0).
    Invoke-Pi "python3 -m venv --system-site-packages ~/rosie-venv" | Out-Null
    Write-Host "       installing paho-mqtt..." -ForegroundColor Gray
    Invoke-Pi '~/rosie-venv/bin/pip install -q "paho-mqtt>=1.6,<2.0"' -TimeoutSec 300 | Out-Null
    # BreezySLAM is not on PyPI for arm64 - build from source on GitHub.
    # The Python C extension is built by setup.py during pip install.
    Write-Host "       building BreezySLAM from source (this is the slow part)..." -ForegroundColor Gray
    Invoke-Pi "~/rosie-venv/bin/pip install -q 'git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python'" -TimeoutSec 900 | Out-Null
    Write-OK

    # --- Maps directory -------------------------------------------------------
    Write-Step "Creating maps directory"
    Invoke-Pi "mkdir -p ~/maps" | Out-Null
    Write-OK

    # --- Environment file -----------------------------------------------------
    Write-Step "Writing ~/rosie-driver.env"
    # Build the env file content and write it safely line by line
    $envLines = @(
        "MQTT_HOST=$MqttHost",
        "MQTT_PORT=$MqttPort",
        "MQTT_USER=$MqttUser",
        "MQTT_PASS=$MqttPass",
        "MQTT_PREFIX=rosie",
        "ROSIE_SERIAL_PORT=/dev/ttyACM0"
    )
    Invoke-Pi "rm -f ~/rosie-driver.env.tmp" | Out-Null
    foreach ($line in $envLines) {
        Invoke-Pi "echo '$line' >> ~/rosie-driver.env.tmp" | Out-Null
    }
    Invoke-Pi "mv ~/rosie-driver.env.tmp ~/rosie-driver.env && chmod 600 ~/rosie-driver.env" | Out-Null
    Write-OK

    # --- Sudoers entry --------------------------------------------------------
    Write-Step "Installing sudoers entry (passwordless service restart)"
    Sudo-Pi "cp /home/$PiUser/rosie/pi/sudoers.d/rosie /etc/sudoers.d/rosie" | Out-Null
    Sudo-Pi "chmod 440 /etc/sudoers.d/rosie" | Out-Null
    $visudoCheck = Sudo-Pi "visudo -c" -AllowFail
    if ($visudoCheck.ExitStatus -ne 0) {
        Write-Warn "visudo -c reported an issue: $($visudoCheck.Output)"
    } else {
        Write-OK
    }

    # --- Make scripts executable ----------------------------------------------
    Write-Step "Making update scripts executable"
    Invoke-Pi "chmod +x ~/rosie/pi/update.sh ~/rosie/pi/check_updates.sh" | Out-Null
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
        Sudo-Pi "cp /home/$PiUser/rosie/pi/systemd/$unit /etc/systemd/system/$unit" | Out-Null
    }
    Sudo-Pi "systemctl daemon-reload" | Out-Null
    Sudo-Pi "systemctl enable rosie.service rosie-update.timer rosie-check-updates.timer" | Out-Null
    Sudo-Pi "systemctl start rosie.service" -AllowFail | Out-Null
    Write-OK

    # --- Unattended OS security updates ---------------------------------------
    Write-Step "Configuring automatic OS security patches"
    # Stage files in user home (no sudo), then move into place with sudo
    Invoke-Pi "cat > /tmp/20auto-upgrades <<'EOF'`nAPT::Periodic::Update-Package-Lists `"1`";`nAPT::Periodic::Unattended-Upgrade `"1`";`nAPT::Periodic::AutocleanInterval `"7`";`nEOF" | Out-Null
    Sudo-Pi "mv /tmp/20auto-upgrades /etc/apt/apt.conf.d/20auto-upgrades" | Out-Null
    Invoke-Pi "echo 'Unattended-Upgrade::Automatic-Reboot `"false`";' > /tmp/50extra" | Out-Null
    Sudo-Pi "bash -c 'cat /tmp/50extra >> /etc/apt/apt.conf.d/50unattended-upgrades && rm /tmp/50extra'" | Out-Null
    Write-OK

    # --- Verify service -------------------------------------------------------
    Write-Step "Waiting for service to stabilise (5 s)"
    Start-Sleep -Seconds 5
    $svcStatus = (Invoke-Pi "systemctl is-active rosie" -AllowFail).Output.Trim()
    if ($svcStatus -eq "active") {
        Write-OK
        Write-Host ""
        Write-Host "  ROSie is running!" -ForegroundColor Green
        Write-Host "  Check Home Assistant entities in ~30 seconds." -ForegroundColor Green
    } else {
        Write-Warn "Service status: $svcStatus"
        Write-Host "  Run this to see why:" -ForegroundColor Yellow
        Write-Host "    ssh ${PiUser}@${PiHost} 'journalctl -u rosie -n 50 --no-pager'" -ForegroundColor Gray
        Write-Host "  Common cause: Neato not plugged in yet (driver retries automatically)" -ForegroundColor Gray
    }

    # --- Summary --------------------------------------------------------------
    Write-Host ""
    Write-Host "  Deployment complete. Useful commands:" -ForegroundColor White
    Write-Host ""
    Write-Host "    ssh ${PiUser}@${PiHost}" -ForegroundColor Gray
    Write-Host "    journalctl -u rosie -f                      # live logs" -ForegroundColor Gray
    Write-Host "    systemctl list-timers                        # scheduled timers" -ForegroundColor Gray
    Write-Host "    ~/rosie/pi/update.sh --force                 # manual update" -ForegroundColor Gray
    Write-Host ""
    Write-Host "  Serial port is set to /dev/ttyACM0 in ~/rosie-driver.env." -ForegroundColor Gray
    Write-Host "  Change ROSIE_SERIAL_PORT if your Neato appears on a different port." -ForegroundColor Gray
    Write-Host ""

} finally {
    if ($script:Session) {
        Remove-SSHSession -SessionId $script:Session.SessionId | Out-Null
    }
}
