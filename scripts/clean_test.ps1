param([string]$Port = 'COM5', [int]$BaudRate = 115200)

$p = [System.IO.Ports.SerialPort]::new($Port, $BaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 200; $p.WriteTimeout = 200; $p.DtrEnable = $false; $p.RtsEnable = $false
$p.Open()

function Q([string]$cmd, [int]$ms = 3000) {
    $b = [System.Text.Encoding]::ASCII.GetBytes($cmd + "`n")
    $p.Write($b, 0, $b.Length)
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $sb = [System.Text.StringBuilder]::new()
    $last = 0
    while ($sw.ElapsedMilliseconds -lt $ms) {
        if ($p.BytesToRead -gt 0) {
            $buf = New-Object byte[] $p.BytesToRead
            $n = $p.Read($buf, 0, $buf.Length)
            [void]$sb.Append([System.Text.Encoding]::ASCII.GetString($buf, 0, $n))
            $last = $sw.ElapsedMilliseconds
        }
        if ($sb.Length -gt 0 -and ($sw.ElapsedMilliseconds - $last) -ge 800) { break }
        Start-Sleep -Milliseconds 10
    }
    return $sb.ToString().Trim()
}

Q 'TestMode On' 800 | Out-Null
$p.DiscardInBuffer()

# Try every plausible WiFi clear / reset command
@(
    'Help SetWifi',
    'SetWifi clear',
    'SetWifi Clear',
    'SetWifi reset',
    'SetWifi Reset',
    'SetWifi delete',
    'SetWifi Delete',
    'SetWifi remove',
    'SetWifi wpa',
    'SetWifi wpa clear',
    'SetWifi config',
    'SetWifi config clear',
    'SetWifi credentials',
    'ClearWifi',
    'DeleteWifi',
    'WifiClear',
    'WifiReset',
    'SetWifiConfig',
    'ClearWifiConfig',
    'SetWifiSettings',
    'SetWifi mode',
    'SetWifi mode ap',
    'SetWifi mode setup',
    'SetWifi setup',
    'SetWifi factory',
    'SetWifi factory reset',
    'SetWifi unpair',
    'SetWifi unprovision',
    'GetNetworkSettings',
    'SetNetworkSettings',
    'GetMDNS',
    'SetMDNS'
) | ForEach-Object {
    $resp = Q $_ 2500
    if ($resp -notmatch 'Unknown Cmd|Unrecognized|Command Not Found') {
        Write-Host "=== $_ ===" -ForegroundColor Green
        Write-Host $resp
    } else {
        Write-Host "--- $_ --- $($resp -split "`n" | Select-Object -First 1)" -ForegroundColor DarkGray
    }
    Write-Host ""
}

$p.Close(); $p.Dispose()
Write-Host "Done." -ForegroundColor Cyan
