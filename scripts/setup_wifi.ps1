param(
    [string]$RobotIP = '192.168.219.1',
    [string]$Ssid = 'YOUR_WIFI_SSID',
    [string]$Password = 'YOUR_WIFI_PASSWORD',
    [string]$RobotName = 'downstairs',
    [string]$Timezone = 'America/Chicago',
    [string]$UtcOffset = 'UTC-6:00UTC-5:00'
)

Add-Type @"
using System.Net;
using System.Security.Cryptography.X509Certificates;
public class TC6 : ICertificatePolicy {
    public bool CheckValidationResult(ServicePoint sp, X509Certificate cert, WebRequest req, int err) { return true; }
}
"@
[System.Net.ServicePointManager]::CertificatePolicy = New-Object TC6
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.SecurityProtocolType]::Tls

$h = "https://${RobotIP}:4443"

function Req($method, $url, $body = $null) {
    try {
        $params = @{ Uri = $url; Method = $method; TimeoutSec = 10; UseBasicParsing = $true }
        if ($null -ne $body) { $params.Body = $body; $params.ContentType = 'application/json' }
        $r = Invoke-WebRequest @params
        return "HTTP $($r.StatusCode) | $($r.Content)"
    } catch [System.Net.WebException] {
        if ($_.Exception.Response) {
            $code = [int]$_.Exception.Response.StatusCode
            $reader = [System.IO.StreamReader]::new($_.Exception.Response.GetResponseStream())
            return "HTTP $code | $($reader.ReadToEnd())"
        }
        return "WebException: $($_.Exception.Message)"
    } catch {
        return "ERROR: $($_.Exception.Message)"
    }
}

# Confirm robot is up
Write-Host "=== /info ===" -ForegroundColor Gray
Write-Host (Req 'GET' "$h/info")
Write-Host ""

# Check wifi_networks to confirm target SSID is visible
Write-Host "=== /wifi_networks ===" -ForegroundColor Gray
Write-Host (Req 'GET' "$h/wifi_networks")
Write-Host ""

# Check current progress state
Write-Host "=== /robot/wifi_networks/new/progress ===" -ForegroundColor Gray
Write-Host (Req 'GET' "$h/robot/wifi_networks/new/progress")
Write-Host ""

# Correct body format from research/setup-network.md (MITM'd from official Neato iOS app)
# Note: beehive/nucleo have .neatocloud.com auto-appended by the robot firmware
$body = @"
{"name":"$RobotName","password":"$Password","server_urls":{"beehive":"beehive","ntp":"pool.ntp.org","nucleo":"nucleo"},"ssid":"$Ssid","timezone":"$Timezone","user_id":"local","utc_offset":"$UtcOffset"}
"@
Write-Host "=== PUT /robot/initialize ===" -ForegroundColor Green
Write-Host "  Body: $($body.Trim())"
Write-Host "  Response: $(Req 'PUT' "$h/robot/initialize" $body.Trim())"
Write-Host ""

# Poll progress after sending initialize
Write-Host "=== Polling /robot/wifi_networks/new/progress ===" -ForegroundColor Cyan
for ($i = 0; $i -lt 15; $i++) {
    Start-Sleep -Seconds 2
    $prog = Req 'GET' "$h/robot/wifi_networks/new/progress"
    Write-Host "  [$($i*2)s] $prog"
    if ($prog -match '"step"\s*:\s*([3-9]|[0-9]{2})') { break }
    if ($prog -match 'WebException|connect') { break }
}

Write-Host ""
Write-Host "Done." -ForegroundColor Green
