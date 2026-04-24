Add-Type @"
using System.Net;
using System.Security.Cryptography.X509Certificates;
public class TC4 : ICertificatePolicy {
    public bool CheckValidationResult(ServicePoint sp, X509Certificate cert, WebRequest req, int err) { return true; }
}
"@
[System.Net.ServicePointManager]::CertificatePolicy = New-Object TC4
[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.SecurityProtocolType]::Tls
try {
    $r = Invoke-WebRequest -Uri 'https://192.168.219.1:4443/info' -TimeoutSec 5 -UseBasicParsing
    Write-Host "Robot still up: $($r.Content)"
} catch {
    Write-Host "Robot UNREACHABLE: $($_.Exception.Message)"
}
