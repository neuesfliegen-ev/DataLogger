# PowerShell reader into a .txt file

# ------------ Arduino serial monitor + logger --------------------
$Port  = 'COM7'          # check in Device Manager – COM = communication port
$Baud  = 115200
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$log   = "$env:USERPROFILE\Desktop\nano_datalog_mag2.txt"   # change path if you like

# open the serial port
$sp = [System.IO.Ports.SerialPort]::new(
        $Port, $Baud,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One)

$sp.DtrEnable = $true    # Data Terminal Ready – makes Nano 33 BLE start talking
# (optional) $sp.RtsEnable = $true   # Request To Send, IDE (Integrated Development Environment) sets it too

try {
    $sp.Open()
    Write-Host "Listening on $Port ($Baud baud)… logging to $log"
    Write-Host "Press Ctrl+C to quit`n"

    while ($true) {
        $line = $sp.ReadLine().TrimEnd("`r", "`n")   # strip CR/LF
        $line | Tee-Object -FilePath $log -Append    # show and append
    }
}
finally {
    $sp.Close()
    Write-Host "`nSerial port closed. Log saved at $log"
}
