[CmdletBinding()]
param(
    [string]$Url = 'rtsp://192.168.4.1:8554/ball',
    [string]$OutputDirectory = 'D:\X\ElectronicDesignContest2026-Rebuild-local\k230\recordings',
    [int]$DurationSeconds = 0,
    [switch]$ViewOnly
)

$ErrorActionPreference = 'Stop'
$ffplay = Get-Command ffplay -ErrorAction Stop
$ffmpeg = Get-Command ffmpeg -ErrorAction Stop

if ($ViewOnly) {
    & $ffplay.Source -rtsp_transport tcp -fflags nobuffer -flags low_delay $Url
    exit $LASTEXITCODE
}

New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$output = Join-Path $OutputDirectory "ball-$stamp.mkv"
$arguments = @('-hide_banner', '-nostdin', '-rtsp_transport', 'tcp', '-i', $Url, '-map', '0:v:0', '-c', 'copy')
if ($DurationSeconds -gt 0) {
    $arguments += @('-t', $DurationSeconds)
}
$arguments += $output
& $ffmpeg.Source @arguments
if ($LASTEXITCODE -ne 0) {
    throw "ffmpeg failed with exit code $LASTEXITCODE"
}
Get-FileHash -LiteralPath $output -Algorithm SHA256 | Format-List
Write-Output "RECORDING=$output"
