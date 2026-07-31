[CmdletBinding()]
param(
    [string]$Url = 'rtsp://192.168.4.1:8554/ball',
    [string]$OutputDirectory = 'D:\X\ElectronicDesignContest2026-Rebuild-local\k230\recordings',
    [ValidateRange(1, 480)][int]$TotalMinutes = 120,
    [ValidateRange(60, 3600)][int]$SegmentSeconds = 600
)

$ErrorActionPreference = 'Stop'
$ffmpeg = Get-Command ffmpeg -ErrorAction Stop
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null

$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$runDirectory = Join-Path $OutputDirectory "endurance-$stamp"
New-Item -ItemType Directory -Force -Path $runDirectory | Out-Null
$pattern = Join-Path $runDirectory 'ball-%03d.mkv'
$duration = $TotalMinutes * 60

& $ffmpeg.Source -hide_banner -nostdin -rtsp_transport tcp -i $Url -t $duration `
    -map 0:v:0 -c copy -f segment -segment_time $SegmentSeconds `
    -reset_timestamps 1 -segment_format matroska $pattern
if ($LASTEXITCODE -ne 0) {
    throw "ffmpeg failed with exit code $LASTEXITCODE"
}

$manifest = Join-Path $runDirectory 'SHA256SUMS.txt'
Get-ChildItem -LiteralPath $runDirectory -Filter '*.mkv' | Sort-Object Name |
    Get-FileHash -Algorithm SHA256 |
    ForEach-Object { "{0}  {1}" -f $_.Hash, $_.Path } |
    Set-Content -LiteralPath $manifest -Encoding utf8
Write-Output "ENDURANCE_RECORDINGS=$runDirectory"
Write-Output "SHA256_MANIFEST=$manifest"
