[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$Path
)

$ErrorActionPreference = 'Stop'
$ffprobe = Get-Command ffprobe -ErrorAction Stop
$files = if (Test-Path -LiteralPath $Path -PathType Container) {
    Get-ChildItem -LiteralPath $Path -Filter '*.mkv' | Sort-Object Name
} else {
    Get-Item -LiteralPath $Path
}
if ($files.Count -eq 0) {
    throw "No MKV files found at $Path"
}

foreach ($file in $files) {
    $probe = & $ffprobe.Source -v error -show_entries format=duration -show_entries stream=codec_name,width,height `
        -of default=noprint_wrappers=1 -select_streams v:0 $file.FullName
    if ($LASTEXITCODE -ne 0) {
        throw "ffprobe failed: $($file.FullName)"
    }
    Write-Output "### $($file.FullName)"
    $probe
    Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256 | Format-List
}
