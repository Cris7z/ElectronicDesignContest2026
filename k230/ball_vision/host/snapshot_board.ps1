[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$SourceRoot,
    [string]$SnapshotRoot = 'D:\X\ElectronicDesignContest2026-Rebuild-local\k230\board_snapshots'
)

$ErrorActionPreference = 'Stop'
$source = (Resolve-Path -LiteralPath $SourceRoot).Path
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$destination = Join-Path $SnapshotRoot $stamp
New-Item -ItemType Directory -Force -Path $destination | Out-Null

$files = Get-ChildItem -LiteralPath $source -Recurse -File |
    Where-Object { $_.Extension -in '.py', '.json', '.kmodel', '.txt' }
if ($files.Count -eq 0) {
    throw "No board script, configuration or model files found below $source"
}

foreach ($file in $files) {
    $relative = $file.FullName.Substring($source.Length).TrimStart('\')
    $target = Join-Path $destination $relative
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $target) | Out-Null
    Copy-Item -LiteralPath $file.FullName -Destination $target
}

$manifest = Join-Path $destination 'SHA256SUMS.txt'
Get-ChildItem -LiteralPath $destination -Recurse -File | Where-Object { $_.Name -ne 'SHA256SUMS.txt' } |
    Sort-Object FullName | Get-FileHash -Algorithm SHA256 |
    ForEach-Object { "{0}  {1}" -f $_.Hash, $_.Path.Substring($destination.Length).TrimStart('\') } |
    Set-Content -LiteralPath $manifest -Encoding utf8
Write-Output "BOARD_SNAPSHOT=$destination"
Write-Output "SHA256_MANIFEST=$manifest"
