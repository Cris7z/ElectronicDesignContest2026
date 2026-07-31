[CmdletBinding()]
param(
    [string]$Root = 'D:\X\ElectronicDesignContest2026-Rebuild-local\k230'
)

$ErrorActionPreference = 'Stop'

$folders = @(
    'archive', 'firmware', 'models', 'dataset', 'board_snapshots',
    'calibration', 'logs', 'recordings', 'evidence'
)
foreach ($folder in $folders) {
    New-Item -ItemType Directory -Force -Path (Join-Path $Root $folder) | Out-Null
}

Write-Output "K230_LOCAL_ROOT=$Root"
Get-ChildItem -LiteralPath $Root -Directory | Select-Object -ExpandProperty FullName
