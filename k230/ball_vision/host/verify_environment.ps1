[CmdletBinding()]
param(
    [string]$ExpectedFfmpegRoot = 'D:\A-Soft\DevTools\FFmpeg\bin',
    [string]$ExpectedCanmvRoot = 'D:\A-Soft\DevTools\CanMV-K230'
)

$ErrorActionPreference = 'Stop'

foreach ($tool in 'ffmpeg', 'ffplay', 'ffprobe') {
    $command = Get-Command $tool -ErrorAction SilentlyContinue
    if ($null -eq $command) {
        throw "$tool is not on PATH"
    }
    $resolved = [IO.Path]::GetFullPath($command.Source)
    if (-not $resolved.StartsWith([IO.Path]::GetFullPath($ExpectedFfmpegRoot), [StringComparison]::OrdinalIgnoreCase)) {
        throw "$tool resolves outside $ExpectedFfmpegRoot : $resolved"
    }
    Write-Output "$tool=$resolved"
    & $command.Source -version | Select-Object -First 1
}

if (-not (Test-Path -LiteralPath $ExpectedCanmvRoot)) {
    throw "CanMV K230 tool root is missing: $ExpectedCanmvRoot"
}
Write-Output "CanMV_K230_ROOT=$ExpectedCanmvRoot"

$python = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $python) {
    throw 'python is not on PATH'
}
Write-Output "python=$($python.Source)"
& $python.Source -m unittest discover -s (Join-Path $PSScriptRoot '..\tests') -v
