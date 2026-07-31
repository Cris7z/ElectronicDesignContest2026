[CmdletBinding()]
param(
    [string]$ExpectedFfmpegRoot = 'D:\A-Soft\DevTools\FFmpeg\bin',
    [string]$ExpectedCanmvRoot = 'D:\A-Soft\DevTools\CanMV-K230',
    [string]$ExpectedCanmvIdeExe = 'D:\A-Soft\DevTools\CanMV-K230\IDE\bin\canmvide.exe'
)

$ErrorActionPreference = 'Stop'

# Codex/PowerShell sessions opened before the installer ran do not inherit an
# updated user PATH.  Require the persistent entry, then add it only to this
# verification process so the commands below exercise the installed binaries.
$userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
$expectedFfmpegFull = [IO.Path]::GetFullPath($ExpectedFfmpegRoot)
$userFfmpegEntries = @($userPath -split ';' | Where-Object {
    $_ -and [IO.Path]::GetFullPath($_).TrimEnd('\\') -eq $expectedFfmpegFull.TrimEnd('\\')
})
if (-not $userFfmpegEntries.Count) {
    throw "User PATH does not contain $ExpectedFfmpegRoot"
}
$processFfmpegEntries = @($env:Path -split ';' | Where-Object {
    $_ -and [IO.Path]::GetFullPath($_).TrimEnd('\\') -eq $expectedFfmpegFull.TrimEnd('\\')
})
if (-not $processFfmpegEntries.Count) {
    $env:Path = "$ExpectedFfmpegRoot;$env:Path"
}

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
if (-not (Test-Path -LiteralPath $ExpectedCanmvIdeExe -PathType Leaf)) {
    throw "CanMV IDE executable is missing: $ExpectedCanmvIdeExe"
}
Write-Output "CanMV_IDE_EXE=$ExpectedCanmvIdeExe"

$python = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $python) {
    throw 'python is not on PATH'
}
Write-Output "python=$($python.Source)"
& $python.Source -m unittest discover -s (Join-Path $PSScriptRoot '..\tests') -v
