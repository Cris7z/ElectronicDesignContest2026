param(
    [string]$SdkRoot = 'D:\A-Soft\DevTools\TI\M0SDK_2_11',
    [string]$CcsRoot = 'D:\A-Soft\DevTools\TI\ccs2100',
    [string]$OutputDirectory = '',
    [string]$AppSource = 'app\main.c'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$sharedRoot = (Resolve-Path (Join-Path $projectRoot '..\h2026_q2')).Path
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $projectRoot 'Build\default'
}
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).Path

$sysconfigCli = Join-Path $CcsRoot 'ccs\utils\sysconfig_1.28.0\sysconfig_cli.bat'
$compilerRoot = Join-Path $CcsRoot 'ccs\tools\compiler\ti-cgt-armllvm_5.1.1.LTS'
$compiler = Join-Path $compilerRoot 'bin\tiarmclang.exe'
$hexTool = Join-Path $compilerRoot 'bin\tiarmhex.exe'
$sourceRoot = Join-Path $SdkRoot 'source'
$cmsisRoot = Join-Path $sourceRoot 'third_party\CMSIS\Core\Include'
$sysconfigScript = Join-Path $sharedRoot 'bsp\h2026_q2.syscfg'
$appPath = Join-Path $projectRoot $AppSource

foreach ($item in @($sysconfigCli, $compiler, $hexTool, $sysconfigScript, $appPath)) {
    if (-not (Test-Path -LiteralPath $item)) {
        throw "Required build input not found: $item"
    }
}

& $sysconfigCli --product (Join-Path $SdkRoot '.metadata\product.json') `
    --compiler ticlang --script $sysconfigScript --output $OutputDirectory `
    --treatWarningsAsErrors
if ($LASTEXITCODE -ne 0) { throw "SysConfig failed: $LASTEXITCODE" }

# Keep the final 1 KiB unallocated, matching the live vehicle's known layout.
$linkerFile = Join-Path $OutputDirectory 'device_linker.cmd'
$linkerText = [IO.File]::ReadAllText($linkerFile)
$pattern = '(?m)^(\s*FLASH\s+\(RX\)\s*:\s*origin\s*=\s*0x00000000,\s*length\s*=\s*)0x00020000'
if (-not [regex]::IsMatch($linkerText, $pattern)) {
    throw "Unexpected Flash region in generated linker file: $linkerFile"
}
[IO.File]::WriteAllText($linkerFile,
    [regex]::Replace($linkerText, $pattern, '${1}0x0001FC00'))

$sources = @(
    $appPath,
    (Join-Path $sharedRoot 'bsp\h2026_bsp.c'),
    (Join-Path $OutputDirectory 'ti_msp_dl_config.c'),
    (Join-Path $sourceRoot 'ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c')
)
if ($AppSource -eq 'app\main.c') {
    $sources = @(
        (Join-Path $projectRoot 'core\line_tracker.c'),
        (Join-Path $projectRoot 'core\wheel_speed_pi.c')
    ) + $sources
} elseif ($AppSource -eq 'app\main_wheel_pi_bench.c') {
    $sources = @((Join-Path $projectRoot 'core\wheel_speed_pi.c')) + $sources
}
$compileArgs = @(
    '-c', "@$(Join-Path $OutputDirectory 'device.opt')", '-std=c11',
    '-mcpu=cortex-m0plus', '-march=thumbv6m', '-mfloat-abi=soft', '-mthumb',
    '-Oz', '-g', '-Wall', '-Wextra', '-Werror',
    "-I$OutputDirectory", "-I$(Join-Path $projectRoot 'core')",
    "-I$(Join-Path $sharedRoot 'bsp')", "-I$sourceRoot", "-I$cmsisRoot"
)
$objects = @()
foreach ($source in $sources) {
    $object = Join-Path $OutputDirectory (([IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
    & $compiler @compileArgs -o $object $source
    if ($LASTEXITCODE -ne 0) { throw "Compile failed for $source" }
    $objects += $object
}

$outputFile = Join-Path $OutputDirectory 'line_tracker.out'
$mapFile = Join-Path $OutputDirectory 'line_tracker.map'
& $compiler "@$(Join-Path $OutputDirectory 'device.opt')" '-mcpu=cortex-m0plus' `
    '-march=thumbv6m' '-mfloat-abi=soft' '-mthumb' '-Oz' '-g' "-Wl,-m$mapFile" `
    "-Wl,-i$sourceRoot" "-Wl,-i$OutputDirectory" "-Wl,-i$(Join-Path $compilerRoot 'lib')" `
    '-Wl,--rom_model' '-Wl,--warn_sections' '-o' $outputFile @objects `
    "-Wl,-l$(Join-Path $OutputDirectory 'device_linker.cmd')" `
    "-Wl,-l$(Join-Path $OutputDirectory 'device.cmd.genlibs')" '-Wl,-llibc.a'
if ($LASTEXITCODE -ne 0) { throw "Link failed: $LASTEXITCODE" }

$hexFile = Join-Path $OutputDirectory 'line_tracker.hex'
& $hexTool --byte --memwidth=8 --romwidth=8 --intel -o $hexFile $outputFile
if ($LASTEXITCODE -ne 0) { throw "Hex conversion failed: $LASTEXITCODE" }

Get-Item -LiteralPath $outputFile, $hexFile, $mapFile |
    Select-Object Name, Length, FullName | Format-Table -AutoSize
Select-String -LiteralPath $mapFile -Pattern '^\s+FLASH\s+', '^\s+SRAM\s+' |
    ForEach-Object { $_.Line }
