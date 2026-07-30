param(
    [string]$SdkRoot = 'D:\A-Soft\DevTools\TI\M0SDK_2_11',
    [string]$CcsRoot = 'D:\A-Soft\DevTools\TI\ccs2100',
    [string]$OutputDirectory = '',
    [switch]$MotorCommissionTest,
    [switch]$MotorCommissionAuto,
    [switch]$EncoderPassiveDiagnostic,
    [switch]$BlsPassiveDiagnostic,
    [switch]$DistanceCalibrationDiagnostic
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
if ($MotorCommissionAuto -and (-not $MotorCommissionTest)) {
    throw '-MotorCommissionAuto requires -MotorCommissionTest.'
}
if (($EncoderPassiveDiagnostic -or $BlsPassiveDiagnostic -or $DistanceCalibrationDiagnostic) -and
    ($MotorCommissionTest -or $MotorCommissionAuto)) {
    throw 'Passive diagnostics cannot be combined with motor-test options.'
}
$diagnosticCount = 0
if ($EncoderPassiveDiagnostic) { ++$diagnosticCount }
if ($BlsPassiveDiagnostic) { ++$diagnosticCount }
if ($DistanceCalibrationDiagnostic) { ++$diagnosticCount }
if ($diagnosticCount -gt 1) {
    throw 'Choose only one diagnostic.'
}
if ([string]::IsNullOrWhiteSpace($OutputDirectory)) {
    $OutputDirectory = Join-Path $projectRoot 'Debug'
}
New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
$OutputDirectory = (Resolve-Path -LiteralPath $OutputDirectory).Path

$sysconfigCli = Join-Path $CcsRoot 'ccs\utils\sysconfig_1.28.0\sysconfig_cli.bat'
$compilerRoot = Join-Path $CcsRoot 'ccs\tools\compiler\ti-cgt-armllvm_5.1.1.LTS'
$compiler = Join-Path $compilerRoot 'bin\tiarmclang.exe'
$hexTool = Join-Path $compilerRoot 'bin\tiarmhex.exe'
$productJson = Join-Path $SdkRoot '.metadata\product.json'
$sysconfigScript = Join-Path $projectRoot 'bsp\h2026_q2.syscfg'

$requiredPaths = @(
    $sysconfigCli,
    $compiler,
    $hexTool,
    $productJson,
    $sysconfigScript
)
foreach ($requiredPath in $requiredPaths) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Required build input not found: $requiredPath"
    }
}

& $sysconfigCli `
    --product $productJson `
    --compiler ticlang `
    --script $sysconfigScript `
    --output $OutputDirectory `
    --treatWarningsAsErrors
if ($LASTEXITCODE -ne 0) {
    throw "SysConfig failed with exit code $LASTEXITCODE"
}

# Reserve the final 1 KiB main-Flash sector (0x1FC00..0x1FFFF) exclusively
# for the line-sensor calibration record. SysConfig regenerates this linker
# file on every build, so make the reservation immediately after generation.
$linkerFile = Join-Path $OutputDirectory 'device_linker.cmd'
$linkerText = [IO.File]::ReadAllText($linkerFile)
$flashRegion = '(?m)^(\s*FLASH\s+\(RX\)\s*:\s*origin\s*=\s*0x00000000,\s*length\s*=\s*)0x00020000'
if (-not [regex]::IsMatch($linkerText, $flashRegion)) {
    throw "Unexpected Flash region in generated linker file: $linkerFile"
}
[IO.File]::WriteAllText($linkerFile,
    [regex]::Replace($linkerText, $flashRegion, '${1}0x0001FC00'))

$sourceRoot = Join-Path $SdkRoot 'source'
$cmsisRoot = Join-Path $sourceRoot 'third_party\CMSIS\Core\Include'
$applicationSource = if ($DistanceCalibrationDiagnostic) {
    Join-Path $projectRoot 'app\main_distance_calibration.c'
} elseif ($BlsPassiveDiagnostic) {
    Join-Path $projectRoot 'app\main_bls_passive.c'
} elseif ($EncoderPassiveDiagnostic) {
    Join-Path $projectRoot 'app\main_encoder_passive.c'
} elseif ($MotorCommissionTest) {
    Join-Path $projectRoot 'app\main_motor_commission.c'
} else {
    Join-Path $projectRoot 'app\main.c'
}
$outputStem = if ($DistanceCalibrationDiagnostic) {
    'h2026_q2_distance_cal'
} elseif ($BlsPassiveDiagnostic) {
    'h2026_q2_bls_passive'
} elseif ($EncoderPassiveDiagnostic) {
    'h2026_q2_encoder_passive'
} elseif ($MotorCommissionTest) {
    'h2026_q2_motor_test'
} else {
    'h2026_q2'
}
$sources = @(
    (Join-Path $projectRoot 'core\h2026_q2.c'),
    (Join-Path $projectRoot 'bsp\h2026_bsp.c'),
    (Join-Path $projectRoot 'app\h2026_q2_app_config.c'),
    (Join-Path $projectRoot 'app\h2026_q2_display.c'),
    $applicationSource,
    (Join-Path $OutputDirectory 'ti_msp_dl_config.c'),
    (Join-Path $sourceRoot 'ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c')
)

$commonCompileArgs = @(
    "-c",
    "@$(Join-Path $OutputDirectory 'device.opt')",
    "-std=c11",
    "-mcpu=cortex-m0plus",
    "-march=thumbv6m",
    "-mfloat-abi=soft",
    "-mthumb",
    "-Oz",
    "-g",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I$OutputDirectory",
    "-I$(Join-Path $projectRoot 'app')",
    "-I$(Join-Path $projectRoot 'bsp')",
    "-I$(Join-Path $projectRoot 'core')",
    "-I$sourceRoot",
    "-I$cmsisRoot"
)
if ($MotorCommissionAuto) {
    $commonCompileArgs += '-DH2026_MOTOR_TEST_AUTORUN=1'
}

$objects = @()
foreach ($source in $sources) {
    if (-not (Test-Path -LiteralPath $source)) {
        throw "Source file not found: $source"
    }
    $objectName = [IO.Path]::GetFileNameWithoutExtension($source) + '.o'
    $object = Join-Path $OutputDirectory $objectName
    & $compiler @commonCompileArgs -o $object $source
    if ($LASTEXITCODE -ne 0) {
        throw "Compile failed for $source with exit code $LASTEXITCODE"
    }
    $objects += $object
}

$outputFile = Join-Path $OutputDirectory "$outputStem.out"
$hexFile = Join-Path $OutputDirectory "$outputStem.hex"
$mapFile = Join-Path $OutputDirectory "$outputStem.map"
$linkArgs = @(
    "@$(Join-Path $OutputDirectory 'device.opt')",
    "-mcpu=cortex-m0plus",
    "-march=thumbv6m",
    "-mfloat-abi=soft",
    "-mthumb",
    "-Oz",
    "-g",
    "-Wl,-m$mapFile",
    "-Wl,-i$sourceRoot",
    "-Wl,-i$OutputDirectory",
    "-Wl,-i$(Join-Path $compilerRoot 'lib')",
    "-Wl,--rom_model",
    "-Wl,--warn_sections",
    "-o",
    $outputFile
) + $objects + @(
    "-Wl,-l$(Join-Path $OutputDirectory 'device_linker.cmd')",
    "-Wl,-l$(Join-Path $OutputDirectory 'device.cmd.genlibs')",
    "-Wl,-llibc.a"
)

& $compiler @linkArgs
if ($LASTEXITCODE -ne 0) {
    throw "Link failed with exit code $LASTEXITCODE"
}

& $hexTool `
    --byte `
    --memwidth=8 `
    --romwidth=8 `
    --intel `
    -o $hexFile `
    $outputFile
if ($LASTEXITCODE -ne 0) {
    throw "Hex conversion failed with exit code $LASTEXITCODE"
}

Write-Host ''
Write-Host "H2026 Q2 build succeeded ($outputStem):"
Get-Item -LiteralPath $outputFile, $hexFile, $mapFile |
    Select-Object Name, Length, FullName |
    Format-Table -AutoSize
Select-String -LiteralPath $mapFile -Pattern '^\s+FLASH\s+', '^\s+SRAM\s+' |
    ForEach-Object { $_.Line }
