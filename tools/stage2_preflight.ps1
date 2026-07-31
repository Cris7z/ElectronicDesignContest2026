[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

function Find-FirstExistingFile {
    param([string[]]$Candidates)

    foreach ($candidate in $Candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    return $null
}

$programmerCommand = Get-Command 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
$programmerPath = if ($programmerCommand) {
    $programmerCommand.Source
} else {
    Find-FirstExistingFile @(
        'D:\A-Soft\DevTools\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'D:\A-Soft\DevTools\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
        'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    )
}

$openOcdCommand = Get-Command 'openocd.exe' -ErrorAction SilentlyContinue
$openOcdPath = if ($openOcdCommand) {
    $openOcdCommand.Source
} else {
    Find-FirstExistingFile @(
        'D:\A-Soft\DevTools\OpenOCD\xpack-openocd-0.12.0-7\xpack-openocd-0.12.0-7\bin\openocd.exe'
    )
}

$devices = @()
if (Get-Command 'Get-PnpDevice' -ErrorAction SilentlyContinue) {
    $devices = @(Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue)
}

$pnpUtilText = ''
if (Get-Command 'pnputil.exe' -ErrorAction SilentlyContinue) {
    $pnpUtilText = (& pnputil.exe /enum-devices /connected 2>$null | Out-String)
}

$relevantDevices = @($devices | Where-Object {
    $_.InstanceId -match 'VID_0483&PID_374[0-9A-F]' -or
    $_.InstanceId -match 'VID_1A86&PID_7523' -or
    $_.InstanceId -match 'VID_1209&PID_ABD1' -or
    $_.FriendlyName -match 'ST-?LINK|STM32|CanMV|CH340'
})

$stLinkDevices = @($relevantDevices | Where-Object {
    $_.InstanceId -match 'VID_0483&PID_374[0-9A-F]' -or
    $_.FriendlyName -match 'ST-?LINK'
})

$stLinkPresent = ($stLinkDevices.Count -gt 0) -or ($pnpUtilText -match 'VID_0483&PID_374[0-9A-F]')
$ch340Present = ($relevantDevices.InstanceId -match 'VID_1A86&PID_7523').Count -gt 0 -or
    $pnpUtilText -match 'VID_1A86&PID_7523'
$canMvPresent = ($relevantDevices.InstanceId -match 'VID_1209&PID_ABD1').Count -gt 0 -or
    $pnpUtilText -match 'VID_1209&PID_ABD1'

Write-Output 'STAGE2_PREFLIGHT_VERSION=1'
Write-Output ("TIMESTAMP={0}" -f (Get-Date -Format 'yyyy-MM-ddTHH:mm:ssK'))
Write-Output ("STLINK_PRESENT={0}" -f [int]$stLinkPresent)
Write-Output ("CH340_PRESENT={0}" -f [int]$ch340Present)
Write-Output ("CANMV_PRESENT={0}" -f [int]$canMvPresent)
Write-Output ("PROGRAMMER_CLI_PRESENT={0}" -f [int](-not [string]::IsNullOrWhiteSpace($programmerPath)))
Write-Output ("PROGRAMMER_CLI={0}" -f $(if ($programmerPath) { $programmerPath } else { 'MISSING' }))
Write-Output ("OPENOCD_PRESENT={0}" -f [int](-not [string]::IsNullOrWhiteSpace($openOcdPath)))
Write-Output ("OPENOCD={0}" -f $(if ($openOcdPath) { $openOcdPath } else { 'MISSING' }))
Write-Output ("READY_FOR_ID_READ={0}" -f [int]($stLinkPresent -and ($programmerPath -or $openOcdPath)))

if ($relevantDevices.Count -eq 0) {
    if ($stLinkPresent) { Write-Output 'DEVICE=PNPUTIL|STLINK|VID_0483' }
    if ($ch340Present) { Write-Output 'DEVICE=PNPUTIL|CH340|VID_1A86&PID_7523' }
    if ($canMvPresent) { Write-Output 'DEVICE=PNPUTIL|CANMV|VID_1209&PID_ABD1' }
    if (-not ($stLinkPresent -or $ch340Present -or $canMvPresent)) {
        Write-Output 'DEVICE=NONE'
    }
} else {
    foreach ($device in $relevantDevices) {
        Write-Output ("DEVICE={0}|{1}|{2}" -f $device.Status, $device.FriendlyName, $device.InstanceId)
    }
}

Write-Output 'NOTE=Read-only detection; no reset, connection, erase, or flash command was issued.'
