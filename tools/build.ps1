<#
Headless build of the STM32CubeIDE project, the same build the IDE runs.

  powershell -ExecutionPolicy Bypass -File tools/build.ps1                 # all configurations
  powershell -ExecutionPolicy Bypass -File tools/build.ps1 Master-Debug    # selected ones
  powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Clean Nozzle-Release

Outputs: firmware/<Configuration>/Stainer_STM_Native-<Board>.elf/.bin/.hex/.map
Logs:    build-logs/<Configuration>.log (ignored by git)
Set STM32CUBEIDE to the IDE folder if it is not C:\ST\STM32CubeIDE_*\STM32CubeIDE.
#>
param([switch]$Clean, [Parameter(ValueFromRemainingArguments = $true)][string[]]$Configurations = @())
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$project = Join-Path $repo 'firmware'

$ide = $env:STM32CUBEIDE
if (-not $ide) {
    $ide = Get-ChildItem 'C:\ST' -Directory -Filter 'STM32CubeIDE_*' -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending | Select-Object -First 1 |
        ForEach-Object { Join-Path $_.FullName 'STM32CubeIDE' }
}
$exe = Join-Path $ide 'stm32cubeidec.exe'
if (-not (Test-Path -LiteralPath $exe)) { throw "stm32cubeidec.exe not found; set STM32CUBEIDE." }

if (-not $Configurations.Count) {
    $Configurations = Select-String -LiteralPath (Join-Path $project '.cproject') -Pattern 'moduleId="org.eclipse.cdt.core.settings" name="([^"]+)"' |
        ForEach-Object { $_.Matches[0].Groups[1].Value }
}

# Fresh workspace outside the repository each run: the project is imported
# in place, so sources and build outputs stay under firmware/.
$workspace = Join-Path ([IO.Path]::GetTempPath()) ('stainer-native-ws-' + [guid]::NewGuid().ToString('N'))
$logs = Join-Path $repo 'build-logs'
New-Item -ItemType Directory -Force -Path $logs | Out-Null
$results = @()
try {
    $first = $true
    foreach ($config in $Configurations) {
        $action = if ($Clean) { '-cleanBuild' } else { '-build' }
        $arguments = @('--launcher.suppressErrors', '-nosplash',
                       '-application', 'org.eclipse.cdt.managedbuilder.core.headlessbuild',
                       '-data', $workspace)
        if ($first) { $arguments += @('-import', $project); $first = $false }
        $arguments += @($action, "Stainer_STM_Native/$config")
        $log = Join-Path $logs "$config.log"
        Write-Output "Building $config"
        # Windows PowerShell turns native stderr into errors under 'Stop'.
        $ErrorActionPreference = 'Continue'
        & $exe @arguments 2>&1 | Out-File -Encoding utf8 -FilePath $log
        $code = $LASTEXITCODE
        $ErrorActionPreference = 'Stop'
        $errors = @(Select-String -LiteralPath $log -Pattern ': error:|Error \d+|undefined reference' | Select-Object -First 5)
        $size = Select-String -LiteralPath $log -Pattern '^\s+\d+\s+\d+\s+\d+\s+\d+\s+[0-9a-f]+\s' | Select-Object -Last 1
        $results += [PSCustomObject]@{
            Configuration = $config
            Result = if ($code -eq 0 -and -not $errors) { 'OK' } else { 'FAILED' }
            'text+data+bss' = if ($size) { ($size.Line -split '\s+' | Where-Object { $_ })[0..2] -join ' / ' } else { '' }
        }
        $errors | ForEach-Object { Write-Output ('  ' + $_.Line.Trim()) }
    }
} finally {
    Remove-Item -Recurse -Force -LiteralPath $workspace -ErrorAction SilentlyContinue
}
$results | Format-Table -AutoSize
if ($results | Where-Object { $_.Result -ne 'OK' }) { exit 1 }
