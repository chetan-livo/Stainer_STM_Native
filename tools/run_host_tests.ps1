<#
Builds and runs the host unit tests with clang (LLVM-MinGW). Equivalence
tests compile third-party Arduino libraries from the local Arduino sketchbook
as references; they are skipped when the library is not installed. Reference
libraries are only linked into these local test programs, never into firmware.

  powershell -ExecutionPolicy Bypass -File tools/run_host_tests.ps1
Set LIVO_CLANG to clang++.exe and ARDUINO_LIBRARIES to the libraries folder if
they are not found automatically.
#>
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$clang = $env:LIVO_CLANG
if (-not $clang) {
    $clang = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter clang++.exe -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -match 'LLVM-MinGW' } | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $clang) { throw 'clang++ (LLVM-MinGW) not found; set LIVO_CLANG.' }
$env:PATH = (Split-Path -Parent $clang) + ';' + $env:PATH
$libs = if ($env:ARDUINO_LIBRARIES) { $env:ARDUINO_LIBRARIES } else { Join-Path $env:USERPROFILE 'Documents\Arduino\libraries' }
$out = Join-Path $repo 'build-host'
New-Item -ItemType Directory -Force -Path $out | Out-Null
$platform = Join-Path $repo 'firmware/Platform'
$devices = Join-Path $repo 'firmware/Devices'
$tests = Join-Path $repo 'tests/host'
# -mno-ms-bitfields: lay out bitfields like ARM GCC (AAPCS), not MSVC.
$flags = @('-std=gnu++17', '-O1', '-g', '-Wall', '-Wextra', '-fsanitize=address,undefined', '-mno-ms-bitfields',
           "-I$tests", "-I$platform/Inc", "-I$devices")

$cases = @(
    @{ Name = 'test_wstring'; Sources = @("$tests/test_wstring.cpp", "$platform/Src/WString.cpp") }
)
$tmc = Join-Path $libs 'TMCStepper/src'
if (Test-Path -LiteralPath $tmc) {
    $tmcSources = Get-ChildItem -LiteralPath "$tmc/source" -Filter *.cpp | Where-Object { $_.Name -notmatch '^bcm2835' } |
        ForEach-Object { $_.FullName }
    $cases += @{ Name = 'test_tmc2209'; Extra = @("-I$tests/arduino_stubs", "-I$tmc", '-w');
                 Sources = @("$tests/test_tmc2209.cpp", "$devices/Tmc2209.cpp", "$platform/Src/Print.cpp", "$platform/Src/WString.cpp") + $tmcSources }
} else { Write-Output 'SKIP test_tmc2209: TMCStepper not installed' }
$accel = Join-Path $libs 'AccelStepper/src'
if (Test-Path -LiteralPath $accel) {
    $cases += @{ Name = 'test_planner'; Extra = @("-I$tests/arduino_stubs", "-I$accel", '-DARDUINO=10819');
                 Sources = @("$tests/test_planner.cpp", "$platform/Src/MotionPlanner.cpp", "$accel/AccelStepper.cpp") }
} else { Write-Output 'SKIP test_planner: AccelStepper not installed' }

$core = if ($env:STM32DUINO_CORE) { $env:STM32DUINO_CORE } else { Join-Path $env:LOCALAPPDATA 'Arduino15/packages/STMicroelectronics/hardware/stm32/3.0.0/cores/arduino' }
if (Test-Path -LiteralPath "$core/api/String.cpp") {
    $cases += @{ Name = 'test_wstring_vs_arduino'; Extra = @("-I$core", '-w');
                 Sources = @("$tests/test_wstring_vs_arduino.cpp", "$platform/Src/WString.cpp", "$tests/arduino_string_ref.cpp");
                 CSources = @("$core/itoa.c", "$core/avr/dtostrf.c") }
    $cases += @{ Name = 'test_print_vs_arduino'; Extra = @("-I$core", '-w');
                 Sources = @("$tests/test_print_vs_arduino.cpp", "$platform/Src/Print.cpp", "$platform/Src/WString.cpp",
                             "$tests/arduino_print_ref.cpp", "$tests/arduino_string_ref.cpp", "$tests/host_platform_stubs.cpp");
                 CSources = @("$core/itoa.c", "$core/avr/dtostrf.c") }
} else { Write-Output 'SKIP test_wstring_vs_arduino: STM32duino 3.0.0 core not installed' }

$failed = 0
foreach ($case in $cases) {
    $exe = Join-Path $out ($case.Name + '.exe')
    $objects = @()
    # Reference C sources build without UBSan: STM32duino itoa.c negates LONG_MIN.
    foreach ($c in @($case.CSources | Where-Object { $_ })) {
        $obj = Join-Path $out ($case.Name + '_' + [IO.Path]::GetFileNameWithoutExtension($c) + '.o')
        & (Join-Path (Split-Path -Parent $clang) 'clang.exe') -c -O1 -g -w -fsanitize=address -mno-ms-bitfields @($case.Extra | Where-Object { $_ }) $c -o $obj
        $objects += $obj
    }
    $args = $flags + @($case.Extra | Where-Object { $_ }) + $case.Sources + $objects + @('-o', $exe)
    & $clang @args
    if ($LASTEXITCODE) { Write-Output "BUILD FAILED $($case.Name)"; $failed++; continue }
    & $exe
    if ($LASTEXITCODE) { $failed++ }
}
if ($failed) { Write-Output "$failed host test program(s) failed"; exit 1 }
Write-Output 'All host tests passed'
