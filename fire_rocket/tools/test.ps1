param([string]$Cxx = $env:CXX)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
if (-not $Cxx) {
    $foundCompiler = Get-Command g++ -ErrorAction SilentlyContinue
    if ($foundCompiler) { $Cxx = $foundCompiler.Source }
    elseif (Test-Path 'C:\Program Files (x86)\Dev-Cpp\MinGW64\bin\g++.exe') {
        $Cxx = 'C:\Program Files (x86)\Dev-Cpp\MinGW64\bin\g++.exe'
    } else { throw 'Specify -Cxx with the full path to a native C++ compiler.' }
}
$testDir = Join-Path $projectRoot 'test-results'
New-Item -ItemType Directory -Force -Path $testDir | Out-Null
$coreDir = Join-Path $projectRoot 'lib\TelemetryCore'
foreach ($entry in @('core_tests','replay_main')) {
    $source = Join-Path $projectRoot "test\host\$entry.cpp"
    $binary = Join-Path $testDir "$entry.exe"
    & $Cxx '-std=c++11' '-O2' '-Wall' '-Wextra' '-Werror' '-static' "-I$coreDir" (Join-Path $coreDir 'TelemetryCore.cpp') $source '-o' $binary
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $entry" }
}
& (Join-Path $testDir 'core_tests.exe')
if ($LASTEXITCODE -ne 0) { throw 'Core tests failed.' }
$integration = Join-Path $testDir 'integration_tests.exe'
& $Cxx '-std=c++11' '-O2' '-Wall' '-Wextra' '-Werror' '-static' "-I$coreDir" "-I$(Join-Path $projectRoot 'test\host\stubs')" "-I$(Join-Path $projectRoot 'include')" (Join-Path $coreDir 'TelemetryCore.cpp') (Join-Path $projectRoot 'src\Recorder.cpp') (Join-Path $projectRoot 'test\host\integration_tests.cpp') '-o' $integration
if ($LASTEXITCODE -ne 0) { throw 'Integration test compile failed.' }
& $integration
if ($LASTEXITCODE -ne 0) { throw 'Integration tests failed.' }
Write-Output "Replay executable: $(Join-Path $testDir 'replay_main.exe')"
