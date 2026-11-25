[CmdletBinding()]
param(
    [string]$QjcamRoot,
    [ValidateSet('Release', 'RelWithDebInfo')]
    [string]$Configuration = 'RelWithDebInfo',
    [string[]]$Plugins = @(),
    [string]$Generator,
    [string]$Toolset
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Invoke-CMake {
    param([string[]]$CMakeArgs)
    & cmake @CMakeArgs
    if ($LASTEXITCODE -ne 0) { throw "cmake failed (exit $LASTEXITCODE)." }
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$installPathFile = Join-Path $repoRoot 'install-path.txt'
if (-not $QjcamRoot) {
    if (Test-Path -LiteralPath $installPathFile) {
        $QjcamRoot = (Get-Content -LiteralPath $installPathFile -Raw -Encoding UTF8).Trim()
        if (-not $QjcamRoot -or $QjcamRoot -match '[\r\n]') {
            throw 'install-path.txt must contain one non-empty installation path.'
        }
    } else {
        $QjcamRoot = $env:QJCAM_ROOT
    }
}
if (-not $QjcamRoot) { throw 'Set install-path.txt, provide -QjcamRoot, or set QJCAM_ROOT.' }
if (-not [System.IO.Path]::IsPathRooted($QjcamRoot)) {
    throw 'The QJCAM installation path must be absolute.'
}
$QjcamRoot = (Resolve-Path -LiteralPath $QjcamRoot).Path
if (-not (Test-Path -LiteralPath (Join-Path $QjcamRoot 'api/qjcam_api.lib'))) {
    throw "Invalid QJCAM installation: $QjcamRoot"
}
foreach ($plugin in $Plugins) {
    if ($plugin -notmatch '^[a-zA-Z0-9][a-zA-Z0-9_-]*$') { throw "Invalid plugin name: $plugin" }
}

if (Get-Process -Name qjcam -ErrorAction SilentlyContinue) {
    throw 'Close QJCAM before building: DLLs are written directly to its plugins directory.'
}

$buildDir = Join-Path $repoRoot 'build'
$configureArgs = @('-S', $repoRoot, '-B', $buildDir, '-A', 'x64', "-DQJCAM_ROOT=$QjcamRoot")
if ($Generator) { $configureArgs += @('-G', $Generator) }
if ($Toolset) { $configureArgs += @('-T', $Toolset) }
Invoke-CMake $configureArgs
$buildArgs = @('--build', $buildDir, '--config', $Configuration, '--parallel')
if ($Plugins.Count -gt 0) { $buildArgs += @('--target') + $Plugins }
Invoke-CMake $buildArgs

Write-Host "Plugins built directly to: $(Join-Path $QjcamRoot 'plugins')"
