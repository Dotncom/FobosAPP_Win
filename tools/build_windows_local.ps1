param(
    [string]$BuildDir = "build\steamdeck-msvc-release",
    [string]$Config = "Release",
    [string]$QtRoot = "C:\Qt\5.15.2\msvc2019_64",
    [switch]$Deploy,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

$Workspace = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$CMake = "C:\Program Files\CMake\bin\cmake.exe"
$VsWhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"

if (-not (Test-Path -LiteralPath $CMake)) {
    throw "CMake not found: $CMake"
}
if (-not (Test-Path -LiteralPath $VsWhere)) {
    throw "vswhere not found: $VsWhere"
}
if (-not (Test-Path -LiteralPath $QtRoot)) {
    throw "Qt root not found: $QtRoot"
}

$VsInstall = & $VsWhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $VsInstall) {
    throw "Visual Studio C++ Build Tools were not found."
}

$VsDevCmd = Join-Path $VsInstall "Common7\Tools\VsDevCmd.bat"
if (-not (Test-Path -LiteralPath $VsDevCmd)) {
    throw "VsDevCmd not found: $VsDevCmd"
}

$RedistRoot = Join-Path $VsInstall "VC\Redist\MSVC"
$VcRedistDir = Get-ChildItem -LiteralPath $RedistRoot -Directory -ErrorAction SilentlyContinue |
    Sort-Object Name -Descending |
    ForEach-Object { Join-Path $_.FullName "x64\Microsoft.VC143.CRT" } |
    Where-Object { Test-Path -LiteralPath (Join-Path $_ "VCRUNTIME140.dll") } |
    Select-Object -First 1
if (-not $VcRedistDir) {
    throw "VC runtime redistributable folder was not found under $RedistRoot"
}

$BuildPath = Join-Path $Workspace $BuildDir
if ($Clean -and (Test-Path -LiteralPath $BuildPath)) {
    Remove-Item -LiteralPath $BuildPath -Recurse -Force
}

$ConfigureArgs = @(
    "-S", $Workspace,
    "-B", $BuildPath,
    "-G", "Visual Studio 17 2022",
    "-A", "x64",
    "-DCMAKE_BUILD_TYPE=$Config",
    "-DQt5_DIR=$QtRoot\lib\cmake\Qt5",
    "-DFOBOS_STANDARD_ROOT=$Workspace\fobos",
    "-DFOBOS_AGILE_ROOT=$Workspace\fobos_agile",
    "-DFFTW_ROOT=$Workspace\fftw-3.3.5-dll64"
)

$BuildArgs = @(
    "--build", $BuildPath,
    "--config", $Config,
    "--target", "FobosAPP"
)

function ConvertTo-CmdArgList {
    param([string[]]$Values)

    $Values | ForEach-Object {
        if ($_ -match '[\s"]') {
            '"' + ($_ -replace '"', '\"') + '"'
        } else {
            $_
        }
    }
}

$CmdPath = Join-Path $env:TEMP "fobos_build_windows_local.cmd"
$ConfigureCommand = ConvertTo-CmdArgList $ConfigureArgs
$BuildCommand = ConvertTo-CmdArgList $BuildArgs
$CmdLines = @(
    "@echo off",
    "call `"$VsDevCmd`" -arch=x64 -host_arch=x64",
    "if errorlevel 1 exit /b %errorlevel%",
    "`"$CMake`" $($ConfigureCommand -join ' ')",
    "if errorlevel 1 exit /b %errorlevel%",
    "`"$CMake`" $($BuildCommand -join ' ')",
    "exit /b %errorlevel%"
)
Set-Content -LiteralPath $CmdPath -Value $CmdLines -Encoding ASCII

Push-Location $Workspace
try {
    & cmd.exe /d /c $CmdPath
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed with exit code $LASTEXITCODE"
    }

    if ($Deploy) {
        & (Join-Path $PSScriptRoot "deploy_windows.ps1") `
            -BuildDir (Join-Path $BuildDir $Config) `
            -DeployDir "release\bin" `
            -QtRoot $QtRoot `
            -VcRedistDir $VcRedistDir
    }
} finally {
    Pop-Location
}
