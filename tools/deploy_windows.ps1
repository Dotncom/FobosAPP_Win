param(
    [string]$BuildDir = "build\Desktop_x86_windows_msvc2022_pe_64bit-Release",
    [string]$DeployDir = "release\bin",
    [string]$QtRoot = "C:\Qt\5.15.2\msvc2019_64",
    [string]$VcRedistDir = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Redist\MSVC\14.40.33807\x64\Microsoft.VC143.CRT"
)

$ErrorActionPreference = "Stop"

$Workspace = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$BuildPath = Join-Path $Workspace $BuildDir
$DeployPath = Join-Path $Workspace $DeployDir

if (-not (Test-Path -LiteralPath $BuildPath)) {
    throw "Build directory not found: $BuildPath"
}

New-Item -ItemType Directory -Path $DeployPath -Force | Out-Null
New-Item -ItemType Directory -Path (Join-Path $DeployPath "platforms") -Force | Out-Null

$RootFiles = @(
    @{ Source = Join-Path $BuildPath "FobosAPP.exe"; Name = "FobosAPP.exe" },
    @{ Source = Join-Path $Workspace "fobos\fobos.dll"; Name = "fobos.dll" },
    @{ Source = Join-Path $Workspace "fobos_agile\fobos_sdr.dll"; Name = "fobos_sdr.dll" },
    @{ Source = Join-Path $Workspace "fftw-3.3.5-dll64\libfftw3f-3.dll"; Name = "libfftw3f-3.dll" },
    @{ Source = Join-Path $Workspace "libusb-1.0.27\VS2022\MS64\dll\libusb-1.0.dll"; Name = "libusb-1.0.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Core.dll"; Name = "Qt5Core.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Gui.dll"; Name = "Qt5Gui.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Widgets.dll"; Name = "Qt5Widgets.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Network.dll"; Name = "Qt5Network.dll" },
    @{ Source = Join-Path $VcRedistDir "MSVCP140.dll"; Name = "MSVCP140.dll" },
    @{ Source = Join-Path $VcRedistDir "MSVCP140_1.dll"; Name = "MSVCP140_1.dll" },
    @{ Source = Join-Path $VcRedistDir "VCRUNTIME140.dll"; Name = "VCRUNTIME140.dll" },
    @{ Source = Join-Path $VcRedistDir "VCRUNTIME140_1.dll"; Name = "VCRUNTIME140_1.dll" }
)

foreach ($File in $RootFiles) {
    if (-not (Test-Path -LiteralPath $File.Source)) {
        throw "Required deploy file not found: $($File.Source)"
    }
    Copy-Item -LiteralPath $File.Source -Destination (Join-Path $DeployPath $File.Name) -Force
}

$QWindows = Join-Path $QtRoot "plugins\platforms\qwindows.dll"
if (-not (Test-Path -LiteralPath $QWindows)) {
    throw "Qt platform plugin not found: $QWindows"
}
Copy-Item -LiteralPath $QWindows -Destination (Join-Path $DeployPath "platforms\qwindows.dll") -Force

$DocFiles = @(
    "README.md",
    "CHANGELOG.md",
    "THIRD_PARTY_LICENSES.txt"
)

foreach ($DocFile in $DocFiles) {
    $DocPath = Join-Path $Workspace $DocFile
    if (Test-Path -LiteralPath $DocPath) {
        Copy-Item -LiteralPath $DocPath -Destination (Join-Path $DeployPath $DocFile) -Force
    }
}

$LicensePath = Join-Path $Workspace "licenses"
if (Test-Path -LiteralPath $LicensePath) {
    Copy-Item -LiteralPath $LicensePath -Destination (Join-Path $DeployPath "licenses") -Recurse -Force
}

Write-Host "Deployed FobosAPP to $DeployPath"
