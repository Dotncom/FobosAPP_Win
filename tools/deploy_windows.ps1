param(
    [string]$BuildDir = "build\Desktop_x86_windows_msvc2022_pe_64bit-Release",
    [string]$DeployDir = "release\bin",
    [string]$QtRoot = "C:\Qt\5.15.2\msvc2019_64",
    [string]$VcRedistDir = "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Redist\MSVC\14.40.33807\x64\Microsoft.VC143.CRT",
    [switch]$IncludeLabTools,
    [switch]$Sign,
    [string]$PfxPath = $env:FOBOSAPP_CODESIGN_PFX,
    [string]$CertPassword = $env:FOBOSAPP_CODESIGN_PASSWORD,
    [string]$CertSubject = $env:FOBOSAPP_CODESIGN_SUBJECT
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
    @{ Source = if (Test-Path -LiteralPath (Join-Path $Workspace "fobos_agile\libusb-1.0.dll")) { Join-Path $Workspace "fobos_agile\libusb-1.0.dll" } else { Join-Path $Workspace "libusb-1.0.27\VS2022\MS64\dll\libusb-1.0.dll" }; Name = "libusb-1.0.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Core.dll"; Name = "Qt5Core.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Gui.dll"; Name = "Qt5Gui.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Widgets.dll"; Name = "Qt5Widgets.dll" },
    @{ Source = Join-Path $QtRoot "bin\Qt5Network.dll"; Name = "Qt5Network.dll" },
    @{ Source = Join-Path $VcRedistDir "MSVCP140.dll"; Name = "MSVCP140.dll" },
    @{ Source = Join-Path $VcRedistDir "MSVCP140_1.dll"; Name = "MSVCP140_1.dll" },
    @{ Source = Join-Path $VcRedistDir "VCRUNTIME140.dll"; Name = "VCRUNTIME140.dll" },
    @{ Source = Join-Path $VcRedistDir "VCRUNTIME140_1.dll"; Name = "VCRUNTIME140_1.dll" }
)

if ($IncludeLabTools) {
    $RootFiles += @{ Source = Join-Path $BuildPath "dmr_lab_replay.exe"; Name = "dmr_lab_replay.exe" }
} else {
    $StaleLabTool = Join-Path $DeployPath "dmr_lab_replay.exe"
    if (Test-Path -LiteralPath $StaleLabTool) {
        Remove-Item -LiteralPath $StaleLabTool -Force
    }
}

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
    "THIRD_PARTY_LICENSES.txt",
    "translations.json"
)

foreach ($DocFile in $DocFiles) {
    $DocPath = Join-Path $Workspace $DocFile
    if (Test-Path -LiteralPath $DocPath) {
        Copy-Item -LiteralPath $DocPath -Destination (Join-Path $DeployPath $DocFile) -Force
    }
}

$LicensePath = Join-Path $Workspace "licenses"
if (Test-Path -LiteralPath $LicensePath) {
    $DeployLicensePath = Join-Path $DeployPath "licenses"
    if (Test-Path -LiteralPath $DeployLicensePath) {
        Remove-Item -LiteralPath $DeployLicensePath -Recurse -Force
    }
    New-Item -ItemType Directory -Path $DeployLicensePath -Force | Out-Null
    Copy-Item -Path (Join-Path $LicensePath "*") -Destination $DeployLicensePath -Recurse -Force
}

$ConfigDeployPath = Join-Path $DeployPath "config"
New-Item -ItemType Directory -Path $ConfigDeployPath -Force | Out-Null
$ReleaseConfigFiles = @(
    "config\fobosapp_defaults_4.0.json",
    "config\dmr_backends.example.json"
)
foreach ($ConfigFile in $ReleaseConfigFiles) {
    $ConfigSource = Join-Path $Workspace $ConfigFile
    if (Test-Path -LiteralPath $ConfigSource) {
        Copy-Item -LiteralPath $ConfigSource -Destination (Join-Path $ConfigDeployPath (Split-Path -Leaf $ConfigSource)) -Force
    }
}

$DmrVoiceBackendBuildPath = Join-Path $Workspace "build\fobos-dmr-voice-backend-gpl-vs\Release"
if (Test-Path -LiteralPath $DmrVoiceBackendBuildPath) {
    $DmrVoiceBackendDeployPath = Join-Path $DeployPath "dmr_voice_backends"
    New-Item -ItemType Directory -Path $DmrVoiceBackendDeployPath -Force | Out-Null
    Get-ChildItem -LiteralPath $DmrVoiceBackendBuildPath -Filter "fobos_dmr_voice_*.dll" -File |
        ForEach-Object {
            Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $DmrVoiceBackendDeployPath $_.Name) -Force
        }
}

$DmrVoiceBackendLicensePath = Join-Path $DeployPath "licenses\dmr_voice_backend"
New-Item -ItemType Directory -Path $DmrVoiceBackendLicensePath -Force | Out-Null
$DmrVoiceBackendNotices = @(
    @{ Source = Join-Path $Workspace "FobosDMRVoiceBackend-gpl\LICENSE"; Destination = Join-Path $DmrVoiceBackendLicensePath "FobosDMRVoiceBackend-LICENSE.txt" },
    @{ Source = Join-Path $Workspace "FobosDMRVoiceBackend-gpl\README.md"; Destination = Join-Path $DmrVoiceBackendLicensePath "FobosDMRVoiceBackend-README.md" },
    @{ Source = Join-Path $Workspace "third_party\mbelib-neo\LICENSE"; Destination = Join-Path $DmrVoiceBackendLicensePath "mbelib-neo-LICENSE.txt" },
    @{ Source = Join-Path $Workspace "third_party\mbelib-neo\README.md"; Destination = Join-Path $DmrVoiceBackendLicensePath "mbelib-neo-README.md" },
    @{ Source = Join-Path $Workspace "third_party\softdmr\LICENSE"; Destination = Join-Path $DmrVoiceBackendLicensePath "softdmr-LICENSE.txt" },
    @{ Source = Join-Path $Workspace "third_party\softdmr\README.md"; Destination = Join-Path $DmrVoiceBackendLicensePath "softdmr-README.md" }
)
foreach ($Notice in $DmrVoiceBackendNotices) {
    if (Test-Path -LiteralPath $Notice.Source) {
        Copy-Item -LiteralPath $Notice.Source -Destination $Notice.Destination -Force
    }
}

if ($IncludeLabTools) {
    $DmrLabDocs = @(
        @{ Source = Join-Path $Workspace "docs\dmr_lab_replay.md"; Destination = Join-Path $DeployPath "docs\dmr_lab_replay.md" },
        @{ Source = Join-Path $Workspace "docs\dmr_external_backend.md"; Destination = Join-Path $DeployPath "docs\dmr_external_backend.md" },
        @{ Source = Join-Path $Workspace "config\dmr_backends.example.json"; Destination = Join-Path $DeployPath "config\dmr_backends.example.json" }
    )

    foreach ($Doc in $DmrLabDocs) {
        if (Test-Path -LiteralPath $Doc.Source) {
            New-Item -ItemType Directory -Path (Split-Path -Parent $Doc.Destination) -Force | Out-Null
            Copy-Item -LiteralPath $Doc.Source -Destination $Doc.Destination -Force
        }
    }
} else {
    $StaleLabDocs = @(
        Join-Path $DeployPath "docs\dmr_lab_replay.md"
    )
    foreach ($StaleDoc in $StaleLabDocs) {
        if (Test-Path -LiteralPath $StaleDoc) {
            Remove-Item -LiteralPath $StaleDoc -Force
        }
    }
}

if ($Sign) {
    $SignScript = Join-Path $PSScriptRoot "sign_windows.ps1"
    & $SignScript -Path (Join-Path $DeployPath "FobosAPP.exe") -PfxPath $PfxPath -CertPassword $CertPassword -CertSubject $CertSubject
}

Write-Host "Deployed FobosAPP to $DeployPath"
