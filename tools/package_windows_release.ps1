param(
    [string]$Version = "",
    [string]$RuntimeDir = "release\bin",
    [string]$OutputPath = ""
)

$ErrorActionPreference = "Stop"

$Workspace = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$RuntimePath = [System.IO.Path]::GetFullPath((Join-Path $Workspace $RuntimeDir))
if (-not $RuntimePath.StartsWith($Workspace, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "RuntimeDir must stay inside the workspace: $Workspace"
}
if (-not (Test-Path -LiteralPath $RuntimePath)) {
    throw "Runtime directory not found: $RuntimePath"
}

if ([string]::IsNullOrWhiteSpace($Version)) {
    $cmakeLists = Get-Content -LiteralPath (Join-Path $Workspace "CMakeLists.txt") -Raw
    $match = [regex]::Match($cmakeLists, "project\s*\([^)]*VERSION\s+([0-9]+\.[0-9]+\.[0-9]+)", "IgnoreCase, Singleline")
    if (-not $match.Success) {
        throw "Could not infer version from CMakeLists.txt. Pass -Version explicitly."
    }
    $Version = $match.Groups[1].Value
}

$ReleaseDir = Join-Path $Workspace "release"
New-Item -ItemType Directory -Path $ReleaseDir -Force | Out-Null

$Stage = Join-Path $ReleaseDir "FobosAPP-v$Version-windows-x64"
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $OutputPath = Join-Path $ReleaseDir "FobosAPP-v$Version-windows-x64.zip"
}
$OutputFullPath = [System.IO.Path]::GetFullPath($OutputPath)
if (-not $OutputFullPath.StartsWith($Workspace, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "OutputPath must stay inside the workspace: $Workspace"
}

if (Test-Path -LiteralPath $Stage) {
    Remove-Item -LiteralPath $Stage -Recurse -Force
}
New-Item -ItemType Directory -Path $Stage -Force | Out-Null

$excludedNames = @(
    "FobosAPP.ini",
    "FobosAPP_diagnostic.log",
    "FobosAPP-backup.exe",
    "dmr_lab_replay.exe",
    "dmr_lab_replay.md"
)
$excludedExtensions = @(
    ".bmp",
    ".csv",
    ".iq",
    ".log",
    ".nmea",
    ".raw",
    ".ubx",
    ".wav"
)
$excludedDirs = @(
    "recordings"
)

$runtimeRoot = $RuntimePath.TrimEnd([System.IO.Path]::DirectorySeparatorChar, [System.IO.Path]::AltDirectorySeparatorChar)
Get-ChildItem -LiteralPath $RuntimePath -Recurse -Force | ForEach-Object {
    $relative = $_.FullName.Substring($runtimeRoot.Length).TrimStart("\", "/")
    $parts = $relative -split "[\\/]"
    if ($parts | Where-Object { $excludedDirs -contains $_ }) {
        return
    }

    if ($_.PSIsContainer) {
        New-Item -ItemType Directory -Path (Join-Path $Stage $relative) -Force | Out-Null
        return
    }

    if ($excludedNames -contains $_.Name) {
        return
    }
    if ($excludedExtensions -contains $_.Extension.ToLowerInvariant()) {
        return
    }

    $dest = Join-Path $Stage $relative
    $destDir = Split-Path -Parent $dest
    if (-not (Test-Path -LiteralPath $destDir)) {
        New-Item -ItemType Directory -Path $destDir -Force | Out-Null
    }
    Copy-Item -LiteralPath $_.FullName -Destination $dest -Force
}

Compress-Archive -Path (Join-Path $Stage "*") -DestinationPath $OutputFullPath -Force
Get-Item -LiteralPath $OutputFullPath | Select-Object FullName, Length, LastWriteTime
