param(
    [string]$OutputPath = ""
)

$ErrorActionPreference = "Stop"
$Workspace = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$ReleaseDir = Join-Path $Workspace "release"
if (-not (Test-Path -LiteralPath $ReleaseDir)) {
    New-Item -ItemType Directory -Path $ReleaseDir | Out-Null
}

if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $OutputPath = Join-Path $ReleaseDir "FobosAPP-raspberry-source.zip"
}

$OutputFullPath = [System.IO.Path]::GetFullPath($OutputPath)
if (-not $OutputFullPath.StartsWith($Workspace, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "OutputPath must stay inside the workspace: $Workspace"
}

$Stage = Join-Path $ReleaseDir "FobosAPP-raspberry-source"
if (Test-Path -LiteralPath $Stage) {
    Remove-Item -LiteralPath $Stage -Recurse -Force
}
New-Item -ItemType Directory -Path $Stage | Out-Null

$files = git -C $Workspace ls-files --cached --others --exclude-standard
$excludePrefixes = @(
    "android/",
    "release/",
    "build/",
    "third_party/patched/libfobos/showimg/",
    "third_party/patched/libfobos-sdr-agile/showimg/",
    "sandbox/",
    "tools/fobos_proxy_logger/",
    "tools/rtlsdr_proxy_logger/"
)
$excludeExactPaths = @(
    "third_party/mbelib-neo",
    "third_party/softdmr",
    "tools/analyze_retune_raw_dump.py",
    "tools/dmr_lab_replay.cpp",
    "docs/dmr_lab_replay.md",
    "docs/dmr_external_backend.md",
    "config/dmr_backends.example.json",
    "tools/build_windows_local.ps1",
    "tools/deploy_windows.ps1",
    "tools/package_windows_release.ps1",
    "tools/sign_windows.ps1"
)

foreach ($file in $files) {
    $normalized = $file -replace "\\", "/"
    $normalizedExact = $normalized.TrimEnd("/")
    if ($excludeExactPaths -contains $normalizedExact) {
        continue
    }
    $excluded = $false
    foreach ($prefix in $excludePrefixes) {
        if ($normalized.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
            $excluded = $true
            break
        }
    }
    if ($excluded) {
        continue
    }

    $source = Join-Path $Workspace $file
    if (-not (Test-Path -LiteralPath $source)) {
        continue
    }
    $dest = Join-Path $Stage $file
    $destDir = Split-Path -Parent $dest
    if (-not (Test-Path -LiteralPath $destDir)) {
        New-Item -ItemType Directory -Path $destDir -Force | Out-Null
    }
    Copy-Item -LiteralPath $source -Destination $dest -Force
}

Compress-Archive -Path (Join-Path $Stage "*") -DestinationPath $OutputFullPath -Force
Get-Item -LiteralPath $OutputFullPath | Select-Object FullName, Length, LastWriteTime
