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
foreach ($file in $files) {
    $source = Join-Path $Workspace $file
    $dest = Join-Path $Stage $file
    $destDir = Split-Path -Parent $dest
    if (-not (Test-Path -LiteralPath $destDir)) {
        New-Item -ItemType Directory -Path $destDir -Force | Out-Null
    }
    Copy-Item -LiteralPath $source -Destination $dest -Force
}

Compress-Archive -Path (Join-Path $Stage "*") -DestinationPath $OutputFullPath -Force
Get-Item -LiteralPath $OutputFullPath | Select-Object FullName, Length, LastWriteTime
