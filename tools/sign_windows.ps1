param(
    [Parameter(Mandatory = $true)]
    [string[]]$Path,
    [string]$PfxPath = $env:FOBOSAPP_CODESIGN_PFX,
    [string]$CertPassword = $env:FOBOSAPP_CODESIGN_PASSWORD,
    [string]$CertSubject = $env:FOBOSAPP_CODESIGN_SUBJECT,
    [string]$TimestampUrl = $(if ($env:FOBOSAPP_TIMESTAMP_URL) { $env:FOBOSAPP_TIMESTAMP_URL } else { "http://timestamp.digicert.com" })
)

$ErrorActionPreference = "Stop"

function Find-SignTool {
    $Roots = @(
        "${env:ProgramFiles(x86)}\Windows Kits\10\bin",
        "${env:ProgramFiles}\Windows Kits\10\bin"
    ) | Where-Object { $_ -and (Test-Path -LiteralPath $_) }

    foreach ($Root in $Roots) {
        $Candidate = Get-ChildItem -LiteralPath $Root -Recurse -Filter signtool.exe -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -match "\\x64\\signtool\.exe$" } |
            Sort-Object FullName -Descending |
            Select-Object -First 1
        if ($Candidate) {
            return $Candidate.FullName
        }
    }

    $FromPath = Get-Command signtool.exe -ErrorAction SilentlyContinue
    if ($FromPath) {
        return $FromPath.Source
    }

    throw "signtool.exe was not found. Install the Windows SDK or add signtool.exe to PATH."
}

$SignTool = Find-SignTool
$BaseArgs = @("sign", "/fd", "SHA256", "/tr", $TimestampUrl, "/td", "SHA256")

if ($PfxPath) {
    if (-not (Test-Path -LiteralPath $PfxPath)) {
        throw "PFX file not found: $PfxPath"
    }
    $BaseArgs += @("/f", (Resolve-Path -LiteralPath $PfxPath).Path)
    if ($CertPassword) {
        $BaseArgs += @("/p", $CertPassword)
    }
} elseif ($CertSubject) {
    $BaseArgs += @("/n", $CertSubject)
} else {
    throw "Provide a PFX file through -PfxPath or FOBOSAPP_CODESIGN_PFX, or a certificate subject through -CertSubject or FOBOSAPP_CODESIGN_SUBJECT."
}

foreach ($Item in $Path) {
    $Resolved = (Resolve-Path -LiteralPath $Item).Path
    & $SignTool @BaseArgs $Resolved
    if ($LASTEXITCODE -ne 0) {
        throw "signtool failed for $Resolved with exit code $LASTEXITCODE"
    }
}
