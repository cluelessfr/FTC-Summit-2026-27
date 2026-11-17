param()
$ErrorActionPreference = 'Stop'
$turretRepo = Split-Path -Parent $PSScriptRoot
$turretOutput = Join-Path $turretRepo 'TeamCode/build/turret-checks'
New-Item -ItemType Directory -Path $turretOutput -Force | Out-Null
$turretGradleRoot = if ($env:GRADLE_USER_HOME) { $env:GRADLE_USER_HOME } else { Join-Path $env:USERPROFILE '.gradle' }
$turretModules = Join-Path $turretGradleRoot 'caches/modules-2/files-2.1'
$turretDependencies = Get-Content (Join-Path $turretRepo 'build.dependencies.gradle') -Raw
Add-Type -AssemblyName System.IO.Compression.FileSystem
function Get-TurretLibraryJar($group, $artifact, $version) {
    $libraryDir = Join-Path $turretModules "$group/$artifact/$version"
    $archive = Get-ChildItem -LiteralPath $libraryDir -Recurse -Filter '*.aar' | Select-Object -First 1
    if (!$archive) {
        return (Get-ChildItem -LiteralPath $libraryDir -Recurse -Filter "$artifact-$version.jar" | Select-Object -First 1).FullName
    }
    $destination = Join-Path $turretOutput "$artifact-$version.jar"
    $zip = [IO.Compression.ZipFile]::OpenRead($archive.FullName)
    try { [IO.Compression.ZipFileExtensions]::ExtractToFile($zip.GetEntry('classes.jar'), $destination, $true) }
    finally { $zip.Dispose() }
    return $destination
}
$turretClasspath = @()
foreach ($library in @(
    @('org.firstinspires.ftc', 'RobotCore'),
    @('org.firstinspires.ftc', 'Hardware')
)) {
    $pattern = [regex]::Escape("$($library[0]):$($library[1]):") + "([^']+)"
    $version = [regex]::Match($turretDependencies, $pattern).Groups[1].Value
    if (!$version) { throw "No declared version for $($library[1])" }
    $turretClasspath += Get-TurretLibraryJar $library[0] $library[1] $version
}
$turretCp = $turretClasspath -join [IO.Path]::PathSeparator
$turretSource = Join-Path $turretRepo 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/turret'
$sources = @('PollenTurret.java', 'HiveOpeningAim.java', 'TurretServo.java') |
    ForEach-Object { Join-Path $turretSource $_ }
& javac -cp $turretCp -d $turretOutput @sources (Join-Path $turretRepo 'tests/turret/BlueHiveTurretChecks.java')
if ($LASTEXITCODE -ne 0) { throw 'Turret checks compilation failed' }
& java -cp ($turretCp + [IO.Path]::PathSeparator + $turretOutput) org.firstinspires.ftc.teamcode.turret.BlueHiveTurretChecks
if ($LASTEXITCODE -ne 0) { throw 'Turret checks failed' }
