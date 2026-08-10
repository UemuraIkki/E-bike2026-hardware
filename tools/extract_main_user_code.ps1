param(
    [Parameter(Mandatory = $true)]
    [string]$RepositoryRoot,

    [Parameter(Mandatory = $true)]
    [string]$OutputPath
)

$resolvedRoot = (Resolve-Path -LiteralPath $RepositoryRoot).Path
$projectRoot = Join-Path $resolvedRoot 'projects'
$outputFullPath = [System.IO.Path]::GetFullPath((Join-Path $resolvedRoot $OutputPath))

if (-not $outputFullPath.StartsWith($resolvedRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'OutputPath must stay inside RepositoryRoot.'
}

$projects = Get-ChildItem -LiteralPath $projectRoot -Directory |
    Where-Object { $_.Name -match '^BLDC\d{4}$' } |
    Sort-Object Name

$pattern = [regex]::new(
    '/\* USER CODE BEGIN (?<name>.*?) \*/(?<body>.*?)/\* USER CODE END \k<name> \*/',
    [System.Text.RegularExpressions.RegexOptions]::Singleline
)

$builder = [System.Text.StringBuilder]::new()
[void]$builder.AppendLine('# main.c USER CODE archive')
[void]$builder.AppendLine()
[void]$builder.AppendLine('This file preserves non-empty `USER CODE` blocks from each experimental `main.c` before generated MCSDK files are removed.')
[void]$builder.AppendLine('Values and implementations are historical test snapshots and must not be treated as the current specification.')
[void]$builder.AppendLine()

foreach ($project in $projects) {
    $mainPath = Join-Path $project.FullName 'Src\main.c'
    if (-not (Test-Path -LiteralPath $mainPath)) {
        continue
    }

    $source = [System.IO.File]::ReadAllText($mainPath)
    $blocks = @()

    foreach ($match in $pattern.Matches($source)) {
        $sectionName = $match.Groups['name'].Value
        $body = $match.Groups['body'].Value.Trim()

        if ($sectionName -in @('Header', 'WHILE', 'Error_Handler_Debug', '6') -or $body.Length -eq 0) {
            continue
        }

        if ($body -match '^/\* Infinite loop \*/$' -or $body -eq ':') {
            continue
        }

        $blocks += [pscustomobject]@{
            Name = $sectionName
            Body = $body
        }
    }

    if ($blocks.Count -eq 0) {
        continue
    }

    [void]$builder.AppendLine("## $($project.Name)")
    [void]$builder.AppendLine()
    [void]$builder.AppendLine(('Source before cleanup: projects/{0}/Src/main.c' -f $project.Name))
    [void]$builder.AppendLine()

    foreach ($block in $blocks) {
        [void]$builder.AppendLine("### USER CODE $($block.Name)")
        [void]$builder.AppendLine()
        [void]$builder.AppendLine('```c')
        [void]$builder.AppendLine($block.Body)
        [void]$builder.AppendLine('```')
        [void]$builder.AppendLine()
    }
}

$outputDirectory = Split-Path -Parent $outputFullPath
[System.IO.Directory]::CreateDirectory($outputDirectory) | Out-Null
[System.IO.File]::WriteAllText(
    $outputFullPath,
    $builder.ToString(),
    [System.Text.UTF8Encoding]::new($false)
)

Write-Output $outputFullPath
