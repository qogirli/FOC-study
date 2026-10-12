param([string]$SourceDir, [string]$OutputDir)
$ErrorActionPreference = 'Stop'
$hadPowerPoint = @(Get-Process POWERPNT -ErrorAction SilentlyContinue).Count -gt 0
$app = New-Object -ComObject PowerPoint.Application
try {
    $names = @('专题8 演讲专用ppt.pptx','专题7 演讲专用ppt .pptx','专题2 PID算法实现电机闭环控制.pptx','第17章 STM32外设之高级控制定时器.pptx','第18章 STM32外设之ADC.pptx')
    foreach ($name in $names) {
        $source = Join-Path $SourceDir $name
        $dest = Join-Path $OutputDir ([IO.Path]::GetFileNameWithoutExtension($name) + '.pdf')
        if (Test-Path -LiteralPath $dest) { throw "Output already exists: $dest" }
        $deck = $app.Presentations.Open($source, -1, 0, 0)
        try {
            $deck.SaveAs($dest, 32)
            Write-Output "$name : $($deck.Slides.Count) slides"
        } finally { $deck.Close() }
    }
} finally {
    if (-not $hadPowerPoint) { $app.Quit() }
    [void][Runtime.InteropServices.Marshal]::ReleaseComObject($app)
}
