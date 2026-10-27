param(
    [string]$PortName = 'COM7',
    [int]$BaudRate = 115200,
    [int]$DurationSeconds = 0,
    [switch]$PlainText
)

# Read-only NMEA monitor. No configuration commands are sent to the GPS.
$ErrorActionPreference = 'Stop'
$gpsPort = $null
$satellites = @{}
$state = @{ Quality = ''; Used = ''; Lat = $null; Lon = $null; LastFix = [datetime]::MinValue; LastData = [datetime]::MinValue; Valid = $false; Lines = 0 }

function Convert-Coordinate([string]$Value, [string]$Hemisphere) {
    if ([string]::IsNullOrWhiteSpace($Value) -or $Hemisphere -notin @('N','S','E','W')) { return $null }
    $raw = [double]::Parse($Value, [Globalization.CultureInfo]::InvariantCulture)
    $degrees = [math]::Floor($raw / 100)
    $result = $degrees + ($raw - $degrees * 100) / 60
    if ($Hemisphere -in @('S','W')) { $result = -$result }
    return $result
}

function Read-Nmea([string]$Line) {
    if ($Line -notmatch '^\$(?<body>[^*]+)\*(?<checksum>[0-9A-Fa-f]{2})$') { return }
    $body = $Matches.body
    $expected = [Convert]::ToInt32($Matches.checksum, 16)
    $actual = 0
    foreach ($character in $body.ToCharArray()) { $actual = $actual -bxor [int]$character }
    if ($actual -ne $expected) { return }
    $fields = $body.Split(',')
    if ($fields[0].Length -ne 5) { return }
    $now = Get-Date
    $state.LastData = $now
    $state.Lines++
    $talker = $fields[0].Substring(0, 2)
    switch ($fields[0].Substring(2)) {
        'GGA' {
            if ($fields.Length -lt 10) { return }
            $state.Quality = $fields[6]
            $state.Used = $fields[7]
            $state.LastFix = $now
            $state.Valid = $fields[6] -in @('1','2','3','4','5')
            $state.Lat = Convert-Coordinate $fields[2] $fields[3]
            $state.Lon = Convert-Coordinate $fields[4] $fields[5]
        }
        'RMC' {
            # Prefer GGA when it is available: it also supplies fix quality and count.
            if ($fields.Length -lt 7 -or ($now - $state.LastFix).TotalSeconds -le 3) { return }
            $state.Quality = 'RMC'
            $state.Used = ''
            $state.Valid = $fields[2] -eq 'A'
            $state.Lat = Convert-Coordinate $fields[3] $fields[4]
            $state.Lon = Convert-Coordinate $fields[5] $fields[6]
            $state.LastFix = $now
        }
        'GSV' {
            # A satellite on several frequencies counts only once. Keep a 15-second window.
            if ($talker -eq 'BD') { $talker = 'GB' }
            for ($i = 4; $i + 3 -lt $fields.Length; $i += 4) {
                if ($fields[$i] -match '^\d+$') {
                    $satellites["${talker}:$($fields[$i])"] = @{ System = $talker; Seen = $now }
                }
            }
        }
    }
}

try {
    $gpsPort = New-Object System.IO.Ports.SerialPort($PortName, $BaudRate, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $gpsPort.ReadTimeout = 250
    $gpsPort.DtrEnable = $false
    $gpsPort.RtsEnable = $false
    $gpsPort.Open()
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $nextDisplay = 0.0
    $pending = ''
    while ($DurationSeconds -le 0 -or $timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
        $pending += $gpsPort.ReadExisting()
        while (($newline = $pending.IndexOf("`n")) -ge 0) {
            $line = $pending.Substring(0, $newline).Trim()
            $pending = $pending.Substring($newline + 1)
            try { Read-Nmea $line } catch { } # Ignore malformed receiver sentences.
        }
        if ($pending.Length -gt 16384) { $pending = '' }
        if ($timer.Elapsed.TotalSeconds -ge $nextDisplay) {
            $now = Get-Date
            foreach ($key in @($satellites.Keys)) {
                if (($now - $satellites[$key].Seen).TotalSeconds -gt 15) { $satellites.Remove($key) }
            }
            $labels = @{ GP='GPS/SBAS'; GB='北斗'; GL='GLONASS'; GA='Galileo'; GQ='QZSS'; GI='NavIC'; GN='混合星座' }
            $groups = @($satellites.Values | Group-Object -Property { $_.System })
            $details = @($groups | ForEach-Object {
                $label = $labels[$_.Name]
                if (-not $label) { $label = $_.Name }
                '{0}: {1}' -f $label, $_.Count
            }) -join '  |  '
            $fresh = ($now - $state.LastFix).TotalSeconds -le 5
            $fixNames = @{ '0'='未定位'; '1'='单点定位'; '2'='差分定位'; '3'='PPS 定位'; '4'='RTK 固定解'; '5'='RTK 浮点解'; '6'='推算位置'; '7'='手动位置'; '8'='模拟位置'; RMC='有效定位' }
            $fix = '等待定位数据'
            if ($fresh) {
                $fix = $fixNames[$state.Quality]
                if (-not $fix) { $fix = '未知定位状态' }
                if ($state.Quality -eq 'RMC' -and -not $state.Valid) { $fix = '未定位' }
            }
            $latitude = '--'
            $longitude = '--'
            if ($fresh -and $state.Valid -and $null -ne $state.Lat -and $null -ne $state.Lon) {
                $latitude = $state.Lat.ToString('F7', [Globalization.CultureInfo]::InvariantCulture)
                $longitude = $state.Lon.ToString('F7', [Globalization.CultureInfo]::InvariantCulture)
            }
            $used = '--'
            if ($fresh -and $state.Used -ne '') { $used = [int]$state.Used }
            $received = '等待数据（检查端口、波特率或是否被其他程序占用）'
            if (($now - $state.LastData).TotalSeconds -le 5) { $received = '正常' }
            $lines = @(
                'GPS 实时测试  |  Ctrl+C 退出',
                ('时间: {0:yyyy-MM-dd HH:mm:ss}  端口: {1}  波特率: {2}' -f $now, $PortName, $BaudRate),
                "数据接收: $received  |  有效 NMEA 报文: $($state.Lines)",
                "定位状态: $fix",
                "参与定位卫星数: $used",
                "最近 15 秒报告卫星数: $($satellites.Count)  $details （启动时需等待 GSV 报文）",
                "纬度: $latitude",
                "经度: $longitude",
                '经纬度为十进制度；南纬、西经为负数。未定位或数据过期时显示 --。',
                '报告卫星数不等于参与定位数；GN 混合与分星座同时输出时可能重复。'
            )
            if (-not $PlainText) { Clear-Host }
            Write-Host ($lines -join [Environment]::NewLine)
            $nextDisplay = $timer.Elapsed.TotalSeconds + 1
        }
        Start-Sleep -Milliseconds 50
    }
} catch {
    Write-Host "GPS 测试失败: $($_.Exception.Message)" -ForegroundColor Red
    Write-Host '请检查设备管理器中的 COM 端口，并关闭其他占用该端口的定位软件。'
    exit 1
} finally {
    if ($null -ne $gpsPort) {
        if ($gpsPort.IsOpen) { $gpsPort.Close() }
        $gpsPort.Dispose()
    }
}
