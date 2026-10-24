$ErrorActionPreference = "Stop"
$dir = "C:\Users\LENOVO\Documents\deepseek-harness\default-workspace\double-pendulum"
$url = & "C:\Program Files\Git\cmd\git.exe" -C $dir config --get remote.origin.url
if ($url -match 'github\.com/[^:]+:([^@]+)@') { $token = $Matches[1] } else { throw "no token in remote url" }
Write-Output ("token len " + $token.Length)

$headers = @{
    Authorization = "Bearer $token"
    "User-Agent"  = "ChaosMap-Release"
    Accept        = "application/vnd.github+json"
}

$body = @{
    tag_name   = "v1.0"
    name       = "ChaosMap v1.0"
    body       = @"
双摆混沌地图 v1.0 —— 中英双语版

ChaosMap v1.0 - bilingual release

**下载 / Downloads**
- ``ChaosMap.exe`` —— 中文界面 / Chinese UI
- ``ChaosMap-en.exe`` —— English UI

**运行要求 / Requirements**
Windows 10/11 + 支持 OpenCL 的显卡（NVIDIA / AMD / Intel 核显均可，无需 CUDA SDK）

**主要特性 / Highlights**
- OpenCL 加速：1200x1200 的 Lyapunov 指数场 0.7 秒
- 三档精度：fp32 / float-float / fp64
- 无限放大、两种参数空间（角度 / 角动量）
- 双摆实时动画 + 相图 + 波形 + 声音合成
- 导出 PNG / 轨迹图 / 10 秒音频

详见 README.md（中文）与 README.en.md（English）。
"@
    draft      = $false
    prerelease = $false
} | ConvertTo-Json -Depth 5

$rel = Invoke-RestMethod -Uri "https://api.github.com/repos/NightMio/double-pendulum-chaos-map/releases" `
    -Method Post -Headers $headers -Body $body -ContentType "application/json"
Write-Output ("release id " + $rel.id + " url " + $rel.html_url)

$upload = $rel.upload_url -replace '\{\?name,label\}', ''

foreach ($f in @("ChaosMap.exe", "ChaosMap-en.exe")) {
    $path = Join-Path $dir $f
    $u = "$upload?name=$f"
    $r = Invoke-RestMethod -Uri $u -Method Post -Headers $headers -InFile $path -ContentType "application/octet-stream"
    Write-Output ("uploaded " + $r.name + " " + $r.size + " bytes -> " + $r.browser_download_url)
}
Write-Output "DONE"
