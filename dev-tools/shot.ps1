Add-Type -AssemblyName System.Drawing
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class WinCap3 {
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
}
"@
[WinCap3]::SetProcessDPIAware() | Out-Null
$proc = Get-Process ChaosMap -ErrorAction SilentlyContinue | Select-Object -First 1
if (-not $proc) { Write-Output "process not found"; exit 1 }
$h = $proc.MainWindowHandle
$r = New-Object WinCap3+RECT
[WinCap3]::GetWindowRect($h, [ref]$r) | Out-Null
$w = $r.Right - $r.Left; $hgt = $r.Bottom - $r.Top
Write-Output "rect $($r.Left),$($r.Top) ${w}x${hgt}"
$bmp = New-Object System.Drawing.Bitmap $w, $hgt
$g = [System.Drawing.Graphics]::FromImage($bmp)
$hdc = $g.GetHdc()
# PW_RENDERFULLCONTENT = 2：即使窗口被遮挡也能抓到
$ok = [WinCap3]::PrintWindow($h, $hdc, 2)
$g.ReleaseHdc($hdc)
$g.Dispose()
if (-not $ok) { Write-Output "PrintWindow failed, fallback CopyFromScreen" }
$bmp.Save("$PSScriptRoot\shot.png", [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()
Write-Output "saved shot.png ${w}x${hgt} printwindow=$ok"
