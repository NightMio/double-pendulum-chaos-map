param([string]$match = "ChaosPendulum", [string]$out = "pend_shot.png")
Add-Type -AssemblyName System.Drawing
Add-Type @"
using System;
using System.Text;
using System.Runtime.InteropServices;
public class ShotWin {
    public delegate bool EnumProc(IntPtr h, IntPtr l);
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumProc cb, IntPtr l);
    [DllImport("user32.dll")] public static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowTextW(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetClassNameW(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr hdc, uint flags);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
}
"@
[ShotWin]::SetProcessDPIAware() | Out-Null
$target = (Get-Process ChaosMap | Select-Object -First 1).Id
$script:found = [IntPtr]::Zero
$script:want = $match
$cb = [ShotWin+EnumProc]{
  param($h, $l)
  $pid2 = 0
  [ShotWin]::GetWindowThreadProcessId($h, [ref]$pid2) | Out-Null
  if ($pid2 -eq $script:target -and [ShotWin]::IsWindowVisible($h)) {
    $sb = New-Object System.Text.StringBuilder 256
    [ShotWin]::GetClassNameW($h, $sb, 256) | Out-Null
    if ($sb.ToString() -eq $script:want) { $script:found = $h }
  }
  return $true
}
$script:target = $target
[ShotWin]::EnumWindows($cb, [IntPtr]::Zero) | Out-Null
if ($script:found -eq [IntPtr]::Zero) { Write-Output "window '$match' not found"; exit 1 }
$r = New-Object ShotWin+RECT
[ShotWin]::GetWindowRect($script:found, [ref]$r) | Out-Null
$w = $r.Right - $r.Left; $hgt = $r.Bottom - $r.Top
Write-Output "rect $($r.Left),$($r.Top) ${w}x${hgt}"
# 用 PrintWindow，避免窗口被其他程序遮挡时抓错内容
[ShotWin]::SetForegroundWindow($script:found) | Out-Null
Start-Sleep -Milliseconds 700
$bmp = New-Object System.Drawing.Bitmap $w, $hgt
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($r.Left, $r.Top, 0, 0, (New-Object System.Drawing.Size $w, $hgt))
$ok = "copyfromscreen"
$g.Dispose()
$bmp.Save((Join-Path $PSScriptRoot $out), [System.Drawing.Imaging.ImageFormat]::Png)
$bmp.Dispose()
Write-Output "saved $out ${w}x${hgt} printwindow=$ok"
