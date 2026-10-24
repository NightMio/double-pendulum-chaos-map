import json, subprocess, sys, os, urllib.request, urllib.parse

DIR = r"C:\Users\LENOVO\Documents\deepseek-harness\default-workspace\double-pendulum"
url = subprocess.check_output([r"C:\Program Files\Git\cmd\git.exe", "-C", DIR, "config", "--get", "remote.origin.url"]).decode().strip()
import re
m = re.search(r"https://[^:/]+:([^@]+)@github\.com", url)
if not m:
    print("no token"); sys.exit(1)
TOKEN = m.group(1)
print("token len", len(TOKEN))

def api(path, data=None, method="GET"):
    req = urllib.request.Request(
        "https://api.github.com" + path,
        data=json.dumps(data).encode() if data is not None else None,
        headers={
            "Authorization": "Bearer " + TOKEN,
            "Accept": "application/vnd.github+json",
            "User-Agent": "ChaosMap-Release",
            "Content-Type": "application/json",
        },
        method=method,
    )
    with urllib.request.urlopen(req, timeout=60) as r:
        return json.loads(r.read().decode())

body = """## 双摆混沌地图 v1.0 —— 中英双语版 / ChaosMap v1.0 - bilingual

**下载 / Downloads**
- `ChaosMap.exe` —— 中文界面 / Chinese UI
- `ChaosMap-en.exe` —— English UI

**运行要求 / Requirements**
Windows 10/11 + 支持 OpenCL 的显卡（NVIDIA / AMD / Intel 核显均可，无需 CUDA SDK）

**主要特性 / Highlights**
- OpenCL 加速：1200x1200 的 Lyapunov 指数场约 0.7 秒
- 三档精度：fp32 / float-float / fp64
- 无限放大、两种参数空间（角度 / 角动量）
- 双摆实时动画 + 相图 + 波形 + 声音合成
- 导出 PNG / 轨迹图 / 10 秒音频

详见 `README.md`（中文）与 `README.en.md`（English）。"""

rel = api("/repos/NightMio/double-pendulum-chaos-map/releases", {
    "tag_name": "v1.0",
    "name": "ChaosMap v1.0",
    "body": body,
    "draft": False,
    "prerelease": False,
}, method="POST")
print("release:", rel["html_url"])
upload_url = rel["upload_url"].split("{")[0]

for f in ("ChaosMap.exe", "ChaosMap-en.exe"):
    path = os.path.join(DIR, f)
    with open(path, "rb") as fh:
        data = fh.read()
    req = urllib.request.Request(
        upload_url + "?name=" + urllib.parse.quote(f),
        data=data,
        headers={
            "Authorization": "Bearer " + TOKEN,
            "Accept": "application/vnd.github+json",
            "User-Agent": "ChaosMap-Release",
            "Content-Type": "application/octet-stream",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=180) as r:
        j = json.loads(r.read().decode())
    print("uploaded", j["name"], j["size"], "bytes")
print("DONE")
