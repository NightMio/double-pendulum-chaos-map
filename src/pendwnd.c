#include "app.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <commctrl.h>
#include <stdint.h>
#include <mmsystem.h>
#include <gdiplus.h>

#include "cl_gpu.h"

extern HWND g_main;
static void export_audio(HWND owner, double th1, double th2, double w1, double w2);
static void export_trajectory(HWND owner, double th1, double th2, double w1, double w2);
void layout(HWND hwnd);
#define IDC_APPLY 1003


static int s_pendFrames = 0;
const WCHAR *PEND_CLASS = L"ChaosPendulum";
#ifndef GET_X_LPARAM
#define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))
#endif
#define IDC_PAUDIO 1020
static RECT g_audioBtn;

/* 摆的几何：保证两个摆任意姿态下都完整落在窗口内 */
static void pend_geom(int cw, int ch, double *pcx, double *pcy, double *pLp)
{
    *pcx = cw * 0.5;
    *pcy = ch * 0.52;
    double m = cw < ch ? (double)cw : (double)ch;
    *pLp = m * 0.21;          /* 最大伸展 2*Lp = 0.42*min(cw,ch) */
}

/* 把角度折到 (-π, π] */
static void pend_push_trail(PendView *pv)
{
    pv->tr1[pv->trailHead] = pv->s.th1;
    pv->tr2[pv->trailHead] = pv->s.th2;
    pv->trailHead = (pv->trailHead + 1) % TRAIL_MAX;
    if (pv->trailCount < TRAIL_MAX) pv->trailCount++;

    if (pv->space == 0) {
        pv->pth1[pv->pHead] = (float)wrap_angle(pv->s.th1);
        pv->pth2[pv->pHead] = (float)wrap_angle(pv->s.th2);
    } else {
        pv->pth1[pv->pHead] = (float)pv->s.w1;
        pv->pth2[pv->pHead] = (float)pv->s.w2;
    }
    pv->pHead = (pv->pHead + 1) % PTRAL_MAX;
    if (pv->pCount < PTRAL_MAX) pv->pCount++;
}

static void snd_push(PendView *pv)
{
    if (pv->space == 0) { pv->sndL[pv->sndHead] = (float)pv->uw1; pv->sndR[pv->sndHead] = (float)pv->uw2; }
    else                { pv->sndL[pv->sndHead] = (float)pv->s.w1; pv->sndR[pv->sndHead] = (float)pv->s.w2; }
    pv->sndHead = (pv->sndHead + 1) % SND_MAX;
    if (pv->sndCount < SND_MAX) pv->sndCount++;
}

static void pend_advance(PendView *pv)
{
    if (pv->paused) return;
    double frame = 0.016 * pv->speed;
    if (frame > 0.2) frame = 0.2;
    int steps = (int)(frame / 0.001 + 0.5);
    if (steps < 1) steps = 1;
    if (steps > 200) steps = 200;
    const double h = frame / steps;
    for (int k = 0; k < steps; ++k) {
        double p1 = pv->s.th1, p2 = pv->s.th2;
        pv->s = rk4(pv->s, h);
        pv->t += h;
        pv->uw1 += wrap_angle(pv->s.th1 - p1);
        pv->uw2 += wrap_angle(pv->s.th2 - p2);
        pv->acc += h;
        pv->sndAcc += h;
        while (pv->acc >= DRAW_DT) {
            pv->acc -= DRAW_DT;
            pend_push_trail(pv);
        }
        while (pv->sndAcc >= 1.0 / SND_HZ) {
            pv->sndAcc -= 1.0 / SND_HZ;
            snd_push(pv);
        }
    }
    if (pv->trailCount < 1) pend_push_trail(pv);
}

#define PANEL_W 380


static void draw_wave(HDC mem, int x, int y, int w, int h,
                      const float *buf, int head, int count, int nshow,
                      COLORREF col, const WCHAR *label)
{
    if (w < 20 || h < 24) return;
    RECT r = { x, y, x + w + 1, y + h + 1 };
    HBRUSH b = CreateSolidBrush(RGB(12, 14, 26));
    FillRect(mem, &r, b);
    DeleteObject(b);
    FrameRect(mem, &r, (HBRUSH)GetStockObject(GRAY_BRUSH));

    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(140, 150, 170));
    TextOutW(mem, x + 4, y + 2, label, (int)wcslen(label));

    int n = count < nshow ? count : nshow;
    if (n < 2) return;
    int start = (head - n + SND_MAX) % SND_MAX;

    double lo = 1e30, hi = -1e30;
    for (int k = 0; k < n; ++k) {
        double v = buf[(start + k) % SND_MAX];
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    if (hi - lo < 1e-6) { lo -= 0.5; hi += 0.5; }

    int top = y + 20, bot = y + h - 4;
    if (bot <= top) return;

    HPEN zp = CreatePen(PS_SOLID, 1, RGB(52, 46, 40));
    HGDIOBJ o1 = SelectObject(mem, zp);
    int yz = bot - (int)((0.0 - lo) / (hi - lo) * (bot - top));
    if (yz >= top && yz <= bot) { MoveToEx(mem, x + 1, yz, NULL); LineTo(mem, x + w - 1, yz); }
    SelectObject(mem, o1);
    DeleteObject(zp);

    HPEN wp = CreatePen(PS_SOLID, 1, col);
    HGDIOBJ o2 = SelectObject(mem, wp);
    for (int k = 0; k < n; ++k) {
        double v = buf[(start + k) % SND_MAX];
        int px = x + 1 + (int)((double)k / (n - 1) * (w - 2));
        int py = bot - (int)((v - lo) / (hi - lo) * (bot - top));
        if (k == 0) MoveToEx(mem, px, py, NULL);
        else LineTo(mem, px, py);
    }
    SelectObject(mem, o2);
    DeleteObject(wp);
}


/* ================= 声音：双摆 -> 音频 ================= */
#define AUDIO_FS     48000
#define AUDIO_BLOCK  1200       /* 25 ms */
#define AUDIO_BUFS   3

typedef struct {
    HWAVEOUT h;
    WAVEHDR  hdr[AUDIO_BUFS];
    short    buf[AUDIO_BUFS][AUDIO_BLOCK * 2];
    int      running, mode;     /* mode 0=流式 1=周期循环 */
    float   *tabL, *tabR;       /* 主波表（周期模式或已生效的流式波表） */
    float   *tabBL, *tabBR;     /* 待切换的流式波表 */
    int      tabN;
    double   xfade;             /* 0..1 交叉淡化进度 */
    double   phase, pitch;
    double   streamPos;         /* 流式模式：播放头（绝对轨迹点索引） */
    int      info;              /* 0 未检测 1 周期 2 准周期 3 混沌 */
    double   period, cover;
    int      refresh;
} AudioCtx;

/* ---------- 实时流式：声音摆独立于演示摆，440 倍速积分 ---------- */
#define RING_BITS  20
#define RING_SIZE  (1 << RING_BITS)          /* 1048576 点，约 2.4 秒音频 */
static double g_sndSpeed = 440.0;   /* 实时流式倍率：音高 = 轨迹频率 × 该值 */
#define SND_GAIN   26000.0

static float *g_ringL, *g_ringR;
static volatile LONG64 g_ringW, g_ringRpos;
static volatile int    g_ringRun;
static HANDLE g_ringThread;
static State  g_sndState;

/* 检测环：声音摆的 wrap 角度，100 Hz，60 秒物理（440 倍速下 0.14 秒就填满） */
#define DET_HZ   100
#define DET_MAX  (60 * DET_HZ)
static float g_detL[DET_MAX], g_detR[DET_MAX];
static volatile LONG g_detW;

static DWORD WINAPI snd_thread(LPVOID param)
{
    (void)param;
    State s = g_sndState;
    double uw1 = s.th1, uw2 = s.th2;
    int dec = 0;
    const double h = 1.0 / SND_HZ;
    while (g_ringRun) {
        for (int k = 0; k < 256 && g_ringRun; ++k) {
            LONG64 w = g_ringW;
            g_ringL[w & (RING_SIZE - 1)] = (float)sin(uw1);
            g_ringR[w & (RING_SIZE - 1)] = (float)sin(uw2);
            g_ringW = w + 1;
            double p1 = s.th1, p2 = s.th2;
            s = rk4(s, h);
            uw1 += wrap_angle(s.th1 - p1);
            uw2 += wrap_angle(s.th2 - p2);
            if (++dec >= SND_HZ / DET_HZ) {
                dec = 0;
                LONG dw = g_detW;
                int di = (int)(dw % DET_MAX);
                g_detL[di] = (float)wrap_angle(uw1);
                g_detR[di] = (float)wrap_angle(uw2);
                g_detW = dw + 1;
            }
        }
        while (g_ringRun && (g_ringW - g_ringRpos) > (LONG64)(RING_SIZE * 3 / 4))
            Sleep(1);
    }
    return 0;
}

static void snd_ring_start(State s)
{
    if (!g_ringL) {
        g_ringL = (float *)malloc(sizeof(float) * RING_SIZE);
        g_ringR = (float *)malloc(sizeof(float) * RING_SIZE);
        if (!g_ringL || !g_ringR) return;
    }
    g_sndState = s;
    g_ringW = g_ringRpos = 0;
    g_detW = 0;
    g_ringRun = 1;
    g_ringThread = CreateThread(NULL, 0, snd_thread, NULL, 0, NULL);
}

static void snd_ring_stop(void)
{
    if (!g_ringRun) return;
    g_ringRun = 0;
    if (g_ringThread) {
        WaitForSingleObject(g_ringThread, 3000);
        CloseHandle(g_ringThread);
        g_ringThread = NULL;
    }
}

static AudioCtx g_au;

/* 自相关找主周期（降采样 10 倍，分辨率 10ms） */
static double snd_autocorr(PendView *pv, int ds, double *rpeak)
{
    int n = pv->sndCount;
    if (n < SND_HZ * 2) { *rpeak = 0; return 0.0; }
    if (n > SND_HZ * 20) n = SND_HZ * 20;
    static float buf[4096];
    int m = n / ds;
    if (m < 64) { *rpeak = 0; return 0.0; }
    if (m > 4096) m = 4096;

    int start = (pv->sndHead - n + SND_MAX * 2) % SND_MAX;
    double mean = 0.0;
    for (int i = 0; i < m; ++i) {
        buf[i] = pv->sndL[(start + i * ds) % SND_MAX];
        mean += buf[i];
    }
    mean /= m;
    double e0 = 0.0;
    for (int i = 0; i < m; ++i) { buf[i] = (float)(buf[i] - mean); e0 += buf[i] * buf[i]; }
    if (e0 < 1e-9) { *rpeak = 0; return 0.0; }

    int lagMin = 30, lagMax = m / 2;
    double rp = 1.0, rc = 0.0, rn = 0.0;
    int stage = 0;
    *rpeak = 0.0;
    for (int lag = lagMin; lag <= lagMax; ++lag) {
        double r = 0.0;
        for (int i = 0; i + lag < m; ++i) r += buf[i] * buf[i + lag];
        r /= e0 * (double)(m - lag) / (double)m;
        if (stage == 0) { rc = r; stage = 1; continue; }
        if (stage == 1) {
            rn = r; stage = 2;
            if (rc > rp && rc > rn && rc > 0.55) {
                *rpeak = rc;
                return (double)lagMin * ds / SND_HZ;
            }
            rp = rc; rc = rn; continue;
        }
        rn = r;
        if (rc > rp && rc > rn && rc > 0.55) {
            *rpeak = rc;
            return (double)(lag - 1) * ds / SND_HZ;
        }
        rp = rc; rc = rn;
    }
    *rpeak = 0.0;
    return 0.0;
}

/* 判断周期性：相空间回归误差（对 60 秒物理轨迹做全局搜索） */
static double snd_detect(int *info, double *cover)
{
    LONG dw = g_detW;
    int n = (int)(dw < DET_MAX ? dw : DET_MAX);
    if (n < 128) { *info = 0; *cover = 0; return 0.0; }

    /* 环形缓冲写满后物理顺序不再等于时间顺序，先按时间顺序拷贝出来 */
    static double dl[DET_MAX], dr[DET_MAX];
    {
        int start = (int)((dw - n) % DET_MAX);
        for (int k = 0; k < n; ++k) {
            int idx = (start + k) % DET_MAX;
            dl[k] = g_detL[idx];
            dr[k] = g_detR[idx];
        }
    }

    static unsigned char grid[64 * 64];
    memset(grid, 0, sizeof grid);
    int cells = 0;
    for (int k = 0; k < n; ++k) {
        int gx = (int)((dl[k] + M_PI) / (2.0 * M_PI) * 64.0);
        int gy = (int)((dr[k] + M_PI) / (2.0 * M_PI) * 64.0);
        if (gx < 0) gx = 0; if (gx > 63) gx = 63;
        if (gy < 0) gy = 0; if (gy > 63) gy = 63;
        if (!grid[gy * 64 + gx]) { grid[gy * 64 + gx] = 1; ++cells; }
    }
    *cover = cells / 4096.0;

    int tmin = 30;                 /* 0.30 s */
    int tmax = n / 2;
    if (tmax > DET_MAX / 2) tmax = DET_MAX / 2;
    if (tmax < tmin + 40) { *info = 2; return 0.0; }

    double bestE = 1e30;
    int bestT = 0;
    for (int T = tmin; T <= tmax; T += 2) {
        double sum = 0.0;
        int cnt = 0;
        for (int i = 0; i + T < n; ++i) {
            double d1 = dl[i + T] - dl[i];
            double d2 = dr[i + T] - dr[i];
            d1 -= 2.0 * M_PI * floor(d1 / (2.0 * M_PI) + 0.5);
            d2 -= 2.0 * M_PI * floor(d2 / (2.0 * M_PI) + 0.5);
            sum += d1 * d1 + d2 * d2;
            ++cnt;
        }
        if (cnt < 8) continue;
        double e = sqrt(sum / cnt);
        if (e < bestE) { bestE = e; bestT = T; }
    }
    logf_("detect n=%d cover=%.3f bestT=%d bestE=%.4f", n, *cover, bestT, bestE);
    if (bestT && bestE < 0.06) {
        *info = 1;
        return (double)bestT / DET_HZ;
    }
    if (*cover > 0.30) { *info = 3; return 0.0; }
    *info = 2;
    return 0.0;
}

static void audio_make_table(PendView *pv)
{
    if (g_au.mode == 0) {
        /* 流式模式：不吃波表，只更新检测信息用于显示 */
        g_au.period = snd_detect(&g_au.info, &g_au.cover);
        return;
    }
    if (!g_au.tabL) {
        g_au.tabL  = (float *)malloc(sizeof(float) * SND_MAX);
        g_au.tabR  = (float *)malloc(sizeof(float) * SND_MAX);
        g_au.tabBL = (float *)malloc(sizeof(float) * SND_MAX);
        g_au.tabBR = (float *)malloc(sizeof(float) * SND_MAX);
        if (!g_au.tabL || !g_au.tabR || !g_au.tabBL || !g_au.tabBR) return;
        g_au.xfade = 1.0;
    }
    double Tp = snd_detect(&g_au.info, &g_au.cover);
    g_au.period = Tp;

    int N;
    if (g_au.mode == 1 && Tp > 0.2) N = (int)(Tp * SND_HZ);
    else N = SND_HZ;
    if (N > SND_MAX) N = SND_MAX;
    if (N < 64) N = 64;

    float *dstL = (g_au.mode == 0) ? g_au.tabBL : g_au.tabL;
    float *dstR = (g_au.mode == 0) ? g_au.tabBR : g_au.tabR;
    int start = (pv->sndHead - N + SND_MAX * 2) % SND_MAX;
    double mL = 0.0, mR = 0.0;
    for (int i = 0; i < N; ++i) {
        dstL[i] = (float)sin(pv->sndL[(start + i) % SND_MAX]);
        dstR[i] = (float)sin(pv->sndR[(start + i) % SND_MAX]);
        mL += dstL[i]; mR += dstR[i];
    }
    mL /= N; mR /= N;
    double dltL = dstL[N - 1] - dstL[0];
    double dltR = dstR[N - 1] - dstR[0];
    double amax = 1e-9;
    for (int i = 0; i < N; ++i) {
        double f = (N > 1) ? (double)i / (N - 1) : 0.0;
        double a = (dstL[i] - mL) - dltL * f;
        double b = (dstR[i] - mR) - dltR * f;
        dstL[i] = (float)a;
        dstR[i] = (float)b;
        if (fabs(a) > amax) amax = fabs(a);
        if (fabs(b) > amax) amax = fabs(b);
    }
    for (int i = 0; i < N; ++i) {
        dstL[i] = (float)(dstL[i] / amax * 0.75);
        dstR[i] = (float)(dstR[i] / amax * 0.75);
    }
    int fade = N / 50;
    if (fade > 4) {
        for (int i = 0; i < fade; ++i) {
            double w = (double)i / fade;
            int j = N - fade + i;
            dstL[j] = (float)(dstL[j] * w + dstL[i] * (1.0 - w));
            dstR[j] = (float)(dstR[j] * w + dstR[i] * (1.0 - w));
        }
    }
    g_au.tabN = N;
    g_au.pitch = 220.0;
    g_au.xfade = (g_au.mode == 0) ? 0.0 : 1.0;
}

static void audio_pump(void)
{
    if (!g_au.running || !g_au.h) return;

    if (g_au.mode == 0) {
        if (!g_ringL) return;
        const double inc = g_sndSpeed / (AUDIO_FS / (double)SND_HZ);
        for (int b = 0; b < AUDIO_BUFS; ++b) {
            WAVEHDR *hh2 = &g_au.hdr[b];
            if (hh2->dwFlags & WHDR_INQUEUE) continue;
            for (int i = 0; i < AUDIO_BLOCK; ++i) {
                LONG64 w = g_ringW;
                double ph = g_au.streamPos;
                double L = 0.0, R = 0.0;
                if (ph + 2.0 < (double)w) {
                    LONG64 i0 = (LONG64)ph;
                    double fr = ph - (double)i0;
                    int a = (int)(i0 & (RING_SIZE - 1));
                    int c2 = (int)((i0 + 1) & (RING_SIZE - 1));
                    L = g_ringL[a] + (g_ringL[c2] - g_ringL[a]) * fr;
                    R = g_ringR[a] + (g_ringR[c2] - g_ringR[a]) * fr;
                }
                double sl = L * SND_GAIN, sr = R * SND_GAIN;
                if (sl > 32000.0) sl = 32000.0;
                if (sl < -32000.0) sl = -32000.0;
                if (sr > 32000.0) sr = 32000.0;
                if (sr < -32000.0) sr = -32000.0;
                g_au.buf[b][2 * i]     = (short)sl;
                g_au.buf[b][2 * i + 1] = (short)sr;
                g_au.streamPos = ph + inc;
            }
            g_ringRpos = (LONG64)g_au.streamPos;
            hh2->dwFlags &= ~WHDR_DONE;
            waveOutWrite(g_au.h, hh2, sizeof(WAVEHDR));
        }
        return;
    }
    if (g_au.tabN < 8) return;
    const int N = g_au.tabN;
    double inc = (double)N * g_au.pitch / AUDIO_FS;
    if (inc < 1e-9) inc = 1e-9;
    int mix = (g_au.mode == 0 && g_au.xfade < 1.0);
    double w = mix ? g_au.xfade : 1.0;
    for (int b = 0; b < AUDIO_BUFS; ++b) {
        WAVEHDR *hh = &g_au.hdr[b];
        if (hh->dwFlags & WHDR_INQUEUE) continue;
        for (int i = 0; i < AUDIO_BLOCK; ++i) {
            int i0 = (int)g_au.phase;
            if (i0 >= N) i0 = N - 1;
            double fr = g_au.phase - i0;
            int i1 = i0 + 1; if (i1 >= N) i1 = 0;
            double La = g_au.tabL[i0] + (g_au.tabL[i1] - g_au.tabL[i0]) * fr;
            double Ra = g_au.tabR[i0] + (g_au.tabR[i1] - g_au.tabR[i0]) * fr;
            if (mix) {
                double Lb = g_au.tabBL[i0] + (g_au.tabBL[i1] - g_au.tabBL[i0]) * fr;
                double Rb = g_au.tabBR[i0] + (g_au.tabBR[i1] - g_au.tabBR[i0]) * fr;
                La = La * (1.0 - w) + Lb * w;
                Ra = Ra * (1.0 - w) + Rb * w;
            }
            g_au.buf[b][2 * i]     = (short)(La * 22000.0);
            g_au.buf[b][2 * i + 1] = (short)(Ra * 22000.0);
            g_au.phase += inc;
            while (g_au.phase >= N) g_au.phase -= N;
        }
        hh->dwFlags &= ~WHDR_DONE;
        waveOutWrite(g_au.h, hh, sizeof(WAVEHDR));
    }
    if (mix) {
        g_au.xfade += (double)AUDIO_BLOCK / (1.5 * AUDIO_FS);
        if (g_au.xfade >= 1.0) {
            g_au.xfade = 1.0;
            memcpy(g_au.tabL, g_au.tabBL, sizeof(float) * N);
            memcpy(g_au.tabR, g_au.tabBR, sizeof(float) * N);
        }
    }
}

static int audio_start(PendView *pv)
{
    if (g_au.running) return 0;
    if (g_au.mode == 0 && !g_ringL && pv) snd_ring_start(pv->s);
    if (g_au.mode == 0) {
        if (g_ringL) { snd_ring_stop(); snd_ring_start(pv->s); }
        g_au.streamPos = 0.0;
        g_ringRpos = 0;
    } else if (g_au.tabN < 8) {
        return -1;
    }
    WAVEFORMATEX wf;
    memset(&wf, 0, sizeof wf);
    wf.wFormatTag = WAVE_FORMAT_PCM;
    wf.nChannels = 2;
    wf.nSamplesPerSec = AUDIO_FS;
    wf.wBitsPerSample = 16;
    wf.nBlockAlign = 4;
    wf.nAvgBytesPerSec = AUDIO_FS * 4;
    if (waveOutOpen(&g_au.h, WAVE_MAPPER, &wf, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
        g_au.h = NULL;
        return -1;
    }
    for (int b = 0; b < AUDIO_BUFS; ++b) {
        memset(&g_au.hdr[b], 0, sizeof(WAVEHDR));
        g_au.hdr[b].lpData = (LPSTR)g_au.buf[b];
        g_au.hdr[b].dwBufferLength = AUDIO_BLOCK * 4;
        waveOutPrepareHeader(g_au.h, &g_au.hdr[b], sizeof(WAVEHDR));
    }
    g_au.running = 1;
    g_au.phase = 0.0;
    audio_pump();
    return 0;
}

static void audio_stop(void)
{
    if (!g_au.running) return;
    g_au.running = 0;
    snd_ring_stop();
    if (g_au.h) {
        waveOutReset(g_au.h);
        for (int b = 0; b < AUDIO_BUFS; ++b)
            waveOutUnprepareHeader(g_au.h, &g_au.hdr[b], sizeof(WAVEHDR));
        waveOutClose(g_au.h);
        g_au.h = NULL;
    }
}

static void pend_paint(HWND hwnd, PendView *pv, HDC hdc)
{
    RECT rc;
    GetClientRect(hwnd, &rc);
    const int cw = rc.right, ch = rc.bottom;
    if (cw < 4 || ch < 4) return;

    OP("paint:alloc");
    HDC mem = CreateCompatibleDC(hdc);
    HBITMAP bmp = CreateCompatibleBitmap(hdc, cw, ch);
    HGDIOBJ old = SelectObject(mem, bmp);

    HBRUSH bg = CreateSolidBrush(RGB(8, 10, 18));
    FillRect(mem, &rc, bg);
    DeleteObject(bg);

    const int leftW = (cw > PANEL_W + 240) ? (cw - PANEL_W) : cw;

    double cx, cy, Lp;
    pend_geom(leftW, ch, &cx, &cy, &Lp);

    double Lmax = g_phys.l1 > g_phys.l2 ? g_phys.l1 : g_phys.l2;
    if (Lmax <= 0.0) Lmax = 1.0;
    const double L1 = Lp * g_phys.l1 / Lmax;   /* 最长那根杆 = Lp，保证不出窗口 */
    const double L2 = Lp * g_phys.l2 / Lmax;
    /* 回看模式下用轨迹缓冲里的历史角度 */
    double cur1 = pv->s.th1, cur2 = pv->s.th2;
    if (pv->cursor >= 0 && pv->trailCount > 0) {
        int ci = (pv->trailHead - pv->trailCount + pv->cursor) % TRAIL_MAX;
        if (ci < 0) ci += TRAIL_MAX;
        cur1 = pv->tr1[ci];
        cur2 = pv->tr2[ci];
    }
    const double x1 = cx + L1 * sin(cur1);
    const double y1 = cy + L1 * cos(cur1);
    const double x2 = x1 + L2 * sin(cur2);
    const double y2 = y1 + L2 * cos(cur2);

    /* 末端轨迹（按角度重算，窗口缩放后依然正确） */
    OP("paint:trail");
    if (pv->trailCount > 1) {
        int n = pv->trailCount;
        POINT *pts = (POINT *)malloc(sizeof(POINT) * (size_t)(n + 1));
        if (pts) {
            int idx = (pv->trailHead - n + TRAIL_MAX) % TRAIL_MAX;
            for (int k = 0; k < n; ++k) {
                double a1 = cx + L1 * sin(pv->tr1[idx]);
                double b1 = cy + L1 * cos(pv->tr1[idx]);
                pts[k].x = (LONG)(a1 + L2 * sin(pv->tr2[idx]));
                pts[k].y = (LONG)(b1 + L2 * cos(pv->tr2[idx]));
                idx = (idx + 1) % TRAIL_MAX;
            }
            /* 越旧越暗：分 24 档，每档一条 Polyline */
            const int SEG = 24;
            for (int sgi = 0; sgi < SEG; ++sgi) {
                int k0 = (int)((double)n * sgi / SEG);
                int k1 = (int)((double)n * (sgi + 1) / SEG);
                if (k1 <= k0) continue;
                if (k1 < n) ++k1;                 /* 与下一档重叠，避免断点 */
                double t = (double)(sgi + 1) / SEG;
                HPEN tp = CreatePen(PS_SOLID, 1,
                                    RGB((int)(18 + 42 * t), (int)(38 + 60 * t), (int)(70 + 110 * t)));
                HGDIOBJ op = SelectObject(mem, tp);
                Polyline(mem, pts + k0, k1 - k0);
                SelectObject(mem, op);
                DeleteObject(tp);
            }
            free(pts);
        }
    }

    /* 杆 */
    HPEN rod = CreatePen(PS_SOLID, 4, RGB(220, 225, 235));
    HGDIOBJ op = SelectObject(mem, rod);
    MoveToEx(mem, (int)cx, (int)cy, NULL);
    LineTo(mem, (int)x1, (int)y1);
    LineTo(mem, (int)x2, (int)y2);
    SelectObject(mem, op);
    DeleteObject(rod);

    /* 两个球 */
    HBRUSH b1 = CreateSolidBrush(RGB(255, 170, 60));
    HBRUSH b2 = CreateSolidBrush(RGB(120, 200, 255));
    HGDIOBJ ob = SelectObject(mem, b1);
    Ellipse(mem, (int)x1 - 11, (int)y1 - 11, (int)x1 + 11, (int)y1 + 11);
    SelectObject(mem, b2);
    Ellipse(mem, (int)x2 - 13, (int)y2 - 13, (int)x2 + 13, (int)y2 + 13);

    /* 支点 */
    HBRUSH b3 = CreateSolidBrush(RGB(200, 200, 210));
    SelectObject(mem, b3);
    Ellipse(mem, (int)cx - 5, (int)cy - 5, (int)cx + 5, (int)cy + 5);
    SelectObject(mem, ob);
    DeleteObject(b1); DeleteObject(b2); DeleteObject(b3);

    /* 左上信息 */
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(210, 215, 225));
    WCHAR buf[256];
    const double e0 = energy(&pv->s);
    swprintf(buf, 256, L"t = %.2f s    倍速 %.2fx   %s",
             pv->t, pv->speed, pv->paused ? L"[暂停]" : L"");
    TextOutW(mem, 16, 12, buf, (int)wcslen(buf));

    swprintf(buf, 256, L"θ1 = %.4f    θ2 = %.4f  (已折到 -π~π)",
             wrap_angle(pv->s.th1), wrap_angle(pv->s.th2));
    TextOutW(mem, 16, 34, buf, (int)wcslen(buf));

    if (g_au.running || g_au.info > 0) {
        const WCHAR *st = (g_au.info == 1) ? L"周期解" :
                          (g_au.info == 2) ? L"准周期" :
                          (g_au.info == 3) ? L"混沌" : L"采样中";
        if (g_au.mode)
            swprintf(buf, 256, L"音频 %s  周期循环  %s 周期%.2fs  %dHz", 
                     g_au.running ? L"▶" : L"■", st, g_au.period, (int)g_au.pitch);
        else
            swprintf(buf, 256, L"音频 %s  实时流式  %s 周期%.2fs  ×%.0f", 
                     g_au.running ? L"▶" : L"■", st, g_au.period, g_sndSpeed);
        SetTextColor(mem, g_au.running ? RGB(120, 220, 160) : RGB(150, 160, 180));
        TextOutW(mem, 16, 56, buf, (int)wcslen(buf));
        SetTextColor(mem, RGB(210, 215, 225));
    }

    /* ---------------- 右侧：状态图（角度空间 θ1-θ2 / 角动量空间 ω1-ω2） ---------------- */
    OP("paint:state");
    const double SR = (pv->space == 0) ? M_PI : 25.0;
    int side = (ch - 150) * 56 / 100;
    int avail = cw - leftW - 48;
    if (side > avail) side = avail;
    if (side < 80) side = 80;
    int gx = leftW + (cw - leftW - side) / 2;
    int gy = 82;

    RECT gr = { gx, gy, gx + side + 1, gy + side + 1 };
    HBRUSH pb = CreateSolidBrush(RGB(12, 14, 26));
    FillRect(mem, &gr, pb);
    DeleteObject(pb);

    IntersectClipRect(mem, gx + 1, gy + 1, gx + side, gy + side);
    FrameRect(mem, &gr, (HBRUSH)GetStockObject(GRAY_BRUSH));

    /* 细网格：每 π/2 一格 */
    HPEN gp = CreatePen(PS_SOLID, 1, RGB(32, 38, 54));
    HGDIOBJ og = SelectObject(mem, gp);
    for (int k = 1; k < 4; ++k) {
        int tx = gx + side * k / 4;
        int ty = gy + side * k / 4;
        MoveToEx(mem, tx, gy, NULL); LineTo(mem, tx, gy + side);
        MoveToEx(mem, gx, ty, NULL); LineTo(mem, gx + side, ty);
    }
    SelectObject(mem, og);
    DeleteObject(gp);

    /* 主轴 θ1=0 / θ2=0 */
    HPEN axp = CreatePen(PS_SOLID, 1, RGB(58, 68, 92));
    HGDIOBJ oa = SelectObject(mem, axp);
    MoveToEx(mem, gx + side / 2, gy, NULL);
    LineTo(mem, gx + side / 2, gy + side);
    MoveToEx(mem, gx, gy + side / 2, NULL);
    LineTo(mem, gx + side, gy + side / 2);
    SelectObject(mem, oa);
    DeleteObject(axp);

    /* 状态轨迹：越旧越暗，跨 ±π 时断开 */
    if (pv->pCount > 1) {
        const int SEG = 24;
        int n = pv->pCount;
        int idx = (pv->pHead - n + PTRAL_MAX) % PTRAL_MAX;
        int prev = -1;
        int curseg = -1;
        HPEN pen = NULL;
        for (int k = 0; k < n; ++k) {
            int seg = (int)((long long)k * SEG / n);
            if (seg >= SEG) seg = SEG - 1;
            if (seg != curseg) {
                if (pen) DeleteObject(pen);
                double t = (double)(seg + 1) / SEG;
                pen = CreatePen(PS_SOLID, 1,
                                RGB((int)(60 + 195 * t), (int)(50 + 164 * t), (int)(20 + 70 * t)));
                SelectObject(mem, pen);
                curseg = seg;
            }
            int curc = idx;
            idx = (idx + 1) % PTRAL_MAX;
            double a = pv->pth1[curc], b = pv->pth2[curc];
            int sx = gx + (int)((a + SR) / (2.0 * SR) * side);
            int sy = gy + side - (int)((b + SR) / (2.0 * SR) * side);
            if (prev < 0 || fabs(a - pv->pth1[prev]) > SR || fabs(b - pv->pth2[prev]) > SR)
                MoveToEx(mem, sx, sy, NULL);
            else
                LineTo(mem, sx, sy);
            prev = curc;
        }
        if (pen) DeleteObject(pen);
    }

    /* 当前点 */
    {
        double a = (pv->space == 0) ? wrap_angle(pv->s.th1) : pv->s.w1;
        double b = (pv->space == 0) ? wrap_angle(pv->s.th2) : pv->s.w2;
        int sx = gx + (int)((a + SR) / (2.0 * SR) * side);
        int sy = gy + side - (int)((b + SR) / (2.0 * SR) * side);
        HBRUSH cb = CreateSolidBrush(RGB(240, 60, 60));
        HGDIOBJ oc = SelectObject(mem, cb);
        HPEN pp = CreatePen(PS_SOLID, 2, RGB(255, 255, 255));
        HGDIOBJ opp = SelectObject(mem, pp);
        Ellipse(mem, sx - 7, sy - 7, sx + 7, sy + 7);
        SelectObject(mem, opp);
        SelectObject(mem, oc);
        DeleteObject(pp);
        DeleteObject(cb);
    }

    SelectClipRgn(mem, NULL);

    /* 面板标注 */
    SetTextColor(mem, RGB(150, 160, 180));
    TextOutW(mem, gx - 2, gy - 22, pv->space ? L"ω2" : L"θ2", 2);
    TextOutW(mem, gx + side - 22, gy + side + 6, pv->space ? L"ω1" : L"θ1", 2);
    SetTextColor(mem, RGB(95, 105, 125));
    if (pv->space == 0) {
        TextOutW(mem, gx + 4, gy + 2, L"π", 1);
        TextOutW(mem, gx + 4, gy + side - 20, L"-π", 2);
        TextOutW(mem, gx + side - 20, gy + side - 20, L"π", 1);
    } else {
        TextOutW(mem, gx + 4, gy + 2, L"25", 2);
        TextOutW(mem, gx + 4, gy + side - 20, L"-25", 3);
        TextOutW(mem, gx + side - 24, gy + side - 20, L"25", 2);
    }

    /* ---------------- 状态图下方：x(t) / y(t) 波形 ---------------- */
    {
        int wy0 = gy + side + 30;
        int wh = ch - wy0 - 44;
        if (wh >= 80) {
            int each = (wh - 8) / 2;
            int nshow = (int)(SCOPE_SEC * SND_HZ);
            draw_wave(mem, gx, wy0, side, each, pv->sndL, pv->sndHead, pv->sndCount,
                      nshow, RGB(255, 170, 60), pv->space ? L"x(t) = ω1" : L"x(t) = θ1");
            draw_wave(mem, gx, wy0 + each + 8, side, each, pv->sndR, pv->sndHead, pv->sndCount,
                      nshow, RGB(120, 200, 255), pv->space ? L"y(t) = ω2" : L"y(t) = θ2");
        }
    }
    if (pv->space == 0) TextOutW(mem, gx + side - 20, gy + 2, L"π", 1);
    else                TextOutW(mem, gx + side - 24, gy + 2, L"25", 2);

    /* 左下信息 */
    SetTextColor(mem, RGB(210, 215, 225));
    if (pv->space == 0)
        swprintf(buf, 256, L"初始 (θ1, θ2) = (%.5f, %.5f)    (ω1, ω2) = (%.5f, %.5f)",
                 wrap_angle(pv->th1_0), wrap_angle(pv->th2_0), pv->w1_0, pv->w2_0);
    else
        swprintf(buf, 256, L"初始 (θ1, θ2) = (0, 0)    (ω1, ω2) = (%.5f, %.5f)",
                 pv->w1_0, pv->w2_0);
    TextOutW(mem, 16, ch - 52, buf, (int)wcslen(buf));

    swprintf(buf, 256, L"空格暂停  ↑↓调速  ←→回看  R重置  P播放  M模式  []倍率  T导出   E = %.6f", e0);
    TextOutW(mem, 16, ch - 28, buf, (int)wcslen(buf));

    OP("paint:blt");
    BitBlt(hdc, 0, 0, cw, ch, mem, 0, 0, SRCCOPY);
    SelectObject(mem, old);
    DeleteObject(bmp);
    DeleteDC(mem);

    /* 保存声音按钮（画在最终位图上，避免被覆盖） */
    {
        RECT rb;
        rb.left = cw - 152; rb.top = ch - 40;
        rb.right = cw - 12; rb.bottom = ch - 8;
        HBRUSH b1 = CreateSolidBrush(RGB(18, 30, 44));
        FillRect(hdc, &rb, b1);
        DeleteObject(b1);
        HPEN p1 = CreatePen(PS_SOLID, 1, RGB(52, 104, 140));
        HGDIOBJ o1 = SelectObject(hdc, p1);
        HGDIOBJ o2 = SelectObject(hdc, GetStockObject(NULL_BRUSH));
        Rectangle(hdc, rb.left, rb.top, rb.right, rb.bottom);
        SelectObject(hdc, o2); SelectObject(hdc, o1);
        DeleteObject(p1);
        g_audioBtn = rb;
        SetBkMode(hdc, TRANSPARENT);
        SetTextColor(hdc, RGB(190, 230, 255));
        DrawTextW(hdc, L"保存声音", -1, &rb,
                  DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    }

}

LRESULT CALLBACK PendWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    PendView *pv = (PendView *)GetWindowLongPtrW(hwnd, GWLP_USERDATA);

    switch (msg) {
    case WM_CREATE: {
        CREATESTRUCTW *cs = (CREATESTRUCTW *)lParam;
        pv = (PendView *)cs->lpCreateParams;
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, (LONG_PTR)pv);
        SetTimer(hwnd, 1, 16, NULL);
        logf_("pend WM_CREATE pv=%p sizeof(PendView)=%lu", (void *)pv,
              (unsigned long)sizeof(PendView));
        return 0;
    }
    case WM_TIMER: {
        if (pv) {
            OP("timer:advance");
            pend_advance(pv);
            if (g_au.running) {
                audio_pump();
                if (g_au.mode == 0 && ++g_au.refresh >= 120) {
                    g_au.refresh = 0;
                    audio_make_table(pv);
                }
            }
            if (((++s_pendFrames) % 120) == 0)
                logf_("tick t=%.1f speed=%.2f trail=%d ptrail=%d",
                      pv->t, pv->speed, pv->trailCount, pv->pCount);
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;
    }
    case WM_SIZE:
        logf_("pend WM_SIZE %dx%d", LOWORD(lParam), HIWORD(lParam));
        return 0;
    case WM_ENTERSIZEMOVE:
        logf_("pend ENTERSIZEMOVE");
        return 0;
    case WM_EXITSIZEMOVE:
        logf_("pend EXITSIZEMOVE");
        return 0;
    case WM_PAINT: {
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        if (pv) pend_paint(hwnd, pv, hdc);
        EndPaint(hwnd, &ps);
        return 0;
    }
    case WM_ERASEBKGND:
        return 1;
    case WM_LBUTTONDOWN: {
        int x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
        RECT rc0; GetClientRect(hwnd, &rc0);
        logf_("pend click %d,%d client=%ldx%ld btn=(%ld,%ld,%ld,%ld)", x, y,
              (long)rc0.right, (long)rc0.bottom,
              (long)g_audioBtn.left, (long)g_audioBtn.top,
              (long)g_audioBtn.right, (long)g_audioBtn.bottom);
        /* 右下角区域（不依赖 paint 是否已赋值）*/
        if (x >= rc0.right - 160 && y >= rc0.bottom - 48) {
            logf_("export_audio triggered");
            if (pv) export_audio(hwnd, pv->th1_0, pv->th2_0, pv->w1_0, pv->w2_0);
            return 0;
        }
        return 0;
    }

    case WM_COMMAND:
        if (LOWORD(wParam) == IDC_PAUDIO && HIWORD(wParam) == BN_CLICKED) {
            if (pv) export_audio(hwnd, pv->th1_0, pv->th2_0, pv->w1_0, pv->w2_0);
            return 0;
        }
        return 0;

    case WM_KEYDOWN:
        if (!pv) return 0;
        if (wParam == 'T' || (wParam == 'S' && (GetKeyState(VK_CONTROL) & 0x8000))) {
            export_trajectory(hwnd, pv->th1_0, pv->th2_0, pv->w1_0, pv->w2_0);
            return 0;
        }
        if (wParam == VK_SPACE) { pv->paused = !pv->paused; if (!pv->paused) pv->cursor = -1; InvalidateRect(hwnd, NULL, FALSE); }
        else if (wParam == VK_LEFT || wParam == VK_RIGHT) {
            if (pv->trailCount > 1) {
                if (pv->cursor < 0) pv->cursor = pv->trailCount - 1;
                /* 普通 0.01 s / Shift 0.1 s / Ctrl 1 s（轨迹 500 帧每秒） */
                int step = 5;
                if (GetKeyState(VK_SHIFT)   & 0x8000) step = 50;
                if (GetKeyState(VK_CONTROL) & 0x8000) step = 500;
                pv->cursor += (wParam == VK_RIGHT) ? step : -step;
                if (pv->cursor < 0) pv->cursor = 0;
                if (pv->cursor >= pv->trailCount) pv->cursor = pv->trailCount - 1;
                pv->paused = TRUE;
                InvalidateRect(hwnd, NULL, FALSE);
            }
            return 0;
        }
        else if (wParam == VK_UP)   { pv->speed *= 1.5; if (pv->speed > 8.0) pv->speed = 8.0; }
        else if (wParam == VK_DOWN) { pv->speed /= 1.5; if (pv->speed < 0.1) pv->speed = 0.1; }
        else if (wParam == 'P') {
            if (g_au.running) audio_stop();
            else {
                audio_make_table(pv);
                if (audio_start(pv) != 0)
                    MessageBoxW(hwnd, L"无法打开音频设备", L"播放失败", MB_OK);
            }
            InvalidateRect(hwnd, NULL, FALSE);
        }
        else if (wParam == 'M') {
            int was = g_au.running;
            if (was) audio_stop();
            g_au.mode = !g_au.mode;
            audio_make_table(pv);
            if (was && audio_start(pv) != 0)
                MessageBoxW(hwnd, L"无法打开音频设备", L"播放失败", MB_OK);
            InvalidateRect(hwnd, NULL, FALSE);
        }
        else if (wParam == VK_OEM_4 || wParam == VK_OEM_MINUS) {
            if (g_au.mode == 0) { g_sndSpeed /= 1.5; if (g_sndSpeed < 27.5) g_sndSpeed = 27.5; }
            else { if (!g_au.running) audio_make_table(pv);
                   g_au.pitch /= 1.5; if (g_au.pitch < 55.0) g_au.pitch = 55.0; }
            InvalidateRect(hwnd, NULL, FALSE);
        }
        else if (wParam == VK_OEM_6 || wParam == VK_OEM_PLUS) {
            if (g_au.mode == 0) { g_sndSpeed *= 1.5; if (g_sndSpeed > 3520.0) g_sndSpeed = 3520.0; }
            else { if (!g_au.running) audio_make_table(pv);
                   g_au.pitch *= 1.5; if (g_au.pitch > 1760.0) g_au.pitch = 1760.0; }
            InvalidateRect(hwnd, NULL, FALSE);
        }
        else if (wParam == 'R') {
            pv->s.th1 = pv->th1_0; pv->s.th2 = pv->th2_0;
            pv->s.w1 = pv->w1_0;   pv->s.w2 = pv->w2_0;
            pv->t = 0.0; pv->acc = 0.0;
            pv->trailHead = pv->trailCount = 0;
            pv->pHead = pv->pCount = 0;
            pv->uw1 = pv->th1_0; pv->uw2 = pv->th2_0;
            pv->sndHead = pv->sndCount = 0; pv->sndAcc = 0.0;
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;
    case WM_DESTROY: {
        KillTimer(hwnd, 1);
        audio_stop();
        if (pv) {
            OP("pend:destroy");
            logf_("pend WM_DESTROY trail=%d ptrail=%d", pv->trailCount, pv->pCount);
            if (pv->bgBmp) DeleteObject(pv->bgBmp);
            free(pv);
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, 0);
        }
        return 0;
    }
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

void open_pendulum_window(double th1, double th2)
{
    PendView *pv = (PendView *)calloc(1, sizeof(PendView));
    if (!pv) return;
    pv->space = g_phys.space;
    if (pv->space == 0) {
        /* 角度空间：横轴 θ1、纵轴 θ2；初速度取高级选项里的 ω1/ω2 */
        pv->th1_0 = th1; pv->th2_0 = th2;
        pv->w1_0 = g_phys.w1_0; pv->w2_0 = g_phys.w2_0;
    } else {
        /* 角动量空间：横轴 ω1、纵轴 ω2；初始角度取高级选项里的 θ1/θ2 */
        pv->th1_0 = g_phys.th1_0; pv->th2_0 = g_phys.th2_0;
        pv->w1_0 = th1;           pv->w2_0 = th2;
    }
    pv->s.th1 = pv->th1_0; pv->s.th2 = pv->th2_0;
    pv->s.w1 = pv->w1_0;   pv->s.w2 = pv->w2_0;
    pv->uw1 = pv->th1_0; pv->uw2 = pv->th2_0;
    pv->t = 0.0;
    pv->speed = 1.0;
    pv->paused = FALSE;
    pv->cursor = -1;

    WCHAR title[128];
    if (pv->space == 0)
        swprintf(title, 128, L"双摆  θ1=%.4f  θ2=%.4f", wrap_angle(pv->th1_0), wrap_angle(pv->th2_0));
    else
        swprintf(title, 128, L"双摆  ω1=%.4f  ω2=%.4f", pv->w1_0, pv->w2_0);

    HWND h = CreateWindowExW(0, PEND_CLASS, title,
                             WS_OVERLAPPEDWINDOW | WS_VISIBLE,
                             CW_USEDEFAULT, CW_USEDEFAULT, 1180, 880,
                             g_main, NULL, GetModuleHandleW(NULL), pv);
    if (!h) { free(pv); return; }
    SetFocus(h);
}

/* ------------------------------------------------------------------ */
/* 主窗口                                                              */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/* 截图：把当前视图的 λ 场按 1:1 渲染成 BMP，存到桌面                    */
/* ------------------------------------------------------------------ */

/* ================= PNG 输出（GDI+ flat API，无第三方库） ================= */
static int   g_gdip_on = 0;
static ULONG_PTR g_gdip_tok = 0;
static CLSID g_png_clsid;

static void png_init(void)
{
    if (g_gdip_on) return;
    GdiplusStartupInput si;
    memset(&si, 0, sizeof si);
    si.GdiplusVersion = 1;
    if (GdiplusStartup(&g_gdip_tok, &si, NULL) != 0) return;
    g_gdip_on = 1;
    CLSIDFromString(L"{557CF406-1A04-11D3-9A73-0000F81EF32E}", &g_png_clsid);
}

/* rgb 为 w*h*3、自顶向下、RGB 顺序 */
static int save_png_rgb(const WCHAR *path, int w, int h, const unsigned char *rgb)
{
    png_init();
    if (!g_gdip_on) return -1;
    GpBitmap *bmp = NULL;
    if (GdipCreateBitmapFromScan0(w, h, 0, PixelFormat32bppARGB, NULL, &bmp) != 0 || !bmp)
        return -2;
    GpRect rc;
    rc.X = 0; rc.Y = 0; rc.Width = w; rc.Height = h;
    BitmapData bd;
    memset(&bd, 0, sizeof bd);
    if (GdipBitmapLockBits(bmp, &rc, ImageLockModeWrite, PixelFormat32bppARGB, &bd) != 0) {
        GdipDisposeImage((GpImage *)bmp);
        return -3;
    }
    for (int y = 0; y < h; ++y) {
        unsigned char *dst = (unsigned char *)bd.Scan0 + (size_t)y * bd.Stride;
        const unsigned char *src = rgb + (size_t)y * w * 3;
        for (int x = 0; x < w; ++x) {
            dst[x * 4 + 0] = src[x * 3 + 2];
            dst[x * 4 + 1] = src[x * 3 + 1];
            dst[x * 4 + 2] = src[x * 3 + 0];
            dst[x * 4 + 3] = 255;
        }
    }
    GdipBitmapUnlockBits(bmp, &bd);
    int r = (int)GdipSaveImageToFile(bmp, path, &g_png_clsid, NULL);
    GdipDisposeImage((GpImage *)bmp);
    return r;
}


/* ================= 导出轨迹图（PNG，左图右参数） ================= */
/* ===== 保存 10 秒音频（OGG Vorbis，元数据里带初始状态） ===== */
static void export_audio(HWND owner, double th1, double th2, double w1, double w2)
{
    const int SR = 44100, SEC = 10;
    const long NS = (long)SR * SEC;
    const double hstep = 0.001;
    double phys = SEC * (g_sndSpeed > 1.0 ? g_sndSpeed : 440.0);
    long   NT = (long)(phys / hstep);
    if (NT > 20000000L) NT = 20000000L;

    short *buf = (short *)calloc((size_t)NS * 2, sizeof(short));
    if (!buf) { MessageBoxW(owner, L"内存不足", L"保存失败", MB_OK); return; }

    State s = { th1, w1, th2, w2 };
    double prev1 = s.th1, prev2 = s.th2, uw1 = 0.0, uw2 = 0.0;
    double t = 0.0, dtOut = phys / (double)NS, nextOut = 0.0;
    long k = 0;

    for (long i = 0; i < NT && k < NS; ++i) {
        s = rk4(s, hstep);
        uw1 += wrap_angle(s.th1 - prev1); prev1 = s.th1;
        uw2 += wrap_angle(s.th2 - prev2); prev2 = s.th2;
        t += hstep;
        while (t >= nextOut && k < NS) {
            double a = sin(uw1), b = sin(uw2);
            buf[k * 2]     = (short)(a * 26000.0);
            buf[k * 2 + 1] = (short)(b * 26000.0);
            ++k;
            nextOut += dtOut;
        }
    }
    while (k < NS) { buf[k*2] = 0; buf[k*2+1] = 0; ++k; }

    WCHAR dir[MAX_PATH], wavPath[MAX_PATH], oggPath[MAX_PATH], tmpDir[MAX_PATH];
    GetEnvironmentVariableW(L"USERPROFILE", dir, MAX_PATH);
    wcscat(dir, L"\\Desktop");
    GetTempPathW(MAX_PATH, tmpDir);

    SYSTEMTIME st; GetLocalTime(&st);
    swprintf(oggPath, MAX_PATH,
             L"%s\\pendulum_audio_%04d%02d%02d_%02d%02d%02d.ogg", dir,
             st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    swprintf(wavPath, MAX_PATH, L"%schaos_tmp_%lu.wav", tmpDir, GetCurrentThreadId());

    /* 写 16bit 立体声 WAV */
    FILE *f = _wfopen(wavPath, L"wb");
    if (!f) { free(buf); MessageBoxW(owner, L"无法写入临时文件", L"保存失败", MB_OK); return; }
    DWORD dataBytes = (DWORD)(NS * 2 * sizeof(short));
    DWORD riffSize = 36 + dataBytes;
    WORD  fmt = 1, ch = 2, bits = 16;
    DWORD rate = SR, byteRate = SR * 4;
    WORD  align = 4;
    fwrite("RIFF", 1, 4, f); fwrite(&riffSize, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
    DWORD fsz = 16;
    fwrite(&fsz, 4, 1, f); fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f);
    fwrite(&rate, 4, 1, f); fwrite(&byteRate, 4, 1, f);
    fwrite(&align, 2, 1, f); fwrite(&bits, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&dataBytes, 4, 1, f);
    fwrite(buf, 1, dataBytes, f);
    fclose(f);
    free(buf);

    /* 调用 ffmpeg 转 OGG Vorbis，并把初始状态写进注释 */
    WCHAR cmd[2048];
    swprintf(cmd, 2048,
        L"ffmpeg -y -loglevel error -i \"%s\" -c:a libvorbis -q:a 5 "
        L"-metadata \"CHAOS_TH1=%.12f\" -metadata \"CHAOS_TH2=%.12f\" "
        L"-metadata \"CHAOS_W1=%.12f\" -metadata \"CHAOS_W2=%.12f\" "
        L"-metadata \"CHAOS_M1=%.12f\" -metadata \"CHAOS_M2=%.12f\" "
        L"-metadata \"CHAOS_L1=%.12f\" -metadata \"CHAOS_L2=%.12f\" "
        L"-metadata \"CHAOS_G=%.12f\" -metadata \"CHAOS_SPACE=%d\" "
        L"-metadata \"CHAOS_SPEED=%.3f\" \"%s\"",
        wavPath, th1, th2, w1, w2, g_phys.m1, g_phys.m2,
        g_phys.l1, g_phys.l2, g_phys.g, g_phys.space, g_sndSpeed, oggPath);

    STARTUPINFOW si; PROCESS_INFORMATION pi;
    memset(&si, 0, sizeof si); si.cb = sizeof si;
    si.dwFlags = STARTF_USESHOWWINDOW; si.wShowWindow = SW_HIDE;
    BOOL ok = CreateProcessW(NULL, cmd, NULL, NULL, FALSE,
                             CREATE_NO_WINDOW, NULL, NULL, &si, &pi);
    int rc = -1;
    if (ok) {
        WaitForSingleObject(pi.hProcess, 60000);
        DWORD code = 1;
        GetExitCodeProcess(pi.hProcess, &code);
        rc = (int)code;
        CloseHandle(pi.hProcess); CloseHandle(pi.hThread);
    }

    WCHAR msg[512];
    if (ok && rc == 0) {
        DeleteFileW(wavPath);
        swprintf(msg, 512, L"已保存 10 秒音频（OGG）\n%d 秒音频 = %.0f 秒物理轨迹\n初始状态已写入文件注释\n\n%s",
                 SEC, phys, oggPath);
        MessageBoxW(owner, msg, L"保存成功", MB_OK | MB_ICONINFORMATION);
    } else {
        /* 找不到 ffmpeg：保留未压缩的 WAV，至少能用 */
        WCHAR wavDst[MAX_PATH];
        swprintf(wavDst, MAX_PATH, L"%s\\pendulum_audio.wav", dir);
        MoveFileExW(wavPath, wavDst, MOVEFILE_REPLACE_EXISTING);
        swprintf(msg, 512, L"未找到 ffmpeg，已保存为 WAV（未压缩）\n\n%s\n\n"
                 L"安装 ffmpeg 后可转 OGG：winget install Gyan.FFmpeg", wavDst);
        MessageBoxW(owner, msg, L"已保存（WAV）", MB_OK | MB_ICONINFORMATION);
    }
}

static void export_trajectory(HWND owner, double th1, double th2, double w1, double w2)
{
    const double TT = 60.0, HSTEP = 0.001, SAMPLE = 0.01;
    const int NS = (int)(TT / SAMPLE + 0.5) + 1;
    double *px = (double *)malloc(sizeof(double) * (size_t)NS);
    double *py = (double *)malloc(sizeof(double) * (size_t)NS);
    if (!px || !py) { free(px); free(py); MessageBoxW(owner, L"内存不足", L"错误", MB_OK); return; }

    State s;
    s.th1 = th1; s.w1 = w1; s.th2 = th2; s.w2 = w2;
    const double e0 = energy(&s);
    const int sub = (int)(SAMPLE / HSTEP + 0.5);
    double mnx = 1e30, mxx = -1e30, mny = 1e30, mxy = -1e30;

    for (int k = 0; k < NS; ++k) {
        const double x1 = g_phys.l1 * sin(s.th1);
        const double y1 = -g_phys.l1 * cos(s.th1);
        const double x2 = x1 + g_phys.l2 * sin(s.th2);
        const double y2 = y1 - g_phys.l2 * cos(s.th2);
        px[k] = x2; py[k] = y2;
        if (x2 < mnx) mnx = x2;
        if (x2 > mxx) mxx = x2;
        if (y2 < mny) mny = y2;
        if (y2 > mxy) mxy = y2;
        for (int m = 0; m < sub; ++m) s = rk4(s, HSTEP);
    }
    const double e1 = energy(&s);

    const int CW = 1600, CH = 900;
    const int LX = 40, LY = 40, LW = 1060, LH = 820;

    HDC screen = GetDC(NULL);
    HDC mem = CreateCompatibleDC(screen);

    BITMAPINFO bi;
    memset(&bi, 0, sizeof bi);
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = CW;
    bi.bmiHeader.biHeight = -CH;
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;

    void *bits = NULL;
    HBITMAP dib = CreateDIBSection(screen, &bi, DIB_RGB_COLORS, &bits, NULL, 0);
    HGDIOBJ oldbm = SelectObject(mem, dib);
    ReleaseDC(NULL, screen);
    if (!dib || !bits) {
        if (oldbm) SelectObject(mem, oldbm);
        DeleteDC(mem);
        free(px); free(py);
        MessageBoxW(owner, L"创建位图失败", L"错误", MB_OK);
        return;
    }

    RECT full = { 0, 0, CW, CH };
    HBRUSH bg = CreateSolidBrush(RGB(8, 11, 18));
    FillRect(mem, &full, bg);
    DeleteObject(bg);

    HPEN bp = CreatePen(PS_SOLID, 1, RGB(36, 46, 64));
    HGDIOBJ obp = SelectObject(mem, bp);
    HGDIOBJ obr = SelectObject(mem, GetStockObject(NULL_BRUSH));
    Rectangle(mem, LX, LY, LX + LW, LY + LH);
    SelectObject(mem, obr);
    SelectObject(mem, obp);
    DeleteObject(bp);

    double wspan = mxx - mnx, hspan = mxy - mny;
    if (wspan < 1e-9) wspan = 1e-9;
    if (hspan < 1e-9) hspan = 1e-9;
    const double pad = 0.04;
    double scale = (LW * (1 - 2 * pad)) / wspan;
    double sc2   = (LH * (1 - 2 * pad)) / hspan;
    if (sc2 < scale) scale = sc2;
    const double ox = LX + LW * 0.5 - (mnx + mxx) * 0.5 * scale;
    const double oy = LY + LH * 0.5 + (mny + mxy) * 0.5 * scale;

    for (int k = 0; k + 1 < NS; ++k) {
        const double t = (double)k / (double)(NS - 1);
        const int r = (int)(40 + 150 * t);
        const int g = (int)(70 + 160 * t);
        const int b = (int)(150 + 105 * t);
        HPEN pen = CreatePen(PS_SOLID, 1, RGB(r, g, b));
        HGDIOBJ op = SelectObject(mem, pen);
        MoveToEx(mem, (int)(ox + px[k] * scale), (int)(oy - py[k] * scale), NULL);
        LineTo(mem, (int)(ox + px[k + 1] * scale), (int)(oy - py[k + 1] * scale));
        SelectObject(mem, op);
        DeleteObject(pen);
    }

    /* 起点红点、终点绿点，以及初始末端速度方向箭头 */
    {
        const double L1 = g_phys.l1, L2 = g_phys.l2;
        double ex0 = L1 * sin(th1) + L2 * sin(th2);
        double ey0 = -L1 * cos(th1) - L2 * cos(th2);
        double ex1 = px[NS - 1], ey1 = py[NS - 1];
        int sx0 = (int)(ox + ex0 * scale), sy0 = (int)(oy - ey0 * scale);
        int sx1 = (int)(ox + ex1 * scale), sy1 = (int)(oy - ey1 * scale);

        HBRUSH brR = CreateSolidBrush(RGB(235, 60, 60));
        HBRUSH brG = CreateSolidBrush(RGB(60, 220, 110));
        HGDIOBJ ob0 = SelectObject(mem, brR);
        Ellipse(mem, sx0 - 7, sy0 - 7, sx0 + 7, sy0 + 7);
        SelectObject(mem, brG);
        Ellipse(mem, sx1 - 7, sy1 - 7, sx1 + 7, sy1 + 7);
        SelectObject(mem, ob0);
        DeleteObject(brR); DeleteObject(brG);

        /* 初始末端速度方向：d/dt 末端位置 */
        double vx = L1 * cos(th1) * w1 + L2 * cos(th2) * w2;
        double vy = L1 * sin(th1) * w1 + L2 * sin(th2) * w2;
        double vn = sqrt(vx * vx + vy * vy);
        if (vn > 1e-9) {
            double ux = vx / vn, uy = vy / vn;
            double len = 70.0;
            int ax = (int)(sx0 + ux * len), ay = (int)(sy0 - uy * len);
            HPEN ap = CreatePen(PS_SOLID, 3, RGB(235, 60, 60));
            HGDIOBJ oap = SelectObject(mem, ap);
            MoveToEx(mem, sx0, sy0, NULL); LineTo(mem, ax, ay);
            /* 箭头两翼 */
            double nx = -uy, ny = ux;
            MoveToEx(mem, ax, ay, NULL);
            LineTo(mem, (int)(ax - ux * 16 + nx * 9), (int)(ay + uy * 16 - ny * 9));
            MoveToEx(mem, ax, ay, NULL);
            LineTo(mem, (int)(ax - ux * 16 - nx * 9), (int)(ay + uy * 16 + ny * 9));
            SelectObject(mem, oap);
            DeleteObject(ap);
        }

        /* 终点（t=60s）处的速度方向，绿色箭头 */
        if (NS >= 2) {
            double dvx = px[NS - 1] - px[NS - 2];
            double dvy = py[NS - 1] - py[NS - 2];
            double dn = sqrt(dvx * dvx + dvy * dvy);
            if (dn > 1e-12) {
                double ux = dvx / dn, uy = dvy / dn;
                double len = 70.0;
                int ax = (int)(sx1 + ux * len), ay = (int)(sy1 - uy * len);
                HPEN gp2 = CreatePen(PS_SOLID, 3, RGB(60, 220, 110));
                HGDIOBJ ogp = SelectObject(mem, gp2);
                MoveToEx(mem, sx1, sy1, NULL); LineTo(mem, ax, ay);
                double nx = -uy, ny = ux;
                MoveToEx(mem, ax, ay, NULL);
                LineTo(mem, (int)(ax - ux * 16 + nx * 9), (int)(ay + uy * 16 - ny * 9));
                MoveToEx(mem, ax, ay, NULL);
                LineTo(mem, (int)(ax - ux * 16 - nx * 9), (int)(ay + uy * 16 + ny * 9));
                SelectObject(mem, ogp);
                DeleteObject(gp2);
            }
        }
    }

    const int RX = 1130;
    HFONT fT = CreateFontW(22, 0, 0, 0, FW_BOLD, 0, 0, 0, DEFAULT_CHARSET,
                           0, 0, CLEARTYPE_QUALITY, 0, L"微软雅黑");
    HFONT fM = CreateFontW(15, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
                           0, 0, CLEARTYPE_QUALITY, 0, L"Consolas");
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(120, 200, 255));
    HGDIOBJ of = SelectObject(mem, fT);
    TextOutW(mem, RX, 34, L"双摆轨迹", 4);
    SetTextColor(mem, RGB(110, 130, 160));
    SelectObject(mem, fM);
    TextOutW(mem, RX, 64, L"t = 0 ~ 60 s", 12);

    WCHAR buf[2048];
    int y = 110;
    SetTextColor(mem, RGB(90, 110, 140));
    TextOutW(mem, RX, y, L"-- 初始状态 (t=0) --", (int)wcslen(L"-- 初始状态 (t=0) --")); y += 24;
    SetTextColor(mem, RGB(210, 225, 245));
    swprintf(buf, 2048, L"th1 = %+.12f", th1); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"th2 = %+.12f", th2); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"w1  = %+.12f", w1);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"w2  = %+.12f", w2);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 30;

    SetTextColor(mem, RGB(90, 110, 140));
    TextOutW(mem, RX, y, L"-- 物理参数 --", (int)wcslen(L"-- 物理参数 --")); y += 24;
    SetTextColor(mem, RGB(210, 225, 245));
    swprintf(buf, 2048, L"m1 = %.12f", g_phys.m1); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"m2 = %.12f", g_phys.m2); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"L1 = %.12f", g_phys.l1); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"L2 = %.12f", g_phys.l2); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"g  = %.12f", g_phys.g);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 30;

    SetTextColor(mem, RGB(90, 110, 140));
    TextOutW(mem, RX, y, L"-- 积分设置 --", (int)wcslen(L"-- 积分设置 --")); y += 24;
    SetTextColor(mem, RGB(210, 225, 245));
    TextOutW(mem, RX, y, L"RK4, fixed step", (int)wcslen(L"RK4, fixed step")); y += 20;
    swprintf(buf, 2048, L"h    = %.12f", HSTEP);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"T    = %.12f", TT);     TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"dt   = %.12f", SAMPLE); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"N    = %d", NS);        TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 30;

    SetTextColor(mem, RGB(90, 110, 140));
    y += 12;
    SetTextColor(mem, RGB(120, 200, 255));
    TextOutW(mem, RX, y, L"-- 图上标记 --", (int)wcslen(L"-- 图上标记 --")); y += 24;
    SetTextColor(mem, RGB(235, 60, 60));
    TextOutW(mem, RX, y, L"● 红点：t=0 的末端位置", (int)wcslen(L"● 红点：t=0 的末端位置")); y += 22;
    SetTextColor(mem, RGB(60, 220, 110));
    TextOutW(mem, RX, y, L"● 绿点：t=60s 的末端位置", (int)wcslen(L"● 绿点：t=60s 的末端位置")); y += 22;
    SetTextColor(mem, RGB(235, 60, 60));
    TextOutW(mem, RX, y, L"→ 红箭头：t=0 末端速度方向", (int)wcslen(L"→ 红箭头：t=0 末端速度方向")); y += 22;
    SetTextColor(mem, RGB(60, 220, 110));
    TextOutW(mem, RX, y, L"→ 绿箭头：t=60s 末端速度方向", (int)wcslen(L"→ 绿箭头：t=60s 末端速度方向")); y += 22;
    SetTextColor(mem, RGB(110, 130, 160));
    TextOutW(mem, RX, y, L"-- 能量 --", (int)wcslen(L"-- 能量 --")); y += 24;
    SetTextColor(mem, RGB(210, 225, 245));
    swprintf(buf, 2048, L"E(0)  = %.12f", e0); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"E(60) = %.12f", e1); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"dE    = %+.6e", e1 - e0); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;

    SelectObject(mem, of);
    DeleteObject(fT);
    DeleteObject(fM);

    unsigned char *rgb = (unsigned char *)malloc((size_t)CW * CH * 3);
    if (rgb) {
        const unsigned char *src = (const unsigned char *)bits;
        for (int yy = 0; yy < CH; ++yy) {
            const unsigned char *row = src + (size_t)yy * CW * 4;
            unsigned char *dst = rgb + (size_t)yy * CW * 3;
            for (int xx = 0; xx < CW; ++xx) {
                dst[xx * 3 + 0] = row[xx * 4 + 2];
                dst[xx * 3 + 1] = row[xx * 4 + 1];
                dst[xx * 3 + 2] = row[xx * 4 + 0];
            }
        }
        WCHAR desk[MAX_PATH] = L"";
        if (!GetEnvironmentVariableW(L"USERPROFILE", desk, MAX_PATH)) wcscpy(desk, L".");
        SYSTEMTIME st;
        GetLocalTime(&st);
        WCHAR path[MAX_PATH];
        swprintf(path, MAX_PATH, L"%s\\Desktop\\trajectory_%04d%02d%02d_%02d%02d%02d.png",
                 desk, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
        if (save_png_rgb(path, CW, CH, rgb) == 0) {
            WCHAR msg[MAX_PATH + 120];
            swprintf(msg, MAX_PATH + 120, L"Trajectory saved:\n%s", path);
            MessageBoxW(owner, msg, L"导出成功", MB_OK | MB_ICONINFORMATION);
        } else {
            MessageBoxW(owner, L"PNG 编码失败。", L"错误", MB_OK | MB_ICONERROR);
        }
        free(rgb);
    }
    SelectObject(mem, oldbm);
    DeleteObject(dib);
    DeleteDC(mem);
    free(px);
    free(py);
}
