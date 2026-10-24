#include "app.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <commctrl.h>
#include <stdint.h>
#include "cl_gpu.h"

/* 主窗口全局 */
extern Phys g_phys;

char g_clogOp[160];
Phys g_phys = { 1.0, 1.0, 1.0, 1.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0 };



/*
 * 双摆混沌地图 —— 交互式图形界面
 * ============================================================
 * 主窗口：把 θ1,θ2 ∈ [-π,π]² 的每个初值点（静止释放）着色，
 *         颜色由最大李雅普诺夫指数 λ 决定：有序=黑，混沌=白。
 * 交互：
 *   左键拖框   -> 以框选区为新范围重新模拟（按当前精度/时长）
 *   Z          -> 回到上一个视图
 *   左键双击   -> 打开该点对应的双摆动画（无时间上限，关窗即停）
 *
 * 编译：
 *   gcc -O3 -march=native -fopenmp chaosmap.c -o ChaosMap.exe \
 *       -mwindows -lgdi32 -lcomctl32 -luser32 -lm
 */
#define WIN32_LEAN_AND_MEAN
#define UNICODE
#define _UNICODE
#include <windows.h>
#include <objbase.h>
#include <propidl.h>
#include <gdiplus.h>
#include <windowsx.h>
#include <commctrl.h>
#include <wchar.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <stdarg.h>
#include <mmsystem.h>
#include "cl_gpu.h"

/* 调试日志：只有定义了 UI_DEBUG 才写入 ui_log.txt */
#ifdef UI_DEBUG
static void dbg_log(const char *fmt, ...)
{
    FILE *f = fopen("ui_log.txt", "a");
    if (!f) return;
    SYSTEMTIME st; GetLocalTime(&st);
    fprintf(f, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
    fprintf(f, "\n");
    fclose(f);
}
#define LOG(...) dbg_log(__VA_ARGS__)
#else
#define LOG(...) ((void)0)
#endif

#define MPI 3.14159265358979323846

/* ------------------------------------------------------------------ */
/* 物理                                                                */
/* ------------------------------------------------------------------ */




/* 前置声明：双摆窗口 Ctrl+S 调用 */
static void export_trajectory(HWND owner, double th1, double th2, double w1, double w2);


/* 双摆运动方程右端 */
static inline double lyapunov(double th1, double w1_0, double th2, double w2_0,
                              long nsteps, double h, double delta0)
{
    State a, b;
    double acc = 0.0;

    a.th1 = th1; a.w1 = w1_0; a.th2 = th2; a.w2 = w2_0;
    b = a; b.th1 += delta0;

    for (long n = 0; n < nsteps; ++n) {
        a = rk4(a, h);
        b = rk4(b, h);

        const double dth1 = b.th1 - a.th1;
        const double dw1  = b.w1  - a.w1;
        const double dth2 = b.th2 - a.th2;
        const double dw2  = b.w2  - a.w2;
        const double d = sqrt(dth1 * dth1 + dw1 * dw1
                              + dth2 * dth2 + dw2 * dw2);

        if (!(d > 0.0) || !isfinite(d)) return 0.0;   /* 数值塌缩，视为有序 */

        acc += log(d / delta0);

        const double sc = delta0 / d;
        b.th1 = a.th1 + dth1 * sc;
        b.w1  = a.w1  + dw1  * sc;
        b.th2 = a.th2 + dth2 * sc;
        b.w2  = a.w2  + dw2  * sc;
    }
    return acc / (nsteps * h);
}

/* ------------------------------------------------------------------ */
/* 配色：黑 -> 深蓝 -> 蓝 -> 亮蓝 -> 白                                 */
/* ------------------------------------------------------------------ */

static const double STOPS[8][4] = {
    { 0.00, 0.000, 0.000, 0.015 },
    { 0.15, 0.004, 0.008, 0.100 },
    { 0.32, 0.008, 0.030, 0.330 },
    { 0.50, 0.020, 0.090, 0.620 },
    { 0.66, 0.060, 0.220, 0.820 },
    { 0.80, 0.260, 0.470, 0.950 },
    { 0.90, 0.620, 0.780, 1.000 },
    { 1.00, 1.000, 1.000, 1.000 },
};

static inline void cmap(double t, BYTE *r, BYTE *g, BYTE *b)
{
    int k = 0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    while (k < 6 && t > STOPS[k + 1][0]) ++k;
    const double f = (t - STOPS[k][0]) / (STOPS[k + 1][0] - STOPS[k][0]);
    double rr = STOPS[k][1] + f * (STOPS[k + 1][1] - STOPS[k][1]);
    double gg = STOPS[k][2] + f * (STOPS[k + 1][2] - STOPS[k][2]);
    double bb = STOPS[k][3] + f * (STOPS[k + 1][3] - STOPS[k][3]);
    *r = (BYTE)(255.0 * rr + 0.5);
    *g = (BYTE)(255.0 * gg + 0.5);
    *b = (BYTE)(255.0 * bb + 0.5);
}

/* λ -> 像素颜色。hi 是映射到纯白的 λ，gain 是 gamma */
static const WCHAR *g_helpText =
    L"主窗口\n"
    L"  左键拖框    放大到选区并重新计算\n"
    L"  Z           返回上一视图\n"
    L"  Home        回到全域\n"
    L"  Ctrl+滚轮   纯视觉缩放（不重算）\n"
    L"  右键拖动    缩放后平移画面\n"
    L"  双击        打开该点对应的双摆\n"
    L"\n"
    L"顶部\n"
    L"  精度        每边的采样点数（64~16384）\n"
    L"  时长        每个双摆的演化时间（秒）\n"
    L"  单/高/双精度  单精度最快；高精度用 float-float；双精度最慢但最准\n"
    L"  快速模式    fast-math 内核，快约 3.7 倍\n"
    L"  锐度        越大，混沌/有序的分界越陡（0.2~8）\n"
    L"  高级选项    质量、摆长、重力、初始角速度/角度、参数空间\n"
    L"\n"
    L"图像含义：黑 = 有序（准周期），白 = 混沌，蓝 = 过渡\n"
    L"\n"
    L"双摆窗口\n"
    L"  空格        暂停 / 继续\n"
    L"  上下箭头    调速\n"
    L"  左右箭头    回看（Shift 十倍，Ctrl 百倍）\n"
    L"  R           重置\n"
    L"  P           播放 / 停止声音\n"
    L"  M           切换声频模式\n"
    L"  [ ]         调整加速倍率 / 音高\n"
    UI(L"  T           导出当前轨迹图到桌面", L"  T           Export trajectory to Desktop");



static double g_sharp = 1.0;   /* 锐度：>1 分界更陡，1.0 原始 */

static inline void lambda_to_rgb(float lam, double hi, double gain,
                                 BYTE *r, BYTE *g, BYTE *b)
{
    double t = (double)lam / hi;
    if (t <= 0.0) t = 0.0;
    else if (t >= 1.0) t = 1.0;
    else t = pow(t, gain);
    t = 0.5 + (t - 0.5) * g_sharp;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
    cmap(t, r, g, b);
}

/* ------------------------------------------------------------------ */
/* 视图                                                                */
/* ------------------------------------------------------------------ */

#define MAXSTACK 32
#define VMAP_HI     1.0     /* λ 映射到白色的阈值 */
#define VMAP_GAMMA  0.7

typedef struct {
    double x0, x1;          /* 横轴范围 */
    double y0, y1;          /* 纵轴范围 */
    int W, H;               /* 位图分辨率 */
    double T;               /* 演化时长 */
    int space;              /* 0 = 角度空间；1 = 角动量空间 */
    float *lambda;          /* W*H 的 λ */
    HBITMAP dib;
    BYTE *bits;
} View;

HWND g_main = NULL;
static View g_cur;
static View g_stack[MAXSTACK];
static int  g_sp = 0;

static volatile LONG g_cancel = 0;      /* 计算取消标志 */
static volatile LONG g_computing = 0;
static HANDLE g_thread = NULL;
static DWORD  g_startTick = 0;

/* 显示区与图像显示矩形（屏幕坐标） */
static int g_dispX, g_dispY, g_dispW, g_dispH;
static int g_imgX, g_imgY, g_imgW, g_imgH;
static int g_panning = 0, g_panX = 0, g_panY = 0;
static WCHAR g_lastTitle[200] = {0};

/* 控件 */
#define IDC_RES     1001
#define IDC_TIME    1002
#define IDC_APPLY   1003
#define IDC_PROG    1004
#define IDC_STATUS  1005
#define IDC_PREC32  1006
#define IDC_FAST    1007
#define IDC_PRECFF  1008
#define IDC_PRECD   1009
#define IDC_SNAP    1010
#define IDC_ADV     1011
#define IDC_SHARP   1012
#define IDC_ZOOMIN  1013
#define IDC_ZOOMOUT 1014
#define IDC_ZOOMRST 1015
#define IDC_HELPBOX    1016

/* 高级选项窗口 */
#define IDC_ADV_M1   1101
#define IDC_ADV_M2   1102
#define IDC_ADV_L1   1103
#define IDC_ADV_L2   1104
#define IDC_ADV_G    1105
#define IDC_ADV_W1   1106
#define IDC_ADV_W2   1107
#define IDC_ADV_SP0  1110
#define IDC_ADV_SP1  1111
#define IDC_ADV_OK   1112
#define IDC_ADV_DEF  1113
#define IDC_ADV_CANCEL 1114
/* ------------------------------------------------------------------ */
/* 运行日志：崩溃时用它还原现场                                         */
/* ------------------------------------------------------------------ */

static FILE *g_log = NULL;
static char  g_lastop[160] = "init";
static int   g_frames = 0;

#define OP(s) do { strncpy(g_lastop, (s), sizeof(g_lastop) - 1); g_lastop[sizeof(g_lastop) - 1] = 0; } while (0)

static void log_open(void)
{
    if (g_log) return;
    WCHAR path[MAX_PATH];
    if (!GetModuleFileNameW(NULL, path, MAX_PATH)) wcscpy(path, L"chaosmap_log.txt");
    WCHAR *p = wcsrchr(path, L'\\');
    if (p) wcscpy(p + 1, L"chaosmap_log.txt");
    g_log = _wfopen(path, L"w");
}

void logf_(const char *fmt, ...)
{
    if (!g_log) return;
    SYSTEMTIME st;
    GetLocalTime(&st);
    fprintf(g_log, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(g_log, fmt, ap);
    va_end(ap);
    fputc(10, g_log);
    fflush(g_log);
}

static LONG WINAPI crash_filter(EXCEPTION_POINTERS *ep)
{
    EXCEPTION_RECORD *er = ep->ExceptionRecord;
    if (g_log) {
        fprintf(g_log, "[CRASH] code=0x%08lX addr=%p flags=0x%lX lastop=%s\n",
                (unsigned long)er->ExceptionCode, er->ExceptionAddress,
                (unsigned long)er->ExceptionFlags, g_lastop);
        if (er->NumberParameters >= 2) {
            const char *kind = er->ExceptionInformation[0] == 0 ? "read" :
                               er->ExceptionInformation[0] == 1 ? "write" : "exec";
            fprintf(g_log, "[CRASH] %s access at 0x%llx\n", kind,
                    (unsigned long long)er->ExceptionInformation[1]);
        }
        fprintf(g_log, "[CRASH] t-of-crash recorded above\n");
        fflush(g_log);
    }
    return EXCEPTION_CONTINUE_SEARCH;
}

#define IDC_PREC32  1006
#define IDC_FAST    1007
#define IDC_PRECFF  1008
#define IDC_PRECD   1009
#define IDC_SNAP    1010
static int g_precision = 1;    /* 0=单精度 1=float-float 2=双精度（默认高精度） */
static HWND g_edRes, g_edTime, g_btnApply, g_rb32, g_rbFF, g_rbD, g_chkFast,
            g_btnSnap, g_btnAdv, g_prog, g_status;
static HWND g_edSharp, g_btnZoomIn, g_btnZoomOut, g_btnZoomRst, g_btnHelp;
static HFONT g_font;

/* 鼠标拖框 */
static BOOL g_dragging = FALSE;
/* ---- 科技感配色 ---- */
#define COL_PANEL   RGB(10, 14, 22)
#define COL_EDITBG  RGB(16, 22, 34)
#define COL_TEXT    RGB(160, 200, 235)
#define COL_TEXT2   RGB(120, 170, 220)
#define COL_ACCENT  RGB(0, 200, 255)
static HBRUSH g_brPanel, g_brEdit;
static HFONT  g_fontUI;

static int  g_dragX0, g_dragY0, g_dragX1, g_dragY1;
/* 纯视觉缩放：不重算，只放大显示。1.0 = 原始 */
static double g_zoom = 1.0;
static double g_zoomCX = 0.5, g_zoomCY = 0.5;   /* 缩放中心（原图归一化坐标） */

/* 自定义消息 */
#define WM_APP_PROGRESS (WM_APP + 1)
#define WM_APP_DONE     (WM_APP + 2)
#define WM_APP_CANCELED (WM_APP + 3)

#define CTRL_H 100

static void view_alloc(View *v, double x0, double x1, double y0, double y1,
                       int W, int H, double T)
{
    memset(v, 0, sizeof(*v));
    v->x0 = x0; v->x1 = x1;
    v->y0 = y0; v->y1 = y1;
    v->W = W; v->H = H;
    v->T = T;
    v->space = g_phys.space;

    v->lambda = (float *)calloc((size_t)W * H, sizeof(float));

    BITMAPINFO bi;
    memset(&bi, 0, sizeof(bi));
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = W;
    bi.bmiHeader.biHeight = -H;          /* 自顶向下 */
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 32;
    bi.bmiHeader.biCompression = BI_RGB;

    void *bits = NULL;
    v->dib = CreateDIBSection(NULL, &bi, DIB_RGB_COLORS, &bits, NULL, 0);
    v->bits = (BYTE *)bits;
    if (v->bits) memset(v->bits, 0, (size_t)W * H * 4);
}

static void view_free(View *v)
{
    if (v->dib) DeleteObject(v->dib);
    if (v->lambda) free(v->lambda);
    memset(v, 0, sizeof(*v));
}

/* 从 λ 数组重建整张位图 */
static void view_render(View *v)
{
    if (!v->bits || !v->lambda) return;
    for (int j = 0; j < v->H; ++j) {
        BYTE *row = v->bits + (size_t)j * v->W * 4;
        for (int i = 0; i < v->W; ++i) {
            BYTE r, g, b;
            lambda_to_rgb(v->lambda[(size_t)j * v->W + i],
                          VMAP_HI, VMAP_GAMMA, &r, &g, &b);
            row[i * 4 + 0] = b;
            row[i * 4 + 1] = g;
            row[i * 4 + 2] = r;
            row[i * 4 + 3] = 255;
        }
    }
}

/* ------------------------------------------------------------------ */
/* 后台计算                                                            */
/* ------------------------------------------------------------------ */

/* 把第 jb..je-1 行从 λ 转成颜色写入位图 */
static void render_rows(View *v, int jb, int je)
{
    for (int j = jb; j < je; ++j) {
        BYTE *row = v->bits + (size_t)j * v->W * 4;
        for (int i = 0; i < v->W; ++i) {
            BYTE r, g, b;
            lambda_to_rgb(v->lambda[(size_t)j * v->W + i],
                          VMAP_HI, VMAP_GAMMA, &r, &g, &b);
            row[i * 4 + 0] = b;
            row[i * 4 + 1] = g;
            row[i * 4 + 2] = r;
            row[i * 4 + 3] = 255;
        }
    }
}

/* GPU 每算完一块回调：把新完成的行刷进位图（j0..j1-1） */
static volatile long g_done_rows = 0;

static void gpu_progress_cb(int j0, int j1, void *ud)
{
    View *v = (View *)ud;
    if (j1 > j0) {
        render_rows(v, j0, j1);
        InterlockedExchangeAdd(&g_done_rows, (LONG)(j1 - j0));
    }
    PostMessage(g_main, WM_APP_PROGRESS, (WPARAM)g_done_rows, (LPARAM)v->H);
}

/* GPU 不可用时的单线程回退（很慢，但保证程序能用） */
static void compute_view_cpu(View *v)
{
    const double h = 0.01;
    const double delta0 = 1e-9;
    const long nsteps = (long)llround(v->T / h);

    for (int j = 0; j < v->H; ++j) {
        if (g_cancel) return;
        const double vv = v->y1 - (v->y1 - v->y0) * ((double)j + 0.5) / (double)v->H;
        for (int i = 0; i < v->W; ++i) {
            const double uu = v->x0 + (v->x1 - v->x0) * ((double)i + 0.5) / (double)v->W;
            double th1, th2, w1, w2;
            if (v->space == 0) { th1 = uu; th2 = vv; w1 = g_phys.w1_0; w2 = g_phys.w2_0; }
            else               { th1 = g_phys.th1_0; th2 = g_phys.th2_0; w1 = uu; w2 = vv; }
            v->lambda[(size_t)j * v->W + i] =
                (float)lyapunov(th1, w1, th2, w2, nsteps, h, delta0);
        }
        render_rows(v, j, j + 1);
        PostMessage(g_main, WM_APP_PROGRESS, (WPARAM)(j + 1), (LPARAM)v->H);
    }
}

static void compute_view(View *v)
{
    g_done_rows = 0;

    gpu_set_physics(g_phys.m1, g_phys.m2, g_phys.l1, g_phys.l2, g_phys.g);

    /* space=0：坐标是初始角度，f1/f2 是两摆初始角速度
       space=1：坐标是初始角速度，f1/f2 是固定初始角度（竖直向下 = 0） */
    double f1 = 0.0, f2 = 0.0;
    if (v->space == 0) { f1 = g_phys.w1_0;   f2 = g_phys.w2_0; }
    else               { f1 = g_phys.th1_0;  f2 = g_phys.th2_0; }

    if (gpu_ready()) {
        const double rect[4] = { v->x0, v->x1, v->y0, v->y1 };
        gpu_compute(rect, v->W, v->H, v->T, g_precision, v->space, f1, f2,
                    v->lambda, &g_cancel, gpu_progress_cb, v);
    } else {
        compute_view_cpu(v);
    }
}

static DWORD WINAPI compute_proc(LPVOID p)
{
    View *v = (View *)p;
    compute_view(v);
    if (g_cancel) PostMessage(g_main, WM_APP_CANCELED, 0, 0);
    else          PostMessage(g_main, WM_APP_DONE, 0, 0);
    return 0;
}

static void stop_job(void)
{
    if (g_thread) {
        InterlockedExchange(&g_cancel, 1);
        WaitForSingleObject(g_thread, INFINITE);
        CloseHandle(g_thread);
        g_thread = NULL;
    }
    g_computing = 0;
}

/* 根据像素数、步数、精度粗估耗时（秒） */
static double estimate_seconds(int W, int H, double T, int prec)
{
    double steps = T / 0.01;
    double derivs = (double)W * (double)H * steps * 8.0;
    double rate = (prec == 0) ? 4.3e10 : (prec == 1 ? 1.0e10 : 4.0e9);
    if (g_phys.space == 1 && prec < 1) rate = 1.0e10;
    double sec = derivs / rate;
    if (sec < 0.05) sec = 0.05;
    return sec;
}

static void start_job(void)
{
    stop_job();
    g_cancel = 0;
    g_computing = 1;
    g_startTick = GetTickCount();
    {
        WCHAR b[160];
        double est = estimate_seconds(g_cur.W, g_cur.H, g_cur.T, g_precision);
        swprintf(b, 160, UI(L"预计 %.1f s · 计算中 0%%", L"Est. %.1f s - computing 0%%"), est);
        SetWindowTextW(g_status, b);
    }
    g_thread = CreateThread(NULL, 0, compute_proc, &g_cur, 0, NULL);
    if (!g_thread) g_computing = 0;
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

static void save_snapshot(HWND hwnd)
{
    if (g_computing || !g_cur.lambda) {
        MessageBoxW(hwnd, UI(L"当前还没有算完的图像，等计算结束再保存。", L"Nothing rendered yet, wait for completion."),
                    UI(L"提示", L"Notice"), MB_OK | MB_ICONINFORMATION);
        return;
    }
    const int W = g_cur.W, H = g_cur.H;
    const size_t n = (size_t)W * H;
    unsigned char *rgb = (unsigned char *)malloc(n * 3);
    if (!rgb) { MessageBoxW(hwnd, UI(L"内存不足，无法生成图片。", L"Out of memory, cannot create image."), UI(L"错误", L"Error"), MB_OK | MB_ICONERROR); return; }
    for (size_t k = 0; k < n; ++k) {
        BYTE r, g, b;
        lambda_to_rgb(g_cur.lambda[k], VMAP_HI, VMAP_GAMMA, &r, &g, &b);
        rgb[k * 3 + 0] = r; rgb[k * 3 + 1] = g; rgb[k * 3 + 2] = b;
    }

    WCHAR desk[MAX_PATH] = L"";
    if (!GetEnvironmentVariableW(L"USERPROFILE", desk, MAX_PATH)) wcscpy(desk, L".");
    SYSTEMTIME st;
    GetLocalTime(&st);
    WCHAR path[MAX_PATH];
    swprintf(path, MAX_PATH,
             L"%s\\Desktop\\chaos_%dx%d_%04d%02d%02d_%02d%02d%02d.png",
             desk, W, H, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
    if (save_png_rgb(path, W, H, rgb) != 0) {
        free(rgb);
        MessageBoxW(hwnd, UI(L"PNG 编码失败。", L"PNG encoding failed."), UI(L"错误", L"Error"), MB_OK | MB_ICONERROR);
        return;
    }
    free(rgb);
    WCHAR msg[MAX_PATH + 80];
    swprintf(msg, MAX_PATH + 80, L"已保存 %d×%d（每个像素一个双摆）：\n%s", W, H, path);
    MessageBoxW(hwnd, msg, UI(L"截图已保存", L"Saved"), MB_OK | MB_ICONINFORMATION);
}

/* ================= 导出轨迹图（PNG，左图右参数） ================= */
static void export_trajectory(HWND owner, double th1, double th2, double w1, double w2)
{
    const double TT = 60.0, HSTEP = 0.001, SAMPLE = 0.01;
    const int NS = (int)(TT / SAMPLE + 0.5) + 1;
    double *px = (double *)malloc(sizeof(double) * (size_t)NS);
    double *py = (double *)malloc(sizeof(double) * (size_t)NS);
    if (!px || !py) { free(px); free(py); MessageBoxW(owner, UI(L"内存不足", L"Out of memory"), UI(L"错误", L"Error"), MB_OK); return; }

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
        MessageBoxW(owner, UI(L"创建位图失败", L"Bitmap creation failed"), UI(L"错误", L"Error"), MB_OK);
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
                           0, 0, CLEARTYPE_QUALITY, 0, UI(L"微软雅黑", L"Segoe UI"));
    HFONT fM = CreateFontW(15, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
                           0, 0, CLEARTYPE_QUALITY, 0, L"Consolas");
    SetBkMode(mem, TRANSPARENT);
    SetTextColor(mem, RGB(120, 200, 255));
    HGDIOBJ of = SelectObject(mem, fT);
    TextOutW(mem, RX, 34, UI(L"双摆轨迹", L"Double pendulum trajectory"), 4);
    SetTextColor(mem, RGB(110, 130, 160));
    SelectObject(mem, fM);
    TextOutW(mem, RX, 64, L"t = 0 ~ 60 s", 12);

    WCHAR buf[2048];
    int y = 110;
    SetTextColor(mem, RGB(90, 110, 140));
    TextOutW(mem, RX, y, UI(L"-- 初始状态 (t=0) --", L"-- Initial state (t=0) --"), (int)wcslen(UI(L"-- 初始状态 (t=0) --", L"-- Initial state (t=0) --"))); y += 24;
    SetTextColor(mem, RGB(210, 225, 245));
    swprintf(buf, 2048, L"th1 = %+.12f", th1); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"th2 = %+.12f", th2); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"w1  = %+.12f", w1);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"w2  = %+.12f", w2);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 30;

    SetTextColor(mem, RGB(90, 110, 140));
    TextOutW(mem, RX, y, UI(L"-- 物理参数 --", L"-- Physical parameters --"), (int)wcslen(UI(L"-- 物理参数 --", L"-- Physical parameters --"))); y += 24;
    SetTextColor(mem, RGB(210, 225, 245));
    swprintf(buf, 2048, L"m1 = %.12f", g_phys.m1); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"m2 = %.12f", g_phys.m2); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"L1 = %.12f", g_phys.l1); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"L2 = %.12f", g_phys.l2); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"g  = %.12f", g_phys.g);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 30;

    SetTextColor(mem, RGB(90, 110, 140));
    TextOutW(mem, RX, y, UI(L"-- 积分设置 --", L"-- Integration --"), (int)wcslen(UI(L"-- 积分设置 --", L"-- Integration --"))); y += 24;
    SetTextColor(mem, RGB(210, 225, 245));
    TextOutW(mem, RX, y, L"RK4, fixed step", (int)wcslen(L"RK4, fixed step")); y += 20;
    swprintf(buf, 2048, L"h    = %.12f", HSTEP);  TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"T    = %.12f", TT);     TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"dt   = %.12f", SAMPLE); TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 20;
    swprintf(buf, 2048, L"N    = %d", NS);        TextOutW(mem, RX, y, buf, (int)wcslen(buf)); y += 30;

    SetTextColor(mem, RGB(90, 110, 140));
    y += 12;
    SetTextColor(mem, RGB(120, 200, 255));
    TextOutW(mem, RX, y, UI(L"-- 图上标记 --", L"-- Markers --"), (int)wcslen(UI(L"-- 图上标记 --", L"-- Markers --"))); y += 24;
    SetTextColor(mem, RGB(235, 60, 60));
    TextOutW(mem, RX, y, UI(L"● 红点：t=0 的末端位置", L"o red dot: tip position at t=0"), (int)wcslen(UI(L"● 红点：t=0 的末端位置", L"o red dot: tip position at t=0"))); y += 22;
    SetTextColor(mem, RGB(60, 220, 110));
    TextOutW(mem, RX, y, UI(L"● 绿点：t=60s 的末端位置", L"o green dot: tip position at t=60s"), (int)wcslen(UI(L"● 绿点：t=60s 的末端位置", L"o green dot: tip position at t=60s"))); y += 22;
    SetTextColor(mem, RGB(235, 60, 60));
    TextOutW(mem, RX, y, UI(L"→ 红箭头：t=0 末端速度方向", L"-> red: tip velocity at t=0"), (int)wcslen(UI(L"→ 红箭头：t=0 末端速度方向", L"-> red: tip velocity at t=0"))); y += 22;
    SetTextColor(mem, RGB(60, 220, 110));
    TextOutW(mem, RX, y, UI(L"→ 绿箭头：t=60s 末端速度方向", L"-> green: tip velocity at t=60s"), (int)wcslen(UI(L"→ 绿箭头：t=60s 末端速度方向", L"-> green: tip velocity at t=60s"))); y += 22;
    SetTextColor(mem, RGB(110, 130, 160));
    TextOutW(mem, RX, y, UI(L"-- 能量 --", L"-- Energy --"), (int)wcslen(UI(L"-- 能量 --", L"-- Energy --"))); y += 24;
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
            MessageBoxW(owner, msg, UI(L"导出成功", L"Export done"), MB_OK | MB_ICONINFORMATION);
        } else {
            MessageBoxW(owner, UI(L"PNG 编码失败。", L"PNG encoding failed."), UI(L"错误", L"Error"), MB_OK | MB_ICONERROR);
        }
        free(rgb);
    }
    SelectObject(mem, oldbm);
    DeleteObject(dib);
    DeleteDC(mem);
    free(px);
    free(py);
}

void layout(HWND hwnd)
{
    RECT rc;
    GetClientRect(hwnd, &rc);
    const int cw = rc.right;

    /* 第一行：参数与精度 */
    MoveWindow(g_edRes,    54,  8,  80, 28, TRUE);
    MoveWindow(g_edTime,  198,  8,  80, 28, TRUE);
    MoveWindow(g_btnApply, 286,  8, 100, 30, TRUE);
    MoveWindow(g_rb32,    398, 10,  80, 24, TRUE);
    MoveWindow(g_rbFF,    482, 10,  80, 24, TRUE);
    MoveWindow(g_rbD,     566, 10,  80, 24, TRUE);
    MoveWindow(g_chkFast, 650, 10,  88, 24, TRUE);
    MoveWindow(g_edSharp, 790,  8,  64, 28, TRUE);

    int avail = cw - 866 - 14;
    if (avail < 120) avail = 120;
    int progW = avail * 30 / 100;
    if (progW < 60) progW = 60;
    MoveWindow(g_prog, 866, 10, progW, 24, TRUE);

    /* 第二行：动作按钮 */
    MoveWindow(g_btnSnap,    10, 54,  90, 30, TRUE);
    MoveWindow(g_btnAdv,    108, 54,  90, 30, TRUE);
    MoveWindow(g_btnZoomIn, 206, 54,  76, 30, TRUE);
    MoveWindow(g_btnZoomOut,290, 54,  76, 30, TRUE);
    MoveWindow(g_btnZoomRst,374, 54,  90, 30, TRUE);
    MoveWindow(g_btnHelp,   472, 54,  90, 30, TRUE);
    int statW = cw - 570 - 16;
    if (statW < 120) statW = 120;
    MoveWindow(g_status, 570, 58, statW, 22, TRUE);

    g_dispX = 0;
    g_dispY = CTRL_H;
    g_dispW = cw;
    g_dispH = rc.bottom - CTRL_H;
    if (g_dispH < 1) g_dispH = 1;

    /* 计算图像在显示区中的矩形（保持模拟域宽高比） */
    double aspect = 1.0;
    if (g_cur.x1 > g_cur.x0 && g_cur.y1 > g_cur.y0)
        aspect = (g_cur.x1 - g_cur.x0) / (g_cur.y1 - g_cur.y0);

    double w = g_dispW, h = w / aspect;
    if (h > g_dispH) { h = g_dispH; w = h * aspect; }
    g_imgW = (int)(w + 0.5);
    g_imgH = (int)(h + 0.5);
    if (g_imgW < 1) g_imgW = 1;
    if (g_imgH < 1) g_imgH = 1;
    g_imgX = g_dispX + (g_dispW - g_imgW) / 2;
    g_imgY = g_dispY + (g_dispH - g_imgH) / 2;

    InvalidateRect(hwnd, NULL, FALSE);
}

static void screen_to_sim(int px, int py, double *t1, double *t2)
{
    double fx = (px - g_imgX) / (double)g_imgW;
    double fy = (py - g_imgY) / (double)g_imgH;
    if (fx < 0.0) fx = 0.0; if (fx > 1.0) fx = 1.0;
    if (fy < 0.0) fy = 0.0; if (fy > 1.0) fy = 1.0;
    /* 反掉纯视觉缩放：屏幕归一化 -> 原图归一化 */
    double ox = g_zoomCX + (fx - 0.5) / g_zoom;
    double oy = g_zoomCY + (fy - 0.5) / g_zoom;
    if (ox < 0.0) ox = 0.0; if (ox > 1.0) ox = 1.0;
    if (oy < 0.0) oy = 0.0; if (oy > 1.0) oy = 1.0;
    *t1 = g_cur.x0 + ox * (g_cur.x1 - g_cur.x0);
    *t2 = g_cur.y1 - oy * (g_cur.y1 - g_cur.y0);
}

/* 当前生效的参数：只有点“重新模拟”或拖框放大时才更新 */
static int    g_res  = 4096;
static double g_time = 30.0;

/* 解析两个输入框；失败时把原因写进 err 并返回 -1 */
static int parse_params(int *res, double *time, WCHAR *err, int errn)
{
    WCHAR buf[64], *end;

    GetWindowTextW(g_edRes, buf, 64);
    long v = wcstol(buf, &end, 10);
    while (end && *end == L' ') ++end;
    if (end == buf || (end && *end)) {
        wcsncpy(err, UI(L"精度请填写整数（64 ~ 16384）", L"Precision must be an integer (64-16384)"), (size_t)errn);
        return -1;
    }
    if (v < 64 || v > 16384) {
        wcsncpy(err, UI(L"精度范围 64 ~ 16384", L"Precision must be 64 - 16384"), (size_t)errn);
        return -1;
    }

    GetWindowTextW(g_edTime, buf, 64);
    double t = wcstod(buf, &end);
    while (end && *end == L' ') ++end;
    if (end == buf || (end && *end)) {
        wcsncpy(err, UI(L"时长请填写数字（0.5 ~ 4000 秒）", L"Duration must be a number (0.5-4000)"), (size_t)errn);
        return -1;
    }
    if (!(t >= 0.5) || t > 4000.0) {
        wcsncpy(err, UI(L"时长范围 0.5 ~ 4000 秒", L"Duration must be 0.5 - 4000 s"), (size_t)errn);
        return -1;
    }

    *res = (int)v;
    *time = t;
    return 0;
}

static void read_params(int *res, double *time)
{
    *res  = g_res;
    *time = g_time;
}

/* 以给定矩形区域创建新视图并开始计算 */
static void launch_region(double x0, double x1, double y0, double y1)
{
    if (x1 - x0 < 1e-12 || y1 - y0 < 1e-12) return;

    int res; double T;
    read_params(&res, &T);

    int W = res;
    double hh = (double)W * (y1 - y0) / (x1 - x0);
    if (hh > 16384.0) hh = 16384.0;
    if (hh < 16.0) hh = 16.0;
    int H = (int)(hh + 0.5);

    /* 保存当前视图 */
    if (g_sp < MAXSTACK) {
        g_stack[g_sp++] = g_cur;
    } else {
        view_free(&g_cur);
    }

    view_alloc(&g_cur, x0, x1, y0, y1, W, H, T);
    layout(g_main);
    start_job();
}

/* 用当前区域 + 当前控件参数重新计算（不压栈） */
static void read_sharp(void)
{
    WCHAR b[32];
    if (!g_edSharp) return;
    GetWindowTextW(g_edSharp, b, 32);
    double v = _wtof(b);
    if (!(v > 0.0)) v = 1.0;
    if (v < 0.2) v = 0.2;
    if (v > 8.0) v = 8.0;
    g_sharp = v;
}

static void resimulate_current(void)
{
    read_sharp();
    int res; double T;
    WCHAR err[128] = { 0 };
    if (parse_params(&res, &T, err, 128) != 0) {
        MessageBoxW(g_main, err, UI(L"参数无效", L"Invalid value"), MB_OK | MB_ICONWARNING);
        return;
    }
    g_res = res;
    g_time = T;

    double x0 = g_cur.x0, x1 = g_cur.x1, y0 = g_cur.y0, y1 = g_cur.y1;
    if (x1 <= x0 || y1 <= y0) return;

    int W = res;
    double hh = (double)W * (y1 - y0) / (x1 - x0);
    if (hh > 16384.0) hh = 16384.0;
    if (hh < 16.0) hh = 16.0;
    int H = (int)(hh + 0.5);

    stop_job();
    view_free(&g_cur);
    view_alloc(&g_cur, x0, x1, y0, y1, W, H, T);
    layout(g_main);
    start_job();
}

#define MP_W_LO (-25.0)
#define MP_W_HI ( 25.0)

/* 参数空间换了（角度空间 <-> 角动量空间）：清空历史并重建全域视图 */
static void reset_domain(void)
{
    if (g_computing) stop_job();
    while (g_sp > 0) { g_sp--; view_free(&g_stack[g_sp]); }

    double lo = (g_phys.space == 0) ? -MPI : MP_W_LO;   /* MP_W_LO 本身是负数 */
    double hi = (g_phys.space == 0) ?  MPI :  MP_W_HI;

    g_zoom = 1.0; g_zoomCX = 0.5; g_zoomCY = 0.5;
    int res; double T;
    read_params(&res, &T);
    view_free(&g_cur);
    view_alloc(&g_cur, lo, hi, lo, hi, res, res, T);
    layout(g_main);
    start_job();
}

/* Z：回到上一个视图 */
static void zoom_back(void)
{
    LOG("ZOOM_BACK sp=%d", g_sp);
    if (g_sp <= 0) return;
    stop_job();
    view_free(&g_cur);
    g_cur = g_stack[--g_sp];
    layout(g_main);
    InvalidateRect(g_main, NULL, FALSE);

    WCHAR buf[128];
    swprintf(buf, 128, UI(L"已返回上一视图（剩余 %d 层）", L"Back to previous view (%d left)"), g_sp);
    SetWindowTextW(g_status, buf);
}

static void update_progress(int done, int total)
{
    if (total <= 0) total = 1;
    int pct = done * 100 / total;
    /* 节流：百分比没变且距上次不到 120ms 就不动控件 */
    static int  s_lastPct = -1;
    static DWORD s_lastTick = 0;
    DWORD now = GetTickCount();
    if (pct == s_lastPct && now - s_lastTick < 120) return;
    s_lastPct = pct;
    s_lastTick = now;
    SendMessageW(g_prog, PBM_SETPOS, pct, 0);
    WCHAR buf[128];
    double el = (now - g_startTick) / 1000.0;
    swprintf(buf, 128, UI(L"计算中 %d%%  (%.1f s)", L"Computing %d%%  (%.1f s)"), pct, el);
    SetWindowTextW(g_status, buf);
}

static void draw_selection(HDC hdc)
{
    if (!g_dragging) return;
    /* 黑色选框：深底上仍清晰，又不刺眼 */
    HPEN pen = CreatePen(PS_SOLID, 2, RGB(0, 0, 0));
    HGDIOBJ op = SelectObject(hdc, pen);
    HGDIOBJ ob = SelectObject(hdc, GetStockObject(NULL_BRUSH));
    Rectangle(hdc, g_dragX0, g_dragY0, g_dragX1, g_dragY1);
    /* 外侧再描一圈细灰边，保证在黑色区域也看得见 */
    HPEN pen2 = CreatePen(PS_SOLID, 1, RGB(70, 78, 96));
    SelectObject(hdc, pen2);
    Rectangle(hdc, g_dragX0 - 1, g_dragY0 - 1, g_dragX1 + 1, g_dragY1 + 1);
    DeleteObject(pen2);
    SelectObject(hdc, ob);
    SelectObject(hdc, op);
    DeleteObject(pen);
}

static void on_paint(HWND hwnd)
{
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);

    RECT rc;
    GetClientRect(hwnd, &rc);

    /* 整窗双缓冲，避免闪烁 */
    HDC mem = CreateCompatibleDC(hdc);
    HBITMAP bmp = CreateCompatibleBitmap(hdc, rc.right, rc.bottom);
    HGDIOBJ old = SelectObject(mem, bmp);

    HBRUSH bg = CreateSolidBrush(RGB(0, 0, 0));
    FillRect(mem, &rc, bg);
    DeleteObject(bg);

    if (g_cur.dib && g_cur.bits) {
        HDC src = CreateCompatibleDC(mem);
        HGDIOBJ so = SelectObject(src, g_cur.dib);
        SetStretchBltMode(mem, HALFTONE);
        SetBrushOrgEx(mem, 0, 0, NULL);
        int sw = (int)(g_cur.W / g_zoom + 0.5);
        int sh = (int)(g_cur.H / g_zoom + 0.5);
        if (sw < 8) sw = 8;
        if (sh < 8) sh = 8;
        if (sw > g_cur.W) sw = g_cur.W;
        if (sh > g_cur.H) sh = g_cur.H;
        int sx = (int)(g_zoomCX * g_cur.W - sw * 0.5);
        int sy = (int)(g_zoomCY * g_cur.H - sh * 0.5);
        if (sx < 0) sx = 0;
        if (sy < 0) sy = 0;
        if (sx > g_cur.W - sw) sx = g_cur.W - sw;
        if (sy > g_cur.H - sh) sy = g_cur.H - sh;
        StretchBlt(mem, g_imgX, g_imgY, g_imgW, g_imgH,
                   src, sx, sy, sw, sh, SRCCOPY);
        SelectObject(src, so);
        DeleteDC(src);
    }

    draw_selection(mem);

#ifdef DEBUG_FRAME
    {
        RECT ir = { g_imgX, g_imgY, g_imgX + g_imgW, g_imgY + g_imgH };
        HBRUSH fb = CreateSolidBrush(RGB(255, 0, 0));
        FrameRect(mem, &ir, fb);
        DeleteObject(fb);
    }
#endif

    BitBlt(hdc, 0, 0, rc.right, rc.bottom, mem, 0, 0, SRCCOPY);
    SelectObject(mem, old);
    DeleteObject(bmp);
    DeleteDC(mem);

    EndPaint(hwnd, &ps);
}

/* 输入框里按回车等同于点“重新模拟” */
static LRESULT CALLBACK edit_subclass(HWND h, UINT m, WPARAM w, LPARAM l,
                                      UINT_PTR id, DWORD_PTR ref)
{
    (void)id; (void)ref;
    if (m == WM_KEYDOWN && w == VK_RETURN) {
        PostMessageW(g_main, WM_COMMAND, MAKEWPARAM(IDC_APPLY, BN_CLICKED), 0);
        return 0;
    }
    if (m == WM_CHAR && w == VK_RETURN) return 0;
    return DefSubclassProc(h, m, w, l);
}

static void create_controls(HWND hwnd)
{
    g_font = CreateFontW(-16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                         DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                         CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE,
                         L"Microsoft YaHei UI");

    HWND lbl1 = CreateWindowW(L"STATIC", UI(L"精度", L"Prec."), WS_CHILD | WS_VISIBLE | SS_CENTERIMAGE,
                              10, 10, 44, 22, hwnd, NULL, NULL, NULL);
    g_edRes = CreateWindowW(L"EDIT", L"4096",
                            WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | ES_CENTER,
                            58, 20, 90, 24, hwnd, (HMENU)IDC_RES, NULL, NULL);
    SendMessageW(g_edRes, EM_SETLIMITTEXT, 8, 0);

    HWND lbl2 = CreateWindowW(L"STATIC", UI(L"时长", L"Dur."), WS_CHILD | WS_VISIBLE | SS_CENTERIMAGE,
                              146, 10, 50, 22, hwnd, NULL, NULL, NULL);
    g_edTime = CreateWindowW(L"EDIT", L"30",
                             WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | ES_CENTER,
                             218, 20, 90, 24, hwnd, (HMENU)IDC_TIME, NULL, NULL);
    SendMessageW(g_edTime, EM_SETLIMITTEXT, 12, 0);

    g_btnApply = CreateWindowW(L"BUTTON", UI(L"重新模拟", L"Recompute"),
                               WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                               320, 16, 100, 30, hwnd, (HMENU)IDC_APPLY, NULL, NULL);

    g_rb32 = CreateWindowW(L"BUTTON", UI(L"单精度", L"Single"),
                           WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP,
                           416, 20, 80, 24, hwnd, (HMENU)IDC_PREC32, NULL, NULL);
    g_rbFF = CreateWindowW(L"BUTTON", UI(L"高精度", L"High"),
                           WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON,
                           500, 20, 80, 24, hwnd, (HMENU)IDC_PRECFF, NULL, NULL);
    g_rbD = CreateWindowW(L"BUTTON", UI(L"双精度", L"Double"),
                          WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON,
                          584, 20, 80, 24, hwnd, (HMENU)IDC_PRECD, NULL, NULL);
    SendMessageW(g_rbFF, BM_SETCHECK, BST_CHECKED, 0);

    g_chkFast = CreateWindowW(L"BUTTON", UI(L"快速模式", L"Fast"),
                              WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
                              668, 20, 88, 24, hwnd, (HMENU)IDC_FAST, NULL, NULL);

    g_brPanel = CreateSolidBrush(COL_PANEL);
    g_brEdit  = CreateSolidBrush(COL_EDITBG);
    g_fontUI  = CreateFontW(-15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                            DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                            CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Microsoft YaHei UI");

    g_btnZoomIn = CreateWindowW(L"BUTTON", UI(L"放大 +", L"Zoom +"),
                                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                                206, 54, 76, 30, hwnd, (HMENU)IDC_ZOOMIN, NULL, NULL);
    g_btnZoomOut = CreateWindowW(L"BUTTON", UI(L"缩小 -", L"Zoom -"),
                                 WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                                 290, 54, 76, 30, hwnd, (HMENU)IDC_ZOOMOUT, NULL, NULL);
    g_btnZoomRst = CreateWindowW(L"BUTTON", UI(L"复位 1:1", L"Reset 1:1"),
                                 WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                                 374, 54, 90, 30, hwnd, (HMENU)IDC_ZOOMRST, NULL, NULL);
    g_btnHelp = CreateWindowW(L"BUTTON", UI(L"使用说明", L"Help"),
                              WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                              472, 54, 90, 30, hwnd, (HMENU)IDC_HELPBOX, NULL, NULL);

    g_btnAdv = CreateWindowW(L"BUTTON", UI(L"高级选项", L"Advanced"),
                             WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                             854, 16, 80, 30, hwnd, (HMENU)IDC_ADV, NULL, NULL);
    CreateWindowW(L"STATIC", UI(L"锐度", L"Sharp"), WS_CHILD | WS_VISIBLE | SS_CENTERIMAGE,
                  740, 10, 44, 22, hwnd, NULL, NULL, NULL);
    g_edSharp = CreateWindowW(L"EDIT", L"1.0",
                              WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL | ES_CENTER,
                              794, 8, 64, 28, hwnd, (HMENU)IDC_SHARP, NULL, NULL);

    g_btnSnap = CreateWindowW(L"BUTTON", UI(L"保存图片", L"Save PNG"),
                              WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                              760, 16, 86, 30, hwnd, (HMENU)IDC_SNAP, NULL, NULL);

    g_prog = CreateWindowW(PROGRESS_CLASSW, NULL, WS_CHILD | WS_VISIBLE,
                           684, 20, 240, 22, hwnd, (HMENU)IDC_PROG, NULL, NULL);
    SendMessageW(g_prog, PBM_SETRANGE, 0, MAKELPARAM(0, 100));

    g_status = CreateWindowW(L"STATIC", UI(L"就绪", L"Ready"), WS_CHILD | WS_VISIBLE | SS_CENTERIMAGE,
                             944, 22, 220, 22, hwnd, (HMENU)IDC_STATUS, NULL, NULL);

    HWND *ctrls[] = { &g_edRes, &g_edTime, &g_btnApply, &g_rb32, &g_rbFF, &g_rbD,
                      &g_chkFast, &g_btnSnap, &g_btnAdv, &g_prog, &g_status };
    if (g_fontUI)
        for (int i = 0; i < (int)(sizeof ctrls / sizeof ctrls[0]); ++i)
            SendMessageW(*ctrls[i], WM_SETFONT, (WPARAM)g_fontUI, TRUE);
    for (int i = 0; i < 10; ++i)
        SendMessageW(*ctrls[i], WM_SETFONT, (WPARAM)g_font, TRUE);
    SendMessageW(lbl1, WM_SETFONT, (WPARAM)g_font, TRUE);
    SendMessageW(lbl2, WM_SETFONT, (WPARAM)g_font, TRUE);

    SetWindowSubclass(g_edRes,  edit_subclass, 1, 0);
    SetWindowSubclass(g_edTime, edit_subclass, 2, 0);

    /* 环境变量 CHAOS_FAST=1 启动时同步到复选框 */
    if (gpu_is_fast()) SendMessageW(g_chkFast, BM_SETCHECK, BST_CHECKED, 0);
}

static void draw_own_button(const DRAWITEMSTRUCT *d);

/* ================= 高级选项窗口 ================= */
#define ADV_CLASS L"ChaosAdvanced"

static HWND g_adv = NULL;
static HWND g_advEd[7];
static HWND g_advLb[7];
static HWND g_advRb[2];
static int  g_advSpace = 0;   /* 窗口内选择的空间，点“确定”后才生效 */

static const WCHAR *ADV_LABEL[7] = {
    UI(L"第一摆质量 m1", L"Pendulum 1 mass m1"), UI(L"第二摆质量 m2", L"Pendulum 2 mass m2"),
    UI(L"第一摆长度 L1", L"Pendulum 1 length L1"), UI(L"第二摆长度 L2", L"Pendulum 2 length L2"),
    UI(L"重力加速度 g", L"Gravity g"),
    UI(L"第一摆初始角速度 ω1", L"Pendulum 1 ang.vel"), UI(L"第二摆初始角速度 ω2", L"Pendulum 2 ang.vel")
};

static void adv_set_labels(void)
{
    int sp = g_advSpace;
    if (g_advLb[5]) SetWindowTextW(g_advLb[5], sp ? UI(L"第一摆初始角度 θ1", L"Pendulum 1 angle")
                                                  : UI(L"第一摆初始角速度 ω1", L"Pendulum 1 ang.vel"));
    if (g_advLb[6]) SetWindowTextW(g_advLb[6], sp ? UI(L"第二摆初始角度 θ2", L"Pendulum 2 angle")
                                                  : UI(L"第二摆初始角速度 ω2", L"Pendulum 2 ang.vel"));
}

static void adv_set_values(void)
{
    WCHAR b[64];
    int sp = g_advSpace;
    double v[7] = { g_phys.m1, g_phys.m2, g_phys.l1, g_phys.l2, g_phys.g,
                    sp ? g_phys.th1_0 : g_phys.w1_0,
                    sp ? g_phys.th2_0 : g_phys.w2_0 };
    for (int i = 0; i < 7; ++i) {
        swprintf(b, 64, L"%.6g", v[i]);
        SetWindowTextW(g_advEd[i], b);
    }
    SendMessageW(g_advRb[g_advSpace ? 1 : 0], BM_SETCHECK, BST_CHECKED, 0);
    adv_set_labels();
}

/* ---- 角度表达式解析：支持 π、pi、π/2、-π/3、2π/3、1.5*π、(π)/4 ---- */
static const WCHAR *ANG_P;
static int ANG_FAIL;

static void ang_skip(void) { while (*ANG_P == L' ') ++ANG_P; }

static double ang_atom(void)
{
    ang_skip();
    /* 用码点比较，避免源码编码不同导致 L'π' 不匹配 */
    if ((unsigned)*ANG_P == 0x03C0u || (unsigned)*ANG_P == 0x03A0u) { ANG_P += 1; return MPI; }
    if ((*ANG_P == L'p' || *ANG_P == L'P') && (ANG_P[1] == L'i' || ANG_P[1] == L'I')) {
        ANG_P += 2; return MPI;
    }
    WCHAR *end = NULL;
    double v = wcstod(ANG_P, &end);
    if (end == ANG_P) { ANG_FAIL = 1; return 0.0; }
    ANG_P = end;
    return v;
}

static double ang_term(void)
{
    double v = ang_atom();
    for (;;) {
        ang_skip();
        if (*ANG_P == L'*') { ANG_P += 1; v *= ang_atom(); }
        else if (*ANG_P == L'/') { ANG_P += 1; v /= ang_atom(); }
        else if ((unsigned)*ANG_P == 0x03C0u || (unsigned)*ANG_P == 0x03A0u) {
            /* 允许 2π/3 这种隐式乘法 */
            ANG_P += 1; v *= MPI;
        }
        else break;
    }
    return v;
}

static double ang_expr(void)
{
    ang_skip();
    double v;
    if (*ANG_P == L'-') { ANG_P += 1; v = -ang_term(); }
    else { if (*ANG_P == L'+') ANG_P += 1; v = ang_term(); }
    for (;;) {
        ang_skip();
        if (*ANG_P == L'+') { ANG_P += 1; v += ang_term(); }
        else if (*ANG_P == L'-') { ANG_P += 1; v -= ang_term(); }
        else break;
    }
    return v;
}

static int parse_angle_expr(const WCHAR *s, double *out)
{
    ANG_P = s; ANG_FAIL = 0;
    double v = ang_expr();
    ang_skip();
    if (ANG_FAIL || *ANG_P != 0) return -1;
    *out = v;
    return 0;
}

static int adv_read_values(double v[7])
{
    WCHAR b[64], *end;
    for (int i = 0; i < 7; ++i) {
        GetWindowTextW(g_advEd[i], b, 64);
        if (i >= 5 && g_advSpace == 1) {
            /* 角动量空间下这两格是初始角度，允许写 π、π/2、-π/3 */
            if (parse_angle_expr(b, &v[i]) != 0 || !isfinite(v[i])) return -1;
            continue;
        }
        v[i] = wcstod(b, &end);
        while (end && *end == L' ') ++end;
        if (end == b || (end && *end) || !isfinite(v[i])) return -1;
    }
    if (v[0] <= 0 || v[1] <= 0 || v[2] <= 0 || v[3] <= 0) return -1;
    if (v[4] == 0) return -1;
    return 0;
}

static LRESULT CALLBACK AdvProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    (void)lParam;
    switch (msg) {
    case WM_CREATE: {
        static const int ids[7] = { IDC_ADV_M1, IDC_ADV_M2, IDC_ADV_L1,
                                    IDC_ADV_L2, IDC_ADV_G, IDC_ADV_W1, IDC_ADV_W2 };
        for (int i = 0; i < 7; ++i) {
            int y = 16 + i * 32;
            g_advLb[i] = CreateWindowW(L"STATIC", ADV_LABEL[i],
                          WS_CHILD | WS_VISIBLE | SS_CENTERIMAGE,
                          16, y + 3, 200, 22, hwnd, NULL, NULL, NULL);
            g_advEd[i] = CreateWindowW(L"EDIT", L"",
                                       WS_CHILD | WS_VISIBLE | WS_BORDER | ES_AUTOHSCROLL,
                                       222, y, 120, 26, hwnd,
                                       (HMENU)(INT_PTR)ids[i], NULL, NULL);
        }
        CreateWindowW(L"STATIC", UI(L"参数空间", L"Space"), WS_CHILD | WS_VISIBLE | SS_CENTERIMAGE,
                      16, 246, 200, 22, hwnd, NULL, NULL, NULL);
        g_advRb[0] = CreateWindowW(L"BUTTON", UI(L"角度空间", L"Angle"),
                                   WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON,
                                   222, 244, 120, 24, hwnd, (HMENU)IDC_ADV_SP0, NULL, NULL);
        g_advRb[1] = CreateWindowW(L"BUTTON", UI(L"角动量空间", L"Ang. velocity"),
                                   WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON,
                                   222, 272, 120, 24, hwnd, (HMENU)IDC_ADV_SP1, NULL, NULL);
        CreateWindowW(L"STATIC",
                      UI(L"角度空间：横纵轴是两摆的初始角度（-π~π），初始角速度取下面两格。 ", L"Angle space: axes are initial angles (-pi..pi); initial ang. velocities from the two fields below. ")
                      UI(L"角动量空间：横纵轴是初始角速度（-25~25），初始角度取下面两格（默认 0，即竖直向下）。 ", L"Angular-velocity space: axes are initial ang. velocities (-25..25); initial angles from the two fields below (default 0 = hanging down). ")
                      UI(L"角速度可为负，负表示顺时针。", L"Negative = clockwise."),
                      WS_CHILD | WS_VISIBLE,
                      16, 304, 508, 74, hwnd, NULL, NULL, NULL);
        CreateWindowW(L"BUTTON", UI(L"确定", L"OK"), WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
                      168, 384, 90, 32, hwnd, (HMENU)IDC_ADV_OK, NULL, NULL);
        CreateWindowW(L"BUTTON", UI(L"恢复默认", L"Defaults"), WS_CHILD | WS_VISIBLE,
                      270, 384, 100, 32, hwnd, (HMENU)IDC_ADV_DEF, NULL, NULL);
        CreateWindowW(L"BUTTON", UI(L"取消", L"Cancel"), WS_CHILD | WS_VISIBLE,
                      382, 384, 90, 32, hwnd, (HMENU)IDC_ADV_CANCEL, NULL, NULL);
        adv_set_values();
        return 0;
    }
    case WM_DRAWITEM: {
        const DRAWITEMSTRUCT *d = (const DRAWITEMSTRUCT *)lParam;
        if (d && d->CtlType == ODT_BUTTON) { draw_own_button(d); return TRUE; }
        break;
    }
    case WM_CTLCOLORSTATIC:
    case WM_CTLCOLORBTN: {
        HDC dc = (HDC)wParam;
        SetTextColor(dc, COL_TEXT);
        SetBkColor(dc, COL_PANEL);
        return (LRESULT)g_brPanel;
    }
    case WM_CTLCOLOREDIT: {
        HDC dc = (HDC)wParam;
        SetTextColor(dc, RGB(235, 245, 255));
        SetBkColor(dc, COL_EDITBG);
        return (LRESULT)g_brEdit;
    }
    case WM_COMMAND:
        switch (LOWORD(wParam)) {
        case IDC_ADV_OK: {
            double v[7];
            if (adv_read_values(v) != 0) {
                MessageBoxW(hwnd, UI(L"请输入有效数字：质量、长度必须为正，重力不能为 0。", L"Invalid numbers: mass/length must be > 0, gravity != 0."),
                            UI(L"参数无效", L"Invalid value"), MB_OK | MB_ICONWARNING);
                return 0;
            }
            int oldspace = g_phys.space;
            g_phys.m1 = v[0]; g_phys.m2 = v[1];
            g_phys.l1 = v[2]; g_phys.l2 = v[3];
            g_phys.g  = v[4];
            int newsp = (SendMessageW(g_advRb[1], BM_GETCHECK, 0, 0) == BST_CHECKED) ? 1 : 0;
            if (newsp) { g_phys.th1_0 = v[5]; g_phys.th2_0 = v[6]; }
            else       { g_phys.w1_0  = v[5]; g_phys.w2_0  = v[6]; }
            g_phys.space = newsp;
            DestroyWindow(hwnd);
            g_adv = NULL;
            if (oldspace != g_phys.space) reset_domain();
            else resimulate_current();
            return 0;
        }
        case IDC_ADV_DEF:
            g_phys.m1 = 1.0; g_phys.m2 = 1.0;
            g_phys.l1 = 1.0; g_phys.l2 = 1.0;
            g_phys.g  = 9.81;
            g_phys.w1_0 = 0.0; g_phys.w2_0 = 0.0;
            g_phys.th1_0 = 0.0; g_phys.th2_0 = 0.0;
            adv_set_values();
            return 0;
        case IDC_ADV_SP0:
        case IDC_ADV_SP1: {
            double v[7];
            int newsp = (LOWORD(wParam) == IDC_ADV_SP1) ? 1 : 0;
            if (newsp == g_advSpace) return 0;
            if (adv_read_values(v) == 0) {
                if (g_advSpace) { g_phys.th1_0 = v[5]; g_phys.th2_0 = v[6]; }
                else            { g_phys.w1_0  = v[5]; g_phys.w2_0  = v[6]; }
            }
            g_advSpace = newsp;
            adv_set_values();
            return 0;
        }
        case IDC_ADV_CANCEL:
            DestroyWindow(hwnd);
            g_adv = NULL;
            return 0;
        }
        return 0;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        g_adv = NULL;
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

static void open_advanced(HWND owner)
{
    g_advSpace = g_phys.space;
    if (g_adv && IsWindow(g_adv)) {
        logf_("adv already open %p", (void *)g_adv);
        ShowWindow(g_adv, SW_SHOW);
        SetForegroundWindow(g_adv);
        return;
    }
    g_adv = CreateWindowExW(0, ADV_CLASS, UI(L"高级选项", L"Advanced"),
                            WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU,
                            CW_USEDEFAULT, CW_USEDEFAULT, 560, 470,
                            owner, NULL, GetModuleHandleW(NULL), NULL);
    if (!g_adv) { logf_("adv CreateWindow failed err=%d", (int)GetLastError()); return; }
    logf_("adv window created %p", (void *)g_adv);
    adv_set_values();
    ShowWindow(g_adv, SW_SHOW);
    SetForegroundWindow(g_adv);
}

/* 自绘按钮：深色底 + 青色描边，按下时高亮 */
static void draw_own_button(const DRAWITEMSTRUCT *d)
{
    if (d->CtlType != ODT_BUTTON) return;
    int pressed = (d->itemState & ODS_SELECTED) != 0;
    int disabled = (d->itemState & ODS_DISABLED) != 0;
    RECT r = d->rcItem;

    COLORREF fill = disabled ? RGB(18, 20, 26)
                             : (pressed ? RGB(0, 86, 118) : RGB(16, 26, 40));
    HBRUSH b = CreateSolidBrush(fill);
    FillRect(d->hDC, &r, b);
    DeleteObject(b);

    COLORREF edge = disabled ? RGB(40, 44, 54)
                             : (pressed ? COL_ACCENT : RGB(46, 108, 148));
    HPEN pen = CreatePen(PS_SOLID, 1, edge);
    HGDIOBJ op = SelectObject(d->hDC, pen);
    HGDIOBJ ob = SelectObject(d->hDC, GetStockObject(NULL_BRUSH));
    Rectangle(d->hDC, r.left, r.top, r.right, r.bottom);
    SelectObject(d->hDC, ob);
    SelectObject(d->hDC, op);
    DeleteObject(pen);

    WCHAR txt[64];
    GetWindowTextW(d->hwndItem, txt, 64);
    SetBkMode(d->hDC, TRANSPARENT);
    SetTextColor(d->hDC, disabled ? RGB(90, 96, 108) : (pressed ? RGB(240, 252, 255) : COL_TEXT));
    HGDIOBJ of = SelectObject(d->hDC, g_fontUI ? g_fontUI : GetStockObject(DEFAULT_GUI_FONT));
    DrawTextW(d->hDC, txt, -1, &r, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    SelectObject(d->hDC, of);
}

static LRESULT CALLBACK MainWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
    switch (msg) {
    case WM_CREATE:
        g_main = hwnd;
        create_controls(hwnd);
        return 0;

    case WM_GETMINMAXINFO: {
        MINMAXINFO *mm = (MINMAXINFO *)lParam;
        mm->ptMinTrackSize.x = 1040;
        mm->ptMinTrackSize.y = 620;
        return 0;
    }

    case WM_SIZE:
        layout(hwnd);
        return 0;

    case WM_ERASEBKGND:
        return 1;

    case WM_PAINT:
        on_paint(hwnd);
        return 0;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDC_APPLY) {
            resimulate_current();
        } else if ((LOWORD(wParam) == IDC_PREC32 || LOWORD(wParam) == IDC_PRECFF ||
                    LOWORD(wParam) == IDC_PRECD) && HIWORD(wParam) == BN_CLICKED) {
            int want = (LOWORD(wParam) == IDC_PRECFF) ? 1 : (LOWORD(wParam) == IDC_PRECD) ? 2 : 0;
            if (want == 2 && !gpu_has_double()) {
                MessageBoxW(hwnd, UI(L"当前设备不支持双精度（cl_khr_fp64）。", L"Device does not support fp64 (cl_khr_fp64)."),
                            UI(L"提示", L"Notice"), MB_OK | MB_ICONINFORMATION);
                SendMessageW(g_rb32, BM_SETCHECK, BST_CHECKED, 0);
                want = 0;
            }
            g_precision = want;
            resimulate_current();
        } else if (LOWORD(wParam) == IDC_ADV && HIWORD(wParam) == BN_CLICKED) {
            open_advanced(hwnd);
        } else if (LOWORD(wParam) == IDC_ZOOMIN && HIWORD(wParam) == BN_CLICKED) {
            g_zoom *= 1.5; if (g_zoom > 64.0) g_zoom = 64.0;
            InvalidateRect(hwnd, NULL, FALSE);
        } else if (LOWORD(wParam) == IDC_ZOOMOUT && HIWORD(wParam) == BN_CLICKED) {
            g_zoom /= 1.5;
            if (g_zoom < 1.0) { g_zoom = 1.0; g_zoomCX = 0.5; g_zoomCY = 0.5; }
            InvalidateRect(hwnd, NULL, FALSE);
        } else if (LOWORD(wParam) == IDC_ZOOMRST && HIWORD(wParam) == BN_CLICKED) {
            g_zoom = 1.0; g_zoomCX = 0.5; g_zoomCY = 0.5;
            InvalidateRect(hwnd, NULL, FALSE);
        } else if (LOWORD(wParam) == IDC_HELPBOX && HIWORD(wParam) == BN_CLICKED) {
            MessageBoxW(hwnd, g_helpText, UI(L"使用说明", L"Help"), MB_OK | MB_ICONINFORMATION);
        } else if (LOWORD(wParam) == IDC_SNAP && HIWORD(wParam) == BN_CLICKED) {
            save_snapshot(hwnd);
        } else if (LOWORD(wParam) == IDC_FAST && HIWORD(wParam) == BN_CLICKED) {
            int on = (SendMessageW(g_chkFast, BM_GETCHECK, 0, 0) == BST_CHECKED);
            /* 重建内核前必须让计算线程彻底停下 */
            if (g_computing) stop_job();
            if (gpu_set_fast(on) != 0) {
                MessageBoxW(hwnd, UI(L"切换快速模式失败（内核重建出错）。", L"Failed to switch fast mode (kernel rebuild error)."),
                            UI(L"提示", L"Notice"), MB_OK | MB_ICONWARNING);
                SendMessageW(g_chkFast, BM_SETCHECK,
                             gpu_is_fast() ? BST_CHECKED : BST_UNCHECKED, 0);
            }
            resimulate_current();
        }
        return 0;

    case WM_LBUTTONDOWN: {
        SetFocus(hwnd);
        int x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
        LOG("DOWN x=%d y=%d dispY=%d", x, y, g_dispY);
        if (y >= g_dispY) {
            g_dragging = TRUE;
            g_dragX0 = g_dragX1 = x;
            g_dragY0 = g_dragY1 = y;
            SetCapture(hwnd);
        }
        return 0;
    }
    case WM_DRAWITEM: {
        const DRAWITEMSTRUCT *d = (const DRAWITEMSTRUCT *)lParam;
        if (d && d->CtlType == ODT_BUTTON) { draw_own_button(d); return TRUE; }
        break;
    }
    case WM_CTLCOLORSTATIC:
    case WM_CTLCOLORBTN: {
        HDC dc = (HDC)wParam;
        SetTextColor(dc, COL_TEXT);
        SetBkColor(dc, COL_PANEL);
        return (LRESULT)g_brPanel;
    }
    case WM_CTLCOLOREDIT: {
        HDC dc = (HDC)wParam;
        SetTextColor(dc, RGB(235, 245, 255));
        SetBkColor(dc, COL_EDITBG);
        return (LRESULT)g_brEdit;
    }
    case WM_RBUTTONDOWN:
        if (g_zoom > 1.001) {
            g_panning = 1;
            g_panX = GET_X_LPARAM(lParam);
            g_panY = GET_Y_LPARAM(lParam);
            SetCapture(hwnd);
        }
        return 0;

    case WM_RBUTTONUP:
        if (g_panning) { g_panning = 0; ReleaseCapture(); }
        return 0;

    case WM_MOUSEWHEEL: {
        /* Ctrl + 滚轮：纯视觉缩放，不重算 */
        if ((wParam >> 16) & MK_CONTROL) {
            int delta = GET_WHEEL_DELTA_WPARAM(wParam);
            POINT pt;
            pt.x = GET_X_LPARAM(lParam);
            pt.y = GET_Y_LPARAM(lParam);
            ScreenToClient(hwnd, &pt);
            /* 以鼠标位置为锚点缩放 */
            double fx = (pt.x - g_imgX) / (double)g_imgW;
            double fy = (pt.y - g_imgY) / (double)g_imgH;
            if (fx < 0.0) fx = 0.0; if (fx > 1.0) fx = 1.0;
            if (fy < 0.0) fy = 0.0; if (fy > 1.0) fy = 1.0;
            double ax = g_zoomCX + (fx - 0.5) / g_zoom;
            double ay = g_zoomCY + (fy - 0.5) / g_zoom;

            double nz = g_zoom * (delta > 0 ? 1.25 : 0.8);
            if (nz < 1.0) nz = 1.0;
            if (nz > 64.0) nz = 64.0;
            if (nz != g_zoom) {
                /* 让锚点在缩放前后落在同一屏幕位置 */
                g_zoomCX = ax - (fx - 0.5) / nz;
                g_zoomCY = ay - (fy - 0.5) / nz;
                g_zoom = nz;
                if (g_zoomCX < 0.0) g_zoomCX = 0.0;
                if (g_zoomCX > 1.0) g_zoomCX = 1.0;
                if (g_zoomCY < 0.0) g_zoomCY = 0.0;
                if (g_zoomCY > 1.0) g_zoomCY = 1.0;
                InvalidateRect(hwnd, NULL, FALSE);
            }
            return 0;
        }
        break;
    }
    case WM_MOUSEMOVE: {
        if (g_dragging) {
            g_dragX1 = GET_X_LPARAM(lParam);
            g_dragY1 = GET_Y_LPARAM(lParam);
            InvalidateRect(hwnd, NULL, FALSE);
        } else if (g_panning) {
            int mx = GET_X_LPARAM(lParam), my = GET_Y_LPARAM(lParam);
            g_zoomCX -= (mx - g_panX) / (double)g_imgW / g_zoom;
            g_zoomCY -= (my - g_panY) / (double)g_imgH / g_zoom;
            if (g_zoomCX < 0) g_zoomCX = 0; if (g_zoomCX > 1) g_zoomCX = 1;
            if (g_zoomCY < 0) g_zoomCY = 0; if (g_zoomCY > 1) g_zoomCY = 1;
            g_panX = mx; g_panY = my;
            InvalidateRect(hwnd, NULL, FALSE);
        } else {
            int mx = GET_X_LPARAM(lParam), my = GET_Y_LPARAM(lParam);
            LOG("mm %d,%d img=(%d,%d %dx%d) disp=(%d,%d %dx%d) zoom=%.3f c=%.3f,%.3f sp=%d",
                mx, my, g_imgX, g_imgY, g_imgW, g_imgH,
                g_dispX, g_dispY, g_dispW, g_dispH,
                g_zoom, g_zoomCX, g_zoomCY, g_phys.space);
            if (mx >= g_imgX && mx < g_imgX + g_imgW &&
                my >= g_imgY && my < g_imgY + g_imgH) {
                double th1, th2;
                screen_to_sim(mx, my, &th1, &th2);
                WCHAR t[200];
                if (g_phys.space == 0)
                    swprintf(t, 200, UI(L"θ1 = %.12f     θ2 = %.12f", L"theta1 = %.12f   theta2 = %.12f"), th1, th2);
                else
                    swprintf(t, 200, UI(L"ω1 = %.12f     ω2 = %.12f", L"omega1 = %.12f   omega2 = %.12f"), th1, th2);
                if (wcscmp(t, g_lastTitle) != 0) {
                    wcsncpy(g_lastTitle, t, 199);
                    g_lastTitle[199] = 0;
                    SetWindowTextW(hwnd, t);
                }
            } else if (g_lastTitle[0]) {
                g_lastTitle[0] = 0;
                SetWindowTextW(hwnd, UI(L"双摆混沌地图", L"ChaosMap"));
            }
        }
        return 0;
    }
    case WM_LBUTTONUP: {
        LOG("UP x=%d y=%d dragging=%d", GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam), (int)g_dragging);
        if (g_dragging) {
            g_dragging = FALSE;
            ReleaseCapture();
            int x1 = GET_X_LPARAM(lParam);
            int y1 = GET_Y_LPARAM(lParam);
            int x0 = g_dragX0, y0 = g_dragY0;

            if (abs(x1 - x0) > 5 && abs(y1 - y0) > 5) {
                if (x1 < x0) { int t = x0; x0 = x1; x1 = t; }
                if (y1 < y0) { int t = y0; y0 = y1; y1 = t; }

                double t1a, t2a, t1b, t2b;
                screen_to_sim(x0, y0, &t1a, &t2a);
                screen_to_sim(x1, y1, &t1b, &t2b);

                double t1lo = t1a < t1b ? t1a : t1b;
                double t1hi = t1a < t1b ? t1b : t1a;
                double t2lo = t2a < t2b ? t2a : t2b;
                double t2hi = t2a < t2b ? t2b : t2a;

                if (g_computing) stop_job();
                LOG("LAUNCH from drag x0=%d y0=%d x1=%d y1=%d", x0, y0, x1, y1);
                g_zoom = 1.0; g_zoomCX = 0.5; g_zoomCY = 0.5;  /* 选区重算后回到 100% 且居中 */
                launch_region(t1lo, t1hi, t2lo, t2hi);
            }
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;
    }
    case WM_LBUTTONDBLCLK: {
        int x = GET_X_LPARAM(lParam), y = GET_Y_LPARAM(lParam);
        /* 双击时结束可能残留的框选捕获，避免鼠标被主窗口抢走 */
        if (g_dragging) { g_dragging = FALSE; ReleaseCapture(); }
        LOG("DBLCLK x=%d y=%d dispY=%d", x, y, g_dispY);
        if (y >= g_dispY && g_cur.W > 0) {
            double th1, th2;
            screen_to_sim(x, y, &th1, &th2);
            open_pendulum_window(th1, th2);
        }
        return 0;
    }
    case WM_KEYDOWN:
        if (wParam == 'Z') { zoom_back(); return 0; }
        if (wParam == VK_HOME) { reset_domain(); return 0; }
        break;

    case WM_APP_PROGRESS: {
        update_progress((int)wParam, (int)lParam);
        /* 只脏图像区域，控件条不参与重绘，避免闪 */
        RECT ir = { g_imgX, g_imgY, g_imgX + g_imgW, g_imgY + g_imgH };
        InvalidateRect(hwnd, &ir, FALSE);
        return 0;
    }

    case WM_APP_DONE: {
        g_computing = 0;
        SendMessageW(g_prog, PBM_SETPOS, 100, 0);
        double el = (GetTickCount() - g_startTick) / 1000.0;
        WCHAR buf[160];
        swprintf(buf, 160, UI(L"完成 %.1fs · 框选放大 · Z返回 · 双击看摆", L"Done %.1fs | drag=zoom | Z=back | dblclick"), el);
        SetWindowTextW(g_status, buf);
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }
    case WM_APP_CANCELED:
        g_computing = 0;
        SetWindowTextW(g_status, UI(L"已取消", L"Cancelled"));
        return 0;

    case WM_DESTROY:
        stop_job();
        for (int i = 0; i < g_sp; ++i) view_free(&g_stack[i]);
        g_sp = 0;
        view_free(&g_cur);
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

/* ------------------------------------------------------------------ */

int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE hPrev, PWSTR cmd, int show)
{
    (void)hPrev; (void)cmd;

    log_open();
    SetUnhandledExceptionFilter(crash_filter);
    logf_("==== ChaosMap start ====");

    SetProcessDPIAware();

    /* 初始化 GPU 后端 */
    char gpuErr[512] = {0};
    int gpuOk = (gpu_init(gpuErr, sizeof(gpuErr)) == 0);
    logf_("gpu_init=%d err=%s", gpuOk, gpuErr);

    INITCOMMONCONTROLSEX icc = { sizeof(icc), ICC_PROGRESS_CLASS | ICC_STANDARD_CLASSES };
    InitCommonControlsEx(&icc);

    WNDCLASSEXW wc;
    memset(&wc, 0, sizeof(wc));
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = MainWndProc;
    wc.hInstance = hInst;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wc.lpszClassName = L"ChaosMapMain";
    wc.style = CS_DBLCLKS;
    RegisterClassExW(&wc);

    memset(&wc, 0, sizeof(wc));
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = AdvProc;      /* 高级选项窗口 */
    wc.lpszClassName = ADV_CLASS;
    wc.hbrBackground = CreateSolidBrush(RGB(24, 28, 38));
    if (!RegisterClassExW(&wc))
        logf_("RegisterClass %ls failed err=%d", ADV_CLASS, (int)GetLastError());
    else
        logf_("RegisterClass %ls ok", ADV_CLASS);

    wc.lpfnWndProc = PendWndProc;
    wc.hInstance = hInst;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    wc.lpszClassName = PEND_CLASS;
    RegisterClassExW(&wc);

    /* 初始空视图：先给出 [-π,π]² 的范围，再分配位图 */
    memset(&g_cur, 0, sizeof(g_cur));
    g_cur.x0 = -MPI; g_cur.x1 = MPI;
    g_cur.y0 = -MPI; g_cur.y1 = MPI;

    /* 标题里带上设备名，让人知道是否在用 GPU */
    WCHAR title[256];
    if (gpuOk) {
        char dn[256];
        gpu_device_name(dn, sizeof(dn));
        swprintf(title, 256, UI(L"双摆混沌地图", L"ChaosMap"));
    } else {
        swprintf(title, 256, UI(L"双摆混沌地图 — CPU（GPU 不可用）", L"ChaosMap - CPU (GPU unavailable)"));
    }

    HWND hwnd = CreateWindowExW(0, L"ChaosMapMain", title,
                                WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
                                CW_USEDEFAULT, CW_USEDEFAULT, 1200, 1000,
                                NULL, NULL, hInst, NULL);
    if (!hwnd) return 1;

    /* 直接铺满工作区：比 SW_MAXIMIZE 可靠（首次 ShowWindow 会被
       STARTUPINFO.wShowWindow 覆盖） */
    ShowWindow(hwnd, SW_SHOW);
    {
        RECT wa;
        if (SystemParametersInfoW(SPI_GETWORKAREA, 0, &wa, 0))
            SetWindowPos(hwnd, NULL, wa.left, wa.top,
                         wa.right - wa.left, wa.bottom - wa.top,
                         SWP_NOZORDER | SWP_NOACTIVATE);
        SetForegroundWindow(hwnd);
    }
    (void)show;
    UpdateWindow(hwnd);

    if (!gpuOk) {
        MessageBoxA(hwnd, gpuErr, "GPU 初始化失败，将改用 CPU（非常慢）",
                    MB_OK | MB_ICONWARNING);
    }

    /* 初始模拟：1200 精度、T=30、全域 */
    view_alloc(&g_cur, -MPI, MPI, -MPI, MPI, 4096, 4096, 30.0);
    layout(hwnd);
    start_job();

    MSG msg;
    while (GetMessageW(&msg, NULL, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return 0;
}
