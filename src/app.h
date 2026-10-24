#ifndef APP_H
#define APP_H

#include <windows.h>
#include <math.h>
#include <stdio.h>

/* ---------- 物理参数 ---------- */
typedef struct {
    double m1, m2, l1, l2, g;
    double w1_0, w2_0;     /* 角度空间：两摆初始角速度 */
    double th1_0, th2_0;   /* 角动量空间：两摆初始角度 */
    int    space;          /* 0 = 角度空间；1 = 角动量空间 */
} Phys;

typedef struct { double th1, w1, th2, w2; } State;

extern Phys g_phys;

/* ---------- 日志 ---------- */
void logf_(const char *fmt, ...);
extern char g_clogOp[160];
#define OP(name) (snprintf(g_clogOp, sizeof g_clogOp, "%s", name), (void)0)

/* ---------- 运动方程（头文件内联，两个编译单元共享） ---------- */
static inline void deriv(const State *s, double d[4])
{
    const double M1 = g_phys.m1, M2 = g_phys.m2;
    const double L1 = g_phys.l1, L2 = g_phys.l2, G = g_phys.g;
    const double st1 = sin(s->th1);
    const double dth = s->th1 - s->th2;
    const double sd = sin(dth), cd = cos(dth);
    const double den = 2.0 * M1 + M2 - M2 * cos(2.0 * dth);
    const double w1s = s->w1 * s->w1, w2s = s->w2 * s->w2;

    const double num1 = -G * (2.0 * M1 + M2) * st1
                        - M2 * G * sin(s->th1 - 2.0 * s->th2)
                        - 2.0 * sd * M2 * (w2s * L2 + w1s * L1 * cd);
    const double num2 = 2.0 * sd * ( w1s * L1 * (M1 + M2)
                                     + G * (M1 + M2) * cos(s->th1)
                                     + w2s * L2 * M2 * cd );
    d[0] = s->w1;
    d[1] = num1 / (L1 * den);
    d[2] = s->w2;
    d[3] = num2 / (L2 * den);
}

static inline State rk4(State s, double h)
{
    double k1[4], k2[4], k3[4], k4[4];
    State t;
    deriv(&s, k1);
    t.th1 = s.th1 + 0.5*h*k1[0]; t.w1 = s.w1 + 0.5*h*k1[1];
    t.th2 = s.th2 + 0.5*h*k1[2]; t.w2 = s.w2 + 0.5*h*k1[3];
    deriv(&t, k2);
    t.th1 = s.th1 + 0.5*h*k2[0]; t.w1 = s.w1 + 0.5*h*k2[1];
    t.th2 = s.th2 + 0.5*h*k2[2]; t.w2 = s.w2 + 0.5*h*k2[3];
    deriv(&t, k3);
    t.th1 = s.th1 + h*k3[0]; t.w1 = s.w1 + h*k3[1];
    t.th2 = s.th2 + h*k3[2]; t.w2 = s.w2 + h*k3[3];
    deriv(&t, k4);
    s.th1 += h*(k1[0] + 2*k2[0] + 2*k3[0] + k4[0]) / 6.0;
    s.w1  += h*(k1[1] + 2*k2[1] + 2*k3[1] + k4[1]) / 6.0;
    s.th2 += h*(k1[2] + 2*k2[2] + 2*k3[2] + k4[2]) / 6.0;
    s.w2  += h*(k1[3] + 2*k2[3] + 2*k3[3] + k4[3]) / 6.0;
    return s;
}

static inline double energy(const State *s)
{
    const double t1 = s->th1, t2 = s->th2;
    const double w1 = s->w1, w2 = s->w2;
    const double T = 0.5 * g_phys.m1 * g_phys.l1 * g_phys.l1 * w1 * w1
                   + 0.5 * g_phys.m2 * (g_phys.l1 * g_phys.l1 * w1 * w1
                       + g_phys.l2 * g_phys.l2 * w2 * w2
                       + 2.0 * g_phys.l1 * g_phys.l2 * w1 * w2 * cos(t1 - t2));
    const double V = -(g_phys.m1 + g_phys.m2) * g_phys.g * g_phys.l1 * cos(t1)
                     - g_phys.m2 * g_phys.g * g_phys.l2 * cos(t2);
    return T + V;
}

static inline double wrap_angle(double a)
{
    a = fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}


/* ---------- 双摆窗口 ---------- */
#define TRAIL_MAX 24000   /* 走完一圈不消失 */
#define PTRAL_MAX 12000
#define DRAW_DT    0.002   /* 轨迹绘制采样间隔（物理秒），与倍速无关 */

/* ---- 声音：物理时间 1kHz 采样，存 30 秒 ---- */
#define SND_HZ   1000
#define SND_MAX  (30 * SND_HZ)
#define SCOPE_SEC 3.0

typedef struct {
    State  s;
    int    space;          /* 0=角度空间 1=角动量空间 */
    double th1_0, th2_0;   /* 初始角度（用于显示，不随演化改变） */
    double w1_0, w2_0;     /* 初始角速度 */
    double t;              /* 已模拟的物理时间 */
    double speed;          /* 倍速 */
    double acc;            /* 绘制采样累加器 */
    BOOL   paused;
    int    trailHead, trailCount;
    int    cursor;          /* -1 = 跟随实时；>=0 = 回看第 cursor 个轨迹点 */
    double tr1[TRAIL_MAX], tr2[TRAIL_MAX];   /* 摆的轨迹（角度，画时再算位置） */
    int    pHead, pCount;
    float  pth1[PTRAL_MAX], pth2[PTRAL_MAX]; /* θ1-θ2 状态图轨迹（已 wrap） */
    HBITMAP bgBmp;
    int     bgW, bgH;

    /* 声音采样（unwrap 的连续角，单位 rad） */
    float   sndL[SND_MAX], sndR[SND_MAX];
    int     sndHead, sndCount;
    double  sndAcc;
    double  uw1, uw2;       /* 连续角 */
} PendView;

/* 双摆窗口入口 */
void open_pendulum_window(double th1, double th2);
extern const WCHAR *PEND_CLASS;
LRESULT CALLBACK PendWndProc(HWND, UINT, WPARAM, LPARAM);


#endif
