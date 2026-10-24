/*
 * cl_gpu.c —— 双摆李雅普诺夫场的 OpenCL 计算后端
 *
 * 特点：
 *   - 运行时动态加载 OpenCL.dll，不需要 OpenCL SDK / 头文件
 *   - 内核在设备上即时编译（NVIDIA 驱动自带编译器）
 *   - 每个像素一个 work-item，按行分块提交，便于渐进显示与取消
 */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "cl_gpu.h"

/* ------------------------------------------------------------------ */
/* OpenCL 最小声明                                                      */
/* ------------------------------------------------------------------ */

typedef int32_t  cl_int;
typedef uint32_t cl_uint;
typedef uint64_t cl_ulong;
typedef struct _cl_platform_id  *cl_platform_id;
typedef struct _cl_device_id    *cl_device_id;
typedef struct _cl_context      *cl_context;
typedef struct _cl_command_queue *cl_command_queue;
typedef struct _cl_mem          *cl_mem;
typedef struct _cl_program      *cl_program;
typedef struct _cl_kernel       *cl_kernel;

#define CL_SUCCESS                  0
#define CL_DEVICE_TYPE_ALL          0xFFFFFFFFu
#define CL_DEVICE_NAME              0x102B
#define CL_DEVICE_EXTENSIONS        0x1030
#define CL_MEM_READ_WRITE           (1u << 0)
#define CL_MEM_WRITE_ONLY           (1u << 1)
#define CL_PROGRAM_BUILD_LOG        0x1183

typedef cl_int  (WINAPI *PFN_clGetPlatformIDs)(cl_uint, cl_platform_id *, cl_uint *);
typedef cl_int  (WINAPI *PFN_clGetDeviceIDs)(cl_platform_id, cl_uint, cl_uint, cl_device_id *, cl_uint *);
typedef cl_int  (WINAPI *PFN_clGetDeviceInfo)(cl_device_id, cl_uint, size_t, void *, size_t *);
typedef cl_context (WINAPI *PFN_clCreateContext)(const cl_int *, cl_uint, const cl_device_id *, void *, void *, cl_int *);
typedef cl_command_queue (WINAPI *PFN_clCreateCommandQueue)(cl_context, cl_device_id, cl_ulong, cl_int *);
typedef cl_program (WINAPI *PFN_clCreateProgramWithSource)(cl_context, cl_uint, const char **, const size_t *, cl_int *);
typedef cl_int  (WINAPI *PFN_clBuildProgram)(cl_program, cl_uint, const cl_device_id *, const char *, void *, void *);
typedef cl_int  (WINAPI *PFN_clGetProgramBuildInfo)(cl_program, cl_device_id, cl_uint, size_t, void *, size_t *);
typedef cl_kernel (WINAPI *PFN_clCreateKernel)(cl_program, const char *, cl_int *);
typedef cl_mem  (WINAPI *PFN_clCreateBuffer)(cl_context, cl_ulong, size_t, void *, cl_int *);
typedef cl_int  (WINAPI *PFN_clSetKernelArg)(cl_kernel, cl_uint, size_t, const void *);
typedef cl_int  (WINAPI *PFN_clEnqueueNDRangeKernel)(cl_command_queue, cl_kernel, cl_uint, const size_t *, const size_t *, const size_t *, cl_uint, const void *, void *);
typedef cl_int  (WINAPI *PFN_clEnqueueReadBuffer)(cl_command_queue, cl_mem, cl_uint, size_t, size_t, void *, cl_uint, const void *, void *);
typedef cl_int  (WINAPI *PFN_clFinish)(cl_command_queue);
typedef cl_int  (WINAPI *PFN_clReleaseMemObject)(cl_mem);
typedef cl_int  (WINAPI *PFN_clReleaseKernel)(cl_kernel);
typedef cl_int  (WINAPI *PFN_clReleaseProgram)(cl_program);
typedef cl_int  (WINAPI *PFN_clReleaseCommandQueue)(cl_command_queue);
typedef cl_int  (WINAPI *PFN_clReleaseContext)(cl_context);

static PFN_clGetPlatformIDs         p_clGetPlatformIDs;
static PFN_clGetDeviceIDs           p_clGetDeviceIDs;
static PFN_clGetDeviceInfo          p_clGetDeviceInfo;
static PFN_clCreateContext          p_clCreateContext;
static PFN_clCreateCommandQueue     p_clCreateCommandQueue;
static PFN_clCreateProgramWithSource p_clCreateProgramWithSource;
static PFN_clBuildProgram           p_clBuildProgram;
static PFN_clGetProgramBuildInfo    p_clGetProgramBuildInfo;
static PFN_clCreateKernel           p_clCreateKernel;
static PFN_clCreateBuffer           p_clCreateBuffer;
static PFN_clSetKernelArg           p_clSetKernelArg;
static PFN_clEnqueueNDRangeKernel   p_clEnqueueNDRangeKernel;
static PFN_clEnqueueReadBuffer      p_clEnqueueReadBuffer;
static PFN_clFinish                 p_clFinish;
static PFN_clReleaseMemObject       p_clReleaseMemObject;
static PFN_clReleaseKernel          p_clReleaseKernel;
static PFN_clReleaseProgram         p_clReleaseProgram;
static PFN_clReleaseCommandQueue    p_clReleaseCommandQueue;
static PFN_clReleaseContext         p_clReleaseContext;

/* ------------------------------------------------------------------ */
/* 状态                                                                */
/* ------------------------------------------------------------------ */

static HMODULE  g_dll = NULL;
static cl_device_id     g_dev = NULL;
static cl_context       g_ctx = NULL;
static cl_command_queue g_queue = NULL;
static cl_program       g_prog = NULL;
typedef struct { double m1, m2, l1, l2, g; } PhysParams;
static PhysParams g_phys  = { 1.0, 1.0, 1.0, 1.0, 9.81 };
static PhysParams g_kphys = { 1.0, 1.0, 1.0, 1.0, 9.81 };
static int phys_changed(void)
{
    return g_phys.m1 != g_kphys.m1 || g_phys.m2 != g_kphys.m2 ||
           g_phys.l1 != g_kphys.l1 || g_phys.l2 != g_kphys.l2 ||
           g_phys.g  != g_kphys.g;
}
void gpu_set_physics(double m1, double m2, double l1, double l2, double g)
{
    g_phys.m1 = m1; g_phys.m2 = m2; g_phys.l1 = l1; g_phys.l2 = l2; g_phys.g = g;
}
static cl_kernel        g_kernel = NULL;
static int              g_fast = 0;       /* float 内核是否用 fast-math */
static cl_program       g_prog64 = NULL;    /* 双精度版，惰性构建 */
static cl_program       g_progFF = NULL;    /* float-float 版，惰性构建 */
static cl_kernel        g_kernelFF = NULL;
static cl_kernel        g_kernel64 = NULL;
static int              g_has_double = 0;
static int              g_ok = 0;
static char             g_devname[256] = "CPU";
static char             g_err[512] = "";

/* ------------------------------------------------------------------ */
/* 内核源码                                                             */
/* ------------------------------------------------------------------ */

/*
 * 与 CPU 版完全一致的方程（m1=m2=1, l1=l2=1, g=9.81）：
 *   den  = 3 - cos(2*(th1-th2))
 *   dd1  = (-9.81*3*sin(th1) - 9.81*sin(th1-2*th2)
 *           - 2*sin(th1-th2)*(w2^2 + w1^2*cos(th1-th2))) / den
 *   dd2  = ( 2*sin(th1-th2)*(2*w1^2 + 19.62*cos(th1) + w2^2*cos(th1-th2)) ) / den
 */
/* 内核正文：用 real 抽象精度；build 时在前面拼上对应的 typedef。
 * 双精度版只是把 real 换成 double，状态积分和坐标都用 fp64，
 * 这样放大到很小的尺度时不会因为 float 量化而出现伪随机噪声。 */
static const char *KERNEL_HEAD_F = "typedef float real;\n";
static const char *KERNEL_HEAD_D =
    "#pragma OPENCL EXTENSION cl_khr_fp64 : enable\n"
    "typedef double real;\n";

static const char *KERNEL_BODY =
"static void deriv(const real *s, real *d)\n"
"{\n"
"  real st1 = sin(s[0]);\n"
"  real ct1 = cos(s[0]);\n"
"  real dth = s[0] - s[2];\n"
"  real sd  = sin(dth);\n"
"  real cd  = cos(dth);\n"
"  real c2  = cos(2.0f * dth);\n"
"  real base = 2.0f * MP1 + MP2 - MP2 * c2;\n"
"  real w1s = s[1] * s[1];\n"
"  real w2s = s[3] * s[3];\n"
"  real num1 = -GP * (2.0f * MP1 + MP2) * st1\n"
"               - MP2 * GP * sin(s[0] - 2.0f * s[2])\n"
"               - 2.0f * sd * MP2 * (w2s * LP2 + w1s * LP1 * cd);\n"
"  real num2 = 2.0f * sd * (w1s * LP1 * (MP1 + MP2)\n"
"               + GP * (MP1 + MP2) * ct1\n"
"               + w2s * LP2 * MP2 * cd);\n"
"  d[0] = s[1];\n"
"  d[1] = num1 / (LP1 * base);\n"
"  d[2] = s[3];\n"
"  d[3] = num2 / (LP2 * base);\n"
"}\n"
"static void rk4(real *s, real h)\n"
"{\n"
"  real k1[4], k2[4], k3[4], k4[4], t[4];\n"
"  int q;\n"
"  deriv(s, k1);\n"
"  for (q = 0; q < 4; ++q) t[q] = s[q] + 0.5f * h * k1[q];\n"
"  deriv(t, k2);\n"
"  for (q = 0; q < 4; ++q) t[q] = s[q] + 0.5f * h * k2[q];\n"
"  deriv(t, k3);\n"
"  for (q = 0; q < 4; ++q) t[q] = s[q] + h * k3[q];\n"
"  deriv(t, k4);\n"
"  for (q = 0; q < 4; ++q)\n"
"    s[q] += h * (k1[q] + 2.0f * k2[q] + 2.0f * k3[q] + k4[q]) * 0.16666667f;\n"
"}\n"
"__kernel void chaos(__global float *out, int W, int H,\n"
"                    real a1, real b1, real a2, real b2,\n"
"                    int nsteps, real h, real d0, int mirror,\n"
"                    int space, real f1, real f2)\n"
"{\n"
"  int i = get_global_id(0);\n"
"  int j = get_global_id(1);\n"
"  int halfw = mirror ? (W >> 1) : W;\n"
"  if (i >= halfw || j >= H) return;\n"
"\n"
"  real u = a1 + (b1 - a1) * ((real)i + 0.5f) / (real)W;\n"
"  real v = b2 - (b2 - a2) * ((real)j + 0.5f) / (real)H;\n"
"\n"
"  real A[4];\n"
"  real B[4];\n"
"  if (space == 0) { A[0] = u; A[1] = f1; A[2] = v; A[3] = f2; }\n"
"  else            { A[0] = f1; A[1] = u; A[2] = f2; A[3] = v; }\n"
"  for (int q = 0; q < 4; ++q) B[q] = A[q];\n"
"  B[space == 0 ? 0 : 1] = A[space == 0 ? 0 : 1] + d0;\n"
"\n"
"  real acc = 0;\n"
"  real lam = 0;\n"
"  int ok = 1;\n"
"  for (int n = 0; n < nsteps; ++n) {\n"
"    rk4(A, h);\n"
"    rk4(B, h);\n"
"    real dt1 = B[0] - A[0];\n"
"    real dw1 = B[1] - A[1];\n"
"    real dt2 = B[2] - A[2];\n"
"    real dw2 = B[3] - A[3];\n"
"    real d = sqrt(dt1 * dt1 + dw1 * dw1 + dt2 * dt2 + dw2 * dw2);\n"
"    if (!(d > 0) || !isfinite(d)) { ok = 0; break; }\n"
"    acc += log(d / d0);\n"
"    real sc = d0 / d;\n"
"    B[0] = A[0] + dt1 * sc;\n"
"    B[1] = A[1] + dw1 * sc;\n"
"    B[2] = A[2] + dt2 * sc;\n"
"    B[3] = A[3] + dw2 * sc;\n"
"  }\n"
"  if (ok) lam = acc / ((real)nsteps * h);\n"
"  out[j * W + i] = (float)lam;\n"
"  if (mirror) out[(H - 1 - j) * W + (W - 1 - i)] = (float)lam;\n"
"}\n";

/* ------------------------------------------------------------------ */
/* 加载与初始化                                                         */
/* ------------------------------------------------------------------ */

static void *sym(const char *name)
{
    void *p = (void *)GetProcAddress(g_dll, name);
    if (!p) {
        snprintf(g_err, sizeof(g_err), "OpenCL.dll 缺少符号 %s", name);
        return NULL;
    }
    return p;
}

static int load_api(void)
{
#define LD(field, name) do { field = (void *)sym(name); if (!field) return -1; } while (0)
    LD(p_clGetPlatformIDs,          "clGetPlatformIDs");
    LD(p_clGetDeviceIDs,            "clGetDeviceIDs");
    LD(p_clGetDeviceInfo,           "clGetDeviceInfo");
    LD(p_clCreateContext,           "clCreateContext");
    LD(p_clCreateCommandQueue,      "clCreateCommandQueue");
    LD(p_clCreateProgramWithSource, "clCreateProgramWithSource");
    LD(p_clBuildProgram,            "clBuildProgram");
    LD(p_clGetProgramBuildInfo,     "clGetProgramBuildInfo");
    LD(p_clCreateKernel,            "clCreateKernel");
    LD(p_clCreateBuffer,            "clCreateBuffer");
    LD(p_clSetKernelArg,            "clSetKernelArg");
    LD(p_clEnqueueNDRangeKernel,    "clEnqueueNDRangeKernel");
    LD(p_clEnqueueReadBuffer,       "clEnqueueReadBuffer");
    LD(p_clFinish,                  "clFinish");
    LD(p_clReleaseMemObject,        "clReleaseMemObject");
    LD(p_clReleaseKernel,           "clReleaseKernel");
    LD(p_clReleaseProgram,          "clReleaseProgram");
    LD(p_clReleaseCommandQueue,     "clReleaseCommandQueue");
    LD(p_clReleaseContext,          "clReleaseContext");
#undef LD
    return 0;
}

/* float-float 内核（两个 fp32 拼约 48 位有效数字，9 阶多项式） */
static const char *KERNEL_FF_BODY =
"typedef struct { float hi, lo; } ff;\n"
"static inline ff ff_mk(float h, float l) { ff r; r.hi = h; r.lo = l; return r; }\n"
"static inline ff ff_f(float h) { ff r; r.hi = h; r.lo = 0.0f; return r; }\n"
"static inline ff ff_add(ff a, ff b)\n"
"{\n"
"  float s = a.hi + b.hi;\n"
"  float bb = s - a.hi;\n"
"  float e = (a.hi - (s - bb)) + (b.hi - bb);\n"
"  e += a.lo + b.lo;\n"
"  float r = s + e;\n"
"  return ff_mk(r, e - (r - s));\n"
"}\n"
"static inline ff ff_sub(ff a, ff b) { return ff_add(a, ff_mk(-b.hi, -b.lo)); }\n"
"static inline ff ff_neg(ff a) { return ff_mk(-a.hi, -a.lo); }\n"
"static inline ff ff_mul(ff a, ff b)\n"
"{\n"
"  float p = a.hi * b.hi;\n"
"  float pe = fma(a.hi, b.hi, -p);\n"
"  pe += a.hi * b.lo + a.lo * b.hi;\n"
"  float r = p + pe;\n"
"  return ff_mk(r, pe - (r - p));\n"
"}\n"
"static inline ff ff_scale(ff a, float s)\n"
"{\n"
"  float p = a.hi * s;\n"
"  float pe = fma(a.hi, s, -p) + a.lo * s;\n"
"  float r = p + pe;\n"
"  return ff_mk(r, pe - (r - p));\n"
"}\n"
"static inline ff ff_div(ff a, ff b)\n"
"{\n"
"  float q1 = a.hi / b.hi;\n"
"  float p = q1 * b.hi;\n"
"  float pe = fma(q1, b.hi, -p);\n"
"  float d = ((a.hi - p) - pe) + a.lo - q1 * b.lo;\n"
"  float q2 = d / b.hi;\n"
"  float q = q1 + q2;\n"
"  return ff_mk(q, (q1 - q) + q2);\n"
"}\n"
"static inline ff ff_log(ff a)\n"
"{\n"
"  float l = log(a.hi);\n"
"  float d = a.lo / a.hi;\n"
"  float r = l + d;\n"
"  return ff_mk(r, (l - r) + d);\n"
"}\n"
"static inline ff ff_sqrt(ff a)\n"
"{\n"
"  float s = sqrt(a.hi);\n"
"  float sq = s * s;\n"
"  float e = fma(s, s, -sq);\n"
"  e = ((a.hi - sq) - e) + a.lo;\n"
"  return ff_mk(s, e / (2.0f * s));\n"
"}\n"
"static const ff CO_S10 = { -2.505210794e-08f, -4.417623045e-16f };\n"
"static const ff CO_S8 = { 2.755731884e-06f, 3.793571224e-14f };\n"
"static const ff CO_S6 = { -1.984127011e-04f, 2.725596875e-12f };\n"
"static const ff CO_S4 = { 8.333333768e-03f, -4.346172033e-10f };\n"
"static const ff CO_S2 = { -1.666666716e-01f, 4.967053879e-09f };\n"
"static const ff CO_C10 = { -2.755731998e-07f, 7.575112209e-15f };\n"
"static const ff CO_C8 = { 2.480158764e-05f, -3.406996094e-13f };\n"
"static const ff CO_C6 = { -1.388888923e-03f, 3.363109444e-11f };\n"
"static const ff CO_C4 = { 4.166666791e-02f, -1.241763470e-09f };\n"
"static const ff CO_C2 = { -5.000000000e-01f, 0.000000000e+00f };\n"
"static const ff PIO2F = { 1.570796371e+00f, -4.371138829e-08f };\n"
"static inline void sincos_ff(ff x, ff *ps, ff *pc)\n"
"{\n"
"  float kf = rint(x.hi * 0.6366197723675813f);\n"
"  int k = (int)kf;\n"
"  ff y = ff_sub(x, ff_scale(PIO2F, kf));\n"
"  ff y2 = ff_mul(y, y);\n"
"  ff t = CO_S8;\n"
"  t = ff_add(ff_mul(t, y2), CO_S6);\n"
"  t = ff_add(ff_mul(t, y2), CO_S4);\n"
"  t = ff_add(ff_mul(t, y2), CO_S2);\n"
"  t = ff_add(ff_mul(t, y2), ff_f(1.0f));\n"
"  ff s = ff_mul(y, t);\n"
"  ff c = CO_C8;\n"
"  c = ff_add(ff_mul(c, y2), CO_C6);\n"
"  c = ff_add(ff_mul(c, y2), CO_C4);\n"
"  c = ff_add(ff_mul(c, y2), CO_C2);\n"
"  c = ff_add(ff_mul(c, y2), ff_f(1.0f));\n"
"  int q = k & 3;\n"
"  if (q == 0)      { *ps = s;         *pc = c; }\n"
"  else if (q == 1) { *ps = c;         *pc = ff_neg(s); }\n"
"  else if (q == 2) { *ps = ff_neg(s); *pc = ff_neg(c); }\n"
"  else             { *ps = ff_neg(c); *pc = s; }\n"
"}\n"
"typedef struct { ff t1, w1, t2, w2; } St;\n"
"static inline void deriv_ff(St *s, ff *k)\n"
"{\n"
"  ff a = ff_sub(s->t1, s->t2);\n"
"  ff SA, CA;  sincos_ff(a, &SA, &CA);\n"
"  ff ST1, CT1; sincos_ff(s->t1, &ST1, &CT1);\n"
"  ff b = ff_sub(s->t1, ff_scale(s->t2, 2.0f));\n"
"  ff SB, CB;  sincos_ff(b, &SB, &CB);\n"
"  (void)CB;\n"
"  ff w1s = ff_mul(s->w1, s->w1);\n"
"  ff w2s = ff_mul(s->w2, s->w2);\n"
"  ff c2  = ff_sub(ff_mul(CA, CA), ff_mul(SA, SA));\n"
"  ff base = ff_sub(ff_f(2.0f * MP1 + MP2), ff_scale(c2, MP2));\n"
"  ff D1 = ff_scale(base, LP1);\n"
"  ff D2 = ff_scale(base, LP2);\n"
"  ff t1 = ff_scale(ST1, -GP * (2.0f * MP1 + MP2));\n"
"  ff t2 = ff_scale(SB, -MP2 * GP);\n"
"  ff t3 = ff_scale(ff_mul(SA, ff_add(ff_scale(w2s, LP2),\n"
"                                     ff_mul(ff_scale(w1s, LP1), CA))), -2.0f * MP2);\n"
"  ff n1 = ff_add(ff_add(t1, t2), t3);\n"
"  ff u1 = ff_scale(w1s, LP1 * (MP1 + MP2));\n"
"  ff u2 = ff_scale(CT1, GP * (MP1 + MP2));\n"
"  ff u3 = ff_mul(ff_scale(w2s, LP2 * MP2), CA);\n"
"  ff n2 = ff_scale(ff_mul(SA, ff_add(ff_add(u1, u2), u3)), 2.0f);\n"
"  k[0] = s->w1;\n"
"  k[1] = ff_div(n1, D1);\n"
"  k[2] = s->w2;\n"
"  k[3] = ff_div(n2, D2);\n"
"}\n"
"static inline void rk4_ff(St *s, float h)\n"
"{\n"
"  ff k1[4], k2[4], k3[4], k4[4];\n"
"  St t;\n"
"  deriv_ff(s, k1);\n"
"  t.t1 = ff_add(s->t1, ff_scale(k1[0], 0.5f * h));\n"
"  t.w1 = ff_add(s->w1, ff_scale(k1[1], 0.5f * h));\n"
"  t.t2 = ff_add(s->t2, ff_scale(k1[2], 0.5f * h));\n"
"  t.w2 = ff_add(s->w2, ff_scale(k1[3], 0.5f * h));\n"
"  deriv_ff(&t, k2);\n"
"  t.t1 = ff_add(s->t1, ff_scale(k2[0], 0.5f * h));\n"
"  t.w1 = ff_add(s->w1, ff_scale(k2[1], 0.5f * h));\n"
"  t.t2 = ff_add(s->t2, ff_scale(k2[2], 0.5f * h));\n"
"  t.w2 = ff_add(s->w2, ff_scale(k2[3], 0.5f * h));\n"
"  deriv_ff(&t, k3);\n"
"  t.t1 = ff_add(s->t1, ff_scale(k3[0], h));\n"
"  t.w1 = ff_add(s->w1, ff_scale(k3[1], h));\n"
"  t.t2 = ff_add(s->t2, ff_scale(k3[2], h));\n"
"  t.w2 = ff_add(s->w2, ff_scale(k3[3], h));\n"
"  deriv_ff(&t, k4);\n"
"  ff c = ff_f(h / 6.0f);\n"
"  s->t1 = ff_add(s->t1, ff_mul(ff_add(ff_add(k1[0], ff_scale(k2[0], 2.0f)),\n"
"                                    ff_add(ff_scale(k3[0], 2.0f), k4[0])), c));\n"
"  s->w1 = ff_add(s->w1, ff_mul(ff_add(ff_add(k1[1], ff_scale(k2[1], 2.0f)),\n"
"                                    ff_add(ff_scale(k3[1], 2.0f), k4[1])), c));\n"
"  s->t2 = ff_add(s->t2, ff_mul(ff_add(ff_add(k1[2], ff_scale(k2[2], 2.0f)),\n"
"                                    ff_add(ff_scale(k3[2], 2.0f), k4[2])), c));\n"
"  s->w2 = ff_add(s->w2, ff_mul(ff_add(ff_add(k1[3], ff_scale(k2[3], 2.0f)),\n"
"                                    ff_add(ff_scale(k3[3], 2.0f), k4[3])), c));\n"
"}\n"
"__kernel void chaos_ff(__global float *out, int W, int H,\n"
"                       real a1, real b1, real a2, real b2,\n"
"                       int nsteps, real h, real d0, int mirror,\n"
"                       int space, real f1, real f2)\n"
"{\n"
"  int i = get_global_id(0), j = get_global_id(1);\n"
"  int halfw = mirror ? (W >> 1) : W;\n"
"  if (i >= halfw || j >= H) return;\n"
"  float u = a1 + (b1 - a1) * ((float)i + 0.5f) / (float)W;\n"
"  float v = b2 - (b2 - a2) * ((float)j + 0.5f) / (float)H;\n"
"  St A, B;\n"
"  if (space == 0) { A.t1 = ff_f(u); A.w1 = ff_f(f1); A.t2 = ff_f(v); A.w2 = ff_f(f2); }\n"
"  else            { A.t1 = ff_f(f1); A.w1 = ff_f(u); A.t2 = ff_f(f2); A.w2 = ff_f(v); }\n"
"  B = A;\n"
"  if (space == 0) B.t1 = ff_add(B.t1, ff_f((float)d0));\n"
"  else            B.w1 = ff_add(B.w1, ff_f((float)d0));\n"
"  ff acc = ff_f(0.0f);\n"
"  float hf = (float)h;\n"
"  ff d0f = ff_f((float)d0);\n"
"  for (int n = 0; n < nsteps; ++n) {\n"
"    rk4_ff(&A, hf);\n"
"    rk4_ff(&B, hf);\n"
"    ff d1 = ff_sub(B.t1, A.t1);\n"
"    ff d2 = ff_sub(B.w1, A.w1);\n"
"    ff d3 = ff_sub(B.t2, A.t2);\n"
"    ff d4 = ff_sub(B.w2, A.w2);\n"
"    ff dd = ff_add(ff_add(ff_mul(d1, d1), ff_mul(d2, d2)),\n"
"                   ff_add(ff_mul(d3, d3), ff_mul(d4, d4)));\n"
"    ff d = ff_sqrt(dd);\n"
"    acc = ff_add(acc, ff_log(ff_div(d, d0f)));\n"
"    ff sc = ff_div(d0f, d);\n"
"    B.t1 = ff_add(A.t1, ff_mul(d1, sc));\n"
"    B.w1 = ff_add(A.w1, ff_mul(d2, sc));\n"
"    B.t2 = ff_add(A.t2, ff_mul(d3, sc));\n"
"    B.w2 = ff_add(A.w2, ff_mul(d4, sc));\n"
"  }\n"
"  float res = acc.hi / ((float)nsteps * hf);\n"
"  out[j * W + i] = res;\n"
"  if (mirror) out[(H - 1 - j) * W + (W - 1 - i)] = res;\n"
"}\n";


/* 用给定的精度头文件构建内核；失败时把编译日志写进 g_err 并返回 NULL */
static cl_kernel build_kernel(const char *head, const char *label, cl_program *out_prog, int fast)
{
    cl_int errc = 0;
    const char *parts[2] = { head, KERNEL_BODY };

    cl_program prog = p_clCreateProgramWithSource(g_ctx, 2, parts, NULL, &errc);
    if (!prog) {
        snprintf(g_err, sizeof(g_err), "clCreateProgramWithSource(%s) 失败 (%d)", label, errc);
        return NULL;
    }

    /* fast 时用 -cl-fast-relaxed-math：sin/cos 走硬件近似（精度约 2^-21），
     * 画混沌图够用，实测快约 3.7 倍。双精度内核永远不开。 */
    char optbuf[256];
    snprintf(optbuf, sizeof optbuf,
             "-D MP1=%.9ff -D MP2=%.9ff -D LP1=%.9ff -D LP2=%.9ff -D GP=%.9ff%s",
             g_phys.m1, g_phys.m2, g_phys.l1, g_phys.l2, g_phys.g,
             fast ? " -cl-fast-relaxed-math" : "");
    const char *opts = optbuf;
    errc = p_clBuildProgram(prog, 1, (const cl_device_id *)&g_dev, opts, NULL, NULL);
    if (errc != CL_SUCCESS) {
        char log[8192] = {0};
        size_t ln = 0;
        p_clGetProgramBuildInfo(prog, g_dev, CL_PROGRAM_BUILD_LOG,
                                sizeof(log) - 1, log, &ln);
        snprintf(g_err, sizeof(g_err), "%s 内核编译失败:\n%s", label, log);
        p_clReleaseProgram(prog);
        return NULL;
    }

    cl_kernel k = p_clCreateKernel(prog, "chaos", &errc);
    if (!k) {
        snprintf(g_err, sizeof(g_err), "clCreateKernel(%s) 失败 (%d)", label, errc);
        p_clReleaseProgram(prog);
        return NULL;
    }
    *out_prog = prog;
    return k;
}

/* 需要双精度时才构建，失败就永久标记不可用 */
static int ensure_double_kernel(void)
{
    if (g_kernel64) return 1;
    if (!g_ok || !g_has_double) return 0;
    g_kernel64 = build_kernel(KERNEL_HEAD_D, "double", &g_prog64, 0);
    if (!g_kernel64) {
        g_has_double = 0;
        return 0;
    }
    return 1;
}

static int ensure_ff_kernel(void)
{
    if (g_kernelFF) return 1;
    if (!g_ok) return 0;
    cl_int errc = 0;
    const char *parts[2] = { KERNEL_HEAD_F, KERNEL_FF_BODY };
    cl_program prog = p_clCreateProgramWithSource(g_ctx, 2, parts, NULL, &errc);
    if (!prog) { snprintf(g_err, sizeof(g_err), "float-float 内核创建失败 (%d)", errc); return 0; }
    /* 必须带上与主内核相同的物理参数宏，否则 MP1/LP1/GP 未定义 */
    char optbuf[256];
    snprintf(optbuf, sizeof optbuf,
             "-D MP1=%.9ff -D MP2=%.9ff -D LP1=%.9ff -D LP2=%.9ff -D GP=%.9ff",
             g_phys.m1, g_phys.m2, g_phys.l1, g_phys.l2, g_phys.g);
    errc = p_clBuildProgram(prog, 1, (const cl_device_id *)&g_dev, optbuf, NULL, NULL);
    if (errc != CL_SUCCESS) {
        char log[8192] = {0}; size_t ln = 0;
        p_clGetProgramBuildInfo(prog, g_dev, CL_PROGRAM_BUILD_LOG, sizeof(log) - 1, log, &ln);
        snprintf(g_err, sizeof(g_err), "float-float 内核编译失败:\n%s", log);
        p_clReleaseProgram(prog);
        return 0;
    }
    g_kernelFF = p_clCreateKernel(prog, "chaos_ff", &errc);
    if (!g_kernelFF) { snprintf(g_err, sizeof(g_err), "chaos_ff 获取失败 (%d)", errc); p_clReleaseProgram(prog); return 0; }
    g_progFF = prog;
    return 1;
}

int gpu_init(char *err, int errn)
{
    if (g_ok) return 0;

    g_err[0] = 0;
    g_dll = LoadLibraryA("OpenCL.dll");
    if (!g_dll) {
        snprintf(g_err, sizeof(g_err), "找不到 OpenCL.dll（请安装显卡驱动）");
        goto fail;
    }
    if (load_api() != 0) goto fail;

    cl_platform_id plats[8];
    cl_uint np = 0;
    if (p_clGetPlatformIDs(8, plats, &np) != CL_SUCCESS || np == 0) {
        snprintf(g_err, sizeof(g_err), "没有可用的 OpenCL 平台");
        goto fail;
    }

    cl_int errc = 0;
    g_dev = NULL;
    for (cl_uint i = 0; i < np && !g_dev; ++i) {
        cl_uint nd = 0;
        if (p_clGetDeviceIDs(plats[i], CL_DEVICE_TYPE_ALL, 1, &g_dev, &nd) != CL_SUCCESS)
            g_dev = NULL;
    }
    if (!g_dev) {
        snprintf(g_err, sizeof(g_err), "没有可用的 OpenCL 设备");
        goto fail;
    }

    g_ctx = p_clCreateContext(NULL, 1, (const cl_device_id *)&g_dev, NULL, NULL, &errc);
    if (!g_ctx) { snprintf(g_err, sizeof(g_err), "clCreateContext 失败 (%d)", errc); goto fail; }

    g_queue = p_clCreateCommandQueue(g_ctx, g_dev, 0, &errc);
    if (!g_queue) { snprintf(g_err, sizeof(g_err), "clCreateCommandQueue 失败 (%d)", errc); goto fail; }

    /* 环境变量 CHAOS_FAST=1 只作为初始默认值，界面上可随时切换 */
    g_fast = getenv("CHAOS_FAST") ? 1 : 0;

    g_kernel = build_kernel(KERNEL_HEAD_F, "float", &g_prog, g_fast);
    if (!g_kernel) goto fail;

    /* 看看设备支不支持 fp64；支持就先记下来，真正构建留到用户勾选时 */
    {
        char ext[4096] = {0};
        if (p_clGetDeviceInfo(g_dev, CL_DEVICE_EXTENSIONS, sizeof(ext) - 1, ext, NULL) == CL_SUCCESS)
            g_has_double = (strstr(ext, "cl_khr_fp64") != NULL);
    }

    p_clGetDeviceInfo(g_dev, CL_DEVICE_NAME, sizeof(g_devname) - 1, g_devname, NULL);
    g_ok = 1;
    return 0;

fail:
    if (err && errn > 0) { strncpy(err, g_err, (size_t)errn - 1); err[errn - 1] = 0; }
    gpu_shutdown();
    return -1;
}

void gpu_shutdown(void)
{
    if (g_kernel64) { p_clReleaseKernel(g_kernel64); g_kernel64 = NULL; }
    if (g_prog64)   { p_clReleaseProgram(g_prog64); g_prog64 = NULL; }
    if (g_kernel) { p_clReleaseKernel(g_kernel); g_kernel = NULL; }
    if (g_prog)   { p_clReleaseProgram(g_prog); g_prog = NULL; }
    if (g_queue)  { p_clReleaseCommandQueue(g_queue); g_queue = NULL; }
    if (g_ctx)    { p_clReleaseContext(g_ctx); g_ctx = NULL; }
    g_dev = NULL;
    g_ok = 0;
    if (g_dll) { FreeLibrary(g_dll); g_dll = NULL; }
}

int gpu_ready(void) { return g_ok; }

int gpu_has_double(void) { return g_ok && g_has_double; }

int gpu_is_fast(void) { return g_fast; }

/* 切换 float 内核的 fast-math 编译选项：销毁旧内核再重建。
 * 调用者必须先保证没有计算任务在跑（gpu_compute 已返回）。 */
int gpu_set_fast(int on)
{
    on = on ? 1 : 0;
    if (!g_ok) return -1;
    if (on == g_fast) return 0;

    if (g_kernel) { p_clReleaseKernel(g_kernel); g_kernel = NULL; }
    if (g_prog)   { p_clReleaseProgram(g_prog); g_prog = NULL; }

    g_fast = on;
    g_kernel = build_kernel(KERNEL_HEAD_F, "float", &g_prog, g_fast);
    if (!g_kernel) {
        /* 构建失败：退回原来的设置，保证还有内核可用 */
        g_fast = !on;
        g_kernel = build_kernel(KERNEL_HEAD_F, "float", &g_prog, g_fast);
        if (g_kernel) g_fast = !on;
        return -1;
    }
    return 0;
}

void gpu_device_name(char *buf, int n)
{
    if (!buf || n <= 0) return;
    strncpy(buf, g_devname, (size_t)n - 1);
    buf[n - 1] = 0;
}

/* ------------------------------------------------------------------ */
/* 计算                                                                 */
/* ------------------------------------------------------------------ */

#define BAND 64          /* 每次提交的行数 */
#define DELTA0 1e-4f     /* float 下可稳定表示的扰动 */

/* 利用点反演对称 lambda(th1,th2) = lambda(-th1,-th2)：只算左半，右半镜像填入。
 * 验证见 verify_sym.c：镜像点用取反的扰动时两个 lambda 逐位相同。 */
static int g_mirror = 1;
void gpu_set_mirror(int on) { g_mirror = on ? 1 : 0; }

int gpu_compute(const double rect[4], int W, int H, double T, int precision,
                int space, double f1, double f2,
                float *out, volatile long *cancel,
                void (*progress)(int, int, void *), void *ud)
{
    if (!g_ok || !out || W <= 0 || H <= 0) return -2;

    /* 物理参数变了：内核里的系数是编译期 -D 常量，必须重建 */
    if (phys_changed()) {
        if (g_kernel)   { p_clReleaseKernel(g_kernel);   g_kernel = NULL; }
        if (g_kernelFF) { p_clReleaseKernel(g_kernelFF); g_kernelFF = NULL;
                          if (g_progFF) { p_clReleaseProgram(g_progFF); g_progFF = NULL; } }
        if (g_kernel64) { p_clReleaseKernel(g_kernel64); g_kernel64 = NULL;
                          if (g_prog64) { p_clReleaseProgram(g_prog64); g_prog64 = NULL; } }
        g_kernel = build_kernel(KERNEL_HEAD_F, "float", &g_prog, g_fast);
        if (!g_kernel) return -2;
        g_kphys = g_phys;
    }

    /* precision: 0=单精度 1=float-float 2=双精度（后两者惰性构建） */
    /* 角动量空间的初始 |ω| 可达 25 rad/s，float 的每步舍入误差在 T=30 内会
       盖过 1e-4 的初始扰动，把 λ 从 0.02 虚抬到 5 左右（实测），所以那里
       至少用 float-float 精度（比 double 快约 10 倍、精度接近）。 */
    if (space == 1 && precision < 1) precision = 1;
    int dbl = (precision == 2 && ensure_double_kernel());
    int usef = (!dbl && precision == 1 && ensure_ff_kernel());
    cl_kernel kern = dbl ? g_kernel64 : (usef ? g_kernelFF : g_kernel);
    if (dbl && !kern) { dbl = 0; kern = g_kernel; }
    if (usef && !kern) { usef = 0; kern = g_kernel; }

    const double hstep = 0.01;
    const int nsteps = (int)lround(T / hstep);

    cl_int errc = 0;
    cl_mem buf = p_clCreateBuffer(g_ctx, CL_MEM_WRITE_ONLY,
                                  (size_t)W * H * sizeof(float), NULL, &errc);
    if (!buf) return -2;

    int rc = 0;
    /* 点反演对称只对以 (0,0) 为中心的全域 [-pi,pi]^2 成立；
     * 放大后的子区域没有这个对称性，必须全算。 */
    int is_global = (fabs(rect[0] + M_PI) < 1e-9 && fabs(rect[1] - M_PI) < 1e-9 &&
                     fabs(rect[2] + M_PI) < 1e-9 && fabs(rect[3] - M_PI) < 1e-9);
    int mirror = (g_mirror && is_global && (W % 2 == 0) && (H % 2 == 0)) ? 1 : 0;

    p_clSetKernelArg(kern, 0, sizeof(cl_mem), &buf);
    p_clSetKernelArg(kern, 1, sizeof(int), &W);
    p_clSetKernelArg(kern, 2, sizeof(int), &H);

    if (dbl) {
        double a1 = rect[0], b1 = rect[1], a2 = rect[2], b2 = rect[3];
        double hd = hstep, d0 = 1e-9;
        p_clSetKernelArg(kern, 3, sizeof(double), &a1);
        p_clSetKernelArg(kern, 4, sizeof(double), &b1);
        p_clSetKernelArg(kern, 5, sizeof(double), &a2);
        p_clSetKernelArg(kern, 6, sizeof(double), &b2);
        p_clSetKernelArg(kern, 7, sizeof(int), &nsteps);
        p_clSetKernelArg(kern, 8, sizeof(double), &hd);
        p_clSetKernelArg(kern, 9, sizeof(double), &d0);
        p_clSetKernelArg(kern, 10, sizeof(int), &mirror);
        double df1 = f1, df2 = f2;
        p_clSetKernelArg(kern, 11, sizeof(int), &space);
        p_clSetKernelArg(kern, 12, sizeof(double), &df1);
        p_clSetKernelArg(kern, 13, sizeof(double), &df2);
    } else {
        float a1 = (float)rect[0], b1 = (float)rect[1];
        float a2 = (float)rect[2], b2 = (float)rect[3];
        float hf = (float)hstep, d0 = DELTA0;
        p_clSetKernelArg(kern, 3, sizeof(float), &a1);
        p_clSetKernelArg(kern, 4, sizeof(float), &b1);
        p_clSetKernelArg(kern, 5, sizeof(float), &a2);
        p_clSetKernelArg(kern, 6, sizeof(float), &b2);
        p_clSetKernelArg(kern, 7, sizeof(int), &nsteps);
        p_clSetKernelArg(kern, 8, sizeof(float), &hf);
        p_clSetKernelArg(kern, 9, sizeof(float), &d0);
        p_clSetKernelArg(kern, 10, sizeof(int), &mirror);
        float ff1 = (float)f1, ff2 = (float)f2;
        p_clSetKernelArg(kern, 11, sizeof(int), &space);
        p_clSetKernelArg(kern, 12, sizeof(float), &ff1);
        p_clSetKernelArg(kern, 13, sizeof(float), &ff2);
    }

    /* fp64 的吞吐只有 fp32 的 1/64 左右，一次提交太多行会超过 Windows
     * 驱动的 TDR 超时（约 2 s），GPU 被重置、上下文失效直接崩溃。
     * 所以双精度时把每次提交的行数压到很小。 */
    const int band_rows = dbl ? 8 : (usef ? 32 : BAND);

    if (!mirror) {
        const size_t gw = (size_t)(((W + 15) / 16) * 16);
        for (int jb = 0; jb < H; jb += band_rows) {
            if (cancel && *cancel) { rc = -1; break; }

            int band = H - jb;
            if (band > band_rows) band = band_rows;

            size_t off[2] = { 0, (size_t)jb };
            size_t gs[2]  = { gw, (size_t)band };

            errc = p_clEnqueueNDRangeKernel(g_queue, kern, 2, off, gs, NULL, 0, NULL, NULL);
            if (errc != CL_SUCCESS) { rc = -2; break; }

            errc = p_clEnqueueReadBuffer(g_queue, buf, 1 /*CL_TRUE*/,
                                         (size_t)jb * W * sizeof(float),
                                         (size_t)band * W * sizeof(float),
                                         out + (size_t)jb * W, 0, NULL, NULL);
            if (errc != CL_SUCCESS) { rc = -2; break; }

            if (progress) progress(jb, jb + band, ud);
        }
    } else {
        /* 成对提交：段 A=[ja0,ja1) 与它的镜像段 B=[H-ja1,H-ja0) 一起算，
         * 两块互为镜像，算完即可回读两段完整的行。 */
        const size_t gw = (size_t)((((W / 2) + 15) / 16) * 16);
        const int halfH = H / 2;

        for (int ja0 = 0; ja0 < halfH; ja0 += band_rows) {
            if (cancel && *cancel) { rc = -1; break; }

            int ja1 = ja0 + band_rows;
            if (ja1 > halfH) ja1 = halfH;
            int band = ja1 - ja0;
            int jb0 = H - ja1;            /* 镜像段起始行 */

            size_t offA[2] = { 0, (size_t)ja0 };
            size_t offB[2] = { 0, (size_t)jb0 };
            size_t gs[2]   = { gw, (size_t)band };

            errc = p_clEnqueueNDRangeKernel(g_queue, kern, 2, offA, gs, NULL, 0, NULL, NULL);
            if (errc != CL_SUCCESS) { rc = -2; break; }
            errc = p_clEnqueueNDRangeKernel(g_queue, kern, 2, offB, gs, NULL, 0, NULL, NULL);
            if (errc != CL_SUCCESS) { rc = -2; break; }

            errc = p_clEnqueueReadBuffer(g_queue, buf, 1 /*CL_TRUE*/,
                                         (size_t)ja0 * W * sizeof(float),
                                         (size_t)band * W * sizeof(float),
                                         out + (size_t)ja0 * W, 0, NULL, NULL);
            if (errc != CL_SUCCESS) { rc = -2; break; }
            errc = p_clEnqueueReadBuffer(g_queue, buf, 1 /*CL_TRUE*/,
                                         (size_t)jb0 * W * sizeof(float),
                                         (size_t)band * W * sizeof(float),
                                         out + (size_t)jb0 * W, 0, NULL, NULL);
            if (errc != CL_SUCCESS) { rc = -2; break; }

            if (progress) {
                progress(ja0, ja1, ud);
                progress(jb0, jb0 + band, ud);
            }
        }
    }

    p_clFinish(g_queue);
    p_clReleaseMemObject(buf);
    return rc;
}
