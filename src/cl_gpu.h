/*
 * cl_gpu.h —— 双摆李雅普诺夫场的 OpenCL 计算后端
 */
#ifndef CL_GPU_H
#define CL_GPU_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 OpenCL（动态加载 OpenCL.dll）。
 * 成功返回 0；失败返回 -1 并把原因写入 err（可为 NULL）。 */
int  gpu_init(char *err, int errn);

/* 释放所有 OpenCL 资源 */
void gpu_shutdown(void);

/* 是否初始化成功 */
int  gpu_ready(void);

/* 设备名，用于界面显示 */
void gpu_device_name(char *buf, int n);

/* 是否利用点反演对称性只算一半（默认开启）。W、H 为奇数时自动回退全算。 */
void gpu_set_mirror(int on);

/* 设备是否支持 fp64（决定“双精度”选项能不能用） */
int  gpu_has_double(void);

/* 是否启用 fast-math（sin/cos 走硬件近似，约 3.7 倍速度，混沌区细节会不同）。
 * 切换会重建内核，调用前必须确保没有计算任务在跑。返回 0 成功 */
int  gpu_set_fast(int on);
int  gpu_is_fast(void);

/* 计算一片参数区域的最大李雅普诺夫指数。
 *   rect = { th1_lo, th1_hi, th2_lo, th2_hi }，注意 th2_hi 对应图像顶部
 *   W,H  = 输出分辨率
 *   T    = 每个双摆的演化时长（秒）
 *   precision = 0 单精度（最快） / 1 float-float（约 48 位有效数字，比双精度快约 10 倍）
 *               / 2 双精度 fp64（最慢但最准）
 *   out  = 长度 W*H 的 float 数组，行优先，out[j*W+i]
 *   cancel = 指向取消标志，非 0 时尽快返回；可为 NULL
 *   progress = 每算完若干行回调一次（j0, j1, ud），表示行 [j0, j1) 已就绪；可为 NULL
 * 返回 0 成功，-1 被取消，-2 出错 */
int  gpu_compute(const double rect[4], int W, int H, double T, int precision,
                 int space, double f1, double f2,
                 float *out, volatile long *cancel,
                 void (*progress)(int, int, void *), void *ud);

/* 设置物理参数（质量、长度、重力）。改变后下次 gpu_compute 会自动重建内核。
 * space: 0 = 坐标是 (θ1,θ2) 初始角度，f1/f2 是初始角速度；
 *        1 = 坐标是 (ω1,ω2) 初始角速度，f1/f2 是固定初始角度。 */
void gpu_set_physics(double m1, double m2, double l1, double l2, double g);

#ifdef __cplusplus
}
#endif

#endif
