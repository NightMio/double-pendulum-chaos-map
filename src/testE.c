/* testE.c —— 用与程序完全相同的 deriv/rk4（来自 app.h）验证能量守恒 */
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include "app.h"

Phys g_phys = { 1.0, 1.0, 1.0, 1.0, 9.81, 0.0, 0.0, 0.0, 0.0, 0 };

int main(void)
{
    State s = { -2.5882439437, 0.0, 0.9505102772, 0.0 };
    const double h = 0.001;
    const double T = 120.0;
    long n = (long)(T / h);
    double e0 = energy(&s);

    for (long i = 0; i < n; ++i) s = rk4(s, h);

    double e1 = energy(&s);
    printf("t=%.1f  E0=%.10f  E1=%.10f  dE=%.3e  rel=%.3e\n",
           T, e0, e1, e1 - e0, fabs((e1 - e0) / e0));
    return 0;
}
