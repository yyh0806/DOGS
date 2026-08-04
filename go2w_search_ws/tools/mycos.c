/* mycos.c: cos/sin ifunc 泰勒实现, LD_PRELOAD 避开 glibc do_cos(0) SIGFPE.
 *
 * 根因 (gdb + objdump): WSL2 glibc 2.35 libm cos 是 ifunc (STT_GNU_IFUNC, objdump iD),
 * resolver 选 __cos_sse2/__cos_avx/__sin_fma → do_cos(dx=0) 整数除零 SIGFPE (所有 SIMD 崩).
 * 普通符号 LD_PRELOAD (DF) 不覆盖 ifunc. 需 mycos cos 也 ifunc (resolver 返回泰勒).
 *
 * 编译: gcc -shared -fPIC -O2 -Wl,--version-script=mycos.map -o mycos.so mycos.c -lm
 * mycos.map: GLIBC_2.2.5 { global: cos; sin; sincos; cosf; sinf; local: *; };
 * 用法: LD_PRELOAD=/path/mycos.so gzserver ...
 *
 * importer/caller: sim_full_bringup SetEnvironmentVariable LD_PRELOAD.
 * 用户原话: "我要的不是mock数据，而是从仿真环境中真实读取的数据".
 */
#define _GNU_SOURCE
#include <math.h>

static double norm_angle(double x) {
    static const double TWO_PI = 2.0 * M_PI;
    double y = fmod(x, TWO_PI);
    if (y > M_PI) y -= TWO_PI;
    if (y < -M_PI) y += TWO_PI;
    return y;
}

static double my_cos(double x) {
    double y = norm_angle(x);
    double y2 = y * y;
    return 1.0 - y2 / 2.0 + y2 * y2 / 24.0 - y2 * y2 * y2 / 720.0
           + y2 * y2 * y2 * y2 / 40320.0;
}

static double my_sin(double x) {
    double y = norm_angle(x);
    double y2 = y * y;
    return y * (1.0 - y2 / 6.0 + y2 * y2 / 120.0 - y2 * y2 * y2 / 5040.0
                + y2 * y2 * y2 * y2 / 362880.0);
}

static void my_sincos(double x, double *s, double *c) {
    *s = my_sin(x);
    *c = my_cos(x);
}

static float my_cosf(float x) { return (float)my_cos((double)x); }
static float my_sinf(float x) { return (float)my_sin((double)x); }

typedef double (*dbl_fn)(double);
typedef void (*sincos_fn)(double, double *, double *);
typedef float (*flt_fn)(float);

static dbl_fn resolve_cos(void) { return my_cos; }
static dbl_fn resolve_sin(void) { return my_sin; }
static sincos_fn resolve_sincos(void) { return my_sincos; }
static flt_fn resolve_cosf(void) { return my_cosf; }
static flt_fn resolve_sinf(void) { return my_sinf; }

double cos(double) __attribute__((ifunc("resolve_cos")));
double sin(double) __attribute__((ifunc("resolve_sin")));
void sincos(double, double *, double *) __attribute__((ifunc("resolve_sincos")));
float cosf(float) __attribute__((ifunc("resolve_cosf")));
float sinf(float) __attribute__((ifunc("resolve_sinf")));
