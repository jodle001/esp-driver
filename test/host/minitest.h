// Minimal test macros for the host builds (no dependency on Unity or gtest).
#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>

static int mt_failures = 0;
static int mt_checks = 0;
static const char *mt_current = "";

#define MT_CHECK(cond, ...)                                                                          \
    do {                                                                                             \
        ++mt_checks;                                                                                 \
        if (!(cond)) {                                                                               \
            ++mt_failures;                                                                           \
            printf("  FAIL %s:%d [%s] %s: ", __FILE__, __LINE__, mt_current, #cond);                 \
            printf(__VA_ARGS__);                                                                     \
            printf("\n");                                                                            \
        }                                                                                            \
    } while (0)

#define MT_TRUE(c) MT_CHECK((c), "expected true")
#define MT_FALSE(c) MT_CHECK(!(c), "expected false")
#define MT_EQ(a, b) MT_CHECK((a) == (b), "%lld != %lld", (long long)(a), (long long)(b))
#define MT_NEAR(a, b, tol) MT_CHECK(fabs((double)(a) - (double)(b)) <= (tol), "%g vs %g (tol %g)", (double)(a), (double)(b), (double)(tol))
#define MT_GT(a, b) MT_CHECK((a) > (b), "%g <= %g", (double)(a), (double)(b))
#define MT_LT(a, b) MT_CHECK((a) < (b), "%g >= %g", (double)(a), (double)(b))
#define MT_STREQ(a, b) MT_CHECK(strcmp((a), (b)) == 0, "'%s' != '%s'", (a), (b))
#define MT_MEMEQ(a, b, n) MT_CHECK(memcmp((a), (b), (n)) == 0, "memory differs over %zu bytes", (size_t)(n))

#define MT_RUN(fn)                                                                                   \
    do {                                                                                             \
        mt_current = #fn;                                                                            \
        const int before = mt_failures;                                                              \
        fn();                                                                                        \
        printf("%s %s\n", mt_failures == before ? "  ok  " : "  FAIL", #fn);                         \
    } while (0)

#define MT_SUMMARY()                                                                                 \
    (printf("%d checks, %d failures\n", mt_checks, mt_failures), mt_failures ? 1 : 0)
