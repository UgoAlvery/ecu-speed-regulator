#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

static int _tests_run    = 0;
static int _tests_failed = 0;

static inline float _tr_fabsf(float x) { return x < 0.0f ? -x : x; }

#define _ASSERT_IMPL(cond, fmt, ...) do {                                   \
    _tests_run++;                                                            \
    if (!(cond)) {                                                           \
        fprintf(stderr, "    ASSERT FAIL %s:%d — " fmt "\n",               \
                __FILE__, __LINE__, ##__VA_ARGS__);                         \
        _tests_failed++;                                                     \
    }                                                                        \
} while (0)

#define TEST_ASSERT(cond) \
    _ASSERT_IMPL((cond), "%s", #cond)

#define TEST_ASSERT_TRUE(cond) \
    _ASSERT_IMPL((cond) != 0, "expected true: %s", #cond)

#define TEST_ASSERT_FALSE(cond) \
    _ASSERT_IMPL((cond) == 0, "expected false: %s", #cond)

#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    _ASSERT_IMPL((int)(expected) == (int)(actual), \
                 "expected %d, got %d", (int)(expected), (int)(actual))

#define TEST_ASSERT_EQUAL_UINT8(expected, actual) \
    _ASSERT_IMPL((uint8_t)(expected) == (uint8_t)(actual), \
                 "expected 0x%02X, got 0x%02X", (uint8_t)(expected), (uint8_t)(actual))

#define TEST_ASSERT_EQUAL_UINT16(expected, actual) \
    _ASSERT_IMPL((uint16_t)(expected) == (uint16_t)(actual), \
                 "expected 0x%04X, got 0x%04X", (uint16_t)(expected), (uint16_t)(actual))

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
    _ASSERT_IMPL(_tr_fabsf((float)(actual) - (float)(expected)) <= (float)(delta), \
                 "expected %.6f +-%.6f, got %.6f", \
                 (float)(expected), (float)(delta), (float)(actual))

#define TEST_ASSERT_MEM_EQUAL(expected, actual, len) \
    _ASSERT_IMPL(memcmp((expected), (actual), (size_t)(len)) == 0, \
                 "memory blocks differ (len=%zu)", (size_t)(len))

#define RUN_TEST(fn) do {                              \
    int _before = _tests_failed;                       \
    fn();                                              \
    if (_tests_failed == _before)                      \
        printf("  PASS  " #fn "\n");                  \
    else                                               \
        printf("  FAIL  " #fn "\n");                  \
} while (0)

#define TEST_SUITE_BEGIN(name) \
    printf("\n=== " name " ===\n")

#define TEST_SUITE_SUMMARY() do {                      \
    printf("\n%d tests, %d failures\n",                \
           _tests_run, _tests_failed);                 \
    return _tests_failed ? 1 : 0;                      \
} while (0)

#endif /* TEST_RUNNER_H */