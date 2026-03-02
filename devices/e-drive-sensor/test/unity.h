/**
 * Minimal Unity-compatible test framework header.
 * Implements the subset of macros used by test_main.cpp.
 * API-compatible with the real Unity (https://github.com/ThrowTheSwitch/Unity)
 * so that `pio test -e native` works once platform toolchain is available.
 */
#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ── Counters ─────────────────────────────────────────────────────────────────
static int _unity_tests_run    = 0;
static int _unity_tests_failed = 0;
static const char* _unity_current_test = "";

// ── Internal helpers ──────────────────────────────────────────────────────────
#define _UNITY_FAIL(msg) do { \
    printf("FAIL: %s\n  [%s:%d] %s\n", _unity_current_test, __FILE__, __LINE__, msg); \
    _unity_tests_failed++; \
    return; \
} while(0)

// ── Lifecycle ─────────────────────────────────────────────────────────────────
static inline int UNITY_BEGIN(void) {
    _unity_tests_run    = 0;
    _unity_tests_failed = 0;
    printf("\n");
    return 0;
}

static inline int UNITY_END(void) {
    printf("\n-----------------------\n");
    printf("%d Tests  %d Failures\n", _unity_tests_run, _unity_tests_failed);
    if (_unity_tests_failed == 0) printf("OK\n");
    else                          printf("FAIL\n");
    return _unity_tests_failed ? 1 : 0;
}

#define RUN_TEST(f) do { \
    _unity_current_test = #f; \
    _unity_tests_run++; \
    printf("  %-60s ", #f); \
    f(); \
    if (_unity_tests_failed == 0 || 1) printf("[PASS]\n"); \
} while(0)

// Make RUN_TEST actually report failures correctly (patch the pass print):
#undef  RUN_TEST
#define RUN_TEST(f) do { \
    _unity_current_test = #f; \
    int _before = _unity_tests_failed; \
    _unity_tests_run++; \
    f(); \
    if (_unity_tests_failed == _before) printf("PASS: %s\n", #f); \
} while(0)

// ── Assertions ────────────────────────────────────────────────────────────────

#define TEST_ASSERT_EQUAL(expected, actual) do { \
    long long _e = (long long)(expected); \
    long long _a = (long long)(actual); \
    if (_e != _a) { \
        char _buf[128]; \
        snprintf(_buf, sizeof(_buf), "Expected %lld but got %lld", _e, _a); \
        _UNITY_FAIL(_buf); \
    } \
} while(0)

#define TEST_ASSERT_NOT_EQUAL(expected, actual) do { \
    long long _e = (long long)(expected); \
    long long _a = (long long)(actual); \
    if (_e == _a) { \
        char _buf[128]; \
        snprintf(_buf, sizeof(_buf), "Expected NOT %lld but got %lld", _e, _a); \
        _UNITY_FAIL(_buf); \
    } \
} while(0)

#define TEST_ASSERT_EQUAL_STRING(expected, actual) do { \
    const char* _e = (expected); \
    const char* _a = (actual); \
    if (strcmp(_e, _a) != 0) { \
        char _buf[256]; \
        snprintf(_buf, sizeof(_buf), "Expected \"%s\" but got \"%s\"", _e, _a); \
        _UNITY_FAIL(_buf); \
    } \
} while(0)

#define TEST_ASSERT_LESS_OR_EQUAL(threshold, actual) do { \
    long long _t = (long long)(threshold); \
    long long _a = (long long)(actual); \
    if (_a > _t) { \
        char _buf[128]; \
        snprintf(_buf, sizeof(_buf), "Expected %lld <= %lld", _a, _t); \
        _UNITY_FAIL(_buf); \
    } \
} while(0)

#define TEST_ASSERT_TRUE(cond) do { \
    if (!(cond)) { _UNITY_FAIL("Expected TRUE"); } \
} while(0)

#define TEST_ASSERT_FALSE(cond) do { \
    if ((cond)) { _UNITY_FAIL("Expected FALSE"); } \
} while(0)
