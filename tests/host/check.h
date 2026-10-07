#pragma once
// Minimal assertion helpers for host tests: count failures, print each one.
#include <stdio.h>
#include <string.h>

static int checks_run = 0, checks_failed = 0;

#define CHECK(cond) do { ++checks_run; if (!(cond)) { ++checks_failed; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_STR(actual, expected) do { ++checks_run; const char* a_ = (actual); \
    if (strcmp(a_, (expected)) != 0) { ++checks_failed; \
    printf("FAIL %s:%d: \"%s\" != \"%s\"\n", __FILE__, __LINE__, a_, (expected)); } } while (0)

static inline int report(const char* name)
{
    printf("%s: %d checks, %d failed\n", name, checks_run, checks_failed);
    return checks_failed ? 1 : 0;
}
