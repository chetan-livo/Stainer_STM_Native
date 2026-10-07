#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Unrecoverable platform failures. The code is kept in platform_last_error
 * for the debugger; Debug builds stop at a breakpoint, Release builds reset. */
typedef enum {
    PLATFORM_ERROR_NONE = 0,
    PLATFORM_ERROR_CLOCK,
    PLATFORM_ERROR_HAL,
    PLATFORM_ERROR_HARDFAULT,
    PLATFORM_ERROR_MEMMANAGE,
    PLATFORM_ERROR_BUSFAULT,
    PLATFORM_ERROR_USAGEFAULT,
    PLATFORM_ERROR_CONFIG,
} platform_error_t;

extern volatile platform_error_t platform_last_error;

void platform_error(platform_error_t code) __attribute__((noreturn));
void Error_Handler(void) __attribute__((noreturn)); /* HAL/ST convention */

#ifdef __cplusplus
}
#endif
