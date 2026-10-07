/*
 * System clock trees. Each matches the STM32duino 3.0.0 generic variant the
 * Arduino firmware used, so UART, timer and USB timing are unchanged: HSI
 * only, no crystal assumed.
 *
 *   STM32F446xx  HSI 16 MHz -> PLL 180 MHz (PLLR), APB1 45 / APB2 90 MHz,
 *                48 MHz USB clock from PLLSAI-P.
 *   STM32F407xx  HSI -> PLL 168 MHz, APB1 42 / APB2 84 MHz, USB 48 MHz (PLLQ).
 *   STM32F401xC  HSI -> PLL 84 MHz, APB1 42 / APB2 84 MHz, USB 48 MHz (PLLQ).
 */
#include "stm32f4xx_hal.h"
#include "platform_error.h"

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
#if defined(STM32F401xC)
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
#else
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
#endif

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 8;
#if defined(STM32F446xx)
    osc.PLL.PLLN = 180;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 8;
    osc.PLL.PLLR = 2;
#elif defined(STM32F407xx)
    osc.PLL.PLLN = 168;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
#elif defined(STM32F401xC)
    osc.PLL.PLLN = 168;
    osc.PLL.PLLP = RCC_PLLP_DIV4;
    osc.PLL.PLLQ = 7;
#else
#error "Unsupported MCU"
#endif
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) platform_error(PLATFORM_ERROR_CLOCK);

#if defined(STM32F446xx)
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) platform_error(PLATFORM_ERROR_CLOCK);
#endif

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
#if defined(STM32F446xx)
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) platform_error(PLATFORM_ERROR_CLOCK);
#elif defined(STM32F407xx)
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) platform_error(PLATFORM_ERROR_CLOCK);
#else
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) platform_error(PLATFORM_ERROR_CLOCK);
#endif

#if defined(STM32F446xx)
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    periph.PLLSAI.PLLSAIM = 16;
    periph.PLLSAI.PLLSAIN = 192;
    periph.PLLSAI.PLLSAIQ = 2;
    periph.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
    periph.PLLSAIDivQ = 1;
    periph.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
    if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) platform_error(PLATFORM_ERROR_CLOCK);
#endif
}
