#pragma once
/* Board and MCU names from the build configuration's defines. */
#if defined(Stainer_Master_PCB)
#define BOARD_NAME "Master"
#elif defined(Nozzle_Mount_PCB)
#define BOARD_NAME "Nozzle"
#elif defined(Stainer_Gantry_PCB_UART)
#define BOARD_NAME "Gantry"
#elif defined(Magazine1_IR_PCB_UART)
#define BOARD_NAME "Magazine1"
#elif defined(Magazine2_IR_PCB_UART)
#define BOARD_NAME "Magazine2"
#elif defined(Gantry_X_Hall_PCB)
#define BOARD_NAME "GantryXHall"
#elif defined(Gantry_Z_Hall_PCB)
#define BOARD_NAME "GantryZHall"
#else
#error "No board selected: build with one of the project's board configurations."
#endif

#if defined(STM32F446xx)
#define MCU_NAME "STM32F446ZE"
#elif defined(STM32F407xx)
#define MCU_NAME "STM32F407VE"
#elif defined(STM32F401xC)
#define MCU_NAME "STM32F401RC"
#endif
