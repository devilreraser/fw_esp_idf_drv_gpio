/* *****************************************************************************
 * File:   drv_gpio.h
 * Author: XX
 *
 * Created on YYYY MM DD
 * 
 * Description: ...
 * 
 **************************************************************************** */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */


/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "esp_err.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "soc/gpio_periph.h"
    
/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
/* Priority Higher to Lower */
#define DRV_GPIO_USE_ONLYMACRO  1   /* use inlined code only (around 50 ns) */
#define DRV_GPIO_USE_MACROFUNC  1   /* use inlined code through function call (around 200 ns) */
#define DRV_GPIO_USE_FUNCMACRO  1   /* use inlined esp library function (around 300 ns) */
#define DRV_GPIO_USE_FUNCTIONS  1   /* use function calls (around 400 ns) */

/* *****************************************************************************
 * Constants and Macros Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Enumeration Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Type Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Function-Like Macro
 **************************************************************************** */
#define REG_WRITE_local(_r, _v)  do { \
            (*(volatile uint32_t *)(_r)) = (_v);   \
        } while(0)
#define REG_READ_local(_r) ({                                                                                                \
            (*(volatile uint32_t *)(_r));                                                                              \
        })

/* *****************************************************************************
 * Variables External Usage
 **************************************************************************** */ 

/* *****************************************************************************
 * Function Prototypes
 **************************************************************************** */

esp_err_t drv_gpio_config_gpio_pin_standard(gpio_num_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull_mode, uint32_t level);
esp_err_t drv_gpio_config_gpio_pin(gpio_num_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull_mode, uint32_t level);

#if DRV_GPIO_USE_ONLYMACRO
static inline esp_err_t drv_gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{  
    uint32_t is_pin_high_port = gpio_num >> 5;  /* gpio_num >> 5 gets if the  pin is  >= 32 */
    uint32_t pin_port_index = gpio_num & ((1 << 5) - 1);  /* gpio_num >> 5 gets if the  pin is  >= 32 */
    uint32_t pin_in_port_mask = 1UL << pin_port_index;  /* gpio_num >> 5 gets if the  pin is  >= 32 */
    uint32_t pin_level = level & 1;
    uint32_t reg_offset_high_port = is_pin_high_port * (GPIO_OUT1_W1TC_REG - GPIO_OUT_W1TC_REG);
    uint32_t reg_offset_set_high_level = pin_level * (GPIO_OUT_W1TS_REG - GPIO_OUT_W1TC_REG);
    uint32_t register_address = GPIO_OUT_W1TC_REG + reg_offset_high_port + reg_offset_set_high_level; 
    //(*(volatile uint32_t *)(register_address)) = pin_in_port_mask;
    REG_WRITE(register_address, pin_in_port_mask);
    return ESP_OK;
}
//#define drv_gpio_set_level(gpio_num, level) (gpio_num >= 32) ? (level > 0) ? REG_WRITE_local(GPIO_OUT1_W1TC_REG, (1ULL << (bus_pin - 32)));//GPIO LOW (clear) // Drives pin low
#elif DRV_GPIO_USE_MACROFUNC
esp_err_t drv_gpio_set_level(gpio_num_t gpio_num, uint32_t level);
#elif DRV_GPIO_USE_FUNCMACRO
#define  drv_gpio_set_level(gpio_num, level) gpio_set_level(gpio_num, level)
#elif DRV_GPIO_USE_FUNCTIONS
esp_err_t drv_gpio_set_level(gpio_num_t gpio_num, uint32_t level);
#endif




#ifdef __cplusplus
}
#endif /* __cplusplus */


