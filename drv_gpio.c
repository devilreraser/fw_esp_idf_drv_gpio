/* *****************************************************************************
 * File:   drv_gpio.c
 * Author: XX
 *
 * Created on YYYY MM DD
 * 
 * Description: ...
 * 
 **************************************************************************** */

/* *****************************************************************************
 * Header Includes
 **************************************************************************** */
#include "drv_gpio.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "soc/gpio_periph.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_hal.h"

/* *****************************************************************************
 * Configuration Definitions
 **************************************************************************** */
#define TAG "drv_gpio"

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
 * Function-Like Macros
 **************************************************************************** */

/* *****************************************************************************
 * Variables Definitions
 **************************************************************************** */

/* *****************************************************************************
 * Prototype of functions definitions
 **************************************************************************** */

/* *****************************************************************************
 * Functions
 **************************************************************************** */

/*
    #define PIN_SLP_INPUT_ENABLE(PIN_NAME)           SET_PERI_REG_MASK(PIN_NAME,SLP_IE)
    #define PIN_SLP_INPUT_DISABLE(PIN_NAME)          CLEAR_PERI_REG_MASK(PIN_NAME,SLP_IE)
    #define PIN_SLP_OUTPUT_ENABLE(PIN_NAME)          SET_PERI_REG_MASK(PIN_NAME,SLP_OE)
    #define PIN_SLP_OUTPUT_DISABLE(PIN_NAME)         CLEAR_PERI_REG_MASK(PIN_NAME,SLP_OE)
    #define PIN_SLP_PULLUP_ENABLE(PIN_NAME)           SET_PERI_REG_MASK(PIN_NAME,SLP_PU)
    #define PIN_SLP_PULLUP_DISABLE(PIN_NAME)          CLEAR_PERI_REG_MASK(PIN_NAME,SLP_PU)
    #define PIN_SLP_PULLDOWN_ENABLE(PIN_NAME)          SET_PERI_REG_MASK(PIN_NAME,SLP_PD)
    #define PIN_SLP_PULLDOWN_DISABLE(PIN_NAME)         CLEAR_PERI_REG_MASK(PIN_NAME,SLP_PD)
    #define PIN_SLP_SEL_ENABLE(PIN_NAME)          SET_PERI_REG_MASK(PIN_NAME,SLP_SEL)
    #define PIN_SLP_SEL_DISABLE(PIN_NAME)         CLEAR_PERI_REG_MASK(PIN_NAME,SLP_SEL)

    #define PIN_INPUT_ENABLE(PIN_NAME)               SET_PERI_REG_MASK(PIN_NAME,FUN_IE)
    #define PIN_INPUT_DISABLE(PIN_NAME)              CLEAR_PERI_REG_MASK(PIN_NAME,FUN_IE)
    #define PIN_SET_DRV(PIN_NAME, drv)            REG_SET_FIELD(PIN_NAME, FUN_DRV, (drv));

    #define PIN_FUNC_SELECT(PIN_NAME, FUNC)      REG_SET_FIELD(PIN_NAME, MCU_SEL, FUNC)
 */

esp_err_t drv_gpio_config_mux_in(gpio_num_t gpio_num, uint32_t signal_idx, bool inv)
{
    esp_rom_gpio_connect_in_signal(gpio_num, signal_idx, inv);
    return ESP_OK;
}

esp_err_t drv_gpio_config_mux_out(gpio_num_t gpio_num, uint32_t signal_idx, bool out_inv, bool oen_inv)
{
    esp_rom_gpio_connect_out_signal(gpio_num, signal_idx, out_inv, oen_inv);
    return ESP_OK;
}

esp_err_t drv_gpio_config_mux_func(gpio_num_t gpio_num, uint32_t func)
{
    uint32_t pin_name = GPIO_PIN_MUX_REG[gpio_num];
    gpio_hal_iomux_func_sel(pin_name, func);      /* func ex.: PIN_FUNC_GPIO */
    return ESP_OK;
}

esp_err_t drv_gpio_config_gpio_pin_standard(gpio_num_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull_mode, uint32_t level)
{
    esp_err_t error = ESP_OK;

    gpio_pullup_t pull_up_mode = GPIO_PULLUP_DISABLE;
    gpio_pulldown_t pull_down_mode = GPIO_PULLDOWN_DISABLE;

    if (pull_mode == GPIO_PULLUP_ONLY)
    {
        pull_up_mode = GPIO_PULLUP_ENABLE;
    }
    else if (pull_mode == GPIO_PULLDOWN_ONLY)
    {
        pull_down_mode = GPIO_PULLDOWN_ENABLE;
    }
    else if (pull_mode == GPIO_PULLUP_PULLDOWN)
    {
        pull_up_mode = GPIO_PULLUP_ENABLE;
        pull_down_mode = GPIO_PULLDOWN_ENABLE;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = mode,
        .pull_up_en = pull_up_mode,
        .pull_down_en = pull_down_mode,
        .intr_type = GPIO_INTR_DISABLE
    };
    error = gpio_config(&io_conf);

    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config() failed with error: %s", esp_err_to_name(error));
    }   
    return error;
}

esp_err_t drv_gpio_config_gpio_pin(gpio_num_t gpio_num, gpio_mode_t mode, gpio_pull_mode_t pull_mode, uint32_t level)
{
    esp_err_t error = ESP_OK;

    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio_num]);

    #if CONFIG_IDF_TARGET_ESP32
    #define GPIO_INPUT_ONLY_PINS  ((1ULL<<GPIO_NUM_34) | (1ULL<<GPIO_NUM_35) | (1ULL<<GPIO_NUM_36) | (1ULL<<GPIO_NUM_37) | (1ULL<<GPIO_NUM_38) | (1ULL<<GPIO_NUM_39))
    if ((pull_mode == GPIO_PULLUP_ONLY) ||
        (pull_mode == GPIO_PULLDOWN_ONLY) ||
        (pull_mode == GPIO_PULLUP_PULLDOWN))
    {
        if (((1ULL<<gpio_num) & GPIO_INPUT_ONLY_PINS) > 0)
        {
            ESP_LOGE(TAG, "GPIO%d can only be used as input and has no pull-up/down resistors", gpio_num);
            return ESP_ERR_NOT_SUPPORTED;
        }
    }
    #endif

    error = gpio_set_pull_mode(gpio_num, pull_mode);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_set_pull_mode() failed with error: %s", esp_err_to_name(error));
        return error;
    }

    if (mode == GPIO_MODE_OUTPUT)
    {
        error = gpio_set_level(gpio_num, level);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "gpio_set_level() failed with error: %s", esp_err_to_name(error));
            return error;
        }
    }

    error = gpio_set_direction(gpio_num, mode);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_set_direction() failed with error: %s", esp_err_to_name(error));
        return error;
    }

    drv_gpio_config_mux_func(gpio_num, PIN_FUNC_GPIO);

    return error;
}

#if DRV_GPIO_USE_ONLYMACRO
#elif DRV_GPIO_USE_MACROFUNC
esp_err_t drv_gpio_set_level(gpio_num_t gpio_num, uint32_t level)
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
}
#elif DRV_GPIO_USE_FUNCMACRO
#elif DRV_GPIO_USE_FUNCTIONS
esp_err_t drv_gpio_set_level(gpio_num_t gpio_num, uint32_t level)
{
    return gpio_set_level(gpio_num, level);
}
#endif