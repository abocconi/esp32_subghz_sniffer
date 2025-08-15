/**
 * @file button_driver.h
 * @brief ESP32-S3 Button Driver with debouncing, long press, and auto-repeat
 * 
 * Features:
 * - Configurable active level (HIGH/LOW) per button
 * - Hardware debouncing with configurable timing
 * - Long press detection 
 * - Auto-repeat for navigation buttons
 * - Thread-safe event queue
 * - Low power GPIO interrupt handling
 */

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Button IDs matching hardware layout
 */
typedef enum {
    BUTTON_UP = 0,
    BUTTON_DOWN,
    BUTTON_ENTER, 
    BUTTON_ESC,
    BUTTON_MENU,
    BUTTON_MAX
} button_id_t;

/**
 * @brief Button event types
 */
typedef enum {
    BUTTON_EVENT_PRESSED,       /**< Button just pressed (after debounce) */
    BUTTON_EVENT_RELEASED,      /**< Button just released (after debounce) */
    BUTTON_EVENT_LONG_PRESS,    /**< Button held for long press duration */
    BUTTON_EVENT_AUTO_REPEAT    /**< Auto-repeat event (for UP/DOWN navigation) */
} button_event_type_t;

/**
 * @brief Button active level configuration
 */
typedef enum {
    BUTTON_ACTIVE_LOW = 0,      /**< Button active when GPIO reads 0 */
    BUTTON_ACTIVE_HIGH = 1      /**< Button active when GPIO reads 1 */
} button_active_level_t;

/**
 * @brief Button event structure
 */
typedef struct {
    button_id_t button;                 /**< Which button generated the event */
    button_event_type_t event;          /**< Type of event */
    uint32_t timestamp_ms;              /**< Event timestamp in milliseconds */
} button_event_t;

/**
 * @brief Single button configuration
 */
typedef struct {
    gpio_num_t gpio_num;                /**< GPIO pin number */
    button_active_level_t active_level; /**< Active level (HIGH or LOW) */
    bool auto_repeat_enabled;           /**< Enable auto-repeat for this button */
} button_config_t;

/**
 * @brief Button driver timing configuration
 */
typedef struct {
    uint32_t debounce_time_ms;          /**< Debounce delay (default: 50ms) */
    uint32_t long_press_time_ms;        /**< Long press threshold (default: 1000ms) */
    uint32_t auto_repeat_delay_ms;      /**< Initial auto-repeat delay (default: 500ms) */
    uint32_t auto_repeat_period_ms;     /**< Auto-repeat period (default: 200ms) */
} button_timing_config_t;

/**
 * @brief Button driver configuration structure
 */
typedef struct {
    button_config_t buttons[BUTTON_MAX]; /**< Per-button configuration */
    button_timing_config_t timing;       /**< Timing parameters */
    size_t event_queue_size;             /**< Event queue depth (default: 16) */
} button_driver_config_t;

/**
 * @brief Get default button driver configuration
 * 
 * @return Default configuration with ESP32-S3 pin mapping
 */
button_driver_config_t button_driver_get_default_config(void);

/**
 * @brief Initialize button driver
 * 
 * @param config Button driver configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t button_driver_init(const button_driver_config_t *config);

/**
 * @brief Deinitialize button driver and cleanup resources
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t button_driver_deinit(void);

/**
 * @brief Get button event queue handle
 * 
 * Use this queue to receive button events in your application task.
 * Events are of type button_event_t.
 * 
 * @return Queue handle, or NULL if driver not initialized
 */
QueueHandle_t button_driver_get_event_queue(void);

/**
 * @brief Check if a button is currently pressed (after debouncing)
 * 
 * @param button Button ID to check
 * @return true if button is pressed, false otherwise
 */
bool button_driver_is_pressed(button_id_t button);

/**
 * @brief Get current state of all buttons as bitmask
 * 
 * @return Bitmask where bit N represents button N state (1 = pressed)
 */
uint8_t button_driver_get_state_mask(void);

/**
 * @brief Enable/disable auto-repeat for a specific button
 * 
 * @param button Button ID
 * @param enable true to enable auto-repeat, false to disable
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if invalid button
 */
esp_err_t button_driver_set_auto_repeat(button_id_t button, bool enable);

/**
 * @brief Update timing configuration at runtime
 * 
 * @param timing New timing configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t button_driver_update_timing(const button_timing_config_t *timing);

/**
 * @brief Get button statistics (for debugging)
 * 
 * @param button Button ID
 * @param press_count Total press count since init
 * @param last_event_time_ms Timestamp of last event
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if invalid button
 */
esp_err_t button_driver_get_stats(button_id_t button, uint32_t *press_count, uint32_t *last_event_time_ms);

#ifdef __cplusplus
}
#endif