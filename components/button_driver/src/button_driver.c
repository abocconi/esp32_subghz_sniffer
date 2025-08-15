/**
 * @file button_driver.c
 * @brief Button driver implementation with state machine and interrupt handling
 */

#include "button_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "button_driver";

/**
 * @brief Button internal states for state machine
 */
typedef enum {
    BUTTON_STATE_IDLE,           /**< Button not pressed */
    BUTTON_STATE_DEBOUNCE_PRESS, /**< Debouncing press transition */
    BUTTON_STATE_PRESSED,        /**< Button confirmed pressed */
    BUTTON_STATE_LONG_PRESS,     /**< Button in long press state */
    BUTTON_STATE_AUTO_REPEAT,    /**< Button in auto-repeat state */
    BUTTON_STATE_DEBOUNCE_RELEASE /**< Debouncing release transition */
} button_state_t;

/**
 * @brief Internal button state structure
 */
typedef struct {
    button_config_t config;          /**< Button configuration */
    button_state_t state;            /**< Current state */
    uint32_t last_change_time;       /**< Last state change timestamp */
    uint32_t press_count;            /**< Total press count for statistics */
    bool raw_level;                  /**< Current raw GPIO level */
    bool debounced_level;            /**< Debounced level */
    esp_timer_handle_t timer;        /**< Per-button timer for timing events */
} button_state_info_t;

/**
 * @brief Driver instance data
 */
typedef struct {
    button_state_info_t buttons[BUTTON_MAX]; /**< Per-button state */
    button_timing_config_t timing;           /**< Timing configuration */
    QueueHandle_t event_queue;               /**< Event queue handle */
    TaskHandle_t task_handle;                /**< Button task handle */
    bool initialized;                        /**< Initialization flag */
} button_driver_instance_t;

static button_driver_instance_t s_button_driver = {0};

/**
 * @brief Read raw button level accounting for active level configuration
 */
static inline bool button_read_raw(button_id_t button_id)
{
    button_state_info_t *btn = &s_button_driver.buttons[button_id];
    int raw_level = gpio_get_level(btn->config.gpio_num);
    
    // Apply active level logic
    if (btn->config.active_level == BUTTON_ACTIVE_LOW) {
        return (raw_level == 0);  // Active low: 0 = pressed
    } else {
        return (raw_level == 1);  // Active high: 1 = pressed  
    }
}

/**
 * @brief Send button event to queue
 */
static void button_send_event(button_id_t button_id, button_event_type_t event_type)
{
    button_event_t event = {
        .button = button_id,
        .event = event_type,
        .timestamp_ms = esp_timer_get_time() / 1000
    };
    
    BaseType_t result = xQueueSend(s_button_driver.event_queue, &event, 0);
    if (result != pdTRUE) {
        ESP_LOGW(TAG, "Event queue full, dropping event for button %d", button_id);
    } else {
        ESP_LOGD(TAG, "Button %d event %d at %lu ms", button_id, event_type, event.timestamp_ms);
    }
}

/**
 * @brief Timer callback for button timing events
 */
static void button_timer_callback(void *arg)
{
    button_id_t button_id = (button_id_t)(uintptr_t)arg;
    button_state_info_t *btn = &s_button_driver.buttons[button_id];
    uint32_t now = esp_timer_get_time() / 1000;
    
    switch (btn->state) {
        case BUTTON_STATE_DEBOUNCE_PRESS:
            // Check if button is still pressed after debounce
            if (button_read_raw(button_id)) {
                btn->state = BUTTON_STATE_PRESSED;
                btn->debounced_level = true;
                button_send_event(button_id, BUTTON_EVENT_PRESSED);
                btn->press_count++;
                
                // Start long press timer
                esp_timer_start_once(btn->timer, s_button_driver.timing.long_press_time_ms * 1000);
            } else {
                // False press, return to idle
                btn->state = BUTTON_STATE_IDLE;
                btn->debounced_level = false;
            }
            break;
            
        case BUTTON_STATE_PRESSED:
            // Long press detected
            btn->state = BUTTON_STATE_LONG_PRESS;
            button_send_event(button_id, BUTTON_EVENT_LONG_PRESS);
            
            // Start auto-repeat if enabled
            if (btn->config.auto_repeat_enabled) {
                esp_timer_start_once(btn->timer, s_button_driver.timing.auto_repeat_delay_ms * 1000);
            }
            break;
            
        case BUTTON_STATE_LONG_PRESS:
            // Start auto-repeat if enabled
            if (btn->config.auto_repeat_enabled) {
                btn->state = BUTTON_STATE_AUTO_REPEAT;
                button_send_event(button_id, BUTTON_EVENT_AUTO_REPEAT);
                esp_timer_start_once(btn->timer, s_button_driver.timing.auto_repeat_period_ms * 1000);
            }
            break;
            
        case BUTTON_STATE_AUTO_REPEAT:
            // Continue auto-repeat
            button_send_event(button_id, BUTTON_EVENT_AUTO_REPEAT);
            esp_timer_start_once(btn->timer, s_button_driver.timing.auto_repeat_period_ms * 1000);
            break;
            
        case BUTTON_STATE_DEBOUNCE_RELEASE:
            // Check if button is still released after debounce
            if (!button_read_raw(button_id)) {
                btn->state = BUTTON_STATE_IDLE;
                btn->debounced_level = false;
                button_send_event(button_id, BUTTON_EVENT_RELEASED);
            } else {
                // False release, return to pressed state
                btn->state = BUTTON_STATE_PRESSED;
                btn->debounced_level = true;
                esp_timer_start_once(btn->timer, s_button_driver.timing.long_press_time_ms * 1000);
            }
            break;
            
        default:
            ESP_LOGW(TAG, "Unexpected timer callback in state %d for button %d", btn->state, button_id);
            break;
    }
}

/**
 * @brief GPIO interrupt handler
 */
static void IRAM_ATTR button_gpio_isr_handler(void *arg)
{
    button_id_t button_id = (button_id_t)(uintptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Just wake up the button task, actual processing happens there
    xTaskNotifyFromISR(s_button_driver.task_handle, (1 << button_id), eSetBits, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Process button state change
 */
static void button_process_change(button_id_t button_id)
{
    button_state_info_t *btn = &s_button_driver.buttons[button_id];
    bool current_level = button_read_raw(button_id);
    
    // Check if level actually changed
    if (current_level == btn->raw_level) {
        return; // No change, possible noise
    }
    
    btn->raw_level = current_level;
    uint32_t now = esp_timer_get_time() / 1000;
    
    if (current_level && btn->state == BUTTON_STATE_IDLE) {
        // Button press detected, start debouncing
        btn->state = BUTTON_STATE_DEBOUNCE_PRESS;
        btn->last_change_time = now;
        esp_timer_start_once(btn->timer, s_button_driver.timing.debounce_time_ms * 1000);
        
    } else if (!current_level && (btn->state >= BUTTON_STATE_PRESSED)) {
        // Button release detected, stop any running timers and start release debounce
        esp_timer_stop(btn->timer);
        btn->state = BUTTON_STATE_DEBOUNCE_RELEASE;
        btn->last_change_time = now;
        esp_timer_start_once(btn->timer, s_button_driver.timing.debounce_time_ms * 1000);
    }
}

/**
 * @brief Button processing task
 */
static void button_task(void *param)
{
    ESP_LOGI(TAG, "Button task started");
    
    while (1) {
        uint32_t notification_value;
        
        // Wait for GPIO interrupt notification
        if (xTaskNotifyWait(0, 0xFFFFFFFF, &notification_value, portMAX_DELAY) == pdTRUE) {
            // Process each button that triggered an interrupt
            for (int i = 0; i < BUTTON_MAX; i++) {
                if (notification_value & (1 << i)) {
                    button_process_change((button_id_t)i);
                }
            }
        }
    }
}

button_driver_config_t button_driver_get_default_config(void)
{
    button_driver_config_t config = {0};
    
    // Default ESP32-S3 pin mapping from specs
    config.buttons[BUTTON_UP]    = (button_config_t){4,  BUTTON_ACTIVE_LOW, true};
    config.buttons[BUTTON_DOWN]  = (button_config_t){5,  BUTTON_ACTIVE_LOW, true};
    config.buttons[BUTTON_ENTER] = (button_config_t){6,  BUTTON_ACTIVE_LOW, false};
    config.buttons[BUTTON_ESC]   = (button_config_t){7,  BUTTON_ACTIVE_LOW, false};
    config.buttons[BUTTON_MENU]  = (button_config_t){15, BUTTON_ACTIVE_HIGH, false};
    
    // Default timing values
    config.timing.debounce_time_ms = 50;
    config.timing.long_press_time_ms = 1000;
    config.timing.auto_repeat_delay_ms = 500;
    config.timing.auto_repeat_period_ms = 200;
    
    config.event_queue_size = 16;
    
    return config;
}

esp_err_t button_driver_init(const button_driver_config_t *config)
{
    esp_err_t ret = ESP_OK;  // Initialize ret to prevent warning
    
    if (s_button_driver.initialized) {
        ESP_LOGW(TAG, "Button driver already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing button driver");
    
    // Copy configuration
    s_button_driver.timing = config->timing;
    
    // Create event queue
    s_button_driver.event_queue = xQueueCreate(config->event_queue_size, sizeof(button_event_t));
    if (!s_button_driver.event_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return ESP_ERR_NO_MEM;
    }
    
    // Install global GPIO ISR service BEFORE configuring individual buttons
    esp_err_t isr_service_ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    if (isr_service_ret != ESP_OK && isr_service_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(isr_service_ret));
        ret = isr_service_ret;
        goto cleanup;
    }
    
    // Initialize GPIO configuration
    gpio_config_t gpio_conf = {0};
    
    for (int i = 0; i < BUTTON_MAX; i++) {
        button_state_info_t *btn = &s_button_driver.buttons[i];
        btn->config = config->buttons[i];
        btn->state = BUTTON_STATE_IDLE;
        btn->raw_level = false;
        btn->debounced_level = false;
        btn->press_count = 0;
        btn->last_change_time = 0;
        
        // Configure GPIO
        gpio_conf.pin_bit_mask = (1ULL << btn->config.gpio_num);
        gpio_conf.mode = GPIO_MODE_INPUT;
        gpio_conf.pull_up_en = (btn->config.active_level == BUTTON_ACTIVE_LOW) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        gpio_conf.pull_down_en = (btn->config.active_level == BUTTON_ACTIVE_HIGH) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
        gpio_conf.intr_type = GPIO_INTR_ANYEDGE;
        
        esp_err_t gpio_ret = gpio_config(&gpio_conf);
        if (gpio_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", btn->config.gpio_num, esp_err_to_name(gpio_ret));
            ret = gpio_ret;
            goto cleanup;
        }
        
        // Create timer for this button
        esp_timer_create_args_t timer_args = {
            .callback = button_timer_callback,
            .arg = (void *)(uintptr_t)i,
            .name = "btn_timer"
        };
        
        esp_err_t timer_ret = esp_timer_create(&timer_args, &btn->timer);
        if (timer_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create timer for button %d: %s", i, esp_err_to_name(timer_ret));
            ret = timer_ret;
            goto cleanup;
        }
        
        // Install interrupt handler
        esp_err_t isr_ret = gpio_isr_handler_add(btn->config.gpio_num, button_gpio_isr_handler, (void *)(uintptr_t)i);
        if (isr_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install ISR for button %d: %s", i, esp_err_to_name(isr_ret));
            ret = isr_ret;
            goto cleanup;
        }
        
        // Read initial state
        btn->raw_level = button_read_raw((button_id_t)i);
        
        ESP_LOGI(TAG, "Button %d: GPIO %d, active %s, auto-repeat %s, initial level %d",
                 i, btn->config.gpio_num,
                 btn->config.active_level ? "HIGH" : "LOW",
                 btn->config.auto_repeat_enabled ? "ON" : "OFF",
                 btn->raw_level);
    }
    
    // Create button processing task on Core 1 (Application Core)
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        button_task,
        "button_task",
        4096,                    // Stack size
        NULL,                    // Parameters
        configMAX_PRIORITIES - 2,// High priority for input responsiveness
        &s_button_driver.task_handle,
        1                        // Core 1 (Application)
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        ret = ESP_ERR_NO_MEM;
        goto cleanup;
    }
    
    s_button_driver.initialized = true;
    ESP_LOGI(TAG, "Button driver initialized successfully");
    return ESP_OK;
    
cleanup:
    button_driver_deinit();
    return ret;
}

esp_err_t button_driver_deinit(void)
{
    if (!s_button_driver.initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Deinitializing button driver");
    
    // Delete task
    if (s_button_driver.task_handle) {
        vTaskDelete(s_button_driver.task_handle);
        s_button_driver.task_handle = NULL;
    }
    
    // Cleanup per-button resources
    for (int i = 0; i < BUTTON_MAX; i++) {
        button_state_info_t *btn = &s_button_driver.buttons[i];
        
        if (btn->timer) {
            esp_timer_stop(btn->timer);
            esp_timer_delete(btn->timer);
            btn->timer = NULL;
        }
        
        gpio_isr_handler_remove(btn->config.gpio_num);
    }
    
    // Delete event queue
    if (s_button_driver.event_queue) {
        vQueueDelete(s_button_driver.event_queue);
        s_button_driver.event_queue = NULL;
    }
    
    s_button_driver.initialized = false;
    ESP_LOGI(TAG, "Button driver deinitialized");
    return ESP_OK;
}

QueueHandle_t button_driver_get_event_queue(void)
{
    return s_button_driver.initialized ? s_button_driver.event_queue : NULL;
}

bool button_driver_is_pressed(button_id_t button)
{
    if (!s_button_driver.initialized || button >= BUTTON_MAX) {
        return false;
    }
    
    return s_button_driver.buttons[button].debounced_level;
}

uint8_t button_driver_get_state_mask(void)
{
    uint8_t mask = 0;
    
    if (!s_button_driver.initialized) {
        return 0;
    }
    
    for (int i = 0; i < BUTTON_MAX; i++) {
        if (s_button_driver.buttons[i].debounced_level) {
            mask |= (1 << i);
        }
    }
    
    return mask;
}

esp_err_t button_driver_set_auto_repeat(button_id_t button, bool enable)
{
    if (!s_button_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (button >= BUTTON_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_button_driver.buttons[button].config.auto_repeat_enabled = enable;
    ESP_LOGI(TAG, "Auto-repeat for button %d %s", button, enable ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t button_driver_update_timing(const button_timing_config_t *timing)
{
    if (!s_button_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!timing) {
        return ESP_ERR_INVALID_ARG;
    }
    
    s_button_driver.timing = *timing;
    ESP_LOGI(TAG, "Button timing updated: debounce=%lu, long_press=%lu, auto_repeat_delay=%lu, auto_repeat_period=%lu",
             timing->debounce_time_ms, timing->long_press_time_ms, 
             timing->auto_repeat_delay_ms, timing->auto_repeat_period_ms);
    
    return ESP_OK;
}

esp_err_t button_driver_get_stats(button_id_t button, uint32_t *press_count, uint32_t *last_event_time_ms)
{
    if (!s_button_driver.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (button >= BUTTON_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    
    button_state_info_t *btn = &s_button_driver.buttons[button];
    
    if (press_count) {
        *press_count = btn->press_count;
    }
    
    if (last_event_time_ms) {
        *last_event_time_ms = btn->last_change_time;
    }
    
    return ESP_OK;
}