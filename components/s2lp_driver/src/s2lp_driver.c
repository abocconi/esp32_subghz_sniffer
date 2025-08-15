/**
 * @file s2lp_driver.c
 * @brief S2LP Sub-GHz Transceiver Driver Implementation
 * 
 * Implementation of the S2LP driver for ESP32-S3.
 * Handles SPI communication, state management, and IRQ processing.
 */

#include "s2lp_driver.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char* TAG = "s2lp_driver";

/**
 * @brief Private driver structure
 */
struct s2lp_handle_t {
    // Hardware configuration
    s2lp_hw_config_t hw_config;
    spi_device_handle_t spi_device;
    
    // RF configuration
    s2lp_rf_config_t rf_config;
    s2lp_packet_config_t packet_config;
    
    // State management
    s2lp_state_t current_state;
    SemaphoreHandle_t state_mutex;
    SemaphoreHandle_t spi_mutex;
    
    // IRQ handling
    QueueHandle_t irq_queue;
    TaskHandle_t irq_task_handle;
    s2lp_irq_callback_t irq_callback;
    void* irq_user_data;
    bool irq_task_running;
    
    // Device info
    uint8_t device_part_number;
    uint8_t device_version;
    bool initialized;
};

/**
 * @brief IRQ task parameters
 */
#define S2LP_IRQ_TASK_STACK_SIZE    4096
#define S2LP_IRQ_TASK_PRIORITY      5
#define S2LP_IRQ_QUEUE_SIZE         10
#define S2LP_IRQ_TASK_NAME          "s2lp_irq_task"

/**
 * @brief Timing constants
 */
#define S2LP_RESET_DELAY_MS         50
#define S2LP_STARTUP_DELAY_MS       100
#define S2LP_STATE_CHANGE_TIMEOUT_MS 1000
#define S2LP_CALIBRATION_TIMEOUT_MS 1000
#define S2LP_SPI_WAIT_DELAY_US      50

/**
 * @brief RSSI conversion constants (from datasheet)
 */
#define S2LP_RSSI_OFFSET_DBM        146     // RSSI offset in dBm
#define S2LP_RSSI_SCALE_DBM         1       // RSSI scale (1 dBm per LSB)

/**
 * @brief Frequency calculation constants
 */
#define S2LP_FREQ_BASE_REG          0x1F000000UL    // Base frequency register value
#define S2LP_FREQ_STEP_HZ           (S2LP_XTAL_FREQUENCY_HZ / (1UL << 25))  // Frequency step per LSB

/**
 * @brief Forward declarations
 */
static esp_err_t s2lp_spi_init(s2lp_handle_t handle);
static esp_err_t s2lp_spi_deinit(s2lp_handle_t handle);
static esp_err_t s2lp_spi_transaction(s2lp_handle_t handle, uint8_t* tx_data, uint8_t* rx_data, size_t length);
static esp_err_t s2lp_configure_rf(s2lp_handle_t handle);
static esp_err_t s2lp_configure_packet(s2lp_handle_t handle);
static esp_err_t s2lp_irq_gpio_init(s2lp_handle_t handle);
static void s2lp_irq_handler(void* arg);
static void s2lp_irq_task(void* param);
static uint32_t s2lp_frequency_to_reg(uint32_t frequency_hz);
static uint32_t s2lp_reg_to_frequency(uint32_t reg_value);

/* ========== PUBLIC API IMPLEMENTATION ========== */

/**
 * @brief Initialize S2LP driver
 */
esp_err_t s2lp_init(const s2lp_hw_config_t* hw_config,
                    const s2lp_rf_config_t* rf_config,
                    const s2lp_packet_config_t* packet_config,
                    s2lp_handle_t* out_handle)
{
    ESP_RETURN_ON_FALSE(hw_config && rf_config && packet_config && out_handle, 
                       ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    ESP_RETURN_ON_FALSE(rf_config->frequency_hz >= S2LP_MIN_FREQUENCY_HZ && 
                       rf_config->frequency_hz <= S2LP_MAX_FREQUENCY_HZ,
                       ESP_ERR_INVALID_ARG, TAG, "Frequency out of range");

    // Allocate handle
    s2lp_handle_t handle = calloc(1, sizeof(struct s2lp_handle_t));
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_NO_MEM, TAG, "Failed to allocate handle");

    esp_err_t ret = ESP_OK;

    // Copy configurations
    handle->hw_config = *hw_config;
    handle->rf_config = *rf_config;
    handle->packet_config = *packet_config;
    handle->current_state = S2LP_STATE_UNKNOWN;

    // Create mutexes
    handle->state_mutex = xSemaphoreCreateMutex();
    handle->spi_mutex = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(handle->state_mutex && handle->spi_mutex, ESP_ERR_NO_MEM, cleanup, TAG, 
                     "Failed to create mutexes");

    // Create IRQ queue
    handle->irq_queue = xQueueCreate(S2LP_IRQ_QUEUE_SIZE, sizeof(s2lp_irq_event_t));
    ESP_GOTO_ON_FALSE(handle->irq_queue, ESP_ERR_NO_MEM, cleanup, TAG, "Failed to create IRQ queue");

    // Initialize SPI
    ESP_GOTO_ON_ERROR(s2lp_spi_init(handle), cleanup, TAG, "SPI initialization failed");

    // Initialize IRQ GPIO
    ESP_GOTO_ON_ERROR(s2lp_irq_gpio_init(handle), cleanup, TAG, "IRQ GPIO initialization failed");

    // Software reset and wait for device ready
    ESP_GOTO_ON_ERROR(s2lp_reset(handle), cleanup, TAG, "Device reset failed");

    // Test communication
    ESP_GOTO_ON_ERROR(s2lp_test_communication(handle), cleanup, TAG, "Communication test failed");

    // Configure RF parameters
    ESP_GOTO_ON_ERROR(s2lp_configure_rf(handle), cleanup, TAG, "RF configuration failed");

    // Configure packet parameters
    ESP_GOTO_ON_ERROR(s2lp_configure_packet(handle), cleanup, TAG, "Packet configuration failed");

    // Start RCO calibration
    ESP_GOTO_ON_ERROR(s2lp_start_rco_calibration(handle), cleanup, TAG, "RCO calibration start failed");

    // Wait for calibration to complete
    bool calibration_complete = false;
    for (int i = 0; i < (S2LP_CALIBRATION_TIMEOUT_MS / 10); i++) {
        ESP_GOTO_ON_ERROR(s2lp_is_rco_calibration_complete(handle, &calibration_complete), cleanup, TAG,
                         "RCO calibration check failed");
        if (calibration_complete) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_GOTO_ON_FALSE(calibration_complete, ESP_ERR_TIMEOUT, cleanup, TAG, "RCO calibration timeout");

    // Configure GPIO0 for IRQ output
    ESP_GOTO_ON_ERROR(s2lp_configure_gpio(handle, 0, S2LP_GPIO_IRQ), cleanup, TAG,
                     "GPIO configuration failed");

    // Start IRQ task
    BaseType_t task_ret = xTaskCreate(s2lp_irq_task, S2LP_IRQ_TASK_NAME, S2LP_IRQ_TASK_STACK_SIZE,
                                     handle, S2LP_IRQ_TASK_PRIORITY, &handle->irq_task_handle);
    ESP_GOTO_ON_FALSE(task_ret == pdPASS, ESP_ERR_NO_MEM, cleanup, TAG, "Failed to create IRQ task");
    handle->irq_task_running = true;

    // Enter READY state
    ESP_GOTO_ON_ERROR(s2lp_enter_ready(handle), cleanup, TAG, "Failed to enter READY state");

    handle->initialized = true;
    *out_handle = handle;

    ESP_LOGI(TAG, "S2LP driver initialized successfully");
    ESP_LOGI(TAG, "Device: Part=0x%02X, Version=0x%02X", handle->device_part_number, handle->device_version);
    ESP_LOGI(TAG, "Frequency: %lu Hz, Datarate: %lu bps", handle->rf_config.frequency_hz, handle->rf_config.datarate_bps);

    return ESP_OK;

cleanup:
    s2lp_deinit(handle);
    return ret;
}

/**
 * @brief Deinitialize S2LP driver
 */
esp_err_t s2lp_deinit(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // Stop IRQ task
    if (handle->irq_task_running && handle->irq_task_handle) {
        handle->irq_task_running = false;
        vTaskDelete(handle->irq_task_handle);
        handle->irq_task_handle = NULL;
    }

    // Put device in standby if possible
    if (handle->initialized) {
        s2lp_enter_standby(handle);
    }

    // Cleanup SPI
    s2lp_spi_deinit(handle);

    // Cleanup IRQ GPIO
    if (handle->hw_config.gpio_irq >= 0) {
        gpio_isr_handler_remove(handle->hw_config.gpio_irq);
        gpio_reset_pin(handle->hw_config.gpio_irq);
    }

    // Cleanup synchronization objects
    if (handle->irq_queue) {
        vQueueDelete(handle->irq_queue);
    }
    if (handle->state_mutex) {
        vSemaphoreDelete(handle->state_mutex);
    }
    if (handle->spi_mutex) {
        vSemaphoreDelete(handle->spi_mutex);
    }

    // Free handle
    free(handle);

    ESP_LOGI(TAG, "S2LP driver deinitialized");
    return ESP_OK;
}

/**
 * @brief Software reset of S2LP device
 */
esp_err_t s2lp_reset(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_LOGI(TAG, "Performing software reset");

    // Send reset command
    ESP_RETURN_ON_ERROR(s2lp_send_command(handle, S2LP_CMD_SRES), TAG, "Reset command failed");

    // Wait for reset to complete
    vTaskDelay(pdMS_TO_TICKS(S2LP_RESET_DELAY_MS));

    // Wait for device to be ready
    ESP_RETURN_ON_ERROR(s2lp_wait_for_state(handle, S2LP_STATE_READY, S2LP_STARTUP_DELAY_MS), TAG,
                       "Device not ready after reset");

    // Get device info
    ESP_RETURN_ON_ERROR(s2lp_get_device_info(handle, &handle->device_part_number, &handle->device_version), TAG,
                       "Failed to get device info");

    ESP_LOGI(TAG, "Reset complete - Device ready");
    return ESP_OK;
}

/**
 * @brief Get current device state
 */
esp_err_t s2lp_get_state(s2lp_handle_t handle, s2lp_state_t* out_state)
{
    ESP_RETURN_ON_FALSE(handle && out_state, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t state_reg;
    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_MC_STATE0, &state_reg), TAG,
                       "Failed to read state register");

    // Extract state from register (bits 7:1)
    *out_state = (s2lp_state_t)((state_reg >> 1) & 0x7F);

    // Update cached state
    if (xSemaphoreTake(handle->state_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        handle->current_state = *out_state;
        xSemaphoreGive(handle->state_mutex);
    }

    return ESP_OK;
}

/**
 * @brief Send command to S2LP
 */
esp_err_t s2lp_send_command(s2lp_handle_t handle, uint8_t command)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t tx_data[2] = {S2LP_SPI_COMMAND, command};
    uint8_t rx_data[2] = {0};

    ESP_RETURN_ON_ERROR(s2lp_spi_transaction(handle, tx_data, rx_data, 2), TAG,
                       "SPI transaction failed");

    // Small delay to allow command processing
    esp_rom_delay_us(S2LP_SPI_WAIT_DELAY_US);

    return ESP_OK;
}

/**
 * @brief Read register from S2LP
 */
esp_err_t s2lp_read_register(s2lp_handle_t handle, uint8_t reg_addr, uint8_t* out_value)
{
    ESP_RETURN_ON_FALSE(handle && out_value, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t tx_data[2] = {S2LP_SPI_READ | reg_addr, 0x00};
    uint8_t rx_data[2] = {0};

    ESP_RETURN_ON_ERROR(s2lp_spi_transaction(handle, tx_data, rx_data, 2), TAG,
                       "SPI transaction failed");

    *out_value = rx_data[1];
    return ESP_OK;
}

/**
 * @brief Write register to S2LP
 */
esp_err_t s2lp_write_register(s2lp_handle_t handle, uint8_t reg_addr, uint8_t value)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t tx_data[2] = {S2LP_SPI_WRITE | reg_addr, value};
    uint8_t rx_data[2] = {0};

    ESP_RETURN_ON_ERROR(s2lp_spi_transaction(handle, tx_data, rx_data, 2), TAG,
                       "SPI transaction failed");

    return ESP_OK;
}

/**
 * @brief Read multiple registers from S2LP
 */
esp_err_t s2lp_read_registers(s2lp_handle_t handle, uint8_t start_addr, uint8_t* buffer, size_t length)
{
    ESP_RETURN_ON_FALSE(handle && buffer && length > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    // Allocate temporary buffers
    uint8_t* tx_data = malloc(length + 1);
    uint8_t* rx_data = malloc(length + 1);
    ESP_RETURN_ON_FALSE(tx_data && rx_data, ESP_ERR_NO_MEM, TAG, "Memory allocation failed");

    esp_err_t ret = ESP_OK;

    // Prepare transaction
    tx_data[0] = S2LP_SPI_READ | start_addr;
    memset(&tx_data[1], 0x00, length);

    // Execute transaction
    ESP_GOTO_ON_ERROR(s2lp_spi_transaction(handle, tx_data, rx_data, length + 1), cleanup, TAG,
                     "SPI transaction failed");

    // Copy received data (skip first byte which is status)
    memcpy(buffer, &rx_data[1], length);

cleanup:
    free(tx_data);
    free(rx_data);
    return ret;
}

/**
 * @brief Write multiple registers to S2LP
 */
esp_err_t s2lp_write_registers(s2lp_handle_t handle, uint8_t start_addr, const uint8_t* buffer, size_t length)
{
    ESP_RETURN_ON_FALSE(handle && buffer && length > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    // Allocate temporary buffers
    uint8_t* tx_data = malloc(length + 1);
    uint8_t* rx_data = malloc(length + 1);
    ESP_RETURN_ON_FALSE(tx_data && rx_data, ESP_ERR_NO_MEM, TAG, "Memory allocation failed");

    esp_err_t ret = ESP_OK;

    // Prepare transaction
    tx_data[0] = S2LP_SPI_WRITE | start_addr;
    memcpy(&tx_data[1], buffer, length);

    // Execute transaction
    ESP_GOTO_ON_ERROR(s2lp_spi_transaction(handle, tx_data, rx_data, length + 1), cleanup, TAG,
                     "SPI transaction failed");

cleanup:
    free(tx_data);
    free(rx_data);
    return ret;
}

/**
 * @brief Set RF frequency
 */
esp_err_t s2lp_set_frequency(s2lp_handle_t handle, uint32_t frequency_hz)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(frequency_hz >= S2LP_MIN_FREQUENCY_HZ && frequency_hz <= S2LP_MAX_FREQUENCY_HZ,
                       ESP_ERR_INVALID_ARG, TAG, "Frequency out of range");

    // Convert frequency to register value
    uint32_t freq_reg = s2lp_frequency_to_reg(frequency_hz);

    // Write frequency registers (SYNT3:SYNT0)
    uint8_t freq_bytes[4] = {
        (freq_reg >> 24) & 0xFF,    // SYNT3
        (freq_reg >> 16) & 0xFF,    // SYNT2
        (freq_reg >> 8) & 0xFF,     // SYNT1
        freq_reg & 0xFF             // SYNT0
    };

    ESP_RETURN_ON_ERROR(s2lp_write_registers(handle, S2LP_REG_SYNT3, freq_bytes, 4), TAG,
                       "Failed to write frequency registers");

    // Update cached frequency
    handle->rf_config.frequency_hz = frequency_hz;

    ESP_LOGD(TAG, "Frequency set to %lu Hz (reg: 0x%08lX)", frequency_hz, freq_reg);
    return ESP_OK;
}

/**
 * @brief Get current RF frequency
 */
esp_err_t s2lp_get_frequency(s2lp_handle_t handle, uint32_t* out_frequency_hz)
{
    ESP_RETURN_ON_FALSE(handle && out_frequency_hz, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t freq_bytes[4];
    ESP_RETURN_ON_ERROR(s2lp_read_registers(handle, S2LP_REG_SYNT3, freq_bytes, 4), TAG,
                       "Failed to read frequency registers");

    // Combine bytes to get register value
    uint32_t freq_reg = ((uint32_t)freq_bytes[0] << 24) |
                        ((uint32_t)freq_bytes[1] << 16) |
                        ((uint32_t)freq_bytes[2] << 8) |
                        freq_bytes[3];

    // Convert register value to frequency
    *out_frequency_hz = s2lp_reg_to_frequency(freq_reg);

    return ESP_OK;
}

/**
 * @brief Set channel (0-19 for 860-879 MHz with 1 MHz steps)
 */
esp_err_t s2lp_set_channel(s2lp_handle_t handle, uint8_t channel)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(channel < 20, ESP_ERR_INVALID_ARG, TAG, "Channel out of range (0-19)");

    // Calculate frequency for channel
    uint32_t frequency_hz = S2LP_MIN_FREQUENCY_HZ + (channel * S2LP_FREQUENCY_STEP_HZ);
    
    return s2lp_set_frequency(handle, frequency_hz);
}

/**
 * @brief Get current channel
 */
esp_err_t s2lp_get_channel(s2lp_handle_t handle, uint8_t* out_channel)
{
    ESP_RETURN_ON_FALSE(handle && out_channel, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint32_t frequency_hz;
    ESP_RETURN_ON_ERROR(s2lp_get_frequency(handle, &frequency_hz), TAG, "Failed to get frequency");

    // Calculate channel from frequency
    if (frequency_hz < S2LP_MIN_FREQUENCY_HZ || frequency_hz > S2LP_MAX_FREQUENCY_HZ) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out_channel = (frequency_hz - S2LP_MIN_FREQUENCY_HZ) / S2LP_FREQUENCY_STEP_HZ;
    return ESP_OK;
}

/**
 * @brief Enter RX mode
 */
esp_err_t s2lp_enter_rx(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_RETURN_ON_ERROR(s2lp_send_command(handle, S2LP_CMD_RX), TAG, "RX command failed");
    return s2lp_wait_for_state(handle, S2LP_STATE_RX, S2LP_STATE_CHANGE_TIMEOUT_MS);
}

/**
 * @brief Enter TX mode
 */
esp_err_t s2lp_enter_tx(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_RETURN_ON_ERROR(s2lp_send_command(handle, S2LP_CMD_TX), TAG, "TX command failed");
    return s2lp_wait_for_state(handle, S2LP_STATE_TX, S2LP_STATE_CHANGE_TIMEOUT_MS);
}

/**
 * @brief Enter READY state
 */
esp_err_t s2lp_enter_ready(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_RETURN_ON_ERROR(s2lp_send_command(handle, S2LP_CMD_READY), TAG, "READY command failed");
    return s2lp_wait_for_state(handle, S2LP_STATE_READY, S2LP_STATE_CHANGE_TIMEOUT_MS);
}

/**
 * @brief Enter STANDBY state (lowest power)
 */
esp_err_t s2lp_enter_standby(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_RETURN_ON_ERROR(s2lp_send_command(handle, S2LP_CMD_STANDBY), TAG, "STANDBY command failed");
    return s2lp_wait_for_state(handle, S2LP_STATE_STANDBY, S2LP_STATE_CHANGE_TIMEOUT_MS);
}

/**
 * @brief Abort current operation and return to READY
 */
esp_err_t s2lp_abort(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    ESP_RETURN_ON_ERROR(s2lp_send_command(handle, S2LP_CMD_SABORT), TAG, "SABORT command failed");
    return s2lp_wait_for_state(handle, S2LP_STATE_READY, S2LP_STATE_CHANGE_TIMEOUT_MS);
}

/**
 * @brief Read current RSSI value
 */
esp_err_t s2lp_read_rssi(s2lp_handle_t handle, s2lp_rssi_t* out_rssi)
{
    ESP_RETURN_ON_FALSE(handle && out_rssi, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t rssi_reg;
    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_RSSI_LEVEL, &rssi_reg), TAG,
                       "Failed to read RSSI register");

    out_rssi->rssi_dbm = s2lp_rssi_reg_to_dbm(rssi_reg);
    out_rssi->valid = true;
    out_rssi->timestamp = xTaskGetTickCount();

    return ESP_OK;
}

/**
 * @brief Read RSSI value continuously (for scanning)
 */
esp_err_t s2lp_read_rssi_run(s2lp_handle_t handle, int16_t* out_rssi_dbm)
{
    ESP_RETURN_ON_FALSE(handle && out_rssi_dbm, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t rssi_reg;
    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_RSSI_LEVEL_RUN, &rssi_reg), TAG,
                       "Failed to read running RSSI register");

    *out_rssi_dbm = s2lp_rssi_reg_to_dbm(rssi_reg);
    return ESP_OK;
}

/**
 * @brief Check if carrier is detected (RSSI above threshold)
 */
esp_err_t s2lp_check_carrier_sense(s2lp_handle_t handle, bool* out_detected)
{
    ESP_RETURN_ON_FALSE(handle && out_detected, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t link_qual_reg;
    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_LINK_QUALIF1, &link_qual_reg), TAG,
                       "Failed to read link quality register");

    // Check CS bit (bit 7)
    *out_detected = (link_qual_reg & 0x80) != 0;
    return ESP_OK;
}

/**
 * @brief Configure IRQ mask
 */
esp_err_t s2lp_set_irq_mask(s2lp_handle_t handle, uint32_t irq_mask)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // IRQ mask is stored in 4 registers (IRQ_MASK3:IRQ_MASK0)
    uint8_t mask_bytes[4] = {
        (irq_mask >> 24) & 0xFF,    // IRQ_MASK3
        (irq_mask >> 16) & 0xFF,    // IRQ_MASK2
        (irq_mask >> 8) & 0xFF,     // IRQ_MASK1
        irq_mask & 0xFF             // IRQ_MASK0
    };

    ESP_RETURN_ON_ERROR(s2lp_write_registers(handle, S2LP_REG_IRQ_MASK3, mask_bytes, 4), TAG,
                       "Failed to write IRQ mask registers");

    return ESP_OK;
}

/**
 * @brief Get current IRQ mask
 */
esp_err_t s2lp_get_irq_mask(s2lp_handle_t handle, uint32_t* out_irq_mask)
{
    ESP_RETURN_ON_FALSE(handle && out_irq_mask, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t mask_bytes[4];
    ESP_RETURN_ON_ERROR(s2lp_read_registers(handle, S2LP_REG_IRQ_MASK3, mask_bytes, 4), TAG,
                       "Failed to read IRQ mask registers");

    *out_irq_mask = ((uint32_t)mask_bytes[0] << 24) |
                    ((uint32_t)mask_bytes[1] << 16) |
                    ((uint32_t)mask_bytes[2] << 8) |
                    mask_bytes[3];

    return ESP_OK;
}

/**
 * @brief Read and clear IRQ status
 */
esp_err_t s2lp_read_irq_status(s2lp_handle_t handle, uint32_t* out_irq_status)
{
    ESP_RETURN_ON_FALSE(handle && out_irq_status, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t status_bytes[4];
    ESP_RETURN_ON_ERROR(s2lp_read_registers(handle, S2LP_REG_IRQ_STATUS3, status_bytes, 4), TAG,
                       "Failed to read IRQ status registers");

    *out_irq_status = ((uint32_t)status_bytes[0] << 24) |
                      ((uint32_t)status_bytes[1] << 16) |
                      ((uint32_t)status_bytes[2] << 8) |
                      status_bytes[3];

    return ESP_OK;
}

/**
 * @brief Register IRQ callback
 */
esp_err_t s2lp_register_irq_callback(s2lp_handle_t handle, 
                                     s2lp_irq_callback_t callback, 
                                     void* user_data)
{
    ESP_RETURN_ON_FALSE(handle && callback, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    handle->irq_callback = callback;
    handle->irq_user_data = user_data;

    return ESP_OK;
}

/**
 * @brief Unregister IRQ callback
 */
esp_err_t s2lp_unregister_irq_callback(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    handle->irq_callback = NULL;
    handle->irq_user_data = NULL;

    return ESP_OK;
}

/**
 * @brief Read data from RX FIFO
 */
esp_err_t s2lp_read_fifo(s2lp_handle_t handle, uint8_t* buffer, size_t buffer_size, size_t* out_bytes_read)
{
    ESP_RETURN_ON_FALSE(handle && buffer && out_bytes_read, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    // Get RX FIFO status first
    uint8_t fifo_bytes;
    ESP_RETURN_ON_ERROR(s2lp_get_rx_fifo_status(handle, &fifo_bytes), TAG, "Failed to get FIFO status");

    // Limit read to available bytes and buffer size
    size_t bytes_to_read = (fifo_bytes < buffer_size) ? fifo_bytes : buffer_size;
    
    if (bytes_to_read == 0) {
        *out_bytes_read = 0;
        return ESP_OK;
    }

    // Read from FIFO register
    ESP_RETURN_ON_ERROR(s2lp_read_registers(handle, S2LP_REG_FIFO, buffer, bytes_to_read), TAG,
                       "Failed to read FIFO");

    *out_bytes_read = bytes_to_read;
    return ESP_OK;
}

/**
 * @brief Write data to TX FIFO
 */
esp_err_t s2lp_write_fifo(s2lp_handle_t handle, const uint8_t* buffer, size_t length)
{
    ESP_RETURN_ON_FALSE(handle && buffer && length > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    // Write to FIFO register
    ESP_RETURN_ON_ERROR(s2lp_write_registers(handle, S2LP_REG_FIFO, buffer, length), TAG,
                       "Failed to write FIFO");

    return ESP_OK;
}

/**
 * @brief Get RX FIFO status
 */
esp_err_t s2lp_get_rx_fifo_status(s2lp_handle_t handle, uint8_t* out_bytes_available)
{
    ESP_RETURN_ON_FALSE(handle && out_bytes_available, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_RX_FIFO_STATUS, out_bytes_available), TAG,
                       "Failed to read RX FIFO status");

    return ESP_OK;
}

/**
 * @brief Get TX FIFO status
 */
esp_err_t s2lp_get_tx_fifo_status(s2lp_handle_t handle, uint8_t* out_bytes_available)
{
    ESP_RETURN_ON_FALSE(handle && out_bytes_available, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_TX_FIFO_STATUS, out_bytes_available), TAG,
                       "Failed to read TX FIFO status");

    return ESP_OK;
}

/**
 * @brief Flush RX FIFO
 */
esp_err_t s2lp_flush_rx_fifo(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    return s2lp_send_command(handle, S2LP_CMD_FLUSHRXFIFO);
}

/**
 * @brief Flush TX FIFO
 */
esp_err_t s2lp_flush_tx_fifo(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    return s2lp_send_command(handle, S2LP_CMD_FLUSHTXFIFO);
}

/**
 * @brief Get device part number and version
 */
esp_err_t s2lp_get_device_info(s2lp_handle_t handle, uint8_t* out_part_number, uint8_t* out_version)
{
    ESP_RETURN_ON_FALSE(handle && out_part_number && out_version, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_DEVICE_INFO1, out_part_number), TAG,
                       "Failed to read part number");
    
    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_DEVICE_INFO0, out_version), TAG,
                       "Failed to read version");

    return ESP_OK;
}

/**
 * @brief Start RCO calibration
 */
esp_err_t s2lp_start_rco_calibration(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    // Enable RCO calibration (bit 0 of XO_RCO_CONF0)
    uint8_t rco_conf;
    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_XO_RCO_CONF0, &rco_conf), TAG,
                       "Failed to read RCO config");

    rco_conf |= 0x01;  // Set RCO_CALIBRATION bit
    ESP_RETURN_ON_ERROR(s2lp_write_register(handle, S2LP_REG_XO_RCO_CONF0, rco_conf), TAG,
                       "Failed to start RCO calibration");

    ESP_LOGD(TAG, "RCO calibration started");
    return ESP_OK;
}

/**
 * @brief Check if RCO calibration is complete
 */
esp_err_t s2lp_is_rco_calibration_complete(s2lp_handle_t handle, bool* out_complete)
{
    ESP_RETURN_ON_FALSE(handle && out_complete, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t state_reg;
    ESP_RETURN_ON_ERROR(s2lp_read_register(handle, S2LP_REG_MC_STATE1, &state_reg), TAG,
                       "Failed to read state register");

    // Check RCO_CAL_OK bit (bit 4)
    *out_complete = (state_reg & 0x10) != 0;
    return ESP_OK;
}

/**
 * @brief Configure GPIO function
 */
esp_err_t s2lp_configure_gpio(s2lp_handle_t handle, uint8_t gpio_num, s2lp_gpio_func_t function)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(gpio_num < 4, ESP_ERR_INVALID_ARG, TAG, "GPIO number out of range");

    uint8_t gpio_reg_addr = S2LP_REG_GPIO0_CONF + gpio_num;
    ESP_RETURN_ON_ERROR(s2lp_write_register(handle, gpio_reg_addr, (uint8_t)function), TAG,
                       "Failed to configure GPIO");

    ESP_LOGD(TAG, "GPIO%d configured for function %d", gpio_num, function);
    return ESP_OK;
}

/**
 * @brief Wait for device state change with timeout
 */
esp_err_t s2lp_wait_for_state(s2lp_handle_t handle, s2lp_state_t expected_state, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    TickType_t start_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        s2lp_state_t current_state;
        ESP_RETURN_ON_ERROR(s2lp_get_state(handle, &current_state), TAG, "Failed to get state");

        if (current_state == expected_state) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms polling interval
    }

    ESP_LOGE(TAG, "Timeout waiting for state %s", s2lp_state_to_string(expected_state));
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Check if device is responsive (communication test)
 */
esp_err_t s2lp_test_communication(s2lp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    uint8_t part_number, version;
    ESP_RETURN_ON_ERROR(s2lp_get_device_info(handle, &part_number, &version), TAG,
                       "Failed to read device info");

    // Check for expected part number (0x03 for S2-LP)
    ESP_RETURN_ON_FALSE(part_number == 0x03, ESP_ERR_INVALID_RESPONSE, TAG,
                       "Unexpected part number: 0x%02X", part_number);

    ESP_LOGI(TAG, "Communication test passed - Part: 0x%02X, Version: 0x%02X", part_number, version);
    return ESP_OK;
}

/**
 * @brief Convert RSSI register value to dBm
 */
int16_t s2lp_rssi_reg_to_dbm(uint8_t rssi_reg)
{
    // RSSI = -146 + register_value (in dBm)
    return -S2LP_RSSI_OFFSET_DBM + (int16_t)rssi_reg;
}

/**
 * @brief Convert dBm to RSSI register value
 */
uint8_t s2lp_rssi_dbm_to_reg(int16_t rssi_dbm)
{
    // register_value = RSSI + 146
    int16_t reg_val = rssi_dbm + S2LP_RSSI_OFFSET_DBM;
    return (reg_val < 0) ? 0 : (reg_val > 255) ? 255 : (uint8_t)reg_val;
}

/**
 * @brief Convert state code to string
 */
const char* s2lp_state_to_string(s2lp_state_t state)
{
    switch (state) {
        case S2LP_STATE_SHUTDOWN:    return "SHUTDOWN";
        case S2LP_STATE_SLEEP_A:     return "SLEEP_A";
        case S2LP_STATE_SLEEP_B:     return "SLEEP_B";
        case S2LP_STATE_READY:       return "READY";
        case S2LP_STATE_STANDBY:     return "STANDBY";
        case S2LP_STATE_LOCKST:      return "LOCKST";
        case S2LP_STATE_LOCKON:      return "LOCKON";
        case S2LP_STATE_RX:          return "RX";
        case S2LP_STATE_TX:          return "TX";
        case S2LP_STATE_SYNTH_SETUP: return "SYNTH_SETUP";
        case S2LP_STATE_WAIT_SLEEP:  return "WAIT_SLEEP";
        default:                     return "UNKNOWN";
    }
}

/**
 * @brief Convert IRQ flags to string (for debugging)
 */
esp_err_t s2lp_irq_flags_to_string(uint32_t irq_flags, char* buffer, size_t buffer_size)
{
    ESP_RETURN_ON_FALSE(buffer && buffer_size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    buffer[0] = '\0';
    size_t pos = 0;

    const struct {
        uint32_t flag;
        const char* name;
    } irq_names[] = {
        {S2LP_IRQ_RX_DATA_READY, "RX_DATA_READY"},
        {S2LP_IRQ_RX_DATA_DISC, "RX_DATA_DISC"},
        {S2LP_IRQ_TX_DATA_SENT, "TX_DATA_SENT"},
        {S2LP_IRQ_MAX_RE_TX_REACH, "MAX_RE_TX_REACH"},
        {S2LP_IRQ_CRC_ERROR, "CRC_ERROR"},
        {S2LP_IRQ_TX_FIFO_ERROR, "TX_FIFO_ERROR"},
        {S2LP_IRQ_RX_FIFO_ERROR, "RX_FIFO_ERROR"},
        {S2LP_IRQ_TX_FIFO_ALMOST_FULL, "TX_FIFO_ALMOST_FULL"},
        {S2LP_IRQ_TX_FIFO_ALMOST_EMPTY, "TX_FIFO_ALMOST_EMPTY"},
        {S2LP_IRQ_RX_FIFO_ALMOST_FULL, "RX_FIFO_ALMOST_FULL"},
        {S2LP_IRQ_RX_FIFO_ALMOST_EMPTY, "RX_FIFO_ALMOST_EMPTY"},
        {S2LP_IRQ_MAX_BO_CCA_REACH, "MAX_BO_CCA_REACH"},
        {S2LP_IRQ_VALID_PREAMBLE, "VALID_PREAMBLE"},
        {S2LP_IRQ_VALID_SYNC, "VALID_SYNC"},
        {S2LP_IRQ_RSSI_ABOVE_TH, "RSSI_ABOVE_TH"},
        {S2LP_IRQ_WKUP_TOUT_LDC, "WKUP_TOUT_LDC"},
        {S2LP_IRQ_READY, "READY"},
        {S2LP_IRQ_STANDBY_DELAYED, "STANDBY_DELAYED"},
        {S2LP_IRQ_LOW_BATT_LVL, "LOW_BATT_LVL"},
        {S2LP_IRQ_POR, "POR"},
        {S2LP_IRQ_RX_TIMEOUT, "RX_TIMEOUT"},
        {S2LP_IRQ_RX_SNIFF_TIMEOUT, "RX_SNIFF_TIMEOUT"}
    };

    for (size_t i = 0; i < sizeof(irq_names) / sizeof(irq_names[0]); i++) {
        if (irq_flags & irq_names[i].flag) {
            if (pos > 0 && pos < buffer_size - 1) {
                buffer[pos++] = '|';
            }
            size_t name_len = strlen(irq_names[i].name);
            if (pos + name_len < buffer_size) {
                strcpy(&buffer[pos], irq_names[i].name);
                pos += name_len;
            }
        }
    }

    if (pos == 0) {
        strcpy(buffer, "NONE");
    }

    return ESP_OK;
}

/* ========== PRIVATE FUNCTION IMPLEMENTATIONS ========== */

/**
 * @brief Initialize SPI interface
 */
static esp_err_t s2lp_spi_init(s2lp_handle_t handle)
{
    // Configure SPI bus
    spi_bus_config_t bus_config = {
        .mosi_io_num = handle->hw_config.gpio_mosi,
        .miso_io_num = handle->hw_config.gpio_miso,
        .sclk_io_num = handle->hw_config.gpio_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,  // Max transfer size for register access
        .flags = 0,
        .intr_flags = 0
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(handle->hw_config.spi_host, &bus_config, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = S2LP_SPI_CLOCK_SPEED_HZ,
        .mode = S2LP_SPI_MODE,
        .spics_io_num = handle->hw_config.gpio_cs,
        .queue_size = 1,  // Single transaction at a time
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL
    };

    // Add device to SPI bus
    ESP_RETURN_ON_ERROR(spi_bus_add_device(handle->hw_config.spi_host, &dev_config, &handle->spi_device), TAG,
                       "SPI device add failed");

    ESP_LOGD(TAG, "SPI initialized - Host: %d, CS: %d, Clock: %d Hz", 
             handle->hw_config.spi_host, handle->hw_config.gpio_cs, S2LP_SPI_CLOCK_SPEED_HZ);

    return ESP_OK;
}

/**
 * @brief Deinitialize SPI interface
 */
static esp_err_t s2lp_spi_deinit(s2lp_handle_t handle)
{
    if (handle->spi_device) {
        spi_bus_remove_device(handle->spi_device);
        handle->spi_device = NULL;
    }

    // Note: We don't free the SPI bus as it might be used by other devices
    return ESP_OK;
}

/**
 * @brief Execute SPI transaction
 */
static esp_err_t s2lp_spi_transaction(s2lp_handle_t handle, uint8_t* tx_data, uint8_t* rx_data, size_t length)
{
    ESP_RETURN_ON_FALSE(handle && handle->spi_device && tx_data && rx_data && length > 0,
                       ESP_ERR_INVALID_ARG, TAG, "Invalid SPI transaction parameters");

    // Take SPI mutex
    ESP_RETURN_ON_FALSE(xSemaphoreTake(handle->spi_mutex, pdMS_TO_TICKS(S2LP_SPI_TRANSACTION_TIMEOUT_MS)) == pdTRUE,
                       ESP_ERR_TIMEOUT, TAG, "SPI mutex timeout");

    esp_err_t ret = ESP_OK;

    spi_transaction_t trans = {
        .length = length * 8,  // Length in bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
        .flags = 0
    };

    ret = spi_device_transmit(handle->spi_device, &trans);

    xSemaphoreGive(handle->spi_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Configure RF parameters
 */
static esp_err_t s2lp_configure_rf(s2lp_handle_t handle)
{
    ESP_LOGI(TAG, "Configuring RF parameters");

    // Set frequency
    ESP_RETURN_ON_ERROR(s2lp_set_frequency(handle, handle->rf_config.frequency_hz), TAG,
                       "Failed to set frequency");

    // Configure modulation (MOD1 register)
    uint8_t mod1_reg = 0x00;
    switch (handle->rf_config.modulation) {
        case S2LP_MOD_2FSK:
            mod1_reg = 0x00;
            break;
        case S2LP_MOD_4FSK:
            mod1_reg = 0x01;
            break;
        case S2LP_MOD_2GFSK_BT05:
            mod1_reg = 0x02;
            break;
        case S2LP_MOD_2GFSK_BT1:
            mod1_reg = 0x03;
            break;
        case S2LP_MOD_4GFSK_BT05:
            mod1_reg = 0x04;
            break;
        case S2LP_MOD_4GFSK_BT1:
            mod1_reg = 0x05;
            break;
        case S2LP_MOD_ASK_OOK:
            mod1_reg = 0x06;
            break;
        case S2LP_MOD_MSK:
            mod1_reg = 0x07;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(s2lp_write_register(handle, S2LP_REG_MOD1, mod1_reg), TAG,
                       "Failed to set modulation");

    // Configure datarate (simplified - would need proper calculation for production)
    // For 10 kbps with 50 MHz crystal: DR = datarate * 2^20 / f_xtal
    uint32_t dr_reg = (uint32_t)((uint64_t)handle->rf_config.datarate_bps * (1ULL << 20) / S2LP_XTAL_FREQUENCY_HZ);
    uint8_t dr_bytes[3] = {
        (dr_reg >> 16) & 0xFF,  // MOD2
        (dr_reg >> 8) & 0xFF,   // MOD1 (lower part)
        dr_reg & 0xFF           // MOD0
    };
    ESP_RETURN_ON_ERROR(s2lp_write_registers(handle, S2LP_REG_MOD2, dr_bytes, 3), TAG,
                       "Failed to set datarate");

    // Configure channel filter bandwidth (simplified)
    uint8_t chflt_reg = 0x23;  // Default value for ~100 kHz BW
    ESP_RETURN_ON_ERROR(s2lp_write_register(handle, S2LP_REG_CHFLT, chflt_reg), TAG,
                       "Failed to set channel filter");

    // Configure RSSI threshold if enabled
    if (handle->rf_config.enable_rssi_threshold) {
        uint8_t rssi_th_reg = s2lp_rssi_dbm_to_reg(handle->rf_config.rssi_threshold_dbm);
        ESP_RETURN_ON_ERROR(s2lp_write_register(handle, S2LP_REG_RSSI_TH, rssi_th_reg), TAG,
                           "Failed to set RSSI threshold");
    }

    // Configure output power (simplified - uses PA_POWER0 for basic power setting)
    uint8_t pa_power = 0x25;  // Default ~10 dBm (would need proper calculation)
    ESP_RETURN_ON_ERROR(s2lp_write_register(handle, S2LP_REG_PA_POWER0, pa_power), TAG,
                       "Failed to set output power");

    ESP_LOGI(TAG, "RF configuration complete");
    return ESP_OK;
}

/**
 * @brief Configure packet parameters
 */
static esp_err_t s2lp_configure_packet(s2lp_handle_t handle)
{
    ESP_LOGI(TAG, "Configuring packet parameters");

    // Configure packet length
    uint16_t pkt_len = handle->packet_config.max_packet_length;
    uint8_t pkt_len_bytes[2] = {
        (pkt_len >> 8) & 0xFF,  // PCKTLEN1
        pkt_len & 0xFF          // PCKTLEN0
    };
    ESP_RETURN_ON_ERROR(s2lp_write_registers(handle, S2LP_REG_PCKTLEN1, pkt_len_bytes, 2), TAG,
                       "Failed to set packet length");

    // Configure sync word
    uint32_t sync_word = handle->packet_config.sync_word;
    uint8_t sync_bytes[4] = {
        (sync_word >> 24) & 0xFF,  // SYNC3
        (sync_word >> 16) & 0xFF,  // SYNC2
        (sync_word >> 8) & 0xFF,   // SYNC1
        sync_word & 0xFF           // SYNC0
    };
    ESP_RETURN_ON_ERROR(s2lp_write_registers(handle, S2LP_REG_SYNC3, sync_bytes, 4), TAG,
                       "Failed to set sync word");

    // Configure preamble length
    uint8_t preamble_reg = (handle->packet_config.preamble_length / 8) - 1;  // Convert bits to bytes
    ESP_RETURN_ON_ERROR(s2lp_write_register(handle, S2LP_REG_PCKT_PSTMBL, preamble_reg), TAG,
                       "Failed to set preamble length");

    // Configure packet control (PCKTCTRL1)
    uint8_t pckt_ctrl1 = 0x00;
    if (handle->packet_config.enable_crc) {
        pckt_ctrl1 |= 0x20;  // Enable CRC
    }
    if (handle->packet_config.enable_fec) {
        pckt_ctrl1 |= 0x01;  // Enable FEC
    }
    ESP_RETURN_ON_ERROR(s2lp_write_register(handle, S2LP_REG_PCKTCTRL1, pckt_ctrl1), TAG,
                       "Failed to set packet control");

    ESP_LOGI(TAG, "Packet configuration complete");
    return ESP_OK;
}

/**
 * @brief Initialize IRQ GPIO
 */
static esp_err_t s2lp_irq_gpio_init(s2lp_handle_t handle)
{
    if (handle->hw_config.gpio_irq < 0) {
        ESP_LOGW(TAG, "IRQ GPIO not configured - IRQ functionality disabled");
        return ESP_OK;
    }

    // Configure GPIO for IRQ input
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << handle->hw_config.gpio_irq),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // S2LP IRQ is active low
    };

    ESP_RETURN_ON_ERROR(gpio_config(&gpio_conf), TAG, "IRQ GPIO config failed");

    // Install ISR service if not already installed
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add ISR handler
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(handle->hw_config.gpio_irq, s2lp_irq_handler, handle), TAG,
                       "IRQ ISR handler add failed");

    ESP_LOGD(TAG, "IRQ GPIO initialized on pin %d", handle->hw_config.gpio_irq);
    return ESP_OK;
}

/**
 * @brief GPIO IRQ handler (ISR)
 */
static void IRAM_ATTR s2lp_irq_handler(void* arg)
{
    s2lp_handle_t handle = (s2lp_handle_t)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Signal IRQ task
    if (handle->irq_queue) {
        s2lp_irq_event_t irq_event = {
            .irq_flags = 0,  // Will be read by IRQ task
            .timestamp = xTaskGetTickCountFromISR(),
            .state = handle->current_state
        };

        xQueueSendFromISR(handle->irq_queue, &irq_event, &higher_priority_task_woken);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief IRQ processing task
 */
static void s2lp_irq_task(void* param)
{
    s2lp_handle_t handle = (s2lp_handle_t)param;
    s2lp_irq_event_t irq_event;

    ESP_LOGI(TAG, "IRQ task started");

    while (handle->irq_task_running) {
        // Wait for IRQ event
        if (xQueueReceive(handle->irq_queue, &irq_event, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Read IRQ status from device
            uint32_t irq_status = 0;
            esp_err_t ret = s2lp_read_irq_status(handle, &irq_status);
            
// Questa è la parte mancante da aggiungere al file s2lp_driver.c
// Inizia dalla riga che è stata troncata nella funzione s2lp_irq_task

            if (ret == ESP_OK && irq_status != 0) {
                irq_event.irq_flags = irq_status;
                
                // Update current state
                s2lp_get_state(handle, &irq_event.state);

                // Call user callback if registered
                if (handle->irq_callback) {
                    handle->irq_callback(handle, &irq_event, handle->irq_user_data);
                }

                // Log IRQ for debugging
                char irq_str[256];
                s2lp_irq_flags_to_string(irq_status, irq_str, sizeof(irq_str));
                ESP_LOGD(TAG, "IRQ: %s (State: %s)", irq_str, s2lp_state_to_string(irq_event.state));
            }
        }
    }

    ESP_LOGI(TAG, "IRQ task stopped");
    vTaskDelete(NULL);
}

/**
 * @brief Convert frequency to register value
 */
static uint32_t s2lp_frequency_to_reg(uint32_t frequency_hz)
{
    // Frequency register = (frequency_hz * 2^25) / f_xtal
    return (uint32_t)((uint64_t)frequency_hz * (1ULL << 25) / S2LP_XTAL_FREQUENCY_HZ);
}

/**
 * @brief Convert register value to frequency
 */
static uint32_t s2lp_reg_to_frequency(uint32_t reg_value)
{
    // frequency_hz = (reg_value * f_xtal) / 2^25
    return (uint32_t)((uint64_t)reg_value * S2LP_XTAL_FREQUENCY_HZ / (1ULL << 25));
}            