/**
 * @file s2lp_driver.h
 * @brief S2LP Sub-GHz Transceiver Driver for ESP32-S3
 * 
 * Driver for STMicroelectronics S2LP sub-GHz transceiver.
 * Supports 860-879 MHz operation with SPI interface.
 * No SHUTDOWN pin connected - uses software reset only.
 * 
 * @note This driver is designed for ESP-IDF and ESP32-S3
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief S2LP Driver Version
 */
#define S2LP_DRIVER_VERSION_MAJOR   1
#define S2LP_DRIVER_VERSION_MINOR   0
#define S2LP_DRIVER_VERSION_PATCH   0

/**
 * @brief Hardware Configuration
 */
#define S2LP_XTAL_FREQUENCY_HZ      25000000    // 25 MHz crystal
#define S2LP_MIN_FREQUENCY_HZ       860000000   // 860 MHz
#define S2LP_MAX_FREQUENCY_HZ       879000000   // 879 MHz
#define S2LP_FREQUENCY_STEP_HZ      1000000     // 1 MHz step

/**
 * @brief SPI Communication Parameters
 */
#define S2LP_SPI_CLOCK_SPEED_HZ     1000000     // 1 MHz SPI clock
#define S2LP_SPI_MODE              0            // SPI Mode 0 (CPOL=0, CPHA=0)
#define S2LP_SPI_COMMAND_TIMEOUT_MS 100
#define S2LP_SPI_TRANSACTION_TIMEOUT_MS 50

/**
 * @brief Register Addresses - Key Registers Only
 */
#define S2LP_REG_GPIO0_CONF         0x00
#define S2LP_REG_GPIO1_CONF         0x01
#define S2LP_REG_GPIO2_CONF         0x02
#define S2LP_REG_GPIO3_CONF         0x03
#define S2LP_REG_SYNT3              0x05    // Frequency synthesis
#define S2LP_REG_SYNT2              0x06
#define S2LP_REG_SYNT1              0x07
#define S2LP_REG_SYNT0              0x08
#define S2LP_REG_IF_OFFSET_ANA      0x09
#define S2LP_REG_IF_OFFSET_DIG      0x0A
#define S2LP_REG_CHSPACE            0x0C    // Channel spacing
#define S2LP_REG_CHNUM              0x0D    // Channel number
#define S2LP_REG_MOD4               0x0E    // Modulation
#define S2LP_REG_MOD3               0x0F
#define S2LP_REG_MOD2               0x10
#define S2LP_REG_MOD1               0x11
#define S2LP_REG_MOD0               0x12
#define S2LP_REG_CHFLT              0x13    // Channel filter
#define S2LP_REG_AFC2               0x14    // AFC
#define S2LP_REG_AFC1               0x15
#define S2LP_REG_AFC0               0x16
#define S2LP_REG_RSSI_FLT           0x17    // RSSI filter
#define S2LP_REG_RSSI_TH            0x18    // RSSI threshold
#define S2LP_REG_AGCCTRL4           0x1A    // AGC control
#define S2LP_REG_AGCCTRL3           0x1B
#define S2LP_REG_AGCCTRL2           0x1C
#define S2LP_REG_AGCCTRL1           0x1D
#define S2LP_REG_AGCCTRL0           0x1E
#define S2LP_REG_ANT_SELECT_CONF    0x1F
#define S2LP_REG_CLOCKREC2          0x20    // Clock recovery
#define S2LP_REG_CLOCKREC1          0x21
#define S2LP_REG_CLOCKREC0          0x22
#define S2LP_REG_PCKTCTRL6          0x2B    // Packet control
#define S2LP_REG_PCKTCTRL5          0x2C
#define S2LP_REG_PCKTCTRL4          0x2D
#define S2LP_REG_PCKTCTRL3          0x2E
#define S2LP_REG_PCKTCTRL2          0x2F
#define S2LP_REG_PCKTCTRL1          0x30
#define S2LP_REG_PCKTLEN1           0x31    // Packet length
#define S2LP_REG_PCKTLEN0           0x32
#define S2LP_REG_SYNC3              0x33    // Sync word
#define S2LP_REG_SYNC2              0x34
#define S2LP_REG_SYNC1              0x35
#define S2LP_REG_SYNC0              0x36
#define S2LP_REG_QI                 0x37    // Quality indicator
#define S2LP_REG_PCKT_PSTMBL        0x38    // Preamble
#define S2LP_REG_PROTOCOL2          0x39    // Protocol
#define S2LP_REG_PROTOCOL1          0x3A
#define S2LP_REG_PROTOCOL0          0x3B
#define S2LP_REG_FIFO_CONFIG3       0x3C    // FIFO configuration
#define S2LP_REG_FIFO_CONFIG2       0x3D
#define S2LP_REG_FIFO_CONFIG1       0x3E
#define S2LP_REG_FIFO_CONFIG0       0x3F
#define S2LP_REG_PCKT_FLT_OPTIONS   0x40
#define S2LP_REG_PCKT_FLT_GOALS4    0x41
#define S2LP_REG_PCKT_FLT_GOALS3    0x42
#define S2LP_REG_PCKT_FLT_GOALS2    0x43
#define S2LP_REG_PCKT_FLT_GOALS1    0x44
#define S2LP_REG_PCKT_FLT_GOALS0    0x45
#define S2LP_REG_TIMERS5            0x46    // Timers
#define S2LP_REG_TIMERS4            0x47
#define S2LP_REG_TIMERS3            0x48
#define S2LP_REG_TIMERS2            0x49
#define S2LP_REG_TIMERS1            0x4A
#define S2LP_REG_TIMERS0            0x4B
#define S2LP_REG_CSMA_CONF3         0x4C    // CSMA
#define S2LP_REG_CSMA_CONF2         0x4D
#define S2LP_REG_CSMA_CONF1         0x4E
#define S2LP_REG_CSMA_CONF0         0x4F
#define S2LP_REG_IRQ_MASK3          0x50    // Interrupt mask
#define S2LP_REG_IRQ_MASK2          0x51
#define S2LP_REG_IRQ_MASK1          0x52
#define S2LP_REG_IRQ_MASK0          0x53
#define S2LP_REG_FAST_RX_TIMER      0x54
#define S2LP_REG_PA_POWER8          0x5A    // PA power
#define S2LP_REG_PA_POWER7          0x5B
#define S2LP_REG_PA_POWER6          0x5C
#define S2LP_REG_PA_POWER5          0x5D
#define S2LP_REG_PA_POWER4          0x5E
#define S2LP_REG_PA_POWER3          0x5F
#define S2LP_REG_PA_POWER2          0x60
#define S2LP_REG_PA_POWER1          0x61
#define S2LP_REG_PA_POWER0          0x62
#define S2LP_REG_PA_CONFIG1         0x63    // PA configuration
#define S2LP_REG_PA_CONFIG0         0x64
#define S2LP_REG_SYNTH_CONFIG2      0x65    // Synthesizer
#define S2LP_REG_SYNTH_CONFIG1      0x66
#define S2LP_REG_SYNTH_CONFIG0      0x67
#define S2LP_REG_VCO_CONFIG         0x68
#define S2LP_REG_VCO_CALIBR_IN2     0x69
#define S2LP_REG_VCO_CALIBR_IN1     0x6A
#define S2LP_REG_VCO_CALIBR_IN0     0x6B
#define S2LP_REG_XO_RCO_CONF1       0x6C    // Oscillator
#define S2LP_REG_XO_RCO_CONF0       0x6D
#define S2LP_REG_RCO_CALIBR_CONF3   0x6E
#define S2LP_REG_RCO_CALIBR_CONF2   0x6F
#define S2LP_REG_RCO_CALIBR_CONF1   0x70
#define S2LP_REG_RCO_CALIBR_CONF0   0x71
#define S2LP_REG_PM_CONF4           0x75    // Power management
#define S2LP_REG_PM_CONF3           0x76
#define S2LP_REG_PM_CONF2           0x77
#define S2LP_REG_PM_CONF1           0x78
#define S2LP_REG_PM_CONF0           0x79
#define S2LP_REG_MC_STATE1          0x8D    // Machine state
#define S2LP_REG_MC_STATE0          0x8E
#define S2LP_REG_TX_FIFO_STATUS     0x8F    // FIFO status
#define S2LP_REG_RX_FIFO_STATUS     0x90
#define S2LP_REG_RCO_CALIBR_OUT4    0x94    // Calibration output
#define S2LP_REG_RCO_CALIBR_OUT3    0x95
#define S2LP_REG_VCO_CALIBR_OUT1    0x99
#define S2LP_REG_VCO_CALIBR_OUT0    0x9A
#define S2LP_REG_TX_PCKT_INFO       0x9C    // Packet info
#define S2LP_REG_RX_PCKT_INFO       0x9D
#define S2LP_REG_AFC_CORR           0x9E
#define S2LP_REG_LINK_QUALIF2       0x9F
#define S2LP_REG_LINK_QUALIF1       0xA0
#define S2LP_REG_RSSI_LEVEL         0xA2    // RSSI level
#define S2LP_REG_RX_PCKT_LEN1       0xA4
#define S2LP_REG_RX_PCKT_LEN0       0xA5
#define S2LP_REG_CRC_FIELD3         0xA6    // CRC fields
#define S2LP_REG_CRC_FIELD2         0xA7
#define S2LP_REG_CRC_FIELD1         0xA8
#define S2LP_REG_CRC_FIELD0         0xA9
#define S2LP_REG_RX_ADDRE_FIELD1    0xAA
#define S2LP_REG_RX_ADDRE_FIELD0    0xAB
#define S2LP_REG_RSSI_LEVEL_RUN     0xEF    // Running RSSI
#define S2LP_REG_DEVICE_INFO1       0xF0    // Device info
#define S2LP_REG_DEVICE_INFO0       0xF1
#define S2LP_REG_IRQ_STATUS3        0xFA    // Interrupt status
#define S2LP_REG_IRQ_STATUS2        0xFB
#define S2LP_REG_IRQ_STATUS1        0xFC
#define S2LP_REG_IRQ_STATUS0        0xFD
#define S2LP_REG_FIFO               0xFF    // FIFO access

/**
 * @brief SPI Command Codes
 */
#define S2LP_CMD_TX                 0x60    // Enter TX state
#define S2LP_CMD_RX                 0x61    // Enter RX state
#define S2LP_CMD_READY              0x62    // Go to READY state
#define S2LP_CMD_STANDBY            0x63    // Go to STANDBY state
#define S2LP_CMD_SLEEP              0x64    // Go to SLEEP state
#define S2LP_CMD_LOCKRX             0x65    // Lock RX frequency
#define S2LP_CMD_LOCKTX             0x66    // Lock TX frequency
#define S2LP_CMD_SABORT             0x67    // Abort current operation
#define S2LP_CMD_LDC_RELOAD         0x68    // Reload LDC timer
#define S2LP_CMD_SRES               0x70    // Software reset
#define S2LP_CMD_FLUSHRXFIFO        0x71    // Flush RX FIFO
#define S2LP_CMD_FLUSHTXFIFO        0x72    // Flush TX FIFO
#define S2LP_CMD_SEQUENCE_UPDATE    0x73    // Update sequence counter

/**
 * @brief SPI Operation Types
 */
#define S2LP_SPI_WRITE              0x00    // Write operation
#define S2LP_SPI_READ               0x01    // Read operation
#define S2LP_SPI_COMMAND            0x80    // Command operation

/**
 * @brief Device States
 */
typedef enum {
    S2LP_STATE_SHUTDOWN         = 0xFF,     // No power (not accessible via SPI)
    S2LP_STATE_SLEEP_A          = 0x01,     // Sleep without FIFO retention
    S2LP_STATE_SLEEP_B          = 0x03,     // Sleep with FIFO retention
    S2LP_STATE_READY            = 0x00,     // Ready state
    S2LP_STATE_STANDBY          = 0x02,     // Standby state
    S2LP_STATE_LOCKST           = 0x14,     // Lock error state
    S2LP_STATE_LOCKON           = 0x0C,     // Lock state
    S2LP_STATE_RX               = 0x30,     // Receive mode
    S2LP_STATE_TX               = 0x5C,     // Transmit mode
    S2LP_STATE_SYNTH_SETUP      = 0x50,     // Synthesizer setup
    S2LP_STATE_WAIT_SLEEP       = 0x7C,     // Wait before sleep
    S2LP_STATE_UNKNOWN          = 0xFE      // Unknown/error state
} s2lp_state_t;

/**
 * @brief IRQ Types
 */
typedef enum {
    S2LP_IRQ_RX_DATA_READY      = (1UL << 0),
    S2LP_IRQ_RX_DATA_DISC       = (1UL << 1),
    S2LP_IRQ_TX_DATA_SENT       = (1UL << 2),
    S2LP_IRQ_MAX_RE_TX_REACH    = (1UL << 3),
    S2LP_IRQ_CRC_ERROR          = (1UL << 4),
    S2LP_IRQ_TX_FIFO_ERROR      = (1UL << 5),
    S2LP_IRQ_RX_FIFO_ERROR      = (1UL << 6),
    S2LP_IRQ_TX_FIFO_ALMOST_FULL = (1UL << 7),
    S2LP_IRQ_TX_FIFO_ALMOST_EMPTY = (1UL << 8),
    S2LP_IRQ_RX_FIFO_ALMOST_FULL = (1UL << 9),
    S2LP_IRQ_RX_FIFO_ALMOST_EMPTY = (1UL << 10),
    S2LP_IRQ_MAX_BO_CCA_REACH   = (1UL << 11),
    S2LP_IRQ_VALID_PREAMBLE     = (1UL << 12),
    S2LP_IRQ_VALID_SYNC         = (1UL << 13),
    S2LP_IRQ_RSSI_ABOVE_TH      = (1UL << 14),
    S2LP_IRQ_WKUP_TOUT_LDC      = (1UL << 15),
    S2LP_IRQ_READY              = (1UL << 16),
    S2LP_IRQ_STANDBY_DELAYED    = (1UL << 17),
    S2LP_IRQ_LOW_BATT_LVL       = (1UL << 18),
    S2LP_IRQ_POR                = (1UL << 19),
    S2LP_IRQ_BOR                = (1UL << 20),
    S2LP_IRQ_LOCK               = (1UL << 21),
    S2LP_IRQ_VCO_CALIBRATION_END = (1UL << 22),
    S2LP_IRQ_PA_CALIBRATION_END = (1UL << 23),
    S2LP_IRQ_RX_TIMEOUT         = (1UL << 28),
    S2LP_IRQ_RX_SNIFF_TIMEOUT   = (1UL << 29),
    S2LP_IRQ_ALL                = 0xFFFFFFFFUL
} s2lp_irq_t;

/**
 * @brief GPIO Functions
 */
typedef enum {
    S2LP_GPIO_IRQ               = 0,        // Interrupt request (active low)
    S2LP_GPIO_POR_INV           = 1,        // POR inverted
    S2LP_GPIO_WUT_EXP           = 2,        // Wake-up timer expiration
    S2LP_GPIO_LOW_BATT_DET      = 3,        // Low battery detection
    S2LP_GPIO_TX_DATA_CLK       = 4,        // TX data clock
    S2LP_GPIO_TX_STATE          = 5,        // TX state indication
    S2LP_GPIO_FIFO_ALMOST_EMPTY = 6,        // FIFO almost empty
    S2LP_GPIO_FIFO_ALMOST_FULL  = 7,        // FIFO almost full
    S2LP_GPIO_RX_DATA           = 8,        // RX data output
    S2LP_GPIO_RX_CLK            = 9,        // RX clock output
    S2LP_GPIO_RX_STATE          = 10,       // RX state indication
    S2LP_GPIO_NOT_SLEEP_STDBY   = 11,       // Not in sleep/standby
    S2LP_GPIO_STANDBY_STATE     = 12,       // Standby state
    S2LP_GPIO_ANT_SWITCH        = 13,       // Antenna switch
    S2LP_GPIO_VALID_PREAMBLE    = 14,       // Valid preamble detected
    S2LP_GPIO_SYNC_DETECTED     = 15,       // Sync word detected
    S2LP_GPIO_RSSI_ABOVE_TH     = 16,       // RSSI above threshold
    S2LP_GPIO_TX_RX_MODE        = 18,       // TX/RX mode indicator
    S2LP_GPIO_VDD               = 19,       // VDD output
    S2LP_GPIO_GND               = 20,       // GND output
    S2LP_GPIO_SMPS_EN           = 21,       // External SMPS enable
    S2LP_GPIO_SLEEP_STATE       = 22,       // Sleep state
    S2LP_GPIO_READY_STATE       = 23,       // Ready state
    S2LP_GPIO_LOCK_STATE        = 24        // Lock state
} s2lp_gpio_func_t;

/**
 * @brief Modulation Types
 */
typedef enum {
    S2LP_MOD_2FSK               = 0,        // 2-FSK
    S2LP_MOD_4FSK               = 1,        // 4-FSK
    S2LP_MOD_2GFSK_BT05         = 2,        // 2-GFSK BT=0.5
    S2LP_MOD_2GFSK_BT1          = 3,        // 2-GFSK BT=1
    S2LP_MOD_4GFSK_BT05         = 4,        // 4-GFSK BT=0.5
    S2LP_MOD_4GFSK_BT1          = 5,        // 4-GFSK BT=1
    S2LP_MOD_ASK_OOK            = 6,        // ASK/OOK
    S2LP_MOD_MSK                = 7         // MSK
} s2lp_modulation_t;

/**
 * @brief Hardware Configuration Structure
 */
typedef struct {
    int gpio_mosi;                          // SPI MOSI pin
    int gpio_miso;                          // SPI MISO pin  
    int gpio_sclk;                          // SPI SCLK pin
    int gpio_cs;                            // SPI CS pin
    int gpio_irq;                           // IRQ pin
    spi_host_device_t spi_host;            // SPI host (SPI2_HOST or SPI3_HOST)
} s2lp_hw_config_t;

/**
 * @brief RF Configuration Structure
 */
typedef struct {
    uint32_t frequency_hz;                  // Carrier frequency in Hz
    s2lp_modulation_t modulation;          // Modulation type
    uint32_t datarate_bps;                 // Data rate in bps
    uint32_t freq_deviation_hz;            // Frequency deviation in Hz
    uint32_t channel_filter_bw_hz;         // Channel filter bandwidth in Hz
    int8_t output_power_dbm;               // Output power in dBm
    bool enable_rssi_threshold;            // Enable RSSI threshold
    int16_t rssi_threshold_dbm;            // RSSI threshold in dBm
} s2lp_rf_config_t;

/**
 * @brief Packet Configuration Structure
 */
typedef struct {
    bool enable_crc;                        // Enable CRC
    bool enable_fec;                        // Enable FEC
    bool variable_length;                   // Variable length packets
    uint16_t preamble_length;              // Preamble length in bits
    uint32_t sync_word;                    // Sync word (32-bit)
    uint8_t sync_length;                   // Sync word length in bits (8-32)
    uint16_t max_packet_length;            // Maximum packet length in bytes
} s2lp_packet_config_t;

/**
 * @brief IRQ Event Structure
 */
typedef struct {
    uint32_t irq_flags;                    // IRQ flags bitmask
    TickType_t timestamp;                  // Event timestamp
    s2lp_state_t state;                    // Device state when IRQ occurred
} s2lp_irq_event_t;

/**
 * @brief RSSI Measurement Structure
 */
typedef struct {
    int16_t rssi_dbm;                      // RSSI value in dBm
    bool valid;                            // Measurement validity
    TickType_t timestamp;                  // Measurement timestamp
} s2lp_rssi_t;

/**
 * @brief Driver Handle
 */
typedef struct s2lp_handle_t* s2lp_handle_t;

/**
 * @brief IRQ Callback Function Type
 * 
 * @param handle S2LP driver handle
 * @param irq_event IRQ event information
 * @param user_data User data passed during callback registration
 */
typedef void (*s2lp_irq_callback_t)(s2lp_handle_t handle, 
                                    const s2lp_irq_event_t* irq_event, 
                                    void* user_data);

/**
 * @brief Default Hardware Configuration for ESP32-S3
 */
#define S2LP_DEFAULT_HW_CONFIG() { \
    .gpio_mosi = 48, \
    .gpio_miso = 47, \
    .gpio_sclk = 45, \
    .gpio_cs = 40, \
    .gpio_irq = 20, \
    .spi_host = SPI3_HOST \
}

/**
 * @brief Default RF Configuration (868 MHz, 2-GFSK, 10 kbps)
 */
#define S2LP_DEFAULT_RF_CONFIG() { \
    .frequency_hz = 868000000, \
    .modulation = S2LP_MOD_2GFSK_BT05, \
    .datarate_bps = 10000, \
    .freq_deviation_hz = 20000, \
    .channel_filter_bw_hz = 100000, \
    .output_power_dbm = 10, \
    .enable_rssi_threshold = true, \
    .rssi_threshold_dbm = -100 \
}

/**
 * @brief Default Packet Configuration
 */
#define S2LP_DEFAULT_PACKET_CONFIG() { \
    .enable_crc = true, \
    .enable_fec = false, \
    .variable_length = true, \
    .preamble_length = 32, \
    .sync_word = 0x88888888, \
    .sync_length = 32, \
    .max_packet_length = 255 \
}

/**
 * @brief Initialize S2LP driver
 * 
 * @param hw_config Hardware configuration
 * @param rf_config RF configuration  
 * @param packet_config Packet configuration
 * @param out_handle Output driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_init(const s2lp_hw_config_t* hw_config,
                    const s2lp_rf_config_t* rf_config,
                    const s2lp_packet_config_t* packet_config,
                    s2lp_handle_t* out_handle);

/**
 * @brief Deinitialize S2LP driver
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_deinit(s2lp_handle_t handle);

/**
 * @brief Software reset of S2LP device
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_reset(s2lp_handle_t handle);

/**
 * @brief Get current device state
 * 
 * @param handle Driver handle
 * @param out_state Output state
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_get_state(s2lp_handle_t handle, s2lp_state_t* out_state);

/**
 * @brief Send command to S2LP
 * 
 * @param handle Driver handle
 * @param command Command code
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_send_command(s2lp_handle_t handle, uint8_t command);

/**
 * @brief Read register from S2LP
 * 
 * @param handle Driver handle
 * @param reg_addr Register address
 * @param out_value Output register value
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_read_register(s2lp_handle_t handle, uint8_t reg_addr, uint8_t* out_value);

/**
 * @brief Write register to S2LP
 * 
 * @param handle Driver handle
 * @param reg_addr Register address
 * @param value Register value
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_write_register(s2lp_handle_t handle, uint8_t reg_addr, uint8_t value);

/**
 * @brief Read multiple registers from S2LP
 * 
 * @param handle Driver handle
 * @param start_addr Starting register address
 * @param buffer Output buffer
 * @param length Number of registers to read
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_read_registers(s2lp_handle_t handle, uint8_t start_addr, uint8_t* buffer, size_t length);

/**
 * @brief Write multiple registers to S2LP
 * 
 * @param handle Driver handle
 * @param start_addr Starting register address
 * @param buffer Input buffer
 * @param length Number of registers to write
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_write_registers(s2lp_handle_t handle, uint8_t start_addr, const uint8_t* buffer, size_t length);

/**
 * @brief Set RF frequency
 * 
 * @param handle Driver handle
 * @param frequency_hz Frequency in Hz (860-879 MHz)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_set_frequency(s2lp_handle_t handle, uint32_t frequency_hz);

/**
 * @brief Get current RF frequency
 * 
 * @param handle Driver handle
 * @param out_frequency_hz Output frequency in Hz
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_get_frequency(s2lp_handle_t handle, uint32_t* out_frequency_hz);

/**
 * @brief Set channel (0-19 for 860-879 MHz with 1 MHz steps)
 * 
 * @param handle Driver handle
 * @param channel Channel number (0-19)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_set_channel(s2lp_handle_t handle, uint8_t channel);

/**
 * @brief Get current channel
 * 
 * @param handle Driver handle
 * @param out_channel Output channel number
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_get_channel(s2lp_handle_t handle, uint8_t* out_channel);

/**
 * @brief Enter RX mode
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_enter_rx(s2lp_handle_t handle);

/**
 * @brief Enter TX mode
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_enter_tx(s2lp_handle_t handle);

/**
 * @brief Enter READY state
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_enter_ready(s2lp_handle_t handle);

/**
 * @brief Enter STANDBY state (lowest power)
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_enter_standby(s2lp_handle_t handle);

/**
 * @brief Abort current operation and return to READY
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_abort(s2lp_handle_t handle);

/**
 * @brief Read current RSSI value
 * 
 * @param handle Driver handle
 * @param out_rssi Output RSSI measurement
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_read_rssi(s2lp_handle_t handle, s2lp_rssi_t* out_rssi);

/**
 * @brief Read RSSI value continuously (for scanning)
 * 
 * @param handle Driver handle
 * @param out_rssi_dbm Output RSSI in dBm
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_read_rssi_run(s2lp_handle_t handle, int16_t* out_rssi_dbm);

/**
 * @brief Check if carrier is detected (RSSI above threshold)
 * 
 * @param handle Driver handle
 * @param out_detected Output carrier detection status
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_check_carrier_sense(s2lp_handle_t handle, bool* out_detected);

/**
 * @brief Configure IRQ mask
 * 
 * @param handle Driver handle
 * @param irq_mask IRQ mask (OR of s2lp_irq_t values)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_set_irq_mask(s2lp_handle_t handle, uint32_t irq_mask);

/**
 * @brief Get current IRQ mask
 * 
 * @param handle Driver handle
 * @param out_irq_mask Output IRQ mask
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_get_irq_mask(s2lp_handle_t handle, uint32_t* out_irq_mask);

/**
 * @brief Read and clear IRQ status
 * 
 * @param handle Driver handle
 * @param out_irq_status Output IRQ status flags
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_read_irq_status(s2lp_handle_t handle, uint32_t* out_irq_status);

/**
 * @brief Register IRQ callback
 * 
 * @param handle Driver handle
 * @param callback Callback function
 * @param user_data User data for callback
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_register_irq_callback(s2lp_handle_t handle, 
                                     s2lp_irq_callback_t callback, 
                                     void* user_data);

/**
 * @brief Unregister IRQ callback
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_unregister_irq_callback(s2lp_handle_t handle);

/**
 * @brief Read data from RX FIFO
 * 
 * @param handle Driver handle
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @param out_bytes_read Number of bytes actually read
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_read_fifo(s2lp_handle_t handle, uint8_t* buffer, size_t buffer_size, size_t* out_bytes_read);

/**
 * @brief Write data to TX FIFO
 * 
 * @param handle Driver handle
 * @param buffer Input buffer
 * @param length Number of bytes to write
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_write_fifo(s2lp_handle_t handle, const uint8_t* buffer, size_t length);

/**
 * @brief Get RX FIFO status
 * 
 * @param handle Driver handle
 * @param out_bytes_available Number of bytes available in RX FIFO
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_get_rx_fifo_status(s2lp_handle_t handle, uint8_t* out_bytes_available);

/**
 * @brief Get TX FIFO status
 * 
 * @param handle Driver handle
 * @param out_bytes_available Number of bytes available in TX FIFO
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_get_tx_fifo_status(s2lp_handle_t handle, uint8_t* out_bytes_available);

/**
 * @brief Flush RX FIFO
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_flush_rx_fifo(s2lp_handle_t handle);

/**
 * @brief Flush TX FIFO
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_flush_tx_fifo(s2lp_handle_t handle);

/**
 * @brief Get device part number and version
 * 
 * @param handle Driver handle
 * @param out_part_number Output part number
 * @param out_version Output version
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_get_device_info(s2lp_handle_t handle, uint8_t* out_part_number, uint8_t* out_version);

/**
 * @brief Start RCO calibration
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_start_rco_calibration(s2lp_handle_t handle);

/**
 * @brief Check if RCO calibration is complete
 * 
 * @param handle Driver handle
 * @param out_complete Output calibration status
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_is_rco_calibration_complete(s2lp_handle_t handle, bool* out_complete);

/**
 * @brief Configure GPIO function
 * 
 * @param handle Driver handle
 * @param gpio_num GPIO number (0-3)
 * @param function GPIO function
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_configure_gpio(s2lp_handle_t handle, uint8_t gpio_num, s2lp_gpio_func_t function);

/**
 * @brief Wait for device state change with timeout
 * 
 * @param handle Driver handle
 * @param expected_state Expected state
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on timeout, error code otherwise
 */
esp_err_t s2lp_wait_for_state(s2lp_handle_t handle, s2lp_state_t expected_state, uint32_t timeout_ms);

/**
 * @brief Check if device is responsive (communication test)
 * 
 * @param handle Driver handle
 * @return esp_err_t ESP_OK if responsive, error code otherwise
 */
esp_err_t s2lp_test_communication(s2lp_handle_t handle);

/**
 * @brief Convert RSSI register value to dBm
 * 
 * @param rssi_reg RSSI register value
 * @return int16_t RSSI value in dBm
 */
int16_t s2lp_rssi_reg_to_dbm(uint8_t rssi_reg);

/**
 * @brief Convert dBm to RSSI register value
 * 
 * @param rssi_dbm RSSI value in dBm
 * @return uint8_t RSSI register value
 */
uint8_t s2lp_rssi_dbm_to_reg(int16_t rssi_dbm);

/**
 * @brief Convert state code to string
 * 
 * @param state State code
 * @return const char* State name string
 */
const char* s2lp_state_to_string(s2lp_state_t state);

/**
 * @brief Convert IRQ flags to string (for debugging)
 * 
 * @param irq_flags IRQ flags
 * @param buffer Output buffer
 * @param buffer_size Buffer size
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t s2lp_irq_flags_to_string(uint32_t irq_flags, char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif