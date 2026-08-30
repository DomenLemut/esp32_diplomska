#include "LSM6DS3_driver.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "LSM6DS3_conf.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

static const char *TAG = "LSM6DS3_DRIVER"; 

i2c_master_dev_handle_t dev_handle;
i2c_master_bus_handle_t bus_handle;

i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x6A,
    .scl_speed_hz = I2C_MASTER_CLK_SPEED,
};

esp_err_t i2c_init(void) {
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    return ESP_OK;
}

esp_err_t i2c_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle, buf, sizeof(buf), 1000);
}

esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data) {
    return i2c_master_transmit_receive(dev_handle, &reg, 1, data, 1, 1000);
}

esp_err_t lsm6ds3_init(void) {
    uint8_t whoami = 0;
    ESP_ERROR_CHECK(i2c_read_reg(WHO_AM_I, &whoami));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", whoami);

    if (whoami != 0x69) return ESP_FAIL;
    return i2c_write_reg(CTRL3_C, 0x01); // Software Reset
}

/**
 * @brief Continuous FIFO Setup with Wakeup routed to INT2
 */
esp_err_t lsm6ds3_setup_continuous_with_wakeup(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(CTRL1_XL, (SAMPLE_DIV << 4) | 0)); // enable accelerometer ODR
    ESP_ERROR_CHECK(i2c_write_reg(CTRL2_G, (SAMPLE_DIV << 4) | 0)); // enable gyroscope ODR
    // ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, ((SAMPLE_DIV << 3) | 6))); // enable FIFO ODR and set FIFO mode to continuous

    return ESP_OK;
}

/**
 * @brief Starts the sensor with setting speed registers of sensors and FIFO
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_stop(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(CTRL1_XL, 0x00)); // Disable accelerometer ODR
    ESP_ERROR_CHECK(i2c_write_reg(CTRL2_G, 0x00)); // Disable gyroscope ODR
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, 0x00)); // Disable FIFO ODR

    return ESP_OK;
}

/**
 * @brief Sets up the wake-up motion interrupt
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_configure_motion_interrupt(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(CTRL9_XL, 0x38));    // Turn on X, Y, Z wakeup
    ESP_ERROR_CHECK(i2c_write_reg(WAKE_UP_THS, 0x01)); // ~200mg threshold (sensitive enough to catch the whole window)
    // 1. Enable Auto-Increment & Block Data Update
    ESP_ERROR_CHECK(i2c_write_reg(CTRL3_C, 0x44)); 

    // 2. Set Accel & Gyro ODR (104 Hz)
    ESP_ERROR_CHECK(i2c_write_reg(CTRL1_XL, (SAMPLE_DIV << 4))); 
    ESP_ERROR_CHECK(i2c_write_reg(CTRL2_G,  (SAMPLE_DIV << 4))); 

    // 3. Set FIFO Threshold (e.g. 312 samples * 6 words = 1872 words = 0x0750)
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL1, 0x50)); // LSB
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL2, 0x07)); // MSB

    // 4. FIFO Configuration: Accel + Gyro in FIFO (No decimation)
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL3, 0x09)); 
    
    // 5. Set FIFO mode to Continuous Mode (0x06)
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, ((SAMPLE_DIV << 3) | 0x06))); 

    // 6. Wakeup Interrupt Engine Setup
    ESP_ERROR_CHECK(i2c_write_reg(CTRL9_XL, 0x38));    
    ESP_ERROR_CHECK(i2c_write_reg(WAKE_UP_THS, 0x04)); 
    ESP_ERROR_CHECK(i2c_write_reg(WAKE_UP_DUR, 0x00)); 

    ESP_ERROR_CHECK(i2c_write_reg(INT1_CTRL, 0x08));
    ESP_ERROR_CHECK(i2c_write_reg(TAP_CFG, 0x9F));     
    ESP_ERROR_CHECK(i2c_write_reg(MD2_CFG, 0x20));     

    return ESP_OK;
}

esp_err_t lsm6ds3_read_fifo(uint8_t *data, size_t len) {
    uint8_t addr = FIFO_DATA_OUT_L;
    return i2c_master_transmit_receive(dev_handle, &addr, 1, data, len, 1000);
}

esp_err_t lsm6ds3_read_wakeup_source(uint8_t *status) {
    return i2c_read_reg(WAKE_UP_SRC, status);
}

bool lsm6ds_check_int2(void) {
    return gpio_get_level(LSM6DS3_INT2) == 1;
}

uint16_t lsm6ds3_get_fifo_word_count(void) {
    uint8_t status[2] = {0, 0};

    i2c_read_reg(FIFO_STATUS1, &status[0]);
    i2c_read_reg(FIFO_STATUS2, &status[1]);

    uint16_t word_count = ((uint16_t)(status[1] & 0x07) << 8) | status[0];

    // Check OVER_RUN flag (Bit 6 of FIFO_STATUS2 -> 0x40)
    if (status[1] & 0x40) {
        word_count = 2048; // Cap at max physical hardware capacity
    }

    return word_count;
}
/**
 * @brief Flushes the FIFO and restarts Continuous Mode
 */
esp_err_t lsm6ds3_reset_fifo(void) {
    // 1. Set FIFO to Bypass mode (clears FIFO content and status flags)
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, 0x00));
    
    // 2. Set FIFO back to Continuous Mode
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, ((SAMPLE_DIV << 3) | 0x06)));
    return ESP_OK;
}

esp_err_t interrupt_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LSM6DS3_INT2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(gpio_wakeup_enable(LSM6DS3_INT2, GPIO_INTR_HIGH_LEVEL));

    return ESP_OK;
}