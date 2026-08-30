#include "LSM6DS3_driver.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "LSM6DS3_conf.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

uint8_t batch_buffer[FIFO_THR_BYTES];

const uint16_t fifo_thr = FIFO_THR_BYTES / 2;

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

/**
 * @brief Enables I2C pins and communication for LSM6DS3 sensor
 * @return esp_err_t Return error values
 */
esp_err_t i2c_init(void) {
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    
    return ESP_OK;
}

/**
 * @brief Writes a single register (1 byte) to the LSM6DS3 sensor
 * @return esp_err_t Return error values
 */
esp_err_t i2c_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(
        dev_handle,
        buf, 
        sizeof(buf),
        1000);
}

/**
 * @brief Reads a single register (1 byte) from the LSM6DS3 sensor
 * @return esp_err_t Return error values
 */
esp_err_t i2c_read_reg(uint8_t reg, uint8_t *data)
{
    return i2c_master_transmit_receive(
        dev_handle,
        &reg,
        1,
        data,
        1,
        1000);
}

/**
 * @brief Checks that the LSM6DS3 sensor has been initialised with reading its ID
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_init(void)
{
    uint8_t whoami = 0;

    ESP_ERROR_CHECK(i2c_read_reg(WHO_AM_I, &whoami));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", whoami);

    if (whoami != 0x69) return ESP_FAIL;
    return i2c_write_reg(CTRL3_C, 0x01);
}

/**
 * @brief Starts the sensor with setting speed registers of sensors and FIFO
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_start(void)
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
    ESP_ERROR_CHECK(i2c_write_reg(WAKE_UP_DUR, 0x00)); 
    
    // CRITICAL: Set Bit 0 (LIR) = 1 to LATCH the interrupt flag.
    // This forces the sensor to keep the FIFO recording active even after motion stops!
    ESP_ERROR_CHECK(i2c_write_reg(TAP_CFG, 0x01)); 
    
    // Route the Wakeup Engine directly to the INT2 line to drive the FIFO trigger
    ESP_ERROR_CHECK(i2c_write_reg(MD2_CFG, 0x20)); 

    return ESP_OK;
}

/**
 * @brief RESETS up the wake-up motion interrupt - silences the INT2 alarm
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_deconfigure_motion_interrupt(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(CTRL9_XL, 0x00));
    ESP_ERROR_CHECK(i2c_write_reg(WAKE_UP_THS, 0x00));
    ESP_ERROR_CHECK(i2c_write_reg(WAKE_UP_DUR, 0x00));
    ESP_ERROR_CHECK(i2c_write_reg(MD2_CFG, 0x00));
    ESP_ERROR_CHECK(i2c_write_reg(TAP_CFG, 0x00));

    return ESP_OK;
}

/**
 * @brief Sets up all the configuration registers on the LSM6DS3
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_setup(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(CTRL3_C, 0x44)); // Enable Block Data Update (BDU) and auto-increment for multi-byte reads

    //ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL1, FIFO_THR)); // FIFO threshold
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL2, 0x80)); // FIFO decimation config
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL3, 0x09)); // Enable accel + gyro in FIFO
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL4, 0x08)); // Timestamp (optional but you had it)
    //ESP_ERROR_CHECK(i2c_write_reg(INT1_CTRL, 0x08)); // Enable theshold interrupt for INT1

    return ESP_OK;
}

esp_err_t lsm6ds3_setup_event_window(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(CTRL3_C, 0x04)); // 1. Enable Block Data Update and Auto-Increment

    ESP_ERROR_CHECK(i2c_write_reg(CTRL10_C, 0x04));

    uint8_t fifo_thr_low =  (uint8_t) (fifo_thr & 0x00FF);         // Lower 8 bits
    uint8_t fifo_thr_high = (uint8_t) ((fifo_thr >> 8) & 0x07);

    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL1, fifo_thr_low)); 
    
    uint8_t ctrl2_val = 0;
    ESP_ERROR_CHECK(i2c_read_reg(FIFO_CTRL2, &ctrl2_val));
    
    ctrl2_val = (ctrl2_val & 0xF8) | fifo_thr_high; 
    
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL2, ctrl2_val));

    // ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL1, FIFO_THR)); // 2. Set the FIFO Watermark Threshold

    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL3, 0x09)); // 3. Enable both Accel and Gyro to store data 1-to-1

    // ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL4, 0x40)); // 4. Route the INT2 signal line directly to the FIFO trigger engine

    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, ((SAMPLE_DIV << 3) | 0x04))); // 5. Put FIFO into Bypass-to-Stream Mode (0x04)

    ESP_ERROR_CHECK(i2c_write_reg(INT2_CTRL, 0x08)); // 6. Route the FIFO Watermark (Full) flag to the physical INT2 pin

    return ESP_OK;
}

/**
 * @brief RESETS up all the configuration registers on the LSM6DS3
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_desetup(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, 0x00));

    return ESP_OK;
}

/**
 * @brief Reads the content of LSM6DS3 fifo
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_read_fifo(uint8_t *data, size_t len)
{
    uint8_t addr = FIFO_DATA_OUT_L;
    
    return i2c_master_transmit_receive(
        dev_handle,
        &addr,
        1,
        data,
        len,
        100
    );
}

bool lsm6ds_check_int1(void) {
    return gpio_get_level(LSM6DS3_INT1) == 1;
}

bool lsm6ds_check_int2(void) {
    return gpio_get_level(LSM6DS3_INT2) == 1;
}

esp_err_t lsm6ds3_read_wakeup_source(uint8_t *status)
{
    uint8_t reg_val = 0;
    ESP_ERROR_CHECK(i2c_read_reg(WAKE_UP_SRC, &reg_val));

    *status = reg_val;

    return ESP_OK;
}

/**
 * @brief Enables external pin interrupts for LSM6DS3 sensor
 * @return esp_err_t Return error values
 */
esp_err_t interrupt_init(void)
{
    gpio_config_t io_conf0 = {
        .pin_bit_mask = (1ULL << LSM6DS3_INT1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    gpio_config_t io_conf1 = {
        .pin_bit_mask = (1ULL << LSM6DS3_INT2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf0));
    ESP_ERROR_CHECK(gpio_config(&io_conf1));

    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    //ESP_ERROR_CHECK(gpio_wakeup_enable(LSM6DS3_INT1, GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(LSM6DS3_INT2, GPIO_INTR_HIGH_LEVEL));

    return ESP_OK;
}

uint16_t lsm6ds3_get_fifo_word_count(void)
{
    uint8_t status1 = 0;
    uint8_t status2 = 0;

    // Read the lower 8 bits of the counter
    if (i2c_read_reg(FIFO_STATUS1, &status1) != ESP_OK) {
        return 0;
    }
    // Read the higher 4 bits of the counter (stored in bits 0-3 of FIFO_STATUS2)
    if (i2c_read_reg(FIFO_STATUS2, &status2) != ESP_OK) {
        return 0;
    }

    // Combine them: status1 holds bits [7:0], status2 bits [3:0] hold bits [11:8]
    uint16_t total_words = status1 | ((status2 & 0x0F) << 8);
    
    return total_words;
}