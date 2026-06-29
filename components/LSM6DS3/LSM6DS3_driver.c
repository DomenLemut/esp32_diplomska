#include "LSM6DS3_driver.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "LSM6DS3_conf.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

uint8_t batch_buffer[FIFO_THR_BYTES];

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
    .scl_speed_hz = 100000,
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
    return ESP_OK;
}

/**
 * @brief Starts the sensor with setting speed registers of sensors and FIFO
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_start(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(CTRL1_XL, (SAMPLE_DIV << 4) | 0));
    ESP_ERROR_CHECK(i2c_write_reg(CTRL2_G, (SAMPLE_DIV << 4) | 0));
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, ((SAMPLE_DIV << 3) | 6)));

    return ESP_OK;
}

/**
 * @brief Sets up the wake-up motion interrupt
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_configure_motion_interrupt(void)
{
    // 1. Ensure all 3 Accelerometer Axes (X, Y, Z) are enabled
    i2c_write_reg(CTRL9_XL, 0x38);

    // 2. Turn on Accelerometer (104 Hz, +/- 2g scale)
    i2c_write_reg(CTRL1_XL, 0x40);   
    i2c_write_reg(CTRL2_G, 0x00);    

    // 3. MASTER INTERRUPT ROUTING ENABLE
    // Set Bit 5 (INT2_on_INT1) = 1 to enable general interrupt pathways to INT1
    i2c_write_reg(CTRL4_C, 0x20);   

    // 4. Enable Axis Slopes + Enable Latched Mode (LIR=1)
    i2c_write_reg(TAP_CFG, 0x79);    

    // 5. Threshold Configuration (Extremely sensitive for testing)
    i2c_write_reg(WAKE_UP_THS, 0x00); // Lowered to 0x01 (approx 31.25 mg)

    // 6. Duration Configuration (Instant trigger)
    i2c_write_reg(WAKE_UP_DUR, 0x00);

    // 7. Route Wake-Up Event to physical INT1 pin via INT1_CTRL
    i2c_write_reg(INT1_CTRL, 0x20); 

    return ESP_OK;
}

/**
 * @brief Sets up all the configuration registers on the LSM6DS3
 * @return esp_err_t Return error values
 */
esp_err_t lsm6ds3_setup(void)
{
    // FIFO threshold
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL1, FIFO_THR));

    // FIFO decimation config
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL2, 0x80));

    // Enable accel + gyro in FIFO
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL3, 0x09));

    // Timestamp (optional but you had it)
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL4, 0x08));

    // FIFO ODR / batching mode
    ESP_ERROR_CHECK(i2c_write_reg(FIFO_CTRL5, ((SAMPLE_DIV << 3) | 0x06)));

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
    ESP_ERROR_CHECK(gpio_wakeup_enable(LSM6DS3_INT1, GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(LSM6DS3_INT2, GPIO_INTR_HIGH_LEVEL));

    return ESP_OK;
}
