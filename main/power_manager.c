/*
 * power management via axp192 driver
 */

#include "sdkconfig.h"

#ifdef CONFIG_LASTLINK_ENABLE_POWER_MANAGEMENT
#include <stdarg.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <ctype.h>

#include "esp_vfs.h"
#include "esp_vfs_dev.h"
//#include "esp_attr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "os_specific.h"

#include "axp192.h"
#include "driver/i2c.h"

#include "power_manager.h"

#define TAG "power_management"

static os_mutex_t axp192_lock;

static int32_t axp_read(void* handle, uint8_t address, uint8_t command, uint8_t* buffer, uint16_t len)
{
    uint32_t i2c_num = (uint32_t) handle;

//ESP_LOGI(TAG, "%s: i2c_num %u address %02x command %02x bytes %d...", __func__, i2c_num, address, command, len);

    os_acquire_recursive_mutex(axp192_lock);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    /* Select register cycle */
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, command, true));

    /* Read cycle */
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read(cmd, buffer, len, I2C_MASTER_LAST_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    //ESP_LOGI(TAG, "%s: i2c_master_cmd_begin(%d, %p, %d)", __func__, i2c_num, cmd, 1000/portTICK_PERIOD_MS);
    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_PERIOD_MS);
    if (err != ESP_OK)  {
        ESP_LOGE(TAG, "%s: i2c_master_cmd_begin error %d", __func__, err);
    }

    i2c_cmd_link_delete(cmd);

    os_release_recursive_mutex(axp192_lock);

//ESP_LOGI(TAG, "%s: i2c_num %u address %02x command %02x bytes %d %02x %02x %02x...", __func__, (uint32_t) i2c_num, address, command, len, buffer[0], buffer[1], buffer[2]);

    return AXP192_OK;
}

static int32_t axp_write(void* handle, uint8_t address, uint8_t command, const uint8_t* buffer, uint16_t len)
{
    uint32_t i2c_num = (uint32_t) handle;

//ESP_LOGI(TAG, "%s: i2c_num %u address %02x command %02x bytes %d %02x %02x %02x...", __func__, i2c_num, address, command, len, buffer[0], buffer[1], buffer[2]);

    os_acquire_recursive_mutex(axp192_lock);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, command, true));
    ESP_ERROR_CHECK(i2c_master_write(cmd, buffer, len, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    ESP_ERROR_CHECK(i2c_master_cmd_begin(i2c_num, cmd, 1000/portTICK_PERIOD_MS));

    i2c_cmd_link_delete(cmd);

    os_release_recursive_mutex(axp192_lock);

    return AXP192_OK;
}

static axp192_t* create_axp192(int i2c_num, int sda_pin, int scl_pin, int speed)
{
    i2c_config_t axp192_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = speed,
        //.clk_flags = <sleep mode flag>,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &axp192_config));

    // ESP_ERROR_CHECK(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0));
    i2c_driver_install(i2c_num, axp192_config.mode, 0, 0, 0);

    ESP_ERROR_CHECK(i2c_set_timeout(i2c_num, speed));

    /* Create axp device */
    axp192_t* axp = (axp192_t*) malloc(sizeof(axp192_t));
    if (axp != NULL) {
        axp->read = axp_read;
        axp->write = axp_write;
        axp->handle = (void*) i2c_num;
    }

    return axp;
}

axp192_t *axp;

bool stop_axp192(void)
{
    if (axp != NULL) {
        free((void*) axp);
        axp = NULL;
        return true;
    } else {
        return false;
    }
}


bool start_axp192(void)
{
    axp192_lock = os_create_recursive_mutex();

    axp = create_axp192(
                  CONFIG_LASTLINK_AXP192_I2C_NUM,
                  CONFIG_LASTLINK_AXP192_SDA_PIN,
                  CONFIG_LASTLINK_AXP192_SCL_PIN,
                  CONFIG_LASTLINK_AXP192_SPEED);

    if (axp != NULL) {
        bool ok;

        float vacin;
        ok = axp192_read(axp, AXP192_ACIN_VOLTAGE, &vacin) == AXP192_OK;
        ESP_LOGI(TAG, "%s: vacin      %.2fV%s", __func__, vacin, ok ? "" : " Error");

        float iacin;
        ok = axp192_read(axp, AXP192_ACIN_CURRENT, &iacin) == AXP192_OK;
        ESP_LOGI(TAG, "%s: iacin      %.2fA%s", __func__, iacin, ok ? "" : " Error");

        float vvbus;
        ok = axp192_read(axp, AXP192_VBUS_VOLTAGE, &vvbus) == AXP192_OK;
        ESP_LOGI(TAG, "%s: vvbus      %.2fV%s", __func__, vvbus, ok ? "" : " Error");

        float ivbus;
        ok = axp192_read(axp, AXP192_VBUS_CURRENT, &ivbus) == AXP192_OK;
        ESP_LOGI(TAG, "%s: ivbus      %.2fV%s", __func__, ivbus, ok ? "" : " Error");

        float temp;
        ok = axp192_read(axp, AXP192_TEMP, &temp) == AXP192_OK;
        ESP_LOGI(TAG, "%s: temp       %.0fC%s", __func__, temp, ok ? "" : " Error");

        float vts;
        ok = axp192_read(axp, AXP192_TS_INPUT, &vts) == AXP192_OK;
        ESP_LOGI(TAG, "%s: vts        %.2fV%s", __func__, vts, ok ? "" : " Error");

        float pbat;
        ok = axp192_read(axp, AXP192_BATTERY_POWER, &pbat) == AXP192_OK;
        ESP_LOGI(TAG, "%s: pbat       %.2fmW%s", __func__, pbat, ok ? "" : " Error");

        float vbat;
        ok = axp192_read(axp, AXP192_BATTERY_VOLTAGE, &vbat) == AXP192_OK;
        ESP_LOGI(TAG, "%s: vbat       %.2fV%s", __func__, vbat, ok ? "" : " Error");

        float icharge;
        ok = axp192_read(axp, AXP192_CHARGE_CURRENT, &icharge) == AXP192_OK;
        ESP_LOGI(TAG, "%s: icharge    %.2fA%s", __func__, icharge, ok ? "" : " Error");

        float idischarge;
        ok = axp192_read(axp, AXP192_DISCHARGE_CURRENT, &idischarge) == AXP192_OK;
        ESP_LOGI(TAG, "%s: idischarge %.2fA%s", __func__, idischarge, ok ? "" : " Error");

        float vaps;
        ok = axp192_read(axp, AXP192_APS_VOLTAGE, &vaps) == AXP192_OK;
        ESP_LOGI(TAG, "%s: vaps       %.2fV%s", __func__, vaps, ok ? "" : " Error");

        uint8_t power;
        ok = axp192_ioctl(axp, AXP192_READ_POWER_STATUS, &power) == AXP192_OK;
        ESP_LOGI(TAG, "%s: power      0x%02x%s", __func__, power, ok ? "" : " Error");

        uint8_t charge;
        ok = axp192_ioctl(axp, AXP192_READ_CHARGE_STATUS, &charge) == AXP192_OK;
        ESP_LOGI(TAG, "%s: charge     0x%02x%s", __func__, charge, ok ? "" : " Error");

        int ldoflag;

        ldoflag = axp192_test_ldo2();
        ESP_LOGI(TAG, "%s: ldo2       %s", __func__, ldoflag >= 0 ? (ldoflag ? "ON": "OFF") : "Error");

        ldoflag = axp192_test_ldo3();
        ESP_LOGI(TAG, "%s: ldo3       %s", __func__, ldoflag >= 0 ? (ldoflag ? "ON": "OFF") : "Error");

        ldoflag = axp192_test_dcdc1();
        ESP_LOGI(TAG, "%s: dcdc1      %s", __func__, ldoflag >= 0 ? (ldoflag ? "ON": "OFF") : "Error");

        ldoflag = axp192_test_dcdc3();
        ESP_LOGI(TAG, "%s: dcdc3      %s", __func__, ldoflag >= 0 ? (ldoflag ? "ON": "OFF") : "Error");
       
    } else {
        ESP_LOGI(TAG, "axp192 not started");
    }

    return axp != NULL;
}

bool axp192_enable_ldo2(bool on)
{
    return axp192_ioctl(axp, on ? AXP192_ENABLE_LDO2 : AXP192_DISABLE_LDO2, NULL) == AXP192_OK;
}

int axp192_test_ldo2(void)
{
    uint8_t buffer[1];
    return axp192_ioctl(axp, AXP192_TEST_LDO2, buffer) == AXP192_OK ? buffer[0] : -1;
}

bool axp192_enable_ldo3(bool on)
{
    return axp192_ioctl(axp, on ? AXP192_ENABLE_LDO3 : AXP192_DISABLE_LDO3, NULL) == AXP192_OK;
}

int axp192_test_ldo3(void)
{
    uint8_t buffer[1];
    return axp192_ioctl(axp, AXP192_TEST_LDO3, buffer) == AXP192_OK ? buffer[0] : -1;
}

bool axp192_endable_dcdc1(bool on)
{
    return axp192_ioctl(axp, on ? AXP192_ENABLE_DCDC1 : AXP192_DISABLE_DCDC1, NULL) == AXP192_OK;
}

int axp192_test_dcdc1(void)
{
    uint8_t buffer[1];
    return axp192_ioctl(axp, AXP192_TEST_DCDC1, buffer) == AXP192_OK ? buffer[0] : -1;
}

bool axp192_enable_dcdc3(bool on)
{
    return axp192_ioctl(axp, on ? AXP192_ENABLE_DCDC3 : AXP192_DISABLE_DCDC3, NULL) == AXP192_OK;
}

int axp192_test_dcdc3(void)
{
    uint8_t buffer[1];
    return axp192_ioctl(axp, AXP192_TEST_DCDC3, buffer) == AXP192_OK ? buffer[0] : -1;
}

#endif /* CONFIG_LASTLINK_ENABLE_POWER_MANAGEMENT */
