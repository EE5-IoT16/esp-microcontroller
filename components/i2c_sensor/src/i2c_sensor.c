#include "i2c_sensor.h"

static const char *TAG = "i2c-sensor";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define GYRO_CONFIG_AD 0x1B
#define ACCEL_CONFIG_1_AD 0x1C
#define ACCEL_CONFIG_2_AD 0x1D
#define CONFIG_AD 0x1A
#define SMPLRT_DIV 0x19

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42

#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48

#define treshH 17
#define treshL 12

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */
#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

#define GravAccel   9.80665

esp_err_t err;
nvs_handle_t my_handle;
int ReadingError;
int WirtingError;

struct accelOffset
{
    short   axOffset;
    short   ayOffset;
    short   azOffset;
};

struct gyroOffset
{
    short  gxOffset;
    short  gyOffset;
    short   gzOffset;
};

static struct gyroOffset  gOffset;
static struct accelOffset aOffset;

enum state 
{
    low = 0,
    high = 1
};

uint32_t step =0;
enum state state=low;

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void getAccelOffset()
{
    uint8_t data[2];
    ESP_ERROR_CHECK(mpu9250_register_read(ACCEL_XOUT_H, data, 2));
    aOffset.axOffset = (int16_t)((data[0] << 8) | data[1]);
    ESP_ERROR_CHECK(mpu9250_register_read(ACCEL_YOUT_H, data, 2));
    aOffset.ayOffset = (int16_t)((data[0] << 8) | data[1]);
    ESP_ERROR_CHECK(mpu9250_register_read(ACCEL_ZOUT_H, data, 2));
    aOffset.azOffset = (int16_t)((data[0] << 8) | data[1])-8196;
}

void getGyroOffset()
{
    uint8_t data[2];
    ESP_ERROR_CHECK(mpu9250_register_read(GYRO_XOUT_H, data, 2));
    gOffset.gxOffset = (int16_t)((data[0] << 8) | data[1]);
    ESP_ERROR_CHECK(mpu9250_register_read(GYRO_YOUT_H, data, 2));
    gOffset.gyOffset = (int16_t)((data[0] << 8) | data[1]);
    ESP_ERROR_CHECK(mpu9250_register_read(GYRO_ZOUT_H, data, 2));
    gOffset.gzOffset = (int16_t)((data[0] << 8) | data[1]);
}

void stepCounter(short ay)
{
    if (state == low)
    {
        if (abs(((ay-aOffset.ayOffset)/32768.0)*4*9.8)>treshH)
        {
            state = high;
            step++;
        }
    }
    else 
    {
        if (abs(((ay-aOffset.ayOffset)/32768.0)*4*9.8)<treshL)
        {
            step++;
            state = low;
        }  
    }
}

void i2cSensor_init(void)
{
    ReadingError = 0;
    WirtingError = 0;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    /*Reset the MPU9250 */
    ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    /*Set the clock reference to X axis gyroscope to get a better accuracy*/
    ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 0x01));

    /*Set the accel scale to 4g*/
    ESP_ERROR_CHECK(mpu9250_register_write_byte(ACCEL_CONFIG_1_AD, 0x08));

    /*Set the gyro scale to 500 °/s and FCHOICE_B*/
    ESP_ERROR_CHECK(mpu9250_register_write_byte(GYRO_CONFIG_AD, 0x08));

    /*Turn on the internal low-pass filter for accelerometer with 10.2Hz bandwidth*/
    ESP_ERROR_CHECK(mpu9250_register_write_byte(ACCEL_CONFIG_2_AD, 0x05));

    /*Turn on the internal low-pass filter for gyroscope with 10Hz bandwidth*/
    ESP_ERROR_CHECK(mpu9250_register_write_byte(CONFIG_AD, 0x05));
   
    /*Set gyro sample rate to 125Hz*/
    ESP_ERROR_CHECK(mpu9250_register_write_byte(SMPLRT_DIV, 0x07));

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err)); 
    }
    //////////////////////////////////////////////////////////////////////////////////////
    char message[100]= " ";
    sprintf(message,"Don't move the sensor. the offset values are being read now");
    uint32_t length = strlen(message);
    esp_blufi_send_custom_data((unsigned char*)message,length);
    vTaskDelay(1500/ portTICK_PERIOD_MS);
    getAccelOffset();
    getGyroOffset();
    sprintf(message,"Offset values reading finished");
    length = strlen(message);
    esp_blufi_send_custom_data((unsigned char*)message,length);
}


void readDataFromSensor(int frequency)
{    
    vTaskDelay(frequency/ portTICK_PERIOD_MS);
    uint8_t data[2];
    ESP_ERROR_CHECK(mpu9250_register_read(ACCEL_XOUT_H, data, 2));
    //ESP_LOGI(TAG, "ACCEL_X = %X", data[0]);
    short ax = (int16_t)((data[0] << 8) | data[1]);
    printf("ax = %2.1fm/s²   ",((ax-aOffset.axOffset)/32768.0)*4*GravAccel);
    ESP_ERROR_CHECK(mpu9250_register_read(ACCEL_YOUT_H, data, 2));
    //ESP_LOGI(TAG, "ACCEL_Y = %X", data[0]);
    short ay = (int16_t)((data[0] << 8) | data[1]);
    printf("ay = %2.1fm/s²   ",((ay-aOffset.ayOffset)/32768.0)*4*GravAccel);
    stepCounter(ay);

    ESP_ERROR_CHECK(mpu9250_register_read(ACCEL_ZOUT_H, data, 2));
    short az = (int16_t)((data[0] << 8) | data[1]);
    printf("az = %2.1fm/s²\n",((az-aOffset.azOffset)/32768.0)*4*GravAccel);

    ESP_ERROR_CHECK(mpu9250_register_read(GYRO_XOUT_H, data, 2));
    //ESP_LOGI(TAG, "ACCEL_X = %X", data[0]);
    short gx = (int16_t)((data[0] << 8) | data[1]);
    printf("gx = %3.1f°/s    ",((gx-gOffset.gxOffset)/32768.0)*500);

    ESP_ERROR_CHECK(mpu9250_register_read(GYRO_YOUT_H, data, 2));
    //ESP_LOGI(TAG, "ACCEL_X = %X", data[0]);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   m
    short gy = (int16_t)((data[0] << 8) | data[1]);
    printf("gy = %3.1f°/s    ",((gy-gOffset.gyOffset)/32768.0)*500);

    ESP_ERROR_CHECK(mpu9250_register_read(GYRO_ZOUT_H, data, 2));
    //ESP_LOGI(TAG, "ACCEL_X = %X", data[0]);
    short gz = (int16_t)((data[0] << 8) | data[1]);
    printf("gz = %3.1f°/s\n",((gz-gOffset.gzOffset)/32768.0)*500);
    
    printf("xAngle = %3.1f°   ",acos(((ax-aOffset.axOffset)/32768.0)*4)*57.29577);
    printf("yAngle = %3.1f°   ",acos(((ay-aOffset.ayOffset)/32768.0)*4)*57.29577);
    printf("zAngle = %3.1f°\n",acos(((az-aOffset.azOffset)/32768.0)*4)*57.29577);
    
    
    ESP_ERROR_CHECK(mpu9250_register_read(TEMP_OUT_H, data, 2));
    short temp = (int16_t)((data[0] << 8) | data[1]) ;
    float temperature = 21.00f + ((((float)temp-333.87f*6))/333.87f);
    printf("temperature = %f°C\n", temperature);
    printf("state:%d, step:%d\n\n",state,step);

        // Write
        //printf("Updating ax in NVS ... ");
        err = nvs_set_u32(my_handle, "step_counter", step);
        //printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        //printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        //printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        /*
        uint32_t test;
        err = nvs_get_u32(my_handle, "step_counter", &test);
        printf("step = %d\n\n", test);*/
    /*
        // Read
        printf("Reading ax from NVS ... ");
        uint32_t test;
        err = nvs_get_i16(my_handle, "step_counter", &test);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("step = %d\n\n", test);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        

        // Close
        //nvs_close(my_handle);*/
}

void unitializedI2C(void)
{
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
    nvs_close(my_handle);
}


