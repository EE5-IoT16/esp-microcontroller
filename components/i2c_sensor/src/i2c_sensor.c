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


static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}


static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}


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
    int ReadingError = 0;
    uint8_t data[2];
    if(mpu9250_register_read(ACCEL_XOUT_H, data, 2) != ESP_OK) ReadingError++;
    aOffset.axOffset = (int16_t)((data[0] << 8) | data[1]);
    if(mpu9250_register_read(ACCEL_YOUT_H, data, 2) )          ReadingError++;
    aOffset.ayOffset = (int16_t)((data[0] << 8) | data[1]);
    if(mpu9250_register_read(ACCEL_ZOUT_H, data, 2)!= ESP_OK)  ReadingError++;
    aOffset.azOffset = (int16_t)((data[0] << 8) | data[1])-8196;
    if(ReadingError != 0 )
    {
        char message[100]= " ";
        sprintf(message,"%d error found while reading offset of accelerometer",ReadingError);
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);

    }
}

void getGyroOffset()
{
    int ReadingError = 0;
    uint8_t data[2];
    if(mpu9250_register_read(GYRO_XOUT_H, data, 2)!= ESP_OK ) ReadingError++;
    gOffset.gxOffset = (int16_t)((data[0] << 8) | data[1]);
    if(mpu9250_register_read(GYRO_YOUT_H, data, 2)!= ESP_OK)  ReadingError++;
    gOffset.gyOffset = (int16_t)((data[0] << 8) | data[1]);
    if(mpu9250_register_read(GYRO_ZOUT_H, data, 2)!= ESP_OK)  ReadingError++;
    gOffset.gzOffset = (int16_t)((data[0] << 8) | data[1]);
    if(ReadingError != 0 )
    {
        char message[100]= " ";
        sprintf(message,"%d error found while reading offset of gyroscope",ReadingError);
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);

    }
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
    char message[100]= " ";
    if(i2c_master_init() !=  ESP_OK)
    {
        sprintf(message,"I2C initializing fail");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }
    ESP_LOGI(TAG, "I2C initialized successfully");

    /*Reset the MPU9250 */
    if(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT) !=  ESP_OK)
    {
        sprintf(message,"Error occurs when resetting sensor MPU9250");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }

    /*Set the clock reference to X axis gyroscope to get a better accuracy*/
    if(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 0x01)!=  ESP_OK)
    {
        sprintf(message,"Error occurs when setting clock reference");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }

    /*Set the accel scale to 4g*/
    if(mpu9250_register_write_byte(ACCEL_CONFIG_1_AD, 0x08)!=  ESP_OK)
    {
        sprintf(message,"Error occurs when setting accelerometer scale fail");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }

    /*Set the gyro scale to 500 °/s and FCHOICE_B*/
    if(mpu9250_register_write_byte(GYRO_CONFIG_AD, 0x08)!=  ESP_OK)
    {
        sprintf(message,"Error occurs when setting gyro scale");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }

    /*Turn on the internal low-pass filter for accelerometer with 10.2Hz bandwidth*/
    if(mpu9250_register_write_byte(ACCEL_CONFIG_2_AD, 0x05)!=  ESP_OK)
    {
        sprintf(message,"Error occurs when turning on the internal low-pass filter for accelerometer");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }

    /*Turn on the internal low-pass filter for gyroscope with 10Hz bandwidth*/
    if(mpu9250_register_write_byte(CONFIG_AD, 0x05)!=  ESP_OK)
    {
        sprintf(message,"Error occurs when turning on the internal low-pass filter for gyroscope");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }
   
    /*Set gyro sample rate to 125Hz*/
    if(mpu9250_register_write_byte(SMPLRT_DIV, 0x07)!=  ESP_OK)
    {
        sprintf(message,"Error occurs when setting sampling rate");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err)); 
    }
    //////////////////////////////////////////////////////////////////////////////////////
    sprintf(message,"Don't move the sensor. the offset values are being read now");
    uint32_t length = strlen(message);
    esp_blufi_send_custom_data((unsigned char*)message,length);
    vTaskDelay(3000/ portTICK_PERIOD_MS);
    getAccelOffset();
    getGyroOffset();
    sprintf(message,"Offset values reading finished");
    length = strlen(message);
    esp_blufi_send_custom_data((unsigned char*)message,length);
}


void readDataFromSensor(int frequency)
{    
    char message[100]= " ";
    int ReadingError = 0;
    vTaskDelay(frequency/ portTICK_PERIOD_MS);
    uint8_t data[2];
    if(mpu9250_register_read(ACCEL_XOUT_H, data, 2)!=  ESP_OK) ReadingError++;
    short ax = (int16_t)((data[0] << 8) | data[1]);
    printf("ax = %2.1fm/s²   ",((ax-aOffset.axOffset)/32768.0)*4*GravAccel);

    if(mpu9250_register_read(ACCEL_YOUT_H, data, 2)!=  ESP_OK) ReadingError++;
    short ay = (int16_t)((data[0] << 8) | data[1]);
    printf("ay = %2.1fm/s²   ",((ay-aOffset.ayOffset)/32768.0)*4*GravAccel);

    stepCounter(ay);

    if(mpu9250_register_read(ACCEL_ZOUT_H, data, 2)!=  ESP_OK) ReadingError++;
    short az = (int16_t)((data[0] << 8) | data[1]);
    printf("az = %2.1fm/s²\n",((az-aOffset.azOffset)/32768.0)*4*GravAccel);

    if(mpu9250_register_read(GYRO_XOUT_H, data, 2)!=  ESP_OK)  ReadingError++;
    short gx = (int16_t)((data[0] << 8) | data[1]);
    printf("gx = %3.1f°/s    ",((gx-gOffset.gxOffset)/32768.0)*500);

    if(mpu9250_register_read(GYRO_YOUT_H, data, 2)!=  ESP_OK) ReadingError++; 
    short gy = (int16_t)((data[0] << 8) | data[1]);
    printf("gy = %3.1f°/s    ",((gy-gOffset.gyOffset)/32768.0)*500);

    if(mpu9250_register_read(GYRO_ZOUT_H, data, 2)!=  ESP_OK)  ReadingError++;
    short gz = (int16_t)((data[0] << 8) | data[1]);
    printf("gz = %3.1f°/s\n",((gz-gOffset.gzOffset)/32768.0)*500);
    
    printf("xAngle = %3.1f°   ",acos(((ax-aOffset.axOffset)/32768.0)*4)*57.29577);
    printf("yAngle = %3.1f°   ",acos(((ay-aOffset.ayOffset)/32768.0)*4)*57.29577);
    printf("zAngle = %3.1f°\n",acos(((az-aOffset.azOffset)/32768.0)*4)*57.29577);
    
    if(mpu9250_register_read(TEMP_OUT_H, data, 2)!=  ESP_OK)  ReadingError++;
    short temp = (int16_t)((data[0] << 8) | data[1]) ;
    float temperature = 21.00f + ((((float)temp-333.87f*6))/333.87f);
    printf("temperature = %f°C\n", temperature);
    printf("state:%d, step:%d\n\n",state,step);

    if(ReadingError != 0)
    {
        sprintf(message,"%d error found while reading data from sensors",ReadingError);
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }

    if(nvs_set_u32(my_handle, "step_counter", step)!= ESP_OK)
    {
        sprintf(message,"Error occurs when saving data in memory");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }
    err = nvs_commit(my_handle);
}

void unitializedI2C(void)
{
    char message[100]= " ";
    if(i2c_driver_delete(I2C_MASTER_NUM)!=  ESP_OK)
    {
        sprintf(message,"Error occurs when closing I2C connection");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }
    ESP_LOGI(TAG, "I2C unitialized successfully");
    nvs_close(my_handle);
}


