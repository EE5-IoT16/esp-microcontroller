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

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ABS(a)   (0 - (a)) > 0 ? (-(a)) : (a)
#define DYNAMIC_PRECISION 200
#define SAMPLE_SIZE 50
#define ACTIVE_NULL 0
#define ACTIVE_X    1
#define ACTIVE_Y    2 
#define ACTIVE_Z    3
#define ACTIVE_PRECISION 400


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
static int readCounter = 0;
static int output_counter = 0;

uint32_t step =0;

typedef struct 
{
    short x;
    short y;
    short z;

}axis_info_t;

typedef struct peak_value
{
    axis_info_t curMAX;
    axis_info_t curMIN;
    axis_info_t preMAX;
    axis_info_t preMIN;

}peak_value_t;

typedef struct fall_info
{
    float At_curr;
    int   Gt_curr;
    float At_prev;
    int   Gt_prev;

}fall_info_t;

typedef struct slid_reg
{
    axis_info_t new;
    axis_info_t old;

}slid_reg_t;

static struct fall_info  FALL;
static struct peak_value PEAK;
static struct slid_reg   SLID; 
static int fall_possibility = 0;

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
    axis_info_t offset;
    offset.x = aOffset.axOffset;
    offset.y = aOffset.ayOffset;
    offset.z = aOffset.azOffset;
    SLID.new = SLID.old = offset;
    PEAK.curMAX = PEAK.curMIN = PEAK.preMAX = PEAK.preMIN = offset;
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

static void peakInit(peak_value_t *peak)
{
    axis_info_t temp;
    temp = peak->curMAX;
    peak->curMAX = peak->curMIN;
    peak->curMIN = temp;
}

static void peakUpdate(peak_value_t* peak, axis_info_t* cur_value)
{
    static unsigned int samplez_size = 0;
    samplez_size++;
    if(samplez_size > SAMPLE_SIZE )
    {
        samplez_size =1;
        peak->preMAX = peak->curMAX;
        peak->preMIN = peak->curMIN;
        peakInit(peak); 
    }
    peak->curMAX.x = MAX(peak->curMAX.x,cur_value->x);
    peak->curMAX.y = MAX(peak->curMAX.y,cur_value->y);
    peak->curMAX.z = MAX(peak->curMAX.z,cur_value->z);

    peak->curMIN.x = MIN(peak->curMIN.x,cur_value->x);
    peak->curMIN.y = MIN(peak->curMIN.y,cur_value->y);
    peak->curMIN.z = MIN(peak->curMIN.x,cur_value->z);
}

static char slid_update(slid_reg_t* slid, axis_info_t* cur_sample)
{
    char res = 0;
    if (ABS(cur_sample->x - slid->new.x) > DYNAMIC_PRECISION)
    {
        slid->old.x = slid->new.x;
        slid->new.x = cur_sample->x;
        res = 1;
    }
    else slid->old.x = slid->new.x;

    if (ABS(cur_sample->y - slid->new.y) > DYNAMIC_PRECISION)
    {
        slid->old.y = slid->new.y;
        slid->new.y = cur_sample->y;
        res = 1;
    }
    else slid->old.y = slid->new.y;

    if (ABS(cur_sample->z - slid->new.z) > DYNAMIC_PRECISION)
    {
        slid->old.z = slid->new.z;
        slid->new.z = cur_sample->z;
        res = 1;
    }
    else slid->old.z = slid->new.z;

    return res;
}

static char findActive(peak_value_t *peak)
{
    char res = ACTIVE_NULL;
    short x_change = ABS(peak->curMAX.x - peak->curMIN.x);
    short y_change = ABS(peak->curMAX.y - peak->curMIN.y);
    short z_change = ABS(peak->curMAX.z - peak->curMIN.z);
    if (x_change > y_change && x_change > z_change && x_change >= ACTIVE_PRECISION)
    {
        res = ACTIVE_X;
    }
    else if (y_change > x_change && y_change > z_change && y_change >= ACTIVE_PRECISION)
    {
        res = ACTIVE_Y;
    }
    else if (z_change > x_change && z_change > y_change && z_change >= ACTIVE_PRECISION)
    {
         res = ACTIVE_Z;
    }
    else
    {
        res = ACTIVE_NULL;
    }
    return res;
}

static void detect_step(peak_value_t *peak, slid_reg_t* slid, axis_info_t *cur_sample)
{
    
    char res = findActive(peak);
    switch (res)
    {
    case ACTIVE_NULL:
        break;

    case ACTIVE_X:{
        short threshold_x = (peak->preMAX.x + peak->preMIN.x)/2;
        if(slid->old.x > threshold_x && slid->new.x < threshold_x)
        {
            step ++;
        }
        break;}

    case ACTIVE_Y:{
        short threshold_y = (peak->preMAX.y + peak->preMIN.y)/2;
        if(slid->old.y > threshold_y && slid->new.y < threshold_y)
        {
            step ++;
        }
        break;}
    
    case ACTIVE_Z:{
        short threshold_z = (peak->preMAX.z + peak->preMIN.z)/2;
        if(slid->old.z > threshold_z && slid->new.z < threshold_z)
        {
            step ++;
        }
        break;}

    default:
        break;
    }
}

void step_counter(void)
{
    readCounter++;
    uint8_t data[2];
    axis_info_t cur_sample;
    mpu9250_register_read(ACCEL_XOUT_H, data, 2);
    cur_sample.x = (int16_t)((data[0] << 8) | data[1]);
    mpu9250_register_read(ACCEL_YOUT_H, data, 2);
    cur_sample.x = (int16_t)((data[0] << 8) | data[1]);
    mpu9250_register_read(ACCEL_ZOUT_H, data, 2);
    cur_sample.x = (int16_t)((data[0] << 8) | data[1]);
    peakUpdate(&PEAK,&cur_sample);
    slid_update(&SLID,&cur_sample);
    detect_step(&PEAK,&SLID,&cur_sample);
    if(readCounter == 10)
    {
        readCounter = 0;
        if(nvs_set_u32(my_handle, "step_counter", step/2)!= ESP_OK)
        {
            char message[100]= " ";
            sprintf(message,"Error occurs when saving step data in memory");
            uint32_t length = strlen(message);
            esp_blufi_send_custom_data((unsigned char*)message,length);
        }
        nvs_commit(my_handle);
    } 
   //printf("step:%d\n",step/2);
}

void measure_temperature(void)
{

    uint8_t data[2];
    if(mpu9250_register_read(TEMP_OUT_H, data, 2)!=  ESP_OK) 
    {
        char message[100]= " ";
        sprintf(message,"Error occurs during reading temperature");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }
    short temp = (int16_t)((data[0] << 8) | data[1]);
    if(nvs_set_i16(my_handle, "temperature", temp)!= ESP_OK)
    {
        char message[100]= " ";
        sprintf(message,"Error occurs when saving tempearature data in memory");
        uint32_t length = strlen(message);
        esp_blufi_send_custom_data((unsigned char*)message,length);
    }
    nvs_commit(my_handle); 
}


int detect_fall(void)
{
    output_counter++;
    uint8_t data[2];
    int ReadingError =0;
    if(mpu9250_register_read(GYRO_XOUT_H, data, 2)!=  ESP_OK)  ReadingError++;
    short gx = (int16_t)((data[0] << 8) | data[1]);
    int Gx = (gx-gOffset.gxOffset)*500/32768;

    if(mpu9250_register_read(GYRO_YOUT_H, data, 2)!=  ESP_OK) ReadingError++; 
    short gy = (int16_t)((data[0] << 8) | data[1]);
    int Gy = (gy-gOffset.gyOffset)*500/32768;

    if(mpu9250_register_read(GYRO_ZOUT_H, data, 2)!=  ESP_OK)  ReadingError++;
    short gz = (int16_t)((data[0] << 8) | data[1]);
    int Gz = (gz-gOffset.gzOffset)*500/32768;

    if(mpu9250_register_read(ACCEL_XOUT_H, data, 2)!=  ESP_OK) ReadingError++;
    short ax = (int16_t)((data[0] << 8) | data[1]);
    if(mpu9250_register_read(ACCEL_YOUT_H, data, 2)!=  ESP_OK) ReadingError++;
    short ay = (int16_t)((data[0] << 8) | data[1]);
    if(mpu9250_register_read(ACCEL_ZOUT_H, data, 2)!=  ESP_OK) ReadingError++;
    short az = (int16_t)((data[0] << 8) | data[1]);

    float Ax = (ax-aOffset.axOffset)*4/32768.0;
    float Ay = (ay-aOffset.ayOffset)*4/32768.0;
    float Az = (az-aOffset.azOffset)*4/32768.0;

    float Ax_peak = (PEAK.curMAX.x-aOffset.axOffset)*4/32768.0;
    float Ay_peak = (PEAK.curMAX.y-aOffset.ayOffset)*4/32768.0;
    float Az_peak = (PEAK.curMAX.z-aOffset.azOffset)*4/32768.0;
    float A_tresh = sqrt(pow(Ax_peak,2)+pow(Ay_peak,2)+pow(Az_peak,2));

    FALL.Gt_prev = FALL.Gt_curr;
    FALL.At_prev = FALL.At_curr;
    FALL.Gt_curr = sqrt(pow(Gy,2)+pow(Gx,2)+pow(Gz,2));
    FALL.At_curr = sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
    
    //printf("Atotal = %.2f g  ",FALL.At_curr);
    //printf("Gtotal = %d °/s\n",FALL.Gt_curr);
    if(abs(FALL.At_curr-FALL.At_prev) >= A_tresh/3 || abs(FALL.Gt_curr-FALL.Gt_prev) >= 260)
    {
        fall_possibility++;
    }

    if (output_counter == 3)
    {
        output_counter = 0;
        int out = fall_possibility;
        fall_possibility = 0;
        if(out >=2) 
        {printf("fall_possibility: %d percent\n",out*33);
        return out;}
        else return 0;
    }
    else return 0;
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


