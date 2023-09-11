/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * sensors_bosch.c
 */

/*********************************************
 * IMPORTANT NOTE relating to sensor_deck v1 *
 * Don't miss to align the axes correctly!   *
 *********************************************/
#define DEBUG_MODULE "IMU"

#include "sensors_bosch.h"

#include <math.h>

#include "stm32fxxx.h"

#include "imu.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "system.h"
#include "configblock.h"
#include "param.h"
#include "debug.h"
#include "nvicconf.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"

/* Bosch Sensortec Drivers */
#include "bmi055.h"
#include "bmi088.h"
#include "bmi160.h"
#include "bmm150.h"
#include "bmp280.h"
#include "bmp3.h"
#include "bstdr_comm_support.h"
#include "static_mem.h"

#define SENSORS_READ_RATE_HZ            1000
#define SENSORS_STARTUP_TIME_MS         1000
#define SENSORS_READ_BARO_HZ            50
#define SENSORS_READ_MAG_HZ             20
#define SENSORS_DELAY_BARO              (SENSORS_READ_RATE_HZ/SENSORS_READ_BARO_HZ)
#define SENSORS_DELAY_MAG               (SENSORS_READ_RATE_HZ/SENSORS_READ_MAG_HZ)

/* calculate constants */
/* BMI160 */
#define SENSORS_BMI160_GYRO_FS_CFG      BMI160_GYRO_RANGE_2000_DPS
#define SENSORS_BMI160_DEG_PER_LSB_CFG  (2.0f *2000.0f) / 65536.0f

#define SENSORS_BMI160_ACCEL_CFG        16
#define SENSORS_BMI160_ACCEL_FS_CFG     BMI160_ACCEL_RANGE_16G
#define SENSORS_BMI160_G_PER_LSB_CFG    (2.0f * (float)SENSORS_BMI160_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI160_1G_IN_LSB        65536 / SENSORS_BMI160_ACCEL_CFG / 2

/* BMI055 */
#define SENSORS_BMI055_GYRO_FS_CFG      BMI055_GYRO_RANGE_2000_DPS
#define SENSORS_BMI055_DEG_PER_LSB_CFG  (2.0f *2000.0f) / 65536.0f

#define SENSORS_BMI055_ACCEL_CFG        16
#define SENSORS_BMI055_ACCEL_FS_CFG     BMI055_ACCEL_RANGE_16G
#define SENSORS_BMI055_G_PER_LSB_CFG    (2.0f * (float)SENSORS_BMI055_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI055_1G_IN_LSB        (65536 / SENSORS_BMI055_ACCEL_CFG / 2)

/* BMI088 */
#define SENSORS_BMI088_GYRO_FS_CFG      BMI088_GYRO_RANGE_2000_DPS
#define SENSORS_BMI088_DEG_PER_LSB_CFG  (2.0f *2000.0f) / 65536.0f

#define SENSORS_BMI088_ACCEL_CFG        24
#define SENSORS_BMI088_ACCEL_FS_CFG     BMI088_ACCEL_RANGE_24G
#define SENSORS_BMI088_G_PER_LSB_CFG    (2.0f * (float)SENSORS_BMI088_ACCEL_CFG) / 65536.0f
#define SENSORS_BMI088_1G_IN_LSB        (65536 / SENSORS_BMI088_ACCEL_CFG / 2)

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT   M2T(1000) // Timeout in ms
#define SENSORS_MAN_TEST_LEVEL_MAX          5.0f      // Max degrees off

#define GYRO_NBR_OF_AXES                3
#define GYRO_MIN_BIAS_TIMEOUT_MS        M2T(1*1000)

// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES  512

// Variance threshold to take zero bias for gyro
#define GYRO_VARIANCE_BASE              2000
#define GYRO_VARIANCE_THRESHOLD_X       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y       (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z       (GYRO_VARIANCE_BASE)



#define SENSORS_TAKE_ACCEL_BIAS

/* available sensors */
#define SENSORS_BMI055          0x01
#define SENSORS_BMI160          0x02
#define SENSORS_BMM150          0x08
#define SENSORS_BMP280          0x10

/* configure sensor's use
 * PRIMARIES are the sensors which are used for stabilization
 * SECONDARIES are only added to the log if compilations is
 * done with CFLAGS += -DLOG_SEC_IMU */
static uint8_t gyroPrimInUse =          SENSORS_BMI055;
static uint8_t accelPrimInUse =         SENSORS_BMI055;
//static uint8_t baroPrimInUse =          SENSORS_BMP280;
#ifdef LOG_SEC_IMU
static uint8_t gyroSecInUse =           SENSORS_BMI160;
static uint8_t accelSecInUse =          SENSORS_BMI160;
#endif

typedef struct {
  Axis3i16      value;
  Axis3i16*     bufStart;
  Axis3i16*     bufPtr;
  uint8_t       found : 1;
  uint8_t       ongoing : 1;
  uint8_t       bufIsFull : 1;
} BiasObj;

/* initialize necessary variables */
static struct bmi160_dev bmi160Dev;
static struct bmi055_dev bmi055Dev;
static struct bmp280_t bmp280Dev;
static struct bmm150_dev bmm150Dev;

static xQueueHandle accelPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelPrimDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroPrimDataQueue, 1, sizeof(Axis3f));
#ifdef LOG_SEC_IMU
static xQueueHandle accelSecDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelSecDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroSecDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroSecDataQueue, 1, sizeof(Axis3f));
#endif
static xQueueHandle baroPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(baroPrimDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magPrimDataQueue;
STATIC_MEM_QUEUE_ALLOC(magPrimDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static bool isInit = false;
static bool allSensorsAreCalibrated = false;
static sensorData_t sensors;

static int32_t varianceSampleTime;
static uint8_t sensorsAccLpfAttFactor;

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;
static uint8_t baroMeasDelayMin = SENSORS_DELAY_BARO;

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static void sensorsDeviceInit(void);
static void sensorsTaskInit(void);
static void sensorsTask(void *param);
static void sensorsApplyBiasAndScale(Axis3f* scaled, Axis3i16* aligned,
                                     Axis3i16* bias, float scale);
static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature);
static bool processGyroBias(BiasObj* bias);
static void processAccelBias(BiasObj* bias);

static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                                  Axis3i32* storedValues, int32_t attenuation);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);
static void sensorsBiasReset(BiasObj* bias);
static void sensorsBiasMalloc(BiasObj* bias);
static void sensorsBiasFree(BiasObj* bias);
static void sensorsBiasBufPtrIncrement(BiasObj* bias);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

void sensorsBoschInit(void)
{
  if (isInit)
    {
      return;
    }

  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

  sensorsDeviceInit();
  sensorsTaskInit();

  isInit = true;
}

static void sensorsDeviceInit(void)
{
  if (isInit)
    return;

  bstdr_ret_t rslt;
  isBarometerPresent = false;

  // Wait for sensors to startup
  vTaskDelay(M2T(SENSORS_STARTUP_TIME_MS));

  /* BMI160 */
  // assign bus read function
  bmi160Dev.read = (bmi160_com_fptr_t)bstdr_burst_read;
  // assign bus write function
  bmi160Dev.write = (bmi160_com_fptr_t)bstdr_burst_write;
  // assign delay function
  bmi160Dev.delay_ms = (bmi160_delay_fptr_t)bstdr_ms_delay;
  bmi160Dev.id = BMI160_I2C_ADDR+1;  // I2C device address

  rslt = bmi160_init(&bmi160Dev); // initialize the device
  if (rslt == BSTDR_OK)
    {
      DEBUG_PRINT("BMI160 I2C connection [OK].\n");
      /* Select the Output data rate, range of Gyroscope sensor
       * ~92Hz BW by OSR4 @ODR=800Hz */
      bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
      bmi160Dev.gyro_cfg.range = SENSORS_BMI160_GYRO_FS_CFG;
      bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;

      /* Select the power mode of Gyroscope sensor */
      bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

      /* Select the Output data rate, range of accelerometer sensor
       * ~92Hz BW by OSR4 @ODR=800Hz */
      bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
      bmi160Dev.accel_cfg.range = SENSORS_BMI160_ACCEL_FS_CFG;
      bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
      /* Select the power mode of accelerometer sensor */
      bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

      /* Set the sensor configuration */
      rslt |= bmi160_set_sens_conf(&bmi160Dev);
      bmi160Dev.delay_ms(50);

      /* read sensor */
      struct bmi160_sensor_data gyr;
      rslt |= bmi160_get_sensor_data(BMI160_GYRO_ONLY, NULL, &gyr,
                                     &bmi160Dev);
      struct bmi160_sensor_data acc;
      rslt |= bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &acc, NULL,
                                     &bmi160Dev);
    }
  else
    {
      DEBUG_PRINT("BMI160 I2C connection [FAIL].\n");
    }

  /* BMI055 */
  bmi055Dev.accel_id = BMI055_ACCEL_I2C_ADDR;
  bmi055Dev.gyro_id = BMI055_GYRO_I2C_ADDR;
  bmi055Dev.interface = BMI055_I2C_INTF;
  bmi055Dev.read = (bmi055_com_fptr_t)bstdr_burst_read;
  bmi055Dev.write = (bmi055_com_fptr_t)bstdr_burst_write;
  bmi055Dev.delay_ms = (bmi055_delay_fptr_t)bstdr_ms_delay;

  /* BMI055 GYRO */
  rslt = bmi055_gyro_init(&bmi055Dev); // initialize the device
  if (rslt == BSTDR_OK)
    {
      DEBUG_PRINT("BMI055 Gyro I2C connection [OK].\n");
      /* set power mode of gyro */
      bmi055Dev.gyro_cfg.power = BMI055_GYRO_PM_NORMAL;
      rslt |= bmi055_set_gyro_power_mode(&bmi055Dev);
      /* set bandwidth and range of gyro */
      bmi055Dev.gyro_cfg.bw = BMI055_GYRO_BW_116_HZ;
      bmi055Dev.gyro_cfg.range = SENSORS_BMI055_GYRO_FS_CFG;
      rslt |= bmi055_set_gyro_sensor_config(CONFIG_ALL, &bmi055Dev);

      bmi055Dev.delay_ms(50);
      struct bmi055_sensor_data gyr;
      rslt |= bmi055_get_gyro_data(&gyr, &bmi055Dev);
    }
  else
    {
      DEBUG_PRINT("BMI055 Gyro I2C connection [FAIL].\n");
    }

  /* BMI055 ACCEL */
  rslt = bmi055_accel_init(&bmi055Dev); // initialize the device
  if (rslt == BSTDR_OK)
    {
      DEBUG_PRINT("BMI055 Accel I2C connection [OK].\n");
      /* set power mode of accel */
      bmi055Dev.accel_cfg.power = BMI055_ACCEL_PM_NORMAL;
      rslt |= bmi055_set_accel_power_mode(&bmi055Dev);
      /* set bandwidth and range of accel */
      bmi055Dev.accel_cfg.bw = BMI055_ACCEL_BW_125_HZ;
      bmi055Dev.accel_cfg.range = SENSORS_BMI055_ACCEL_FS_CFG;
      rslt |= bmi055_set_accel_sensor_config(CONFIG_ALL, &bmi055Dev);

      bmi055Dev.delay_ms(10);
      struct bmi055_sensor_data acc;
      rslt |= bmi055_get_accel_data(&acc, &bmi055Dev);
    }
  else
    {
      DEBUG_PRINT("BMI055 Accel I2C connection [FAIL].\n");
    }

  /* BMM150 */
  rslt = BSTDR_E_GEN_ERROR;

  /* Sensor interface over I2C */
  bmm150Dev.id = BMM150_DEFAULT_I2C_ADDRESS;
  bmm150Dev.interface = BMM150_I2C_INTF;
  bmm150Dev.read = (bmm150_com_fptr_t)bstdr_burst_read;
  bmm150Dev.write = (bmm150_com_fptr_t)bstdr_burst_write;
  bmm150Dev.delay_ms = bstdr_ms_delay;

  rslt = bmm150_init(&bmm150Dev);

  if (rslt == BMM150_OK){
      bmm150Dev.settings.pwr_mode = BMM150_NORMAL_MODE;
      rslt |= bmm150_set_op_mode(&bmm150Dev);
      bmm150Dev.settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;
      rslt |= bmm150_set_presetmode(&bmm150Dev);

      DEBUG_PRINT("BMM150 I2C connection [OK].\n");
      isMagnetometerPresent = true;
  }

  /* BMP280 */
  rslt = BSTDR_E_GEN_ERROR;

  bmp280Dev.bus_read = bstdr_burst_read;
  bmp280Dev.bus_write = bstdr_burst_write;
  bmp280Dev.delay_ms = bstdr_ms_delay;
  bmp280Dev.dev_addr = BMP280_I2C_ADDRESS1;
  rslt = bmp280_init(&bmp280Dev);
  if (rslt == BSTDR_OK)
    {
      isBarometerPresent = true;
      DEBUG_PRINT("BMP280 I2C connection [OK].\n");
      bmp280_set_filter(BMP280_FILTER_COEFF_OFF);
      bmp280_set_oversamp_temperature(BMP280_OVERSAMP_2X);
      bmp280_set_oversamp_pressure(BMP280_OVERSAMP_8X);
      bmp280_set_power_mode(BMP280_NORMAL_MODE);
      bmp280Dev.delay_ms(20); // wait before first read out
      // read out data
      int32_t v_temp_s32;
      uint32_t v_pres_u32;
      bmp280_read_pressure_temperature(&v_pres_u32, &v_temp_s32);
      baroMeasDelayMin = SENSORS_DELAY_BARO;
    }

  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  sensorsAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI / 180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI / 180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI / 180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI / 180);
}

static void sensorsTaskInit(void)
{
  accelPrimDataQueue = STATIC_MEM_QUEUE_CREATE(accelPrimDataQueue);
  gyroPrimDataQueue = STATIC_MEM_QUEUE_CREATE(gyroPrimDataQueue);
#ifdef LOG_SEC_IMU
  accelSecDataQueue = STATIC_MEM_QUEUE_CREATE(accelSecDataQueue);
  gyroSecDataQueue= STATIC_MEM_QUEUE_CREATE(gyroSecDataQueue);
#endif
  magPrimDataQueue = STATIC_MEM_QUEUE_CREATE(magPrimDataQueue);
  baroPrimDataQueue = STATIC_MEM_QUEUE_CREATE(baroPrimDataQueue);

  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

static void sensorsGyroGet(Axis3i16* dataOut, uint8_t device) {
  static struct bmi160_sensor_data temp;
  switch(device) {
    case SENSORS_BMI160:
      bmi160_get_sensor_data(BMI160_GYRO_ONLY, NULL, &temp, &bmi160Dev);
      dataOut->x = temp.x;
      dataOut->y = temp.y;
      dataOut->z = temp.z;
      break;
    case SENSORS_BMI055:
      bmi055_get_gyro_data(
          (struct bmi055_sensor_data*)dataOut, &bmi055Dev);
      break;
  }
}

static void sensorsAccelGet(Axis3i16* dataOut, uint8_t device) {
  static struct bmi160_sensor_data temp;
  switch(device) {
    case SENSORS_BMI160:
      bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &temp, NULL, &bmi160Dev);
      dataOut->x = temp.x;
      dataOut->y = temp.y;
      dataOut->z = temp.z;
      break;
    case SENSORS_BMI055:
      bmi055_get_accel_data(
          (struct bmi055_sensor_data*)dataOut, &bmi055Dev);
      /* scale to 16 bit */
      dataOut->x = dataOut->x << 4;
      dataOut->y = dataOut->y << 4;
      dataOut->z = dataOut->z << 4;
      break;
  }
}

static void sensorsGyroCalibrate(BiasObj* gyro, uint8_t type) {
  if (gyro->found == 0)
    {
      if (gyro->ongoing == 0)
        {
          sensorsBiasMalloc(gyro);
        }
      /* write directly into buffer */
      sensorsGyroGet(gyro->bufPtr, type);
      /* FIXME: for sensor deck v1 realignment has to be added her */
      sensorsBiasBufPtrIncrement(gyro);

      if (gyro->bufIsFull == 1)
        {
          if (processGyroBias(gyro))
            sensorsBiasFree(gyro);
        }
    }
}

static void __attribute__((used))
sensorsAccelCalibrate(BiasObj* accel, BiasObj* gyro, uint8_t type) {
  if (accel->found == 0)
    {
      if (accel->ongoing == 0)
        {
          sensorsBiasMalloc(accel);
        }
      /* write directly into buffer */
      sensorsAccelGet(accel->bufPtr, type);
      /* FIXME: for sensor deck v1 realignment has to be added her */
      sensorsBiasBufPtrIncrement(accel);
      if ( (accel->bufIsFull == 1) && (gyro->found == 1) )
        {
          processAccelBias(accel);
          switch(type) {
            case SENSORS_BMI160:
              accel->value.z -= SENSORS_BMI160_1G_IN_LSB;
              break;
            case SENSORS_BMI055:
              accel->value.z -= SENSORS_BMI055_1G_IN_LSB;
              break;
          }
          sensorsBiasFree(accel);
        }
    }
}

static void sensorsTask(void *param)
{
  systemWaitStart();

  uint32_t lastWakeTime = xTaskGetTickCount();
  static BiasObj bmi160GyroBias;
  static BiasObj bmi055GyroBias;
#ifdef SENSORS_TAKE_ACCEL_BIAS
  static BiasObj bmi160AccelBias;
  static BiasObj bmi055AccelBias;
#endif
  Axis3i16 gyroPrim;
  Axis3i16 accelPrim;
  Axis3f accelPrimScaled;
  Axis3i16 accelPrimLPF;
  Axis3i32 accelPrimStoredFilterValues;
#ifdef LOG_SEC_IMU
  Axis3i16 gyroSec;
  Axis3i16 accelSec;
  Axis3f accelSecScaled;
  Axis3i16 accelSecLPF;
  Axis3i32 accelSecStoredFilterValues;
#endif /* LOG_SEC_IMU */
  /* wait an additional second the keep bus free
   * this is only required by the z-ranger, since the
   * configuration will be done after system start-up */
  //vTaskDelayUntil(&lastWakeTime, M2T(1500));
  while (1)
    {
      vTaskDelayUntil(&lastWakeTime, F2T(SENSORS_READ_RATE_HZ));
      /* calibrate if necessary */
      if (!allSensorsAreCalibrated)
        {
          if (!bmi160GyroBias.found) {
              sensorsGyroCalibrate(&bmi160GyroBias, SENSORS_BMI160);
#ifdef SENSORS_TAKE_ACCEL_BIAS
              sensorsAccelCalibrate(&bmi160AccelBias,
                                    &bmi160GyroBias, SENSORS_BMI160);
#endif
          }

          if (!bmi055GyroBias.found)
            {
              sensorsGyroCalibrate(&bmi055GyroBias, SENSORS_BMI055);
#ifdef SENSORS_TAKE_ACCEL_BIAS
              sensorsAccelCalibrate(&bmi055AccelBias,
                                    &bmi055GyroBias, SENSORS_BMI055);
#endif
            }
          if ( bmi160GyroBias.found && bmi055GyroBias.found
#ifdef SENSORS_TAKE_ACCEL_BIAS
              && bmi160AccelBias.found && bmi055AccelBias.found
#endif
          )
            {
              // soundSetEffect(SND_CALIB);
              DEBUG_PRINT("Sensor calibration [OK].\n");
              ledseqRun(&seq_calibrated);
              allSensorsAreCalibrated= true;
            }
        }
      else {
          /* get data from chosen sensors */
          sensorsGyroGet(&gyroPrim, gyroPrimInUse);
          sensorsAccelGet(&accelPrim, accelPrimInUse);
#ifdef LOG_SEC_IMU
          sensorsGyroGet(&gyroSec, gyroSecInUse);
          sensorsAccelGet(&accelSec, accelSecInUse);
#endif
          /* FIXME: for sensor deck v1 realignment has to be added her */

          switch(gyroPrimInUse) {
            case SENSORS_BMI160:
              sensorsApplyBiasAndScale(&sensors.gyro, &gyroPrim,
                                       &bmi160GyroBias.value,
                                       SENSORS_BMI160_DEG_PER_LSB_CFG);
              break;
            case SENSORS_BMI055:
              sensorsApplyBiasAndScale(&sensors.gyro, &gyroPrim,
                                       &bmi055GyroBias.value,
                                       SENSORS_BMI055_DEG_PER_LSB_CFG);
              break;
          }

          sensorsAccIIRLPFilter(&accelPrim, &accelPrimLPF,
                                &accelPrimStoredFilterValues,
                                (int32_t)sensorsAccLpfAttFactor);

          switch(accelPrimInUse) {
            case SENSORS_BMI160:
              sensorsApplyBiasAndScale(&accelPrimScaled, &accelPrimLPF,
                                       &bmi160AccelBias.value,
                                       SENSORS_BMI160_G_PER_LSB_CFG);
              break;
            case SENSORS_BMI055:
              sensorsApplyBiasAndScale(&accelPrimScaled, &accelPrimLPF,
                                       &bmi055AccelBias.value,
                                       SENSORS_BMI055_G_PER_LSB_CFG);
              break;
          }

          sensorsAccAlignToGravity(&accelPrimScaled, &sensors.acc);

#ifdef LOG_SEC_IMU
          switch(gyroSecInUse) {
            case SENSORS_BMI160:
              sensorsApplyBiasAndScale(&sensors.gyroSec, &gyroSec,
                                       &bmi160GyroBias.value,
                                       SENSORS_BMI160_DEG_PER_LSB_CFG);
              break;
            case SENSORS_BMI055:
              sensorsApplyBiasAndScale(&sensors.gyroSec, &gyroSec,
                                       &bmi055GyroBias.value,
                                       SENSORS_BMI055_DEG_PER_LSB_CFG);
              break;
          }

          sensorsAccIIRLPFilter(&accelSec, &accelSecLPF,
                                &accelSecStoredFilterValues,
                                (int32_t)sensorsAccLpfAttFactor);

          switch(accelSecInUse) {
            case SENSORS_BMI160:
              sensorsApplyBiasAndScale(&accelSecScaled, &accelSecLPF,
                                       &bmi160AccelBias.value,
                                       SENSORS_BMI160_G_PER_LSB_CFG);
              break;
            case SENSORS_BMI055:
              sensorsApplyBiasAndScale(&accelSecScaled, &accelSecLPF,
                                       &bmi055AccelBias.value,
                                       SENSORS_BMI055_G_PER_LSB_CFG);
              break;
          }

          sensorsAccAlignToGravity(&accelSecScaled, &sensors.accSec);
#endif
      }
      if (isMagnetometerPresent)
        {
          static uint8_t magMeasDelay = SENSORS_DELAY_MAG;

          if (--magMeasDelay == 0)
            {
              bmm150_read_mag_data(&bmm150Dev);
              sensors.mag.x = bmm150Dev.data.x;
              sensors.mag.y = bmm150Dev.data.y;
              sensors.mag.z = bmm150Dev.data.z;
              magMeasDelay = SENSORS_DELAY_MAG;
            }
        }

      if (isBarometerPresent)
        {
          static uint8_t baroMeasDelay = SENSORS_DELAY_BARO;
          static int32_t v_temp_s32;
          static uint32_t v_pres_u32;
          static baro_t* baro280 = &sensors.baro;

          if (--baroMeasDelay == 0)
            {
              bmp280_read_pressure_temperature(&v_pres_u32, &v_temp_s32);
              sensorsScaleBaro(baro280, (float)v_pres_u32, (float)v_temp_s32/100.0f);
              baroMeasDelay = baroMeasDelayMin;
            }
        }
      xQueueOverwrite(accelPrimDataQueue, &sensors.acc);
      xQueueOverwrite(gyroPrimDataQueue, &sensors.gyro);

#ifdef LOG_SEC_IMU
      xQueueOverwrite(gyroSecDataQueue, &sensors.gyroSec);
      xQueueOverwrite(accelSecDataQueue, &sensors.accSec);
#endif

      if (isBarometerPresent)
        {
          xQueueOverwrite(baroPrimDataQueue, &sensors.baro);
        }

      if (isMagnetometerPresent)
        {
          xQueueOverwrite(magPrimDataQueue, &sensors.mag);
        }

      xSemaphoreGive(dataReady);
    }
}

void sensorsBoschWaitDataReady(void)
{
  xSemaphoreTake(dataReady, portMAX_DELAY);
}

static void sensorsBiasMalloc(BiasObj* bias)
{
  /* allocate memory for buffer */
  bias->bufStart =
      pvPortMalloc(SENSORS_NBR_OF_BIAS_SAMPLES * sizeof(Axis3i16));
  bias->bufPtr = bias->bufStart;
  /* set ongoing bit */
  bias->ongoing = 1;
}

static void __attribute__((used)) sensorsBiasReset(BiasObj* bias)
{
  /* unset bias found and buffer full status bits */
  bias->found = 0;
  bias->bufIsFull = 0;
  /* set bufPtr to start to ensure the buffer has to be refilled
   * completly before bufferIsFull Status bit will be set */
  bias->bufPtr = bias->bufStart;
  /* clear any exisiting bias value */
  bias->value.x = 0;
  bias->value.y = 0;
  bias->value.z = 0;
  allSensorsAreCalibrated = false;
}

static void sensorsBiasFree(BiasObj* bias)
{
  /* unset buffer is full */
  bias->bufIsFull = 0;
  /* free buffer memory */
  vPortFree(bias->bufStart);
  bias->bufStart = NULL;
  bias->bufPtr = NULL;
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void sensorsBiasBufPtrIncrement(BiasObj* bias)
{
  bias->bufPtr++;
  if (bias->bufPtr >= bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES)
    {
      bias->bufPtr = bias->bufStart;
      bias->bufIsFull = 1;
    }
}

/**
 * Calculates the mean for the bias buffer.
 */
static void calcMean(BiasObj* bias, Axis3f* mean) {
  Axis3i16* elem;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};

  for (elem = bias->bufStart;
       elem != (bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES); elem++)
    {
      sum[0] += elem->x;
      sum[1] += elem->y;
      sum[2] += elem->z;
    }

  mean->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  mean->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void calcVarianceAndMean(BiasObj* bias, Axis3f* variance, Axis3f* mean)
{
  Axis3i16* elem;
  int64_t sumSquared[GYRO_NBR_OF_AXES] = {0};

  for (elem = bias->bufStart;
      elem != (bias->bufStart+SENSORS_NBR_OF_BIAS_SAMPLES); elem++)
    {
      sumSquared[0] += elem->x * elem->x;
      sumSquared[1] += elem->y * elem->y;
      sumSquared[2] += elem->z * elem->z;
    }
  calcMean(bias, mean);

  variance->x = fabs(sumSquared[0] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->x * mean->x);
  variance->y = fabs(sumSquared[1] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->y * mean->y);
  variance->z = fabs(sumSquared[2] / SENSORS_NBR_OF_BIAS_SAMPLES
                     - mean->z * mean->z);
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool processGyroBias(BiasObj* bias)
{
  Axis3f mean, variance;
  calcVarianceAndMean(bias, &variance, &mean);

  if (variance.x < GYRO_VARIANCE_THRESHOLD_X
      && variance.y < GYRO_VARIANCE_THRESHOLD_Y
      && variance.z < GYRO_VARIANCE_THRESHOLD_Z
      && (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->value.x = (int16_t)(mean.x + 0.5f);
      bias->value.y = (int16_t)(mean.y + 0.5f);
      bias->value.z = (int16_t)(mean.z + 0.5f);
      bias->found = 1;
      return true;
    }
  return false;
}

static void processAccelBias(BiasObj* bias)
{
  Axis3f mean;
  calcMean(bias, &mean);

  varianceSampleTime = xTaskGetTickCount();
  bias->value.x = (int16_t)(mean.x + 0.5f);
  bias->value.y = (int16_t)(mean.y + 0.5f);
  bias->value.z = (int16_t)(mean.z + 0.5f);
  bias->found = 1;
}
static void sensorsApplyBiasAndScale(Axis3f* scaled, Axis3i16* aligned,
                                     Axis3i16* bias, float scale) {
  scaled->x = ((float)aligned->x - (float)bias->x) * scale;
  scaled->y = ((float)aligned->y - (float)bias->y) * scale;
  scaled->z = ((float)aligned->z - (float)bias->z) * scale;
}

static void sensorsScaleBaro(baro_t* baroScaled, float pressure,
                             float temperature) {
  baroScaled->pressure = pressure*0.01f;
  baroScaled->temperature = temperature;
  baroScaled->asl = ((powf((1015.7f / baroScaled->pressure), 0.1902630958f)
      - 1.0f) * (25.0f + 273.15f)) / 0.0065f;
}

bool sensorsBoschReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroPrimDataQueue, gyro, 0));
}

#ifdef LOG_SEC_IMU
bool sensorsReadGyroSec(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroSecDataQueue, gyro, 0));
}

bool sensorsReadAccSec(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelSecDataQueue, acc, 0));
}
#endif

bool sensorsBoschReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelPrimDataQueue, acc, 0));
}

bool sensorsBoschReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magPrimDataQueue, mag, 0));
}

bool sensorsBoschReadBaro(baro_t *baro)
{
  return (pdTRUE == xQueueReceive(baroPrimDataQueue, baro, 0));
}

void sensorsBoschAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  sensorsReadBaro(&sensors->baro);
#ifdef LOG_SEC_IMU
  sensorsReadGyroSec(&sensors->gyroSec);
  sensorsReadAccSec(&sensors->accSec);
#endif
}

bool sensorsBoschAreCalibrated()
{
  return allSensorsAreCalibrated;
}

bool sensorsBoschTest(void)
{
  bool testStatus = true;

  if (!isInit)
    {
      DEBUG_PRINT("Uninitialized\n");
      testStatus = false;
    }

  return testStatus;
}

bool sensorsBoschManufacturingTest(void)
{
  return true;
}

bool sensorsHasBarometer(void)
{
  return isBarometerPresent;
}

bool sensorsHasMangnetometer(void)
{
  return isMagnetometerPresent;
}

static void sensorsAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                                  Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
  Axis3f rx;
  Axis3f ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void sensorsBoschSetAccMode(accModes accMode)
{
  // Difficult to switch mode so do nothing.
  switch (accMode)
  {
    case ACC_MODE_PROPTEST:
      break;
    case ACC_MODE_FLIGHT:
    default:
      break;
  }
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8, BoschGyrSel, &gyroPrimInUse)
PARAM_ADD(PARAM_UINT8, BoschAccSel, &accelPrimInUse)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMM150, &isMagnetometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP285, &isBarometerPresent)
PARAM_GROUP_STOP(imu_sensors)
