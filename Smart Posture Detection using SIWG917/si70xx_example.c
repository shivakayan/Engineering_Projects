/***************************************************************************/ /**
 * @file si70xx_example.c
 * @brief si70xx example APIs
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "rsi_debug.h"
#include "sl_si91x_si70xx.h"
#include "si70xx_example.h"
#include "sl_si91x_driver_gpio.h"
#include <math.h>

/*******************************************************************************
 ***************************  Defines / Macros  ********************************
 ******************************************************************************/

#define TX_THRESHOLD       0                   // tx threshold value
#define RX_THRESHOLD       0                   // rx threshold value
#define I2C                SI70XX_I2C_INSTANCE // I2C instance
#define USER_REG_1         0xBA                // writing data into user register
#define DELAY_PERIODIC_MS1 2000                //sleeptimer1 periodic timeout in ms
#define MODE_0             0                   // Initializing GPIO MODE_0 value
#define OUTPUT_VALUE       1                   // GPIO output value
#define MPU6050_ADDR      0x68                //Addres if MPU6050 Sensor
#define RX_LEN           14    ///< Read buffer length 2 bytes
#define TX_LEN           2    ///< Write buffer length 2 bytes
#define RD_BUF           6    ///< Read buffer length 6 bytes
#define WR_BUF           1    ///< Write buffer length 1 byte
#define MS_DELAY_COUNTER 4600 // Delay count
#define M_PI 3.14159265358979323846

/*******************************************************************************
 ******************************  Data Types  ***********************************
 ******************************************************************************/
/*******************************************************************************
 *************************** LOCAL VARIABLES   *******************************
 ******************************************************************************/
typedef sl_i2c_config_t sl_i2c_configuration_t;
boolean_t delay_timeout = false;     //Indicates sleeptimer1 timeout
/*******************************************************************************
 **********************  Local Function prototypes   ***************************
 ******************************************************************************/
static void i2c_leader_callback(sl_i2c_instance_t i2c_instance, uint32_t status);
sl_status_t si70xx_send_command1(sl_i2c_instance_t, uint8_t, uint32_t*, uint8_t);
void wait_till_i2c_gets_idle1(sl_i2c_instance_t i2c_instance);
void *si70xx_get_i2c_base_address1(sl_i2c_instance_t i2c_instance);
/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/
/*******************************************************************************
 * RHT example initialization function
 ******************************************************************************/
void si70xx_example_init(void)
{
  sl_status_t status;
  uint8_t firm_rev;
  sl_i2c_configuration_t i2c_config;
  uint32_t humidity;
  int32_t temperature;
  uint8_t value;
  i2c_config.mode           = SL_I2C_LEADER_MODE;
  i2c_config.transfer_type  = SL_I2C_USING_NON_DMA;
  i2c_config.operating_mode = SL_I2C_STANDARD_MODE;
  i2c_config.i2c_callback   = i2c_leader_callback;

  do {
#if defined(SENSOR_ENABLE_GPIO_MAPPED_TO_UULP)
    if (sl_si91x_gpio_driver_get_uulp_npss_pin(SENSOR_ENABLE_GPIO_PIN) != 1) {
      // Enable GPIO ULP_CLK
      status = sl_si91x_gpio_driver_enable_clock((sl_si91x_gpio_select_clock_t)ULPCLK_GPIO);
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_si91x_gpio_driver_enable_clock, Error code: %lu", status);
        break;
      }
      DEBUGOUT("GPIO driver clock enable is successful \n");
      // Set NPSS GPIO pin MUX
      status = sl_si91x_gpio_driver_set_uulp_npss_pin_mux(SENSOR_ENABLE_GPIO_PIN, NPSS_GPIO_PIN_MUX_MODE0);
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_si91x_gpio_driver_set_uulp_npss_pin_mux, Error code: %lu", status);
        break;
      }
      DEBUGOUT("GPIO driver uulp pin mux selection is successful \n");
      // Set NPSS GPIO pin direction
      status =
        sl_si91x_gpio_driver_set_uulp_npss_direction(SENSOR_ENABLE_GPIO_PIN, (sl_si91x_gpio_direction_t)GPIO_OUTPUT);
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_si91x_gpio_driver_set_uulp_npss_direction, Error code: %lu", status);
        break;
      }
      DEBUGOUT("GPIO driver uulp pin direction selection is successful \n");
      // Set UULP GPIO pin
      status = sl_si91x_gpio_driver_set_uulp_npss_pin_value(SENSOR_ENABLE_GPIO_PIN, SET);
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_si91x_gpio_driver_set_uulp_npss_pin_value, Error code: %lu", status);
        break;
      }
      DEBUGOUT("GPIO driver set uulp pin value is successful \n");
    }
#else
    sl_gpio_t sensor_enable_port_pin = { SENSOR_ENABLE_GPIO_PORT, SENSOR_ENABLE_GPIO_PIN };
    uint8_t pin_value;

    status = sl_gpio_driver_get_pin(&sensor_enable_port_pin, &pin_value);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_gpio_driver_get_pin, Error code: %lu", status);
      break;
    }
    if (pin_value != 1) {
      // Enable GPIO CLK
#ifdef SENSOR_ENABLE_GPIO_MAPPED_TO_ULP
      status = sl_si91x_gpio_driver_enable_clock((sl_si91x_gpio_select_clock_t)ULPCLK_GPIO);
#else
      status = sl_si91x_gpio_driver_enable_clock((sl_si91x_gpio_select_clock_t)M4CLK_GPIO);
#endif
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_si91x_gpio_driver_enable_clock, Error code: %lu", status);
        break;
      }
      DEBUGOUT("GPIO driver clock enable is successful \n");

      // Set the pin mode for GPIO pins.
      status = sl_gpio_driver_set_pin_mode(&sensor_enable_port_pin, MODE_0, OUTPUT_VALUE);
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_gpio_driver_set_pin_mode, Error code: %lu", status);
        break;
      }
      DEBUGOUT("GPIO driver pin mode select is successful \n");
      // Select the direction of GPIO pin whether Input/ Output
      status = sl_si91x_gpio_driver_set_pin_direction(SENSOR_ENABLE_GPIO_PORT,
                                                      SENSOR_ENABLE_GPIO_PIN,
                                                      (sl_si91x_gpio_direction_t)GPIO_OUTPUT);
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_si91x_gpio_driver_set_pin_direction, Error code: %lu", status);
        break;
      }
      // Set GPIO pin
      status = sl_gpio_driver_set_pin(&sensor_enable_port_pin); // Set ULP GPIO pin
      if (status != SL_STATUS_OK) {
        DEBUGOUT("sl_gpio_driver_set_pin, Error code: %lu", status);
        break;
      }
      DEBUGOUT("GPIO driver set pin value is successful \n");
    }
#endif

    /* Wait for sensor to become ready */
    delay(80);

    //Start 2000 ms periodic timer
    // Initialize I2C bus
    status = sl_i2c_driver_init(I2C, &i2c_config);
    if (status != SL_I2C_SUCCESS) {
      DEBUGOUT("sl_i2c_driver_init : Invalid Parameters, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Successfully initialized and configured i2c leader\n");
    }
    status = sl_i2c_driver_configure_fifo_threshold(I2C, TX_THRESHOLD, RX_THRESHOLD);
    if (status != SL_I2C_SUCCESS) {
      DEBUGOUT("sl_i2c_driver_configure_fifo_threshold : Invalid Parameters, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Successfully configured i2c TX & RX FIFO thresholds\n");
    }
    // reset the sensor
    status = sl_si91x_si70xx_reset(I2C, SI70XX_SLAVE_ADDR);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor reset un-successful, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Successfully reset sensor\n");
    }
    // Initializes sensor and reads electronic ID 1st byte
    status = sl_si91x_si70xx_init(I2C, SI70XX_SLAVE_ADDR, SL_EID_FIRST_BYTE);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor initialization un-successful, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Successfully initialized sensor\n");
    }
    // Initializes sensor and reads electronic ID 2nd byte
    status = sl_si91x_si70xx_init(I2C, SI70XX_SLAVE_ADDR, SL_EID_SECOND_BYTE);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor initialization un-successful, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Successfully reset sensor\n");
    }
    // Get sensor internal firmware version of sensor
    status = sl_si91x_si70xx_get_firmware_revision(I2C, SI70XX_SLAVE_ADDR, &firm_rev);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor firmware version un-successful, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Successfully firmware version of sensor is read\n");
    }
    DEBUGOUT("firmware version:%x\n", firm_rev);
    // write register data into sensor
    status = sl_si91x_si70xx_write_control_register(I2C, SI70XX_SLAVE_ADDR, SL_RH_T_USER_REG, USER_REG_1);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor user register 1 write data failed, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Sensor user register 1 write data is successful\n");
    }
    // Reads register data from sensor
    status = sl_si91x_si70xx_read_control_register(I2C, SI70XX_SLAVE_ADDR, SL_RH_T_USER_REG, &value);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor user register 1 read failed, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Sensor user register 1 read is successful\n");
    }
    DEBUGOUT("user register data:%x\n", value);
    // Reads temperature from humidity from sensor
    status = sl_si91x_si70xx_read_temp_from_rh(I2C, SI70XX_SLAVE_ADDR, &humidity, &temperature);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor temperature read failed, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("Sensor temperature read is successful\n");
    }
    DEBUGOUT("sensor humidity :%ld\n", humidity);
    DEBUGOUT("sensor temperature :%ld\n", temperature);
    // measure humidity data from sensor
    status = sl_si91x_si70xx_measure_humidity(I2C, SI70XX_SLAVE_ADDR, &humidity);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor humidity read failed, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("\nSensor humidity read is successful\n");
    }
    DEBUGOUT("sensor humidity :%ld\n", humidity);
    // measure temperature data from sensor
    status = sl_si91x_si70xx_measure_temperature(I2C, SI70XX_SLAVE_ADDR, &temperature);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("Sensor temperature read failed, Error Code: 0x%ld \n", status);
      break;
    } else {
      DEBUGOUT("\nSensor temperature read is successful\n");
    }
    DEBUGOUT("sensor temperature :%ld\n", temperature);
  } while (false);
}

/*******************************************************************************
 * Function will run continuously in while loop and reads relative humidity and
 * temperature from sensor
 ******************************************************************************/
void si70xx_example_process_action(uint8_t *posture)
{
  sl_status_t status;
  float acceler[3];
  float gyros[3];
  float roll;
  status = mpu6050_read_all_data(I2C, MPU6050_ADDR, acceler, gyros, &roll);
  DEBUGOUT("Roll: %.2f°\n", roll);
  if (status != SL_STATUS_OK) {
    DEBUGOUT("Gyro sensor read, Error Code: 0x%ld \n", status);
  } else {
    DEBUGOUT("\nGyro sensor read is successful\n");
  }
//  printf("Accel (g): X=%.2f, Y=%.2f, Z=%.2f\n", acceler[0], acceler[1], acceler[2]);
//  printf("Gyro (°/s): X=%.2f, Y=%.2f, Z=%.2f\n", gyros[0], gyros[1], gyros[2]);
//  printf("Roll: %.2f°\n", roll);
  if(roll<-90)
    *posture = 1;
  else if(roll<-80)
    *posture = 2;
  else
    *posture = 3;
}

/*******************************************************************************
 * Callback Function
 ******************************************************************************/
void i2c_leader_callback(sl_i2c_instance_t i2c_instance, uint32_t status)
{
  (void)i2c_instance;
  switch (status) {
    case SL_I2C_DATA_TRANSFER_COMPLETE:
      break;
    default:
      break;
  }
}
/*******************************************************************************
* Function to measure light intensity using BH1750 i2c base adress
*******************************************************************************/
void *si70xx_get_i2c_base_address1(sl_i2c_instance_t i2c_instance)
{
  I2C0_Type *i2c = NULL;
  // Updating i2c pointer as per instance number
  if (i2c_instance == SL_I2C0) {
    i2c = ((I2C0_Type *)I2C0_BASE);
  } else if (i2c_instance == SL_I2C1) {
    i2c = ((I2C0_Type *)I2C1_BASE);
  } else if (i2c_instance == SL_ULP_I2C) {
    i2c = ((I2C0_Type *)I2C2_BASE);
  }
  return i2c;
}
/*******************************************************************************
* Function to measure light intensity using BH1750 wait till idle
*******************************************************************************/
void wait_till_i2c_gets_idle1(sl_i2c_instance_t i2c_instance)
{
  I2C0_Type *i2c;

  // Updating i2c pointer as per instance number
  i2c = (I2C0_Type *)si70xx_get_i2c_base_address1(i2c_instance);
  // Checking I2C ACTIVITY bit status
  while (i2c->IC_STATUS_b.ACTIVITY)
    ;
}
/*******************************************************************************
* Function to measure light intensity using BH1750 send command
*******************************************************************************/
sl_status_t si70xx_send_command1(sl_i2c_instance_t i2c_instance, uint8_t addr, uint32_t *data, uint8_t command)
{
  sl_status_t status;
  uint8_t read_buffer_size  = RX_LEN;
  uint8_t write_buffer_size = WR_BUF;
  uint8_t i2c_read_data[read_buffer_size];
  uint8_t i2c_write_data[write_buffer_size];
  i2c_write_data[0] = command;
  // Validate invalid parameters
  if (i2c_instance >= SL_I2C_LAST) {
    return SL_STATUS_INVALID_PARAMETER;
  }
  // Validate NULL parameters
  if (data == NULL) {
    return SL_STATUS_NULL_POINTER;
  }
  // Send command to sensor
  status = sl_i2c_driver_send_data_blocking(i2c_instance, addr, i2c_write_data, write_buffer_size);
  if (status != SL_STATUS_OK) {
    return status;
  }
  wait_till_i2c_gets_idle1(i2c_instance);
  // Receive response from sensor
  status = sl_i2c_driver_receive_data_blocking(i2c_instance, addr, i2c_read_data, read_buffer_size);
  if (status != SL_STATUS_OK) {
    return status;
  }
  wait_till_i2c_gets_idle1(i2c_instance);
  *data = (uint32_t)((i2c_read_data[0] << 8) | (i2c_read_data[1]));
  return SL_STATUS_OK;
}
/*******************************************************************************
* Function to read the mpu6050 init data
*******************************************************************************/
sl_status_t mpu6050_init(void)
{
  sl_i2c_instance_t i2c_instance = I2C;
  uint8_t addr = MPU6050_ADDR;
    sl_status_t status;
    uint8_t i2c_write_data[2];  // Buffer to hold register address and data

    // 1. Wake up MPU6050 (Clear sleep bit in PWR_MGMT_1)
    i2c_write_data[0] = 0x6B;  // PWR_MGMT_1 register
    i2c_write_data[1] = 0x01;  // Use PLL with X-axis gyroscope as clock source
    status = sl_i2c_driver_send_data_blocking(i2c_instance, addr, i2c_write_data, 2);
    if (status != SL_STATUS_OK) return status;

    wait_till_i2c_gets_idle1(i2c_instance);

    // 2. Set accelerometer range to ±2g (ACCEL_CONFIG register)
    i2c_write_data[0] = 0x1C;  // ACCEL_CONFIG register
    i2c_write_data[1] = 0x00;  // ±2g (default)
    status = sl_i2c_driver_send_data_blocking(i2c_instance, addr, i2c_write_data, 2);
    if (status != SL_STATUS_OK) return status;

    wait_till_i2c_gets_idle1(i2c_instance);

    // 3. Set gyroscope range to ±250°/s (GYRO_CONFIG register)
    i2c_write_data[0] = 0x1B;  // GYRO_CONFIG register
    i2c_write_data[1] = 0x00;  // ±250°/s (default)
    status = sl_i2c_driver_send_data_blocking(i2c_instance, addr, i2c_write_data, 2);
    if (status != SL_STATUS_OK) return status;

    wait_till_i2c_gets_idle1(i2c_instance);

    // 4. Configure the digital low-pass filter (DLPF_CFG in CONFIG register)
    i2c_write_data[0] = 0x1A;  // CONFIG register
    i2c_write_data[1] = 0x03;  // DLPF enabled, 44Hz bandwidth
    status = sl_i2c_driver_send_data_blocking(i2c_instance, addr, i2c_write_data, 2);
    if (status != SL_STATUS_OK) return status;

    wait_till_i2c_gets_idle1(i2c_instance);

    // 5. Set sample rate divider (SMPLRT_DIV register)
    i2c_write_data[0] = 0x19;  // SMPLRT_DIV register
    i2c_write_data[1] = 0x07;  // Sample rate = Gyro output rate / (1 + 7) = 1kHz / 8 = 125Hz
    status = sl_i2c_driver_send_data_blocking(i2c_instance, addr, i2c_write_data, 2);
    if (status != SL_STATUS_OK) return status;

    wait_till_i2c_gets_idle1(i2c_instance);

    return SL_STATUS_OK;
}

/*******************************************************************************
* Function to read the mpu6050 data
*******************************************************************************/
sl_status_t mpu6050_read_all_data(sl_i2c_instance_t i2c_instance, uint8_t addr, float *accel, float *gyro, float *roll)
{
    sl_status_t status;
    uint8_t i2c_write_data[1];  // Buffer to hold register address
    uint8_t i2c_read_data[14];  // Buffer to hold received data
    int16_t raw_accel[3], raw_gyro[3];

    i2c_write_data[0] = 0x3B;  // Start reading from ACCEL_XOUT_H

    // Send register address to the MPU6050
    status = sl_i2c_driver_send_data_blocking(i2c_instance, addr, i2c_write_data, 1);
    if (status != SL_STATUS_OK) {
        return status;
    }
    wait_till_i2c_gets_idle1(i2c_instance);

    // Read all 14 bytes from MPU6050 (Accel, Temp, Gyro)
    status = sl_i2c_driver_receive_data_blocking(i2c_instance, addr, i2c_read_data, 14);
    if (status != SL_STATUS_OK) {
        return status;
    }
    wait_till_i2c_gets_idle1(i2c_instance);

    // Convert received data into raw integer values
    raw_accel[0] = (int16_t)((i2c_read_data[0] << 8) | i2c_read_data[1]);  // ACCEL_X
    raw_accel[1] = (int16_t)((i2c_read_data[2] << 8) | i2c_read_data[3]);  // ACCEL_Y
    raw_accel[2] = (int16_t)((i2c_read_data[4] << 8) | i2c_read_data[5]);  // ACCEL_Z

    raw_gyro[0] = (int16_t)((i2c_read_data[8] << 8) | i2c_read_data[9]);   // GYRO_X
    raw_gyro[1] = (int16_t)((i2c_read_data[10] << 8) | i2c_read_data[11]); // GYRO_Y
    raw_gyro[2] = (int16_t)((i2c_read_data[12] << 8) | i2c_read_data[13]); // GYRO_Z

    // Convert raw values to meaningful data (Accelerometer: g, Gyroscope: °/s)
    accel[0] = raw_accel[0] / 16384.0;  // ACCEL_X in g
    accel[1] = raw_accel[1] / 16384.0;  // ACCEL_Y in g
    accel[2] = raw_accel[2] / 16384.0;  // ACCEL_Z in g

    gyro[0] = raw_gyro[0] / 131.0;  // GYRO_X in °/s
    gyro[1] = raw_gyro[1] / 131.0;  // GYRO_Y in °/s
    gyro[2] = raw_gyro[2] / 131.0;  // GYRO_Z in °/s

    // Print data (Optional for debugging)
    printf("Accel (g): X=%.2f, Y=%.2f, Z=%.2f\n", accel[0], accel[1], accel[2]);
    printf("Gyro (°/s): X=%.2f, Y=%.2f, Z=%.2f\n", gyro[0], gyro[1], gyro[2]);
    *roll = atan2(accel[1], accel[2]) * 180.0 / M_PI;

    // Print values
     printf("Roll: %.2f°", *roll);

    return SL_STATUS_OK;
}
/*******************************************************************************
* Function to provide 1 ms Delay
*******************************************************************************/
void delay(uint32_t idelay)
{
  for (uint32_t x = 0; x < 4600 * idelay; x++) //1.002ms delay
  {
    __NOP();
  }
}
