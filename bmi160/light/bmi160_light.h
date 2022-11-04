
#ifndef BMI160_H_
#define BMI160_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define BMI160_GET_BITS(regvar, bitname) \
    ((regvar & bitname##_MSK) >> bitname##_POS)
#define BMI160_SET_BITS(regvar, bitname, val) \
    ((regvar & ~bitname##_MSK) | \
     ((val << bitname##_POS) & bitname##_MSK))
#define BMI160_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))
#define BMI160_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))
#define BMI160_SET_LOW_BYTE                       UINT16_C(0x00FF)
#define BMI160_SET_HIGH_BYTE                      UINT16_C(0xFF00)
#define BMI160_GET_LSB(var)                       (uint8_t)(var & BMI160_SET_LOW_BYTE)
#define BMI160_GET_MSB(var)                       (uint8_t)((var & BMI160_SET_HIGH_BYTE) >> 8)


//settings
#define BMI160_I2C_ADDR                           0x68

#define BMI160_SPI_INTF                           UINT8_C(0)
#define BMI160_I2C_INTF                           UINT8_C(1)

#define BMI160_ACCEL_SEL                          UINT8_C(0x01)
#define BMI160_GYRO_SEL                           UINT8_C(0x02)
#define BMI160_TIME_SEL                           UINT8_C(0x04)

#define BMI160_ACCEL_NORMAL_MODE                  UINT8_C(0x11)
#define BMI160_ACCEL_LOWPOWER_MODE                UINT8_C(0x12)
#define BMI160_ACCEL_SUSPEND_MODE                 UINT8_C(0x10)
#define BMI160_GYRO_SUSPEND_MODE                  UINT8_C(0x14)
#define BMI160_GYRO_NORMAL_MODE                   UINT8_C(0x15)
#define BMI160_GYRO_FASTSTARTUP_MODE              UINT8_C(0x17)
#define BMI160_AUX_SUSPEND_MODE                   UINT8_C(0x18)
#define BMI160_AUX_NORMAL_MODE                    UINT8_C(0x19)
#define BMI160_AUX_LOWPOWER_MODE                  UINT8_C(0x1A)

#define BMI160_ACCEL_RANGE_2G                     UINT8_C(0x03)
#define BMI160_ACCEL_RANGE_4G                     UINT8_C(0x05)
#define BMI160_ACCEL_RANGE_8G                     UINT8_C(0x08)
#define BMI160_ACCEL_RANGE_16G                    UINT8_C(0x0C)
#define BMI160_GYRO_RANGE_2000_DPS                UINT8_C(0x00)
#define BMI160_GYRO_RANGE_1000_DPS                UINT8_C(0x01)
#define BMI160_GYRO_RANGE_500_DPS                 UINT8_C(0x02)
#define BMI160_GYRO_RANGE_250_DPS                 UINT8_C(0x03)
#define BMI160_GYRO_RANGE_125_DPS                 UINT8_C(0x04)
#define BMI160_ACCEL_BW_OSR4_AVG1                 UINT8_C(0x00)
#define BMI160_ACCEL_BW_OSR2_AVG2                 UINT8_C(0x01)
#define BMI160_ACCEL_BW_NORMAL_AVG4               UINT8_C(0x02)
#define BMI160_ACCEL_BW_RES_AVG8                  UINT8_C(0x03)
#define BMI160_ACCEL_BW_RES_AVG16                 UINT8_C(0x04)
#define BMI160_ACCEL_BW_RES_AVG32                 UINT8_C(0x05)
#define BMI160_ACCEL_BW_RES_AVG64                 UINT8_C(0x06)
#define BMI160_ACCEL_BW_RES_AVG128                UINT8_C(0x07)
#define BMI160_GYRO_BW_OSR4_MODE                  UINT8_C(0x00)
#define BMI160_GYRO_BW_OSR2_MODE                  UINT8_C(0x01)
#define BMI160_GYRO_BW_NORMAL_MODE                UINT8_C(0x02)

#define BMI160_ACCEL_ODR_RESERVED                 UINT8_C(0x00)
#define BMI160_ACCEL_ODR_0_78HZ                   UINT8_C(0x01)
#define BMI160_ACCEL_ODR_1_56HZ                   UINT8_C(0x02)
#define BMI160_ACCEL_ODR_3_12HZ                   UINT8_C(0x03)
#define BMI160_ACCEL_ODR_6_25HZ                   UINT8_C(0x04)
#define BMI160_ACCEL_ODR_12_5HZ                   UINT8_C(0x05)
#define BMI160_ACCEL_ODR_25HZ                     UINT8_C(0x06)
#define BMI160_ACCEL_ODR_50HZ                     UINT8_C(0x07)
#define BMI160_ACCEL_ODR_100HZ                    UINT8_C(0x08)
#define BMI160_ACCEL_ODR_200HZ                    UINT8_C(0x09)
#define BMI160_ACCEL_ODR_400HZ                    UINT8_C(0x0A)
#define BMI160_ACCEL_ODR_800HZ                    UINT8_C(0x0B)
#define BMI160_ACCEL_ODR_1600HZ                   UINT8_C(0x0C)
#define BMI160_ACCEL_ODR_RESERVED0                UINT8_C(0x0D)
#define BMI160_ACCEL_ODR_RESERVED1                UINT8_C(0x0E)
#define BMI160_ACCEL_ODR_RESERVED2                UINT8_C(0x0F)
#define BMI160_GYRO_ODR_RESERVED                  UINT8_C(0x00)

#define BMI160_GYRO_ODR_25HZ                      UINT8_C(0x06)
#define BMI160_GYRO_ODR_50HZ                      UINT8_C(0x07)
#define BMI160_GYRO_ODR_100HZ                     UINT8_C(0x08)
#define BMI160_GYRO_ODR_200HZ                     UINT8_C(0x09)
#define BMI160_GYRO_ODR_400HZ                     UINT8_C(0x0A)
#define BMI160_GYRO_ODR_800HZ                     UINT8_C(0x0B)
#define BMI160_GYRO_ODR_1600HZ                    UINT8_C(0x0C)
#define BMI160_GYRO_ODR_3200HZ                    UINT8_C(0x0D)

//address macro
#define BMI160_CHIP_ID                            UINT8_C(0xD1)
#define BMI160_SOFT_RESET_CMD                     UINT8_C(0xb6)
#define BMI160_SOFT_RESET_DELAY_MS                UINT8_C(1)
#define BMI160_START_FOC_CMD                      UINT8_C(0x03)
#define BMI160_NVM_BACKUP_EN                      UINT8_C(0xA0)
#define BMI160_ACCEL_DELAY_MS                     UINT8_C(5)
#define BMI160_GYRO_DELAY_MS                      UINT8_C(80)
#define BMI160_ONE_MS_DELAY                       UINT8_C(1)

#define BMI160_STATUS_ADDR                        UINT8_C(0x1B)
#define BMI160_ACCEL_CONFIG_ADDR                  UINT8_C(0x40)
#define BMI160_ACCEL_RANGE_ADDR                   UINT8_C(0x41)
#define BMI160_GYRO_CONFIG_ADDR                   UINT8_C(0x42)
#define BMI160_GYRO_RANGE_ADDR                    UINT8_C(0x43)
#define BMI160_OFFSET_CONF_ADDR                   UINT8_C(0x77)
#define BMI160_COMMAND_REG_ADDR                   UINT8_C(0x7E)
#define BMI160_INT_DATA_0_ADDR                    UINT8_C(0x58)
#define BMI160_INT_DATA_1_ADDR                    UINT8_C(0x59)
#define BMI160_FOC_CONF_ADDR                      UINT8_C(0x69)
#define BMI160_CONF_ADDR                          UINT8_C(0x6A)
#define BMI160_IF_CONF_ADDR                       UINT8_C(0x6B)
#define BMI160_SPI_COMM_TEST_ADDR                 UINT8_C(0x7F)
#define BMI160_OFFSET_ADDR                        UINT8_C(0x71)
#define BMI160_OFFSET_CONF_ADDR                   UINT8_C(0x77)
#define BMI160_CHIP_ID_ADDR                       UINT8_C(0x00)
#define BMI160_ERROR_REG_ADDR                     UINT8_C(0x02)
#define BMI160_PMU_STATUS_ADDR                    UINT8_C(0x03)
#define BMI160_GYRO_DATA_ADDR                     UINT8_C(0x0C)
#define BMI160_ACCEL_DATA_ADDR                    UINT8_C(0x12)

//data macro

#define BMI160_ENABLE                             UINT8_C(0x01)
#define BMI160_DISABLE                            UINT8_C(0x00)

#define FIFO_CONFIG_MSB_CHECK                     UINT8_C(0x80)
#define FIFO_CONFIG_LSB_CHECK                     UINT8_C(0x00)
#define BMI160_FOC_ACCEL_DISABLED                 UINT8_C(0x00)
#define BMI160_FOC_ACCEL_POSITIVE_G               UINT8_C(0x01)
#define BMI160_FOC_ACCEL_NEGATIVE_G               UINT8_C(0x02)
#define BMI160_FOC_ACCEL_0G                       UINT8_C(0x03)
#define BMI160_SENSOR_TIME_LENGTH                 UINT8_C(3)
#define BMI160_SENSOR_TIME_LSB_BYTE               UINT8_C(0)
#define BMI160_SENSOR_TIME_XLSB_BYTE              UINT8_C(1)
#define BMI160_SENSOR_TIME_MSB_BYTE               UINT8_C(2)

//mask and pos macro
#define BMI160_MANUAL_MODE_EN_POS                 UINT8_C(7)
#define BMI160_MANUAL_MODE_EN_MSK                 UINT8_C(0x80)
#define BMI160_ACCEL_BW_MASK                      UINT8_C(0x70)
#define BMI160_ACCEL_ODR_MASK                     UINT8_C(0x0F)
#define BMI160_ACCEL_UNDERSAMPLING_MASK           UINT8_C(0x80)
#define BMI160_ACCEL_RANGE_MASK                   UINT8_C(0x0F)
#define BMI160_GYRO_BW_MASK                       UINT8_C(0x30)
#define BMI160_GYRO_ODR_MASK                      UINT8_C(0x0F)
#define BMI160_GYRO_RANGE_MASK                    UINT8_C(0x07)
#define BMI160_ACCEL_BW_POS                       UINT8_C(4)
#define BMI160_GYRO_BW_POS                        UINT8_C(4)
#define BMI160_SPI_RD_MASK                        UINT8_C(0x80)
#define BMI160_SPI_WR_MASK                        UINT8_C(0x7F)
#define BMI160_SEN_SEL_MASK                       UINT8_C(0x07)
#define BMI160_ERR_REG_MASK                       UINT8_C(0x0F)
#define BMI160_GYRO_FOC_EN_POS                    UINT8_C(6)
#define BMI160_GYRO_FOC_EN_MSK                    UINT8_C(0x40)
#define BMI160_ACCEL_FOC_X_CONF_POS               UINT8_C(4)
#define BMI160_ACCEL_FOC_X_CONF_MSK               UINT8_C(0x30)
#define BMI160_ACCEL_FOC_Y_CONF_POS               UINT8_C(2)
#define BMI160_ACCEL_FOC_Y_CONF_MSK               UINT8_C(0x0C)
#define BMI160_ACCEL_FOC_Z_CONF_MSK               UINT8_C(0x03)
#define BMI160_FOC_STATUS_POS                     UINT8_C(3)
#define BMI160_FOC_STATUS_MSK                     UINT8_C(0x08)
#define BMI160_GYRO_OFFSET_X_MSK                  UINT8_C(0x03)
#define BMI160_GYRO_OFFSET_Y_POS                  UINT8_C(2)
#define BMI160_GYRO_OFFSET_Y_MSK                  UINT8_C(0x0C)
#define BMI160_GYRO_OFFSET_Z_POS                  UINT8_C(4)
#define BMI160_GYRO_OFFSET_Z_MSK                  UINT8_C(0x30)
#define BMI160_GYRO_OFFSET_EN_POS                 UINT8_C(7)
#define BMI160_GYRO_OFFSET_EN_MSK                 UINT8_C(0x80)
#define BMI160_ACCEL_OFFSET_EN_POS                UINT8_C(6)
#define BMI160_ACCEL_OFFSET_EN_MSK                UINT8_C(0x40)
#define BMI160_GYRO_OFFSET_POS                    UINT16_C(8)
#define BMI160_GYRO_OFFSET_MSK                    UINT16_C(0x0300)
#define BMI160_NVM_UPDATE_POS                     UINT8_C(1)
#define BMI160_NVM_UPDATE_MSK                     UINT8_C(0x02)
#define BMI160_NVM_STATUS_POS                     UINT8_C(4)
#define BMI160_NVM_STATUS_MSK                     UINT8_C(0x10)
#define BMI160_MAG_POWER_MODE_MSK                 UINT8_C(0x03)
#define BMI160_ACCEL_POWER_MODE_MSK               UINT8_C(0x30)
#define BMI160_ACCEL_POWER_MODE_POS               UINT8_C(4)
#define BMI160_GYRO_POWER_MODE_MSK                UINT8_C(0x0C)
#define BMI160_GYRO_POWER_MODE_POS                UINT8_C(2)

//status macro
#define BMI160_OK                                 INT8_C(0x00)
#define BMI160_E_NULL_PTR                         INT8_C(-1)
#define BMI160_E_COM_FAIL                         INT8_C(-2)
#define BMI160_E_DEV_NOT_FOUND                    INT8_C(-3)
#define BMI160_E_OUT_OF_RANGE                     INT8_C(-4)
#define BMI160_E_INVALID_INPUT                    INT8_C(-5)
#define BMI160_E_ACCEL_ODR_BW_INVALID             INT8_C(-6)
#define BMI160_E_GYRO_ODR_BW_INVALID              INT8_C(-7)
#define BMI160_E_LWP_PRE_FLTR_INT_INVALID         INT8_C(-8)
#define BMI160_E_LWP_PRE_FLTR_INVALID             INT8_C(-9)
#define BMI160_E_FOC_FAILURE                      INT8_C(-11)
#define BMI160_E_READ_WRITE_LENGTH_INVALID        INT8_C(-12)
#define BMI160_E_INVALID_CONFIG                   INT8_C(-13)


typedef int8_t (*bmi160_read_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef int8_t (*bmi160_write_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *read_data, uint16_t len);
typedef void (*bmi160_delay_fptr_t)(uint32_t ms);
 
struct bmi160_foc_conf{
    uint8_t foc_gyr_en;    
    uint8_t foc_acc_x;
    uint8_t foc_acc_y;
    uint8_t foc_acc_z;    
    uint8_t acc_off_en;    
    uint8_t gyro_off_en;
};

struct bmi160_offsets{
    int8_t off_acc_x;    
    int8_t off_acc_y;    
    int8_t off_acc_z;    
    int16_t off_gyro_x;    
    int16_t off_gyro_y;    
    int16_t off_gyro_z;
};

struct bmi160_sensor_data{
    int16_t x;    
    int16_t y;    
    int16_t z;    
    uint32_t sensortime;
};

enum bmi160_select_sensor {
    BMI160_ACCEL_ONLY = 1,
    BMI160_GYRO_ONLY,
    BMI160_BOTH_ACCEL_AND_GYRO
};

enum bmi160_step_detect_mode {
    BMI160_STEP_DETECT_NORMAL,
    BMI160_STEP_DETECT_SENSITIVE,
    BMI160_STEP_DETECT_ROBUST,    
    BMI160_STEP_DETECT_USER_DEFINE
};

struct bmi160_cfg{
    uint8_t power;    
    uint8_t odr;    
    uint8_t range;    
    uint8_t bw;
};

enum bmi160_any_sig_motion_active_interrupt_state {
    BMI160_BOTH_ANY_SIG_MOTION_DISABLED = -1,    
    BMI160_ANY_MOTION_ENABLED,    
    BMI160_SIG_MOTION_ENABLED
};
 
struct bmi160_dev{
    uint8_t chip_id;    
    uint8_t id;    
    uint8_t intf;    
    enum bmi160_any_sig_motion_active_interrupt_state any_sig_sel;    
    struct bmi160_cfg accel_cfg;    
    struct bmi160_cfg prev_accel_cfg;    
    struct bmi160_cfg gyro_cfg;    
    struct bmi160_cfg prev_gyro_cfg;
    bmi160_read_fptr_t read;    
    bmi160_write_fptr_t write;    
    bmi160_delay_fptr_t delay_ms;   
}; 

int8_t bmi160_set_offsets(const struct bmi160_foc_conf *foc_conf, const struct bmi160_offsets *offset);
int8_t bmi160_get_regs(uint16_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160_set_regs(uint16_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160_get_offsets(struct bmi160_offsets *offset);
int8_t bmi160_set_sens_conf(void);
int8_t bmi160_get_sens_conf(void);
int8_t bmi160_set_power_mode(void);
int8_t bmi160_get_power_mode(void);
int8_t bmi160_soft_reset(void);

int8_t bmi160_init(struct bmi160_dev *dev);
int8_t bmi160_get_sensor_data(uint8_t select_sensor, struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro);
int8_t bmi160_start_foc(const struct bmi160_foc_conf *foc_conf, struct bmi160_offsets *offset);
int8_t bmi160_update_nvm(void);


/*************user code****************

int8_t init_bmi160(struct bmi160_offsets *offset);
int8_t bmi_clibration(struct bmi160_offsets *offset, struct bmi160_foc_conf *foc_confg);*/

#ifdef __cplusplus
}
#endif
#endif /* BMI160_H_ */
														 
