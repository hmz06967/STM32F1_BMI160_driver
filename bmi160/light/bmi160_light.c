#include "bmi160.h"

/**********************************************************************************************************************************/

struct bmi160_dev * bmi160_dev;

/*
int8_t init_bmi160(struct bmi160_offsets *offset){
    int8_t rslt;

    struct bmi160_dev bmi160dev;
	bmi160dev.write = i2c_write;
	bmi160dev.read = i2c_read;
	bmi160dev.delay_ms = delay;
	bmi160dev.id = BMI160_I2C_ADDR;
	bmi160dev.intf = BMI160_I2C_INTF;
    rslt = bmi160_init(&bmi160dev);

    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_50HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    // Set the sensor configuration 
    if(rslt=BMI160_OK)
        rslt = bmi160_set_sens_conf();
		
    if (rslt != BMI160_OK){
        failureMode();
    }else{
        struct bmi160_foc_conf foc_confg;
        rslt = bmi_clibration(offset, &foc_confg);
    }
    return rslt;
}

int8_t bmi_clibration(struct bmi160_offsets *offset, struct bmi160_foc_conf *foc_confg){
    int8_t rslt;
    rslt = bmi160_start_foc(foc_confg, offset);
    if(rslt == BMI160_OK)
	    rslt = bmi160_get_offsets(offset);
    if(rslt == BMI160_OK)
	    rslt = bmi160_update_nvm();
    return rslt;
}
*/
/**********************************************************************************************************************************/

static void default_param_settg(void);
static int8_t null_ptr_check(void);
static int8_t set_accel_conf(void);
static int8_t get_accel_conf(void);
static int8_t set_gyro_conf(void);
static int8_t get_gyro_conf(void);
static int8_t check_accel_config(uint8_t * data);
static int8_t check_invalid_settg(void);
static int8_t check_gyro_config(uint8_t * data);
static int8_t set_gyro_pwr(void);
static int8_t set_accel_pwr(void);
static int8_t process_accel_odr(uint8_t * data);
static int8_t process_accel_bw(uint8_t * data);
static int8_t process_accel_range(uint8_t * data);
static int8_t process_gyro_odr(uint8_t * data);
static int8_t process_gyro_bw(uint8_t * data);
static int8_t process_gyro_range(uint8_t * data);
static int8_t process_under_sampling(uint8_t * data);
static int8_t get_accel_data(uint8_t len, struct bmi160_sensor_data * accel);
static int8_t get_gyro_data(uint8_t len, struct bmi160_sensor_data * gyro);
static int8_t get_accel_gyro_data(uint8_t len, struct bmi160_sensor_data * accel, struct bmi160_sensor_data * gyro);
static int8_t get_foc_status(uint8_t * foc_status);
static int8_t configure_offset_enable(const struct bmi160_foc_conf * foc_conf);
static int8_t trigger_foc(struct bmi160_offsets * offset);

/**********************************************************************************************************************************/

int8_t bmi160_get_regs(uint16_t reg_addr, uint8_t * data, uint16_t len) {
    int8_t rslt = BMI160_OK;
    if ((bmi160_dev == NULL) || (bmi160_dev -> read == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else if (len == 0) {
        rslt = BMI160_E_READ_WRITE_LENGTH_INVALID;
    } else {
        if (bmi160_dev -> intf == BMI160_SPI_INTF) {
            reg_addr = (reg_addr | BMI160_SPI_RD_MASK);
        }
        rslt = bmi160_dev -> read((bmi160_dev -> id << 1), reg_addr, (uint8_t * ) data, len);
    }
    return rslt;
}

int8_t bmi160_set_regs(uint16_t reg_addr, uint8_t * data, uint16_t len) {
    int8_t rslt = BMI160_OK;
    uint8_t count = 0;
    if ((bmi160_dev == NULL) || (bmi160_dev -> write == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else if (len == 0) {
        rslt = BMI160_E_READ_WRITE_LENGTH_INVALID;
    } else {
        if (bmi160_dev -> intf == BMI160_SPI_INTF) {
            reg_addr = (reg_addr & BMI160_SPI_WR_MASK);
        }
        if ((bmi160_dev -> prev_accel_cfg.power == BMI160_ACCEL_NORMAL_MODE) || (bmi160_dev -> prev_gyro_cfg.power == BMI160_GYRO_NORMAL_MODE)) {
            rslt = bmi160_dev -> write((bmi160_dev -> id << 1), reg_addr, data, len);
            bmi160_dev -> delay_ms(5);
        } else {
            for (; count < len; count++) {
                rslt = bmi160_dev -> write((bmi160_dev -> id << 1), reg_addr, & data[count], len);
                reg_addr++;
                bmi160_dev -> delay_ms(1);
            }
        }
        if (rslt != BMI160_OK) {
            rslt = BMI160_E_COM_FAIL;
        }
    }
    return rslt;
}

int8_t bmi160_init(struct bmi160_dev * bmidev) {
    bmi160_dev = bmidev;
    int8_t rslt;
    uint8_t data;
    uint8_t
    try = 3;
    rslt = null_ptr_check();
    bmi160_dev -> chip_id = 0;
    if ((rslt == BMI160_OK) && (bmi160_dev -> intf == BMI160_SPI_INTF)) {
        rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, & data, 1);
    }
    if (rslt == BMI160_OK) {
        while ((try --) && (bmi160_dev -> chip_id != BMI160_CHIP_ID)) {
            rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR, & bmi160_dev -> chip_id, 1);
        }
        if ((rslt == BMI160_OK) && (bmi160_dev -> chip_id == BMI160_CHIP_ID)) {
            bmi160_dev -> any_sig_sel = BMI160_BOTH_ANY_SIG_MOTION_DISABLED;
            rslt = bmi160_soft_reset();
        } else {
            rslt = BMI160_E_DEV_NOT_FOUND;
        }
    }
    return rslt;
}
int8_t bmi160_soft_reset() {
    int8_t rslt = BMI160_OK;
    uint8_t data = BMI160_SOFT_RESET_CMD;
    if ((bmi160_dev == NULL) || (bmi160_dev -> delay_ms == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        bmi160_dev -> delay_ms(BMI160_SOFT_RESET_DELAY_MS);
        if ((rslt == BMI160_OK) && (bmi160_dev -> intf == BMI160_SPI_INTF)) {
            rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, & data, 1);
        }
        if (rslt == BMI160_OK) {
            default_param_settg();
        }
    }
    return rslt;
}
int8_t bmi160_set_sens_conf() {
    int8_t rslt = BMI160_OK;
    if ((bmi160_dev == NULL) || (bmi160_dev -> delay_ms == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = set_accel_conf();
        if (rslt == BMI160_OK) {
            rslt = set_gyro_conf();
            if (rslt == BMI160_OK) {
                rslt = bmi160_set_power_mode();
                if (rslt == BMI160_OK) {
                    rslt = check_invalid_settg();
                }
            }
        }
    }
    return rslt;
}
int8_t bmi160_get_sens_conf() {
    int8_t rslt = BMI160_OK;
    if ((bmi160_dev == NULL) || (bmi160_dev -> delay_ms == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = get_accel_conf();
        if (rslt == BMI160_OK) {
            rslt = get_gyro_conf();
        }
    }
    return rslt;
}
int8_t bmi160_set_power_mode() {
    int8_t rslt = 0;
    if ((bmi160_dev == NULL) || (bmi160_dev -> delay_ms == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = set_accel_pwr();
        if (rslt == BMI160_OK) {
            rslt = set_gyro_pwr();
        }
    }
    return rslt;
}
int8_t bmi160_get_power_mode() {
    int8_t rslt = 0;
    uint8_t power_mode = 0;
    if ((bmi160_dev == NULL) || (bmi160_dev -> delay_ms == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = bmi160_get_regs(BMI160_PMU_STATUS_ADDR, & power_mode, 1);
        if (rslt == BMI160_OK) {
            bmi160_dev -> gyro_cfg.power = BMI160_GET_BITS(power_mode, BMI160_GYRO_POWER_MODE);
            bmi160_dev -> accel_cfg.power = BMI160_GET_BITS(power_mode, BMI160_ACCEL_POWER_MODE);
        }
    }
    return rslt;
}
int8_t bmi160_get_sensor_data(uint8_t select_sensor, struct bmi160_sensor_data * accel, struct bmi160_sensor_data * gyro) {
    int8_t rslt = BMI160_OK;
    uint8_t time_sel;
    uint8_t sen_sel;
    uint8_t len = 0;
    sen_sel = select_sensor & BMI160_SEN_SEL_MASK;
    time_sel = ((sen_sel & BMI160_TIME_SEL) >> 2);
    sen_sel = sen_sel & (BMI160_ACCEL_SEL | BMI160_GYRO_SEL);
    if (time_sel == 1) {
        len = 3;
    }
    if (bmi160_dev != NULL) {
        switch (sen_sel) {
        case BMI160_ACCEL_ONLY:
            if (accel == NULL) {
                rslt = BMI160_E_NULL_PTR;
            } else {
                rslt = get_accel_data(len, accel);
            }
            break;
        case BMI160_GYRO_ONLY:
            if (gyro == NULL) {
                rslt = BMI160_E_NULL_PTR;
            } else {
                rslt = get_gyro_data(len, gyro);
            }
            break;
        case BMI160_BOTH_ACCEL_AND_GYRO:
            if ((gyro == NULL) || (accel == NULL)) {
                rslt = BMI160_E_NULL_PTR;
            } else {
                rslt = get_accel_gyro_data(len, accel, gyro);
            }
            break;
        default:
            rslt = BMI160_E_INVALID_INPUT;
            break;
        }
    } else {
        rslt = BMI160_E_NULL_PTR;
    }
    return rslt;
}


int8_t bmi160_start_foc(const struct bmi160_foc_conf * foc_conf, struct bmi160_offsets * offset) {
    int8_t rslt;
    uint8_t data;
    rslt = null_ptr_check();
    if (rslt != BMI160_OK) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = configure_offset_enable(foc_conf);
        if (rslt == BMI160_OK) {
            rslt = bmi160_get_regs(BMI160_FOC_CONF_ADDR, & data, 1);
            data = BMI160_SET_BITS(data, BMI160_GYRO_FOC_EN, foc_conf -> foc_gyr_en);
            data = BMI160_SET_BITS(data, BMI160_ACCEL_FOC_X_CONF, foc_conf -> foc_acc_x);
            data = BMI160_SET_BITS(data, BMI160_ACCEL_FOC_Y_CONF, foc_conf -> foc_acc_y);
            data = BMI160_SET_BITS_POS_0(data, BMI160_ACCEL_FOC_Z_CONF, foc_conf -> foc_acc_z);
            if (rslt == BMI160_OK) {
                rslt = bmi160_set_regs(BMI160_FOC_CONF_ADDR, & data, 1);
                if (rslt == BMI160_OK) {
                    rslt = trigger_foc(offset);
                }
            }
        }
    }
    return rslt;
}
int8_t bmi160_get_offsets(struct bmi160_offsets * offset) {
    int8_t rslt;
    uint8_t data[7];
    uint8_t lsb, msb;
    int16_t offset_msb, offset_lsb;
    int16_t offset_data;
    rslt = null_ptr_check();
    if (rslt != BMI160_OK) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = bmi160_get_regs(BMI160_OFFSET_ADDR, data, 7);
        offset -> off_acc_x = (int8_t) data[0];
        offset -> off_acc_y = (int8_t) data[1];
        offset -> off_acc_z = (int8_t) data[2];
        lsb = data[3];
        msb = BMI160_GET_BITS_POS_0(data[6], BMI160_GYRO_OFFSET_X);
        offset_msb = (int16_t)(msb << 14);
        offset_lsb = lsb << 6;
        offset_data = offset_msb | offset_lsb;
        offset -> off_gyro_x = (int16_t)(offset_data / 64);
        lsb = data[4];
        msb = BMI160_GET_BITS(data[6], BMI160_GYRO_OFFSET_Y);
        offset_msb = (int16_t)(msb << 14);
        offset_lsb = lsb << 6;
        offset_data = offset_msb | offset_lsb;
        offset -> off_gyro_y = (int16_t)(offset_data / 64);
        lsb = data[5];
        msb = BMI160_GET_BITS(data[6], BMI160_GYRO_OFFSET_Z);
        offset_msb = (int16_t)(msb << 14);
        offset_lsb = lsb << 6;
        offset_data = offset_msb | offset_lsb;
        offset -> off_gyro_z = (int16_t)(offset_data / 64);
    }
    return rslt;
}
int8_t bmi160_set_offsets(const struct bmi160_foc_conf * foc_conf,
    const struct bmi160_offsets * offset) {
    int8_t rslt;
    uint8_t data[7];
    uint8_t x_msb, y_msb, z_msb;
    rslt = null_ptr_check();
    if (rslt != BMI160_OK) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        data[0] = (uint8_t) offset -> off_acc_x;
        data[1] = (uint8_t) offset -> off_acc_y;
        data[2] = (uint8_t) offset -> off_acc_z;
        data[3] = BMI160_GET_LSB(offset -> off_gyro_x);
        data[4] = BMI160_GET_LSB(offset -> off_gyro_y);
        data[5] = BMI160_GET_LSB(offset -> off_gyro_z);
        x_msb = BMI160_GET_BITS(offset -> off_gyro_x, BMI160_GYRO_OFFSET);
        y_msb = BMI160_GET_BITS(offset -> off_gyro_y, BMI160_GYRO_OFFSET);
        z_msb = BMI160_GET_BITS(offset -> off_gyro_z, BMI160_GYRO_OFFSET);
        data[6] = (uint8_t)(z_msb << 4 | y_msb << 2 | x_msb);
        data[6] = BMI160_SET_BITS(data[6], BMI160_GYRO_OFFSET_EN, foc_conf -> gyro_off_en);
        data[6] = BMI160_SET_BITS(data[6], BMI160_ACCEL_OFFSET_EN, foc_conf -> acc_off_en);
        rslt = bmi160_set_regs(BMI160_OFFSET_ADDR, data, 7);
    }
    return rslt;
}
int8_t bmi160_update_nvm() {
    int8_t rslt;
    uint8_t data;
    uint8_t cmd = BMI160_NVM_BACKUP_EN;
    rslt = bmi160_get_regs(BMI160_CONF_ADDR, & data, 1);
    if (rslt == BMI160_OK) {
        data = BMI160_SET_BITS(data, BMI160_NVM_UPDATE, 1);
        rslt = bmi160_set_regs(BMI160_CONF_ADDR, & data, 1);
        if (rslt == BMI160_OK) {
            rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, & cmd, 1);
            if (rslt == BMI160_OK) {
                rslt = bmi160_get_regs(BMI160_STATUS_ADDR, & data, 1);
                if (rslt == BMI160_OK) {
                    data = BMI160_GET_BITS(data, BMI160_NVM_STATUS);
                    if (data != BMI160_ENABLE) {
                        bmi160_dev -> delay_ms(25);
                    }
                }
            }
        }
    }
    return rslt;
}
static int8_t null_ptr_check() {
    int8_t rslt;
    if ((bmi160_dev == NULL) || (bmi160_dev -> read == NULL) || (bmi160_dev -> write == NULL) || (bmi160_dev -> delay_ms == NULL)) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = BMI160_OK;
    }
    return rslt;
}
static void default_param_settg() {
    bmi160_dev -> accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi160_dev -> accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    bmi160_dev -> accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
    bmi160_dev -> accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160_dev -> gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi160_dev -> gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    bmi160_dev -> gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
    bmi160_dev -> gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160_dev -> prev_accel_cfg = bmi160_dev -> accel_cfg;
    bmi160_dev -> prev_gyro_cfg = bmi160_dev -> gyro_cfg;
}
static int8_t set_accel_conf() {
    int8_t rslt;
    uint8_t data[2] = {
        0
    };
    rslt = check_accel_config(data);
    if (rslt == BMI160_OK) {
        rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, & data[0], 1);
        if (rslt == BMI160_OK) {
            bmi160_dev -> prev_accel_cfg.odr = bmi160_dev -> accel_cfg.odr;
            bmi160_dev -> prev_accel_cfg.bw = bmi160_dev -> accel_cfg.bw;
            rslt = bmi160_set_regs(BMI160_ACCEL_RANGE_ADDR, & data[1], 1);
            if (rslt == BMI160_OK) {
                bmi160_dev -> prev_accel_cfg.range = bmi160_dev -> accel_cfg.range;
            }
        }
    }
    return rslt;
}
static int8_t get_accel_conf() {
    int8_t rslt;
    uint8_t data[2] = {
        0
    };
    rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 2);
    if (rslt == BMI160_OK) {
        bmi160_dev -> accel_cfg.odr = (data[0] & BMI160_ACCEL_ODR_MASK);
        bmi160_dev -> accel_cfg.bw = (data[0] & BMI160_ACCEL_BW_MASK) >> BMI160_ACCEL_BW_POS;
        bmi160_dev -> accel_cfg.range = (data[1] & BMI160_ACCEL_RANGE_MASK);
    }
    return rslt;
}
static int8_t check_accel_config(uint8_t * data) {
    int8_t rslt;
    rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 2);
    if (rslt == BMI160_OK) {
        rslt = process_accel_odr( & data[0]);
        if (rslt == BMI160_OK) {
            rslt = process_accel_bw( & data[0]);
            if (rslt == BMI160_OK) {
                rslt = process_accel_range( & data[1]);
            }
        }
    }
    return rslt;
}
static int8_t process_accel_odr(uint8_t * data) {
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t odr = 0;
    if (bmi160_dev -> accel_cfg.odr <= BMI160_ACCEL_ODR_1600HZ) {
        if (bmi160_dev -> accel_cfg.odr != bmi160_dev -> prev_accel_cfg.odr) {
            odr = (uint8_t) bmi160_dev -> accel_cfg.odr;
            temp = * data & ~BMI160_ACCEL_ODR_MASK;
            * data = temp | (odr & BMI160_ACCEL_ODR_MASK);
        }
    } else {
        rslt = BMI160_E_OUT_OF_RANGE;
    }
    return rslt;
}
static int8_t process_accel_bw(uint8_t * data) {
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t bw = 0;
    if (bmi160_dev -> accel_cfg.bw <= BMI160_ACCEL_BW_RES_AVG128) {
        if (bmi160_dev -> accel_cfg.bw != bmi160_dev -> prev_accel_cfg.bw) {
            bw = (uint8_t) bmi160_dev -> accel_cfg.bw;
            temp = * data & ~BMI160_ACCEL_BW_MASK;
            * data = temp | ((bw << 4) & BMI160_ACCEL_BW_MASK);
        }
    } else {
        rslt = BMI160_E_OUT_OF_RANGE;
    }
    return rslt;
}
static int8_t process_accel_range(uint8_t * data) {
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t range = 0;
    if (bmi160_dev -> accel_cfg.range <= BMI160_ACCEL_RANGE_16G) {
        if (bmi160_dev -> accel_cfg.range != bmi160_dev -> prev_accel_cfg.range) {
            range = (uint8_t) bmi160_dev -> accel_cfg.range;
            temp = * data & ~BMI160_ACCEL_RANGE_MASK;
            * data = temp | (range & BMI160_ACCEL_RANGE_MASK);
        }
    } else {
        rslt = BMI160_E_OUT_OF_RANGE;
    }
    return rslt;
}
static int8_t check_invalid_settg() {
    int8_t rslt;
    uint8_t data = 0;
    rslt = bmi160_get_regs(BMI160_ERROR_REG_ADDR, & data, 1);
    data = data >> 1;
    data = data & BMI160_ERR_REG_MASK;
    if (data == 1) {
        rslt = BMI160_E_ACCEL_ODR_BW_INVALID;
    } else if (data == 2) {
        rslt = BMI160_E_GYRO_ODR_BW_INVALID;
    } else if (data == 3) {
        rslt = BMI160_E_LWP_PRE_FLTR_INT_INVALID;
    } else if (data == 7) {
        rslt = BMI160_E_LWP_PRE_FLTR_INVALID;
    }
    return rslt;
}
static int8_t set_gyro_conf() {
    int8_t rslt;
    uint8_t data[2] = {
        0
    };
    rslt = check_gyro_config(data);
    if (rslt == BMI160_OK) {
        rslt = bmi160_set_regs(BMI160_GYRO_CONFIG_ADDR, & data[0], 1);
        if (rslt == BMI160_OK) {
            bmi160_dev -> prev_gyro_cfg.odr = bmi160_dev -> gyro_cfg.odr;
            bmi160_dev -> prev_gyro_cfg.bw = bmi160_dev -> gyro_cfg.bw;
            rslt = bmi160_set_regs(BMI160_GYRO_RANGE_ADDR, & data[1], 1);
            if (rslt == BMI160_OK) {
                bmi160_dev -> prev_gyro_cfg.range = bmi160_dev -> gyro_cfg.range;
            }
        }
    }
    return rslt;
}
static int8_t get_gyro_conf() {
    int8_t rslt;
    uint8_t data[2] = {
        0
    };
    rslt = bmi160_get_regs(BMI160_GYRO_CONFIG_ADDR, data, 2);
    if (rslt == BMI160_OK) {
        bmi160_dev -> gyro_cfg.odr = (data[0] & BMI160_GYRO_ODR_MASK);
        bmi160_dev -> gyro_cfg.bw = (data[0] & BMI160_GYRO_BW_MASK) >> BMI160_GYRO_BW_POS;
        bmi160_dev -> gyro_cfg.range = (data[1] & BMI160_GYRO_RANGE_MASK);
    }
    return rslt;
}
static int8_t check_gyro_config(uint8_t * data) {
    int8_t rslt;
    rslt = bmi160_get_regs(BMI160_GYRO_CONFIG_ADDR, data, 2);
    if (rslt == BMI160_OK) {
        rslt = process_gyro_odr( & data[0]);
        if (rslt == BMI160_OK) {
            rslt = process_gyro_bw( & data[0]);
            if (rslt == BMI160_OK) {
                rslt = process_gyro_range( & data[1]);
            }
        }
    }
    return rslt;
}
static int8_t process_gyro_odr(uint8_t * data) {
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t odr = 0;
    if (bmi160_dev -> gyro_cfg.odr <= BMI160_GYRO_ODR_3200HZ) {
        if (bmi160_dev -> gyro_cfg.odr != bmi160_dev -> prev_gyro_cfg.odr) {
            odr = (uint8_t) bmi160_dev -> gyro_cfg.odr;
            temp = ( * data & ~BMI160_GYRO_ODR_MASK);
            * data = temp | (odr & BMI160_GYRO_ODR_MASK);
        }
    } else {
        rslt = BMI160_E_OUT_OF_RANGE;
    }
    return rslt;
}
static int8_t process_gyro_bw(uint8_t * data) {
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t bw = 0;
    if (bmi160_dev -> gyro_cfg.bw <= BMI160_GYRO_BW_NORMAL_MODE) {
        bw = (uint8_t) bmi160_dev -> gyro_cfg.bw;
        temp = * data & ~BMI160_GYRO_BW_MASK;
        * data = temp | ((bw << 4) & BMI160_GYRO_BW_MASK);
    } else {
        rslt = BMI160_E_OUT_OF_RANGE;
    }
    return rslt;
}
static int8_t process_gyro_range(uint8_t * data) {
    int8_t rslt = 0;
    uint8_t temp = 0;
    uint8_t range = 0;
    if (bmi160_dev -> gyro_cfg.range <= BMI160_GYRO_RANGE_125_DPS) {
        if (bmi160_dev -> gyro_cfg.range != bmi160_dev -> prev_gyro_cfg.range) {
            range = (uint8_t) bmi160_dev -> gyro_cfg.range;
            temp = * data & ~BMI160_GYRO_RANGE_MASK;
            * data = temp | (range & BMI160_GYRO_RANGE_MASK);
        }
    } else {
        rslt = BMI160_E_OUT_OF_RANGE;
    }
    return rslt;
}
static int8_t set_accel_pwr() {
    int8_t rslt = 0;
    uint8_t data = 0;
    if ((bmi160_dev -> accel_cfg.power >= BMI160_ACCEL_SUSPEND_MODE) && (bmi160_dev -> accel_cfg.power <= BMI160_ACCEL_LOWPOWER_MODE)) {
        if (bmi160_dev -> accel_cfg.power != bmi160_dev -> prev_accel_cfg.power) {
            rslt = process_under_sampling( & data);
            if (rslt == BMI160_OK) {
                rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, & bmi160_dev -> accel_cfg.power, 1);
                if (bmi160_dev -> prev_accel_cfg.power == BMI160_ACCEL_SUSPEND_MODE) {
                    bmi160_dev -> delay_ms(BMI160_ACCEL_DELAY_MS);
                }
                bmi160_dev -> prev_accel_cfg.power = bmi160_dev -> accel_cfg.power;
            }
        }
    } else {
        rslt = BMI160_E_INVALID_CONFIG;
    }
    return rslt;
}
static int8_t process_under_sampling(uint8_t * data) {
    int8_t rslt;
    uint8_t temp = 0;
    uint8_t pre_filter[2] = {
        0
    };
    rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1);
    if (rslt == BMI160_OK) {
        if (bmi160_dev -> accel_cfg.power == BMI160_ACCEL_LOWPOWER_MODE) {
            temp = * data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
            * data = temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);
            rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1);
            if (rslt == BMI160_OK) {
                rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, pre_filter, 2);
            }
        } else if ( * data & BMI160_ACCEL_UNDERSAMPLING_MASK) {
            temp = * data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
            * data = temp;
            rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1);
        }
    }
    return rslt;
}
static int8_t set_gyro_pwr() {
    int8_t rslt = 0;
    if ((bmi160_dev -> gyro_cfg.power == BMI160_GYRO_SUSPEND_MODE) || (bmi160_dev -> gyro_cfg.power == BMI160_GYRO_NORMAL_MODE) ||
        (bmi160_dev -> gyro_cfg.power == BMI160_GYRO_FASTSTARTUP_MODE)) {
        if (bmi160_dev -> gyro_cfg.power != bmi160_dev -> prev_gyro_cfg.power) {
            rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, & bmi160_dev -> gyro_cfg.power, 1);
            if (bmi160_dev -> prev_gyro_cfg.power == BMI160_GYRO_SUSPEND_MODE) {
                bmi160_dev -> delay_ms(BMI160_GYRO_DELAY_MS);
            } else if ((bmi160_dev -> prev_gyro_cfg.power == BMI160_GYRO_FASTSTARTUP_MODE) &&
                (bmi160_dev -> gyro_cfg.power == BMI160_GYRO_NORMAL_MODE)) {
                bmi160_dev -> delay_ms(10);
            } else {}
            bmi160_dev -> prev_gyro_cfg.power = bmi160_dev -> gyro_cfg.power;
        }
    } else {
        rslt = BMI160_E_INVALID_CONFIG;
    }
    return rslt;
}
static int8_t get_accel_data(uint8_t len, struct bmi160_sensor_data * accel) {
    int8_t rslt;
    uint8_t idx = 0;
    uint8_t data_array[9] = {
        0
    };
    uint8_t time_0 = 0;
    uint16_t time_1 = 0;
    uint32_t time_2 = 0;
    uint8_t lsb;
    uint8_t msb;
    int16_t msblsb;
    rslt = bmi160_get_regs(BMI160_ACCEL_DATA_ADDR, data_array, 6 + len);
    if (rslt == BMI160_OK) {
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel -> x = msblsb;
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel -> y = msblsb;
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel -> z = msblsb;
        if (len == 3) {
            time_0 = data_array[idx++];
            time_1 = (uint16_t)(data_array[idx++] << 8);
            time_2 = (uint32_t)(data_array[idx++] << 16);
            accel -> sensortime = (uint32_t)(time_2 | time_1 | time_0);
        } else {
            accel -> sensortime = 0;
        }
    } else {
        rslt = BMI160_E_COM_FAIL;
    }
    return rslt;
}
static int8_t get_gyro_data(uint8_t len, struct bmi160_sensor_data * gyro) {
    int8_t rslt;
    uint8_t idx = 0;
    uint8_t data_array[15] = {
        0
    };
    uint8_t time_0 = 0;
    uint16_t time_1 = 0;
    uint32_t time_2 = 0;
    uint8_t lsb;
    uint8_t msb;
    int16_t msblsb;
    if (len == 0) {
        rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 6);
        if (rslt == BMI160_OK) {
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro -> x = msblsb;
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro -> y = msblsb;
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro -> z = msblsb;
            gyro -> sensortime = 0;
        } else {
            rslt = BMI160_E_COM_FAIL;
        }
    } else {
        rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len);
        if (rslt == BMI160_OK) {
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro -> x = msblsb;
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro -> y = msblsb;
            lsb = data_array[idx++];
            msb = data_array[idx++];
            msblsb = (int16_t)((msb << 8) | lsb);
            gyro -> z = msblsb;
            idx = idx + 6;
            time_0 = data_array[idx++];
            time_1 = (uint16_t)(data_array[idx++] << 8);
            time_2 = (uint32_t)(data_array[idx++] << 16);
            gyro -> sensortime = (uint32_t)(time_2 | time_1 | time_0);
        } else {
            rslt = BMI160_E_COM_FAIL;
        }
    }
    return rslt;
}
static int8_t get_accel_gyro_data(uint8_t len, struct bmi160_sensor_data * accel, struct bmi160_sensor_data * gyro) {
    int8_t rslt;
    uint8_t idx = 0;
    uint8_t data_array[15] = {
        0
    };
    uint8_t time_0 = 0;
    uint16_t time_1 = 0;
    uint32_t time_2 = 0;
    uint8_t lsb;
    uint8_t msb;
    int16_t msblsb;
    rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len);
    if (rslt == BMI160_OK) {
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        gyro -> x = msblsb;
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        gyro -> y = msblsb;
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        gyro -> z = msblsb;
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel -> x = (int16_t) msblsb;
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel -> y = (int16_t) msblsb;
        lsb = data_array[idx++];
        msb = data_array[idx++];
        msblsb = (int16_t)((msb << 8) | lsb);
        accel -> z = (int16_t) msblsb;
        if (len == 3) {
            time_0 = data_array[idx++];
            time_1 = (uint16_t)(data_array[idx++] << 8);
            time_2 = (uint32_t)(data_array[idx++] << 16);
            accel -> sensortime = (uint32_t)(time_2 | time_1 | time_0);
            gyro -> sensortime = (uint32_t)(time_2 | time_1 | time_0);
        } else {
            accel -> sensortime = 0;
            gyro -> sensortime = 0;
        }
        //							//UART_DEBUG
        //								sprintf(Buffer, "SENSOR: %X\n", (int16_t)msblsb);
        //								dev->uart_write(dev->huart, Buffer, sizeof(Buffer), 100); 
    } else {
        rslt = BMI160_E_COM_FAIL;
    }
    return rslt;
}

static int8_t get_foc_status(uint8_t * foc_status) {
    int8_t rslt;
    uint8_t data;
    rslt = bmi160_get_regs(BMI160_STATUS_ADDR, & data, 1);
    if (rslt == BMI160_OK) {
        * foc_status = BMI160_GET_BITS(data, BMI160_FOC_STATUS);
    }
    return rslt;
}
static int8_t configure_offset_enable(const struct bmi160_foc_conf * foc_conf) {
    int8_t rslt;
    uint8_t data;
    rslt = null_ptr_check();
    if (rslt != BMI160_OK) {
        rslt = BMI160_E_NULL_PTR;
    } else {
        rslt = bmi160_get_regs(BMI160_OFFSET_CONF_ADDR, & data, 1);
        if (rslt == BMI160_OK) {
            data = BMI160_SET_BITS(data, BMI160_GYRO_OFFSET_EN, foc_conf -> gyro_off_en);
            data = BMI160_SET_BITS(data, BMI160_ACCEL_OFFSET_EN, foc_conf -> acc_off_en);
            rslt = bmi160_set_regs(BMI160_OFFSET_CONF_ADDR, & data, 1);
        }
    }
    return rslt;
}
static int8_t trigger_foc(struct bmi160_offsets * offset) {
    int8_t rslt;
    uint8_t foc_status = BMI160_ENABLE;
    uint8_t cmd = BMI160_START_FOC_CMD;
    uint8_t timeout = 0;
    uint8_t data_array[20];
    rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, & cmd, 1);
    if (rslt == BMI160_OK) {
        rslt = get_foc_status( & foc_status);
        if ((rslt != BMI160_OK) || (foc_status != BMI160_ENABLE)) {
            while ((foc_status != BMI160_ENABLE) && (timeout < 11)) {
                bmi160_dev -> delay_ms(25);
                rslt = get_foc_status( & foc_status);
                timeout++;
            }
            if ((rslt == BMI160_OK) && (foc_status == BMI160_ENABLE)) {
                rslt = bmi160_get_offsets(offset);
            } else {
                rslt = BMI160_E_FOC_FAILURE;
            }
        }
        if (rslt == BMI160_OK) {
            rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 20);
        }
    }
    return rslt;
}
