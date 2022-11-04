#include "bmi160.h"
#ifdef BMI_INT_TYPE
const uint8_t int_mask_lookup_table[13] = {
 BMI160_INT1_SLOPE_MASK, BMI160_INT1_SLOPE_MASK, BMI160_INT2_LOW_STEP_DETECT_MASK, BMI160_INT1_DOUBLE_TAP_MASK, BMI160_INT1_SINGLE_TAP_MASK, BMI160_INT1_ORIENT_MASK, BMI160_INT1_FLAT_MASK, BMI160_INT1_HIGH_G_MASK, BMI160_INT1_LOW_G_MASK, BMI160_INT1_NO_MOTION_MASK, BMI160_INT2_DATA_READY_MASK, BMI160_INT2_FIFO_FULL_MASK, BMI160_INT2_FIFO_WM_MASK
};

static int8_t set_intr_pin_config(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_any_motion_int(struct bmi160_int_settg *int_config, struct bmi160_dev *dev);
static int8_t set_accel_tap_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_gyro_data_ready_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_sig_motion_int(struct bmi160_int_settg *int_config, struct bmi160_dev *dev);
static int8_t set_accel_no_motion_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_step_detect_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_orientation_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_flat_detect_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_low_g_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_accel_high_g_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t enable_accel_any_motion_int(const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,struct bmi160_dev *dev);
static int8_t disable_sig_motion_int(const struct bmi160_dev *dev);
static int8_t config_any_motion_src(const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,const struct bmi160_dev *dev);
static int8_t config_any_dur_threshold(const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,const struct bmi160_dev *dev);
static int8_t config_any_motion_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,const struct bmi160_dev *dev);
static int8_t enable_data_ready_int(const struct bmi160_dev *dev);
static int8_t enable_no_motion_int(const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,const struct bmi160_dev *dev);
static int8_t config_no_motion_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,const struct bmi160_dev *dev);
static int8_t enable_sig_motion_int(const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg, struct bmi160_dev *dev);
static int8_t config_sig_motion_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,const struct bmi160_dev *dev);
static int8_t config_no_motion_data_src(const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg, const struct bmi160_dev *dev);
static int8_t config_no_motion_dur_thr(const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,const struct bmi160_dev *dev);

static int8_t config_sig_motion_data_src(const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,const struct bmi160_dev *dev);
static int8_t config_sig_dur_threshold(const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,const struct bmi160_dev *dev);
static int8_t enable_step_detect_int(const struct bmi160_acc_step_detect_int_cfg *step_detect_int_cfg,const struct bmi160_dev *dev);
static int8_t config_step_detect(const struct bmi160_acc_step_detect_int_cfg *step_detect_int_cfg, const struct bmi160_dev *dev);
static int8_t enable_tap_int(const struct bmi160_int_settg *int_config,const struct bmi160_acc_tap_int_cfg *tap_int_cfg,const struct bmi160_dev *dev);
static int8_t config_tap_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_tap_int_cfg *tap_int_cfg,const struct bmi160_dev *dev);
static int8_t enable_orient_int(const struct bmi160_acc_orient_int_cfg *orient_int_cfg, const struct bmi160_dev *dev);
static int8_t config_orient_int_settg(const struct bmi160_acc_orient_int_cfg *orient_int_cfg,const struct bmi160_dev *dev);
static int8_t enable_flat_int(const struct bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev);
static int8_t config_flat_int_settg(const struct bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev);
static int8_t enable_low_g_int(const struct bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev);
static int8_t config_low_g_data_src(const struct bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev);
static int8_t config_low_g_int_settg(const struct bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev);
static int8_t enable_high_g_int(const struct bmi160_acc_high_g_int_cfg *high_g_int_cfg, const struct bmi160_dev *dev);
static int8_t config_high_g_data_src(const struct bmi160_acc_high_g_int_cfg *high_g_int_cfg,const struct bmi160_dev *dev);
static int8_t config_high_g_int_settg(const struct bmi160_acc_high_g_int_cfg *high_g_int_cfg,const struct bmi160_dev *dev);			
static int8_t config_tap_data_src(const struct bmi160_acc_tap_int_cfg *tap_int_cfg, const struct bmi160_dev *dev);
static int8_t config_tap_param(const struct bmi160_int_settg *int_config,const struct bmi160_acc_tap_int_cfg *tap_int_cfg,const struct bmi160_dev *dev);					
static int8_t config_int_out_ctrl(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t config_int_latch(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);			
static int8_t set_fifo_full_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t enable_fifo_full_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t set_fifo_watermark_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t enable_fifo_wtm_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t map_hardware_interrupt(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);
static int8_t map_feature_interrupt(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev);


static int8_t map_read_len(uint16_t *len, const struct bmi160_dev *dev);
static int8_t config_sec_if(const struct bmi160_dev *dev);

#endif

static void default_param_settg(struct bmi160_dev *dev);
static int8_t null_ptr_check(const struct bmi160_dev *dev);
static int8_t set_accel_conf(struct bmi160_dev *dev);
static int8_t get_accel_conf(struct bmi160_dev *dev);
static int8_t set_gyro_conf(struct bmi160_dev *dev);
static int8_t get_gyro_conf(struct bmi160_dev *dev);
static int8_t check_accel_config(uint8_t *data, const struct bmi160_dev *dev);
static int8_t check_invalid_settg(const struct bmi160_dev *dev);
static int8_t check_gyro_config(uint8_t *data, const struct bmi160_dev *dev);
static int8_t set_gyro_pwr(struct bmi160_dev *dev);
static int8_t set_accel_pwr(struct bmi160_dev *dev);
static int8_t process_accel_odr(uint8_t *data, const struct bmi160_dev *dev);
static int8_t process_accel_bw(uint8_t *data, const struct bmi160_dev *dev);
static int8_t process_accel_range(uint8_t *data, const struct bmi160_dev *dev);
static int8_t process_gyro_odr(uint8_t *data, const struct bmi160_dev *dev);
static int8_t process_gyro_bw(uint8_t *data, const struct bmi160_dev *dev);
static int8_t process_gyro_range(uint8_t *data, const struct bmi160_dev *dev);
static int8_t process_under_sampling(uint8_t *data, const struct bmi160_dev *dev);
static int8_t get_accel_data(uint8_t len, struct bmi160_sensor_data *accel, const struct bmi160_dev *dev);
static int8_t get_gyro_data(uint8_t len, struct bmi160_sensor_data *gyro, const struct bmi160_dev *dev);
static int8_t get_accel_gyro_data(uint8_t len,struct bmi160_sensor_data *accel,struct bmi160_sensor_data *gyro,const struct bmi160_dev *dev);


static void check_frame_validity(uint16_t *data_index, const struct bmi160_dev *dev);
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi160_dev *dev);
static void unpack_sensortime_frame(uint16_t *data_index, const struct bmi160_dev *dev);
static void unpack_skipped_frame(uint16_t *data_index, const struct bmi160_dev *dev);
static int8_t get_foc_status(uint8_t *foc_status, struct bmi160_dev const *dev);
static int8_t configure_offset_enable(const struct bmi160_foc_conf *foc_conf, struct bmi160_dev const *dev);
static int8_t trigger_foc(struct bmi160_offsets *offset, struct bmi160_dev const *dev);
static void reset_fifo_data_structure(const struct bmi160_dev *dev);
static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, struct bmi160_dev const *dev);
static void get_accel_len_to_parse(uint16_t *data_index,uint16_t *data_read_length,const uint8_t *acc_frame_count,const struct bmi160_dev *dev);
static void unpack_accel_frame(struct bmi160_sensor_data *acc,uint16_t *idx,uint8_t *acc_idx,uint8_t frame_info,const struct bmi160_dev *dev);
static void unpack_accel_data(struct bmi160_sensor_data *accel_data,uint16_t data_start_index,const struct bmi160_dev *dev);
static void extract_accel_header_mode(struct bmi160_sensor_data *accel_data, uint8_t *accel_length, const struct bmi160_dev *dev);
static void get_gyro_len_to_parse(uint16_t *data_index, uint16_t *data_read_length, const uint8_t *gyro_frame_count, const struct bmi160_dev *dev);
static void unpack_gyro_frame(struct bmi160_sensor_data *gyro,uint16_t *idx,uint8_t *gyro_idx,uint8_t frame_info,const struct bmi160_dev *dev);
static void unpack_gyro_data(struct bmi160_sensor_data *gyro_data,uint16_t data_start_index,const struct bmi160_dev *dev);
static void extract_gyro_header_mode(struct bmi160_sensor_data *gyro_data,uint8_t *gyro_length,const struct bmi160_dev *dev);

int8_t bmi160_get_regs(uint16_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev){
 int8_t rslt = BMI160_OK;
 if ((dev == NULL) || (dev->read == NULL)){
rslt = BMI160_E_NULL_PTR;
 }else if (len == 0){
rslt = BMI160_E_READ_WRITE_LENGTH_INVALID;
 }else{
if (dev->intf == BMI160_SPI_INTF){
reg_addr = (reg_addr | BMI160_SPI_RD_MASK);
}
				rslt = dev->read(dev->hi2cx, (uint16_t)(dev->id<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100);
 }
		return rslt;
}
int8_t bmi160_set_regs(uint16_t reg_addr, uint8_t *data, uint16_t len, const struct bmi160_dev *dev){
 int8_t rslt = BMI160_OK;
 uint8_t count = 0;
 if ((dev == NULL) || (dev->write == NULL)){
rslt = BMI160_E_NULL_PTR;
 }else if (len == 0){
rslt = BMI160_E_READ_WRITE_LENGTH_INVALID;
 }else{
if (dev->intf == BMI160_SPI_INTF){
reg_addr = (reg_addr & BMI160_SPI_WR_MASK);
}
				if ((dev->prev_accel_cfg.power == BMI160_ACCEL_NORMAL_MODE) || (dev->prev_gyro_cfg.power == BMI160_GYRO_NORMAL_MODE)){
rslt = dev->write(dev->hi2cx, (uint16_t)(dev->id<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
dev->delay_ms(5);	
}else{
for (; count < len; count++){
							rslt = dev->write(dev->hi2cx, (uint16_t)(dev->id<<1), reg_addr, I2C_MEMADD_SIZE_8BIT, &data[count], len, 100);
 reg_addr++; 
 dev->delay_ms(1);
						}
}
if (rslt != BMI160_OK){
rslt = BMI160_E_COM_FAIL;
}
 }
		return rslt;
}

int8_t bmi160_init(struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data;
	uint8_t try = 3; 
 rslt = null_ptr_check(dev);
		dev->chip_id = 0;	
	
		if ((rslt == BMI160_OK) && (dev->intf == BMI160_SPI_INTF)){
			rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, &data, 1, dev);
		}
		
		if (rslt == BMI160_OK){
			 
			while ((try--) && (dev->chip_id != BMI160_CHIP_ID)){
					rslt = bmi160_get_regs(BMI160_CHIP_ID_ADDR, &dev->chip_id, 1, dev);
					if (rslt == BMI160_OK)break;
			}
			
			if ((rslt == BMI160_OK) && (dev->chip_id == BMI160_CHIP_ID)){
					dev->any_sig_sel = BMI160_BOTH_ANY_SIG_MOTION_DISABLED;
					rslt = bmi160_soft_reset(dev);
			}else{
					rslt = BMI160_E_DEV_NOT_FOUND;
			}
 }
		return rslt;
}

int8_t bmi160_soft_reset(struct bmi160_dev *dev){
 int8_t rslt = BMI160_OK;
 uint8_t data = BMI160_SOFT_RESET_CMD; 
 if ((dev == NULL) || (dev->delay_ms == NULL)){
rslt = BMI160_E_NULL_PTR;
 }else{
dev->delay_ms(BMI160_SOFT_RESET_DELAY_MS);
			if ((rslt == BMI160_OK) && (dev->intf == BMI160_SPI_INTF)){ 
rslt = bmi160_get_regs(BMI160_SPI_COMM_TEST_ADDR, &data, 1, dev);
}
				if (rslt == BMI160_OK){
default_param_settg(dev);
}
 }
		return rslt;
}

int8_t bmi160_set_sens_conf(struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK; 
 if ((dev == NULL) || (dev->delay_ms == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = set_accel_conf(dev);if (rslt == BMI160_OK)
{
rslt = set_gyro_conf(dev);
if (rslt == BMI160_OK)
{
 
 rslt = bmi160_set_power_mode(dev);
 if (rslt == BMI160_OK)
 {
rslt = check_invalid_settg(dev);
 }
}
}
 } return rslt;
}
int8_t bmi160_get_sens_conf(struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK; 
 if ((dev == NULL) || (dev->delay_ms == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = get_accel_conf(dev);
if (rslt == BMI160_OK)
{
rslt = get_gyro_conf(dev);
}
 } return rslt;
}
int8_t bmi160_set_power_mode(struct bmi160_dev *dev)
{
 int8_t rslt = 0; 
 if ((dev == NULL) || (dev->delay_ms == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = set_accel_pwr(dev);
if (rslt == BMI160_OK)
{
rslt = set_gyro_pwr(dev);
}
 } return rslt;
}
int8_t bmi160_get_power_mode(struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t power_mode = 0; 
 if ((dev == NULL) || (dev->delay_ms == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = bmi160_get_regs(BMI160_PMU_STATUS_ADDR, &power_mode, 1, dev);
if (rslt == BMI160_OK)
{

dev->gyro_cfg.power = BMI160_GET_BITS(power_mode, BMI160_GYRO_POWER_MODE);
dev->accel_cfg.power = BMI160_GET_BITS(power_mode, BMI160_ACCEL_POWER_MODE);
}
 } return rslt;
}
int8_t bmi160_get_sensor_data(uint8_t select_sensor, struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro, const struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK;
 uint8_t time_sel;
 uint8_t sen_sel;
 uint8_t len = 0; 
 sen_sel = select_sensor & BMI160_SEN_SEL_MASK;
 time_sel = ((sen_sel & BMI160_TIME_SEL) >> 2);
 sen_sel = sen_sel & (BMI160_ACCEL_SEL | BMI160_GYRO_SEL);
 if (time_sel == 1)
 {
len = 3;
 } 
 if (dev != NULL)
 {
switch (sen_sel)
{
case BMI160_ACCEL_ONLY: 
 if (accel == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = get_accel_data(len, accel, dev);
 } break;
case BMI160_GYRO_ONLY: 
 if (gyro == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = get_gyro_data(len, gyro, dev);
 } break;
case BMI160_BOTH_ACCEL_AND_GYRO: 
 if ((gyro == NULL) || (accel == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = get_accel_gyro_data(len, accel, gyro, dev);
 } break;
default:
 rslt = BMI160_E_INVALID_INPUT;
 break;
}
 }
 else
 {
rslt = BMI160_E_NULL_PTR;
 } return rslt;
}


int8_t bmi160_get_fifo_data(struct bmi160_dev const *dev)
{
 int8_t rslt = 0;
 uint16_t bytes_to_read = 0;
 uint16_t user_fifo_len = 0; 
 if ((dev == NULL) || (dev->fifo->data == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
reset_fifo_data_structure(dev);
rslt = get_fifo_byte_counter(&bytes_to_read, dev);
if (rslt == BMI160_OK)
{
user_fifo_len = dev->fifo->length;
if ((dev->fifo->length > bytes_to_read))
{
 
 dev->fifo->length = bytes_to_read;
}if ((dev->fifo->fifo_time_enable == BMI160_FIFO_TIME_ENABLE) &&
 (bytes_to_read + BMI160_FIFO_BYTES_OVERREAD <= user_fifo_len))
{
 
 dev->fifo->length = dev->fifo->length + BMI160_FIFO_BYTES_OVERREAD;
}
rslt = bmi160_get_regs(BMI160_FIFO_DATA_ADDR, dev->fifo->data, dev->fifo->length, dev);
}
 } return rslt;
}
int8_t bmi160_set_fifo_flush(const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t data = BMI160_FIFO_FLUSH_VALUE;
 uint8_t reg_addr = BMI160_COMMAND_REG_ADDR; 
 if (dev == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = bmi160_set_regs(reg_addr, &data, BMI160_ONE, dev);
 } return rslt;
}
int8_t bmi160_set_fifo_config(uint8_t config, uint8_t enable, struct bmi160_dev const *dev)
{
 int8_t rslt = 0;
 uint8_t data = 0;
 uint8_t reg_addr = BMI160_FIFO_CONFIG_1_ADDR;
 uint8_t fifo_config = config & BMI160_FIFO_CONFIG_1_MASK; 
 if (dev == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = bmi160_get_regs(reg_addr, &data, BMI160_ONE, dev);
if (rslt == BMI160_OK)
{
if (fifo_config > 0)
{
 if (enable == BMI160_ENABLE)
 {
data = data | fifo_config;
 }
 else
 {
data = data & (~fifo_config);
 }
}
rslt = bmi160_set_regs(reg_addr, &data, BMI160_ONE, dev);
if (rslt == BMI160_OK)
{
 
 rslt = bmi160_get_regs(reg_addr, &data, BMI160_ONE, dev);
 if (rslt == BMI160_OK)
 {

dev->fifo->fifo_header_enable = data & BMI160_FIFO_HEAD_ENABLE;
dev->fifo->fifo_data_enable = data & BMI160_FIFO_M_G_A_ENABLE;
dev->fifo->fifo_time_enable = data & BMI160_FIFO_TIME_ENABLE;
 }
}
}
 } return rslt;
}
int8_t bmi160_set_fifo_down(uint8_t fifo_down, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t data = 0;
 uint8_t reg_addr = BMI160_FIFO_DOWN_ADDR; 
 if (dev == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = bmi160_get_regs(reg_addr, &data, BMI160_ONE, dev);
if (rslt == BMI160_OK)
{
data = data | fifo_down;
rslt = bmi160_set_regs(reg_addr, &data, BMI160_ONE, dev);
}
 } return rslt;
}
int8_t bmi160_set_fifo_wm(uint8_t fifo_wm, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t data = fifo_wm;
 uint8_t reg_addr = BMI160_FIFO_CONFIG_0_ADDR; 
 if (dev == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = bmi160_set_regs(reg_addr, &data, BMI160_ONE, dev);
 } return rslt;
}
int8_t bmi160_extract_accel(struct bmi160_sensor_data *accel_data, uint8_t *accel_length, struct bmi160_dev const *dev)
{
 int8_t rslt = 0;
 uint16_t data_index = 0;
 uint16_t data_read_length = 0;
 uint8_t accel_index = 0;
 uint8_t fifo_data_enable = 0; if (dev == NULL || dev->fifo == NULL || dev->fifo->data == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

if (dev->fifo->fifo_header_enable == 0)
{

get_accel_len_to_parse(&data_index, &data_read_length, accel_length, dev);
for (; data_index < data_read_length;)
{
 
 check_frame_validity(&data_index, dev);
 fifo_data_enable = dev->fifo->fifo_data_enable;
 unpack_accel_frame(accel_data, &data_index, &accel_index, fifo_data_enable, dev);
}
*accel_length = accel_index;
dev->fifo->accel_byte_start_idx = data_index;
}
else
{

extract_accel_header_mode(accel_data, accel_length, dev);
}
 } return rslt;
}
int8_t bmi160_extract_gyro(struct bmi160_sensor_data *gyro_data, uint8_t *gyro_length, struct bmi160_dev const *dev)
{
 int8_t rslt = 0;
 uint16_t data_index = 0;
 uint16_t data_read_length = 0;
 uint8_t gyro_index = 0;
 uint8_t fifo_data_enable = 0; if (dev == NULL || dev->fifo->data == NULL)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

if (dev->fifo->fifo_header_enable == 0)
{

get_gyro_len_to_parse(&data_index, &data_read_length, gyro_length, dev);
for (; data_index < data_read_length;)
{
 
 check_frame_validity(&data_index, dev);
 fifo_data_enable = dev->fifo->fifo_data_enable;
 unpack_gyro_frame(gyro_data, &data_index, &gyro_index, fifo_data_enable, dev);
}
*gyro_length = gyro_index;
dev->fifo->gyro_byte_start_idx = data_index;
}
else
{

extract_gyro_header_mode(gyro_data, gyro_length, dev);
}
 } return rslt;
}
int8_t bmi160_start_foc(const struct bmi160_foc_conf *foc_conf,struct bmi160_offsets *offset,struct bmi160_dev const *dev)
{
 int8_t rslt;
 uint8_t data; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

rslt = configure_offset_enable(foc_conf, dev);
if (rslt == BMI160_OK)
{

rslt = bmi160_get_regs(BMI160_FOC_CONF_ADDR, &data, 1, dev);
data = BMI160_SET_BITS(data, BMI160_GYRO_FOC_EN, foc_conf->foc_gyr_en);
data = BMI160_SET_BITS(data, BMI160_ACCEL_FOC_X_CONF, foc_conf->foc_acc_x);
data = BMI160_SET_BITS(data, BMI160_ACCEL_FOC_Y_CONF, foc_conf->foc_acc_y);
data = BMI160_SET_BITS_POS_0(data, BMI160_ACCEL_FOC_Z_CONF, foc_conf->foc_acc_z);
if (rslt == BMI160_OK)
{
 
 rslt = bmi160_set_regs(BMI160_FOC_CONF_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {

rslt = trigger_foc(offset, dev);
 }
}
}
 } return rslt;
}
int8_t bmi160_get_offsets(struct bmi160_offsets *offset, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data[7];
 uint8_t lsb, msb;
 int16_t offset_msb, offset_lsb;
 int16_t offset_data; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

rslt = bmi160_get_regs(BMI160_OFFSET_ADDR, data, 7, dev);
offset->off_acc_x = (int8_t)data[0];
offset->off_acc_y = (int8_t)data[1];
offset->off_acc_z = (int8_t)data[2];
lsb = data[3];
msb = BMI160_GET_BITS_POS_0(data[6], BMI160_GYRO_OFFSET_X);
offset_msb = (int16_t)(msb << 14);
offset_lsb = lsb << 6;
offset_data = offset_msb | offset_lsb;
offset->off_gyro_x = (int16_t)(offset_data / 64);
lsb = data[4];
msb = BMI160_GET_BITS(data[6], BMI160_GYRO_OFFSET_Y);
offset_msb = (int16_t)(msb << 14);
offset_lsb = lsb << 6;
offset_data = offset_msb | offset_lsb;
offset->off_gyro_y = (int16_t)(offset_data / 64);
lsb = data[5];
msb = BMI160_GET_BITS(data[6], BMI160_GYRO_OFFSET_Z);
offset_msb = (int16_t)(msb << 14);
offset_lsb = lsb << 6;
offset_data = offset_msb | offset_lsb;
offset->off_gyro_z = (int16_t)(offset_data / 64);
 } return rslt;
}
int8_t bmi160_set_offsets(const struct bmi160_foc_conf *foc_conf,const struct bmi160_offsets *offset,struct bmi160_dev const *dev)
{
 int8_t rslt;
 uint8_t data[7];
 uint8_t x_msb, y_msb, z_msb; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

data[0] = (uint8_t)offset->off_acc_x;
data[1] = (uint8_t)offset->off_acc_y;
data[2] = (uint8_t)offset->off_acc_z;
data[3] = BMI160_GET_LSB(offset->off_gyro_x);
data[4] = BMI160_GET_LSB(offset->off_gyro_y);
data[5] = BMI160_GET_LSB(offset->off_gyro_z);
x_msb = BMI160_GET_BITS(offset->off_gyro_x, BMI160_GYRO_OFFSET);
y_msb = BMI160_GET_BITS(offset->off_gyro_y, BMI160_GYRO_OFFSET);
z_msb = BMI160_GET_BITS(offset->off_gyro_z, BMI160_GYRO_OFFSET);
data[6] = (uint8_t)(z_msb << 4 | y_msb << 2 | x_msb);
data[6] = BMI160_SET_BITS(data[6], BMI160_GYRO_OFFSET_EN, foc_conf->gyro_off_en);
data[6] = BMI160_SET_BITS(data[6], BMI160_ACCEL_OFFSET_EN, foc_conf->acc_off_en);
rslt = bmi160_set_regs(BMI160_OFFSET_ADDR, data, 7, dev);
 } return rslt;
}
int8_t bmi160_update_nvm(struct bmi160_dev const *dev)
{
 int8_t rslt;
 uint8_t data;
 uint8_t cmd = BMI160_NVM_BACKUP_EN; 
 rslt = bmi160_get_regs(BMI160_CONF_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
data = BMI160_SET_BITS(data, BMI160_NVM_UPDATE, 1);
rslt = bmi160_set_regs(BMI160_CONF_ADDR, &data, 1, dev);
if (rslt == BMI160_OK)
{

rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &cmd, 1, dev);
if (rslt == BMI160_OK)
{
 
 rslt = bmi160_get_regs(BMI160_STATUS_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
data = BMI160_GET_BITS(data, BMI160_NVM_STATUS);
if (data != BMI160_ENABLE)
{

dev->delay_ms(25);
}
 }
}
}
 } return rslt;
}
static int8_t null_ptr_check(const struct bmi160_dev *dev)
{
 int8_t rslt; if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

rslt = BMI160_OK;
 } return rslt;
}
static void default_param_settg(struct bmi160_dev *dev)
{
 
 dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
 dev->accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
 dev->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;
 dev->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
 dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
 dev->gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
 dev->gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
 dev->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS; 
 dev->prev_accel_cfg = dev->accel_cfg; 
 dev->prev_gyro_cfg = dev->gyro_cfg;
}
static int8_t set_accel_conf(struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data[2] = { 0 }; rslt = check_accel_config(data, dev);
 if (rslt == BMI160_OK)
 {

rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, &data[0], 1, dev);if (rslt == BMI160_OK)
{		
dev->prev_accel_cfg.odr = dev->accel_cfg.odr;
dev->prev_accel_cfg.bw = dev->accel_cfg.bw;
rslt = bmi160_set_regs(BMI160_ACCEL_RANGE_ADDR, &data[1], 1, dev);
if (rslt == BMI160_OK)
{
 dev->prev_accel_cfg.range = dev->accel_cfg.range;
}
}
 } return rslt;
}
static int8_t get_accel_conf(struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data[2] = { 0 }; 
 rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 2, dev);
 if (rslt == BMI160_OK)
 {
dev->accel_cfg.odr = (data[0] & BMI160_ACCEL_ODR_MASK);
dev->accel_cfg.bw = (data[0] & BMI160_ACCEL_BW_MASK) >> BMI160_ACCEL_BW_POS;
dev->accel_cfg.range = (data[1] & BMI160_ACCEL_RANGE_MASK);
 } return rslt;
}
static int8_t check_accel_config(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 2, dev);
 if (rslt == BMI160_OK)
 {
rslt = process_accel_odr(&data[0], dev);
if (rslt == BMI160_OK)
{
rslt = process_accel_bw(&data[0], dev);
if (rslt == BMI160_OK)
{
 rslt = process_accel_range(&data[1], dev);
}
}
 } return rslt;
}
static int8_t process_accel_odr(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t temp = 0;
 uint8_t odr = 0; if (dev->accel_cfg.odr <= BMI160_ACCEL_ODR_1600HZ)
 {
if (dev->accel_cfg.odr != dev->prev_accel_cfg.odr)
{
odr = (uint8_t)dev->accel_cfg.odr;
temp = *data & ~BMI160_ACCEL_ODR_MASK;
*data = temp | (odr & BMI160_ACCEL_ODR_MASK);
}
 }
 else
 {
rslt = BMI160_E_OUT_OF_RANGE;
 } return rslt;
}
static int8_t process_accel_bw(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t temp = 0;
 uint8_t bw = 0; if (dev->accel_cfg.bw <= BMI160_ACCEL_BW_RES_AVG128)
 {
if (dev->accel_cfg.bw != dev->prev_accel_cfg.bw)
{
bw = (uint8_t)dev->accel_cfg.bw;
temp = *data & ~BMI160_ACCEL_BW_MASK;
*data = temp | ((bw << 4) & BMI160_ACCEL_BW_MASK);
}
 }
 else
 {
rslt = BMI160_E_OUT_OF_RANGE;
 } return rslt;
}
static int8_t process_accel_range(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t temp = 0;
 uint8_t range = 0; if (dev->accel_cfg.range <= BMI160_ACCEL_RANGE_16G)
 {
if (dev->accel_cfg.range != dev->prev_accel_cfg.range)
{
range = (uint8_t)dev->accel_cfg.range;
temp = *data & ~BMI160_ACCEL_RANGE_MASK;
*data = temp | (range & BMI160_ACCEL_RANGE_MASK);
}
 }
 else
 {
rslt = BMI160_E_OUT_OF_RANGE;
 } return rslt;
}
static int8_t check_invalid_settg(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0; 
 rslt = bmi160_get_regs(BMI160_ERROR_REG_ADDR, &data, 1, dev);
 data = data >> 1;
 data = data & BMI160_ERR_REG_MASK;
 if (data == 1)
 {
rslt = BMI160_E_ACCEL_ODR_BW_INVALID;
 }
 else if (data == 2)
 {
rslt = BMI160_E_GYRO_ODR_BW_INVALID;
 }
 else if (data == 3)
 {
rslt = BMI160_E_LWP_PRE_FLTR_INT_INVALID;
 }
 else if (data == 7)
 {
rslt = BMI160_E_LWP_PRE_FLTR_INVALID;
 } return rslt;
}
static int8_t set_gyro_conf(struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data[2] = { 0 }; rslt = check_gyro_config(data, dev);
 if (rslt == BMI160_OK)
 {

rslt = bmi160_set_regs(BMI160_GYRO_CONFIG_ADDR, &data[0], 1, dev);
if (rslt == BMI160_OK)
{
dev->prev_gyro_cfg.odr = dev->gyro_cfg.odr;
dev->prev_gyro_cfg.bw = dev->gyro_cfg.bw;
rslt = bmi160_set_regs(BMI160_GYRO_RANGE_ADDR, &data[1], 1, dev);
if (rslt == BMI160_OK)
{
 dev->prev_gyro_cfg.range = dev->gyro_cfg.range;
}
}
 } return rslt;
}
static int8_t get_gyro_conf(struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data[2] = { 0 }; 
 rslt = bmi160_get_regs(BMI160_GYRO_CONFIG_ADDR, data, 2, dev);
 if (rslt == BMI160_OK)
 {
dev->gyro_cfg.odr = (data[0] & BMI160_GYRO_ODR_MASK);
dev->gyro_cfg.bw = (data[0] & BMI160_GYRO_BW_MASK) >> BMI160_GYRO_BW_POS;
dev->gyro_cfg.range = (data[1] & BMI160_GYRO_RANGE_MASK);
 } return rslt;
}
static int8_t check_gyro_config(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = bmi160_get_regs(BMI160_GYRO_CONFIG_ADDR, data, 2, dev);
 if (rslt == BMI160_OK)
 {
rslt = process_gyro_odr(&data[0], dev);
if (rslt == BMI160_OK)
{
rslt = process_gyro_bw(&data[0], dev);
if (rslt == BMI160_OK)
{
 rslt = process_gyro_range(&data[1], dev);
}
}
 } return rslt;
}
static int8_t process_gyro_odr(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t temp = 0;
 uint8_t odr = 0; if (dev->gyro_cfg.odr <= BMI160_GYRO_ODR_3200HZ)
 {
if (dev->gyro_cfg.odr != dev->prev_gyro_cfg.odr)
{
odr = (uint8_t)dev->gyro_cfg.odr;
temp = (*data & ~BMI160_GYRO_ODR_MASK);
*data = temp | (odr & BMI160_GYRO_ODR_MASK);
}
 }
 else
 {
rslt = BMI160_E_OUT_OF_RANGE;
 } return rslt;
}
static int8_t process_gyro_bw(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t temp = 0;
 uint8_t bw = 0; if (dev->gyro_cfg.bw <= BMI160_GYRO_BW_NORMAL_MODE)
 {
bw = (uint8_t)dev->gyro_cfg.bw;
temp = *data & ~BMI160_GYRO_BW_MASK;
*data = temp | ((bw << 4) & BMI160_GYRO_BW_MASK);
 }
 else
 {
rslt = BMI160_E_OUT_OF_RANGE;
 } return rslt;
}
static int8_t process_gyro_range(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t temp = 0;
 uint8_t range = 0; if (dev->gyro_cfg.range <= BMI160_GYRO_RANGE_125_DPS)
 {
if (dev->gyro_cfg.range != dev->prev_gyro_cfg.range)
{
range = (uint8_t)dev->gyro_cfg.range;
temp = *data & ~BMI160_GYRO_RANGE_MASK;
*data = temp | (range & BMI160_GYRO_RANGE_MASK);
}
 }
 else
 {
rslt = BMI160_E_OUT_OF_RANGE;
 } return rslt;
}
static int8_t set_accel_pwr(struct bmi160_dev *dev)
{
 int8_t rslt = 0;
 uint8_t data = 0; if ((dev->accel_cfg.power >= BMI160_ACCEL_SUSPEND_MODE) && (dev->accel_cfg.power <= BMI160_ACCEL_LOWPOWER_MODE))
 {
if (dev->accel_cfg.power != dev->prev_accel_cfg.power)
{
rslt = process_under_sampling(&data, dev);
if (rslt == BMI160_OK)
{
 
 rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &dev->accel_cfg.power, 1, dev); 
 if (dev->prev_accel_cfg.power == BMI160_ACCEL_SUSPEND_MODE)
 {
dev->delay_ms(BMI160_ACCEL_DELAY_MS);
 } dev->prev_accel_cfg.power = dev->accel_cfg.power;
}
}
 }
 else
 {
rslt = BMI160_E_INVALID_CONFIG;
 } return rslt;
}
static int8_t process_under_sampling(uint8_t *data, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t temp = 0;
 uint8_t pre_filter[2] = { 0 }; rslt = bmi160_get_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
 if (rslt == BMI160_OK)
 {
if (dev->accel_cfg.power == BMI160_ACCEL_LOWPOWER_MODE)
{
temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
*data = temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);
rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
if (rslt == BMI160_OK)
{
 
 rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, pre_filter, 2, dev);
}
}
else if (*data & BMI160_ACCEL_UNDERSAMPLING_MASK)
{
temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
*data = temp;
rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
}
 } return rslt;
}
static int8_t set_gyro_pwr(struct bmi160_dev *dev)
{
 int8_t rslt = 0; if ((dev->gyro_cfg.power == BMI160_GYRO_SUSPEND_MODE) || (dev->gyro_cfg.power == BMI160_GYRO_NORMAL_MODE) ||
(dev->gyro_cfg.power == BMI160_GYRO_FASTSTARTUP_MODE))
 {
if (dev->gyro_cfg.power != dev->prev_gyro_cfg.power)
{

rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &dev->gyro_cfg.power, 1, dev);
if (dev->prev_gyro_cfg.power == BMI160_GYRO_SUSPEND_MODE)
{
 
 dev->delay_ms(BMI160_GYRO_DELAY_MS);
}
else if ((dev->prev_gyro_cfg.power == BMI160_GYRO_FASTSTARTUP_MODE) &&
(dev->gyro_cfg.power == BMI160_GYRO_NORMAL_MODE))
{
 
 dev->delay_ms(10);
}
else
{
 
}dev->prev_gyro_cfg.power = dev->gyro_cfg.power;
}
 }
 else
 {
rslt = BMI160_E_INVALID_CONFIG;
 } return rslt;
}
static int8_t get_accel_data(uint8_t len, struct bmi160_sensor_data *accel, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t idx = 0;
 uint8_t data_array[9] = { 0 };
 uint8_t time_0 = 0;
 uint16_t time_1 = 0;
 uint32_t time_2 = 0;
 uint8_t lsb;
 uint8_t msb;
 int16_t msblsb; 
 rslt = bmi160_get_regs(BMI160_ACCEL_DATA_ADDR, data_array, 6 + len, dev);
 if (rslt == BMI160_OK)
 {

lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
accel->x = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
accel->y = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
accel->z = msblsb; 
if (len == 3)
{
time_0 = data_array[idx++];
time_1 = (uint16_t)(data_array[idx++] << 8);
time_2 = (uint32_t)(data_array[idx++] << 16);
accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
}
else
{
accel->sensortime = 0;
}
 }
 else
 {
rslt = BMI160_E_COM_FAIL;
 } return rslt;
}
static int8_t get_gyro_data(uint8_t len, struct bmi160_sensor_data *gyro, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t idx = 0;
 uint8_t data_array[15] = { 0 };
 uint8_t time_0 = 0;
 uint16_t time_1 = 0;
 uint32_t time_2 = 0;
 uint8_t lsb;
 uint8_t msb;
 int16_t msblsb; if (len == 0)
 {

rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 6, dev);
if (rslt == BMI160_OK)
{

lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->x = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->y = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->z = msblsb; 
gyro->sensortime = 0;
}
else
{
rslt = BMI160_E_COM_FAIL;
}
 }
 else
 {

rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
if (rslt == BMI160_OK)
{

lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->x = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->y = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->z = msblsb; 
idx = idx + 6;
time_0 = data_array[idx++];
time_1 = (uint16_t)(data_array[idx++] << 8);
time_2 = (uint32_t)(data_array[idx++] << 16);
gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
}
else
{
rslt = BMI160_E_COM_FAIL;
}
 } return rslt;
}
static int8_t get_accel_gyro_data(uint8_t len,struct bmi160_sensor_data *accel,struct bmi160_sensor_data *gyro,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t idx = 0;
 uint8_t data_array[15] = { 0 };
 uint8_t time_0 = 0;
 uint16_t time_1 = 0;
 uint32_t time_2 = 0;
 uint8_t lsb;
 uint8_t msb;
 int16_t msblsb;
 
 rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
 if (rslt == BMI160_OK)
 {

lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->x = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->y = msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
gyro->z = msblsb; 

lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
accel->x = (int16_t)msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
accel->y = (int16_t)msblsb; 
lsb = data_array[idx++];
msb = data_array[idx++];
msblsb = (int16_t)((msb << 8) | lsb);
accel->z = (int16_t)msblsb; 
if (len == 3)
{
time_0 = data_array[idx++];
time_1 = (uint16_t)(data_array[idx++] << 8);
time_2 = (uint32_t)(data_array[idx++] << 16);
accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
}
else
{
accel->sensortime = 0;
gyro->sensortime = 0;
}
				
//							//UART_DEBUG
//								sprintf(Buffer, "SENSOR: %X\n", (int16_t)msblsb);
//								dev->uart_write(dev->huart, Buffer, sizeof(Buffer), 100); 
				
 }
 else
 {
rslt = BMI160_E_COM_FAIL;
 } return rslt;
}

static void reset_fifo_data_structure(const struct bmi160_dev *dev)
{
 
 dev->fifo->accel_byte_start_idx = 0;
 dev->fifo->gyro_byte_start_idx = 0;
 dev->fifo->aux_byte_start_idx = 0;
 dev->fifo->sensor_time = 0;
 dev->fifo->skipped_frame_count = 0;
}
static int8_t get_fifo_byte_counter(uint16_t *bytes_to_read, struct bmi160_dev const *dev)
{
 int8_t rslt = 0;
 uint8_t data[2];
 uint8_t addr = BMI160_FIFO_LENGTH_ADDR; rslt |= bmi160_get_regs(addr, data, 2, dev);
 data[1] = data[1] & BMI160_FIFO_BYTE_COUNTER_MASK; 
 *bytes_to_read = (((uint16_t)data[1] << 8) | ((uint16_t)data[0])); return rslt;
}
static void get_accel_len_to_parse(uint16_t *data_index,uint16_t *data_read_length,const uint8_t *acc_frame_count,const struct bmi160_dev *dev)
{
 
 *data_index = dev->fifo->accel_byte_start_idx;
 if (dev->fifo->fifo_data_enable == BMI160_FIFO_A_ENABLE)
 {
*data_read_length = (*acc_frame_count) * BMI160_FIFO_A_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_G_A_ENABLE)
 {
*data_read_length = (*acc_frame_count) * BMI160_FIFO_GA_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_A_ENABLE)
 {
*data_read_length = (*acc_frame_count) * BMI160_FIFO_MA_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_A_ENABLE)
 {
*data_read_length = (*acc_frame_count) * BMI160_FIFO_MGA_LENGTH;
 }
 else
 {

*data_index = dev->fifo->length;
 } if (*data_read_length > dev->fifo->length)
 {

*data_read_length = dev->fifo->length;
 }
}
static void unpack_accel_frame(struct bmi160_sensor_data *acc,uint16_t *idx,uint8_t *acc_idx,uint8_t frame_info,const struct bmi160_dev *dev)
{
 switch (frame_info)
 {
case BMI160_FIFO_HEAD_A:
case BMI160_FIFO_A_ENABLE:
if ((*idx + BMI160_FIFO_A_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_accel_data(&acc[*acc_idx], *idx, dev);
*idx = *idx + BMI160_FIFO_A_LENGTH;
(*acc_idx)++;
break;
case BMI160_FIFO_HEAD_G_A:
case BMI160_FIFO_G_A_ENABLE:
if ((*idx + BMI160_FIFO_GA_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_accel_data(&acc[*acc_idx], *idx + BMI160_FIFO_G_LENGTH, dev);
*idx = *idx + BMI160_FIFO_GA_LENGTH;
(*acc_idx)++;
break;
case BMI160_FIFO_HEAD_M_A:
case BMI160_FIFO_M_A_ENABLE:
if ((*idx + BMI160_FIFO_MA_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_accel_data(&acc[*acc_idx], *idx + BMI160_FIFO_M_LENGTH, dev);
*idx = *idx + BMI160_FIFO_MA_LENGTH;
(*acc_idx)++;
break;
case BMI160_FIFO_HEAD_M_G_A:
case BMI160_FIFO_M_G_A_ENABLE:
if ((*idx + BMI160_FIFO_MGA_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_accel_data(&acc[*acc_idx], *idx + BMI160_FIFO_MG_LENGTH, dev);
*idx = *idx + BMI160_FIFO_MGA_LENGTH;
(*acc_idx)++;
break;
case BMI160_FIFO_HEAD_M:
case BMI160_FIFO_M_ENABLE:
(*idx) = (*idx) + BMI160_FIFO_M_LENGTH;
break;
case BMI160_FIFO_HEAD_G:
case BMI160_FIFO_G_ENABLE:
(*idx) = (*idx) + BMI160_FIFO_G_LENGTH;
break;
case BMI160_FIFO_HEAD_M_G:
case BMI160_FIFO_M_G_ENABLE:
(*idx) = (*idx) + BMI160_FIFO_MG_LENGTH;
break;
default:
break;
 }
}
static void unpack_accel_data(struct bmi160_sensor_data *accel_data, uint16_t data_start_index, const struct bmi160_dev *dev)
{
 uint16_t data_lsb;
 uint16_t data_msb; 
 data_lsb = dev->fifo->data[data_start_index++];
 data_msb = dev->fifo->data[data_start_index++];
 accel_data->x = (int16_t)((data_msb << 8) | data_lsb); 
 data_lsb = dev->fifo->data[data_start_index++];
 data_msb = dev->fifo->data[data_start_index++];
 accel_data->y = (int16_t)((data_msb << 8) | data_lsb); 
 data_lsb = dev->fifo->data[data_start_index++];
 data_msb = dev->fifo->data[data_start_index++];
 accel_data->z = (int16_t)((data_msb << 8) | data_lsb);
}
static void extract_accel_header_mode(struct bmi160_sensor_data *accel_data,uint8_t *accel_length,const struct bmi160_dev *dev)
{
 uint8_t frame_header = 0;
 uint16_t data_index;
 uint8_t accel_index = 0; for (data_index = dev->fifo->accel_byte_start_idx; data_index < dev->fifo->length;)
 {

frame_header = (dev->fifo->data[data_index] & BMI160_FIFO_TAG_INTR_MASK);
data_index++;
switch (frame_header)
{

case BMI160_FIFO_HEAD_A:
case BMI160_FIFO_HEAD_M_A:
case BMI160_FIFO_HEAD_G_A:
case BMI160_FIFO_HEAD_M_G_A:
 unpack_accel_frame(accel_data, &data_index, &accel_index, frame_header, dev);
 break;
case BMI160_FIFO_HEAD_M:
 move_next_frame(&data_index, BMI160_FIFO_M_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_G:
 move_next_frame(&data_index, BMI160_FIFO_G_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_M_G:
 move_next_frame(&data_index, BMI160_FIFO_MG_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_SENSOR_TIME:
 unpack_sensortime_frame(&data_index, dev);
 break;
case BMI160_FIFO_HEAD_SKIP_FRAME:
 unpack_skipped_frame(&data_index, dev);
 break;
case BMI160_FIFO_HEAD_INPUT_CONFIG:
 move_next_frame(&data_index, 1, dev);
 break;
case BMI160_FIFO_HEAD_OVER_READ: 
 data_index = dev->fifo->length;
 break;
default:
 break;
}
if (*accel_length == accel_index)
{

break;
}
 } 
 *accel_length = accel_index; 
 dev->fifo->accel_byte_start_idx = data_index;
}
static void get_gyro_len_to_parse(uint16_t *data_index,uint16_t *data_read_length,const uint8_t *gyro_frame_count,const struct bmi160_dev *dev)
{
 
 *data_index = dev->fifo->gyro_byte_start_idx;
 if (dev->fifo->fifo_data_enable == BMI160_FIFO_G_ENABLE)
 {
*data_read_length = (*gyro_frame_count) * BMI160_FIFO_G_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_G_A_ENABLE)
 {
*data_read_length = (*gyro_frame_count) * BMI160_FIFO_GA_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_ENABLE)
 {
*data_read_length = (*gyro_frame_count) * BMI160_FIFO_MG_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_A_ENABLE)
 {
*data_read_length = (*gyro_frame_count) * BMI160_FIFO_MGA_LENGTH;
 }
 else
 {

*data_index = dev->fifo->length;
 } if (*data_read_length > dev->fifo->length)
 {

*data_read_length = dev->fifo->length;
 }
}
static void unpack_gyro_frame(struct bmi160_sensor_data *gyro, uint16_t *idx, uint8_t *gyro_idx, uint8_t frame_info, const struct bmi160_dev *dev)
{
 switch (frame_info)
 {
case BMI160_FIFO_HEAD_G:
case BMI160_FIFO_G_ENABLE:
if ((*idx + BMI160_FIFO_G_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_gyro_data(&gyro[*gyro_idx], *idx, dev);
(*idx) = (*idx) + BMI160_FIFO_G_LENGTH;
(*gyro_idx)++;
break;
case BMI160_FIFO_HEAD_G_A:
case BMI160_FIFO_G_A_ENABLE:
if ((*idx + BMI160_FIFO_GA_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_gyro_data(&gyro[*gyro_idx], *idx, dev);
*idx = *idx + BMI160_FIFO_GA_LENGTH;
(*gyro_idx)++;
break;
case BMI160_FIFO_HEAD_M_G_A:
case BMI160_FIFO_M_G_A_ENABLE:
if ((*idx + BMI160_FIFO_MGA_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_gyro_data(&gyro[*gyro_idx], *idx + BMI160_FIFO_M_LENGTH, dev);
*idx = *idx + BMI160_FIFO_MGA_LENGTH;
(*gyro_idx)++;
break;
case BMI160_FIFO_HEAD_M_A:
case BMI160_FIFO_M_A_ENABLE:
*idx = *idx + BMI160_FIFO_MA_LENGTH;
break;
case BMI160_FIFO_HEAD_M:
case BMI160_FIFO_M_ENABLE:
(*idx) = (*idx) + BMI160_FIFO_M_LENGTH;
break;
case BMI160_FIFO_HEAD_M_G:
case BMI160_FIFO_M_G_ENABLE:
if ((*idx + BMI160_FIFO_MG_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_gyro_data(&gyro[*gyro_idx], *idx + BMI160_FIFO_M_LENGTH, dev);
(*idx) = (*idx) + BMI160_FIFO_MG_LENGTH;
(*gyro_idx)++;
break;
case BMI160_FIFO_HEAD_A:
case BMI160_FIFO_A_ENABLE:
*idx = *idx + BMI160_FIFO_A_LENGTH;
break;
default:
break;
 }
}
static void unpack_gyro_data(struct bmi160_sensor_data *gyro_data,uint16_t data_start_index,const struct bmi160_dev *dev)
{
 uint16_t data_lsb;
 uint16_t data_msb; 
 data_lsb = dev->fifo->data[data_start_index++];
 data_msb = dev->fifo->data[data_start_index++];
 gyro_data->x = (int16_t)((data_msb << 8) | data_lsb); 
 data_lsb = dev->fifo->data[data_start_index++];
 data_msb = dev->fifo->data[data_start_index++];
 gyro_data->y = (int16_t)((data_msb << 8) | data_lsb); 
 data_lsb = dev->fifo->data[data_start_index++];
 data_msb = dev->fifo->data[data_start_index++];
 gyro_data->z = (int16_t)((data_msb << 8) | data_lsb);
}
static void extract_gyro_header_mode(struct bmi160_sensor_data *gyro_data,uint8_t *gyro_length,const struct bmi160_dev *dev)
{
 uint8_t frame_header = 0;
 uint16_t data_index;
 uint8_t gyro_index = 0; for (data_index = dev->fifo->gyro_byte_start_idx; data_index < dev->fifo->length;)
 {

frame_header = (dev->fifo->data[data_index] & BMI160_FIFO_TAG_INTR_MASK);
data_index++;
switch (frame_header)
{

case BMI160_FIFO_HEAD_G:
case BMI160_FIFO_HEAD_G_A:
case BMI160_FIFO_HEAD_M_G:
case BMI160_FIFO_HEAD_M_G_A:
 unpack_gyro_frame(gyro_data, &data_index, &gyro_index, frame_header, dev);
 break;
case BMI160_FIFO_HEAD_A:
 move_next_frame(&data_index, BMI160_FIFO_A_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_M:
 move_next_frame(&data_index, BMI160_FIFO_M_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_M_A:
 move_next_frame(&data_index, BMI160_FIFO_M_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_SENSOR_TIME:
 unpack_sensortime_frame(&data_index, dev);
 break;
case BMI160_FIFO_HEAD_SKIP_FRAME:
 unpack_skipped_frame(&data_index, dev);
 break;
case BMI160_FIFO_HEAD_INPUT_CONFIG:
 move_next_frame(&data_index, 1, dev);
 break;
case BMI160_FIFO_HEAD_OVER_READ: 
 data_index = dev->fifo->length;
 break;
default:
 break;
}
if (*gyro_length == gyro_index)
{

break;
}
 } 
 *gyro_length = gyro_index; 
 dev->fifo->gyro_byte_start_idx = data_index;
}

static void check_frame_validity(uint16_t *data_index, const struct bmi160_dev *dev)
{
 if ((*data_index + 2) < dev->fifo->length)
 {

if ((dev->fifo->data[*data_index] == FIFO_CONFIG_MSB_CHECK) &&
(dev->fifo->data[*data_index + 1] == FIFO_CONFIG_LSB_CHECK))
{

*data_index = dev->fifo->length;
}
 }
}
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bmi160_dev *dev)
{
 
 if ((*data_index + current_frame_length) > dev->fifo->length)
 {

*data_index = dev->fifo->length;
 }
 else
 {

*data_index = *data_index + current_frame_length;
 }
}
static void unpack_sensortime_frame(uint16_t *data_index, const struct bmi160_dev *dev)
{
 uint32_t sensor_time_byte3 = 0;
 uint16_t sensor_time_byte2 = 0;
 uint8_t sensor_time_byte1 = 0; 
 if ((*data_index + BMI160_SENSOR_TIME_LENGTH) > dev->fifo->length)
 {

*data_index = dev->fifo->length;
 }
 else
 {
sensor_time_byte3 = dev->fifo->data[(*data_index) + BMI160_SENSOR_TIME_MSB_BYTE] << 16;
sensor_time_byte2 = dev->fifo->data[(*data_index) + BMI160_SENSOR_TIME_XLSB_BYTE] << 8;
sensor_time_byte1 = dev->fifo->data[(*data_index)];
dev->fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
*data_index = (*data_index) + BMI160_SENSOR_TIME_LENGTH;
 }
}
static void unpack_skipped_frame(uint16_t *data_index, const struct bmi160_dev *dev)
{
 
 if (*data_index >= dev->fifo->length)
 {

*data_index = dev->fifo->length;
 }
 else
 {
dev->fifo->skipped_frame_count = dev->fifo->data[*data_index];
*data_index = (*data_index) + 1;
 }
}
static int8_t get_foc_status(uint8_t *foc_status, struct bmi160_dev const *dev)
{
 int8_t rslt;
 uint8_t data; 
 rslt = bmi160_get_regs(BMI160_STATUS_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {

*foc_status = BMI160_GET_BITS(data, BMI160_FOC_STATUS);
 } return rslt;
}
static int8_t configure_offset_enable(const struct bmi160_foc_conf *foc_conf, struct bmi160_dev const *dev)
{
 int8_t rslt;
 uint8_t data; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

rslt = bmi160_get_regs(BMI160_OFFSET_CONF_ADDR, &data, 1, dev);
if (rslt == BMI160_OK)
{

data = BMI160_SET_BITS(data, BMI160_GYRO_OFFSET_EN, foc_conf->gyro_off_en);
data = BMI160_SET_BITS(data, BMI160_ACCEL_OFFSET_EN, foc_conf->acc_off_en);
rslt = bmi160_set_regs(BMI160_OFFSET_CONF_ADDR, &data, 1, dev);
}
 } return rslt;
}static int8_t trigger_foc(struct bmi160_offsets *offset, struct bmi160_dev const *dev)
{
 int8_t rslt;
 uint8_t foc_status = BMI160_ENABLE;
 uint8_t cmd = BMI160_START_FOC_CMD;
 uint8_t timeout = 0;
 uint8_t data_array[20]; 
 rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &cmd, 1, dev);
 if (rslt == BMI160_OK)
 {

rslt = get_foc_status(&foc_status, dev);if ((rslt != BMI160_OK) || (foc_status != BMI160_ENABLE))
{
while ((foc_status != BMI160_ENABLE) && (timeout < 11))
{
 
 dev->delay_ms(25); 
 rslt = get_foc_status(&foc_status, dev);
 timeout++;
}if ((rslt == BMI160_OK) && (foc_status == BMI160_ENABLE))
{
 
 rslt = bmi160_get_offsets(offset, dev);
}
else
{
 
 rslt = BMI160_E_FOC_FAILURE;
}
}if (rslt == BMI160_OK)
{

rslt = bmi160_get_regs(BMI160_GYRO_DATA_ADDR, data_array, 20, dev);
}
 } return rslt;
}
#ifdef BMI_STEP_COUNTER
int8_t bmi160_set_step_counter(uint8_t step_cnt_enable, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = bmi160_get_regs(BMI160_INT_STEP_CONFIG_1_ADDR, &data, 1, dev);
if (rslt == BMI160_OK)
{
if (step_cnt_enable == BMI160_ENABLE)
{
 data |= (uint8_t)(step_cnt_enable << 3);
}
else
{
 data &= ~BMI160_STEP_COUNT_EN_BIT_MASK;
}rslt = bmi160_set_regs(BMI160_INT_STEP_CONFIG_1_ADDR, &data, 1, dev);
}
 } return rslt;
}
int8_t bmi160_read_step_counter(uint16_t *step_val, const struct bmi160_dev *dev)
{

 int8_t rslt;
 uint8_t data[2] = { 0, 0 };
 uint16_t msb = 0;
 uint8_t lsb = 0; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = bmi160_get_regs(BMI160_INT_STEP_CNT_0_ADDR, data, 2, dev);
if (rslt == BMI160_OK)
{
lsb = data[0];
msb = data[1] << 8;
*step_val = msb | lsb;
}
 } return rslt;
}

#endif
#ifdef BMI_AUX_TYPE
static void get_aux_len_to_parse(uint16_t *data_index, uint16_t *data_read_length, const uint8_t *aux_frame_count, const struct bmi160_dev *dev);
static void unpack_aux_frame(struct bmi160_aux_data *aux_data,uint16_t *idx,uint8_t *aux_index,uint8_t frame_info,const struct bmi160_dev *dev);
static void unpack_aux_data(struct bmi160_aux_data *aux_data, uint16_t data_start_index, const struct bmi160_dev *dev);
static void extract_aux_header_mode(struct bmi160_aux_data *aux_data, uint8_t *aux_length, const struct bmi160_dev *dev);
static int8_t config_aux_odr(const struct bmi160_dev *dev);
static int8_t config_aux_settg(const struct bmi160_dev *dev);
static int8_t extract_aux_read(uint16_t map_len,uint8_t reg_addr,uint8_t *aux_data,uint16_t len,const struct bmi160_dev *dev);
#endif

																	
#ifdef BMI_TEST_TYPE
static int8_t perform_accel_self_test(struct bmi160_dev *dev);
static int8_t enable_accel_self_test(struct bmi160_dev *dev);
static int8_t accel_self_test_positive_excitation(struct bmi160_sensor_data *accel_pos, const struct bmi160_dev *dev);
static int8_t accel_self_test_negative_excitation(struct bmi160_sensor_data *accel_neg, const struct bmi160_dev *dev);
static int8_t validate_accel_self_test(const struct bmi160_sensor_data *accel_pos,const struct bmi160_sensor_data *accel_neg);
static int8_t perform_gyro_self_test(const struct bmi160_dev *dev);
static int8_t enable_gyro_self_test(const struct bmi160_dev *dev);
static int8_t validate_gyro_self_test(const struct bmi160_dev *dev);
#endif


#ifdef BMI_INT_TYPE
int8_t bmi160_set_int_config(struct bmi160_int_settg *int_config, struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK; switch (int_config->int_type)
 {
case BMI160_ACC_ANY_MOTION_INT:
rslt = set_accel_any_motion_int(int_config, dev);
break;
case BMI160_ACC_SIG_MOTION_INT:
rslt = set_accel_sig_motion_int(int_config, dev);
break;
case BMI160_ACC_SLOW_NO_MOTION_INT:
rslt = set_accel_no_motion_int(int_config, dev);
break;
case BMI160_ACC_DOUBLE_TAP_INT:
case BMI160_ACC_SINGLE_TAP_INT:
rslt = set_accel_tap_int(int_config, dev);
break;
case BMI160_STEP_DETECT_INT:
rslt = set_accel_step_detect_int(int_config, dev);
break;
case BMI160_ACC_ORIENT_INT:
rslt = set_accel_orientation_int(int_config, dev);
break;
case BMI160_ACC_FLAT_INT:
rslt = set_accel_flat_detect_int(int_config, dev);
break;
case BMI160_ACC_LOW_G_INT:
rslt = set_accel_low_g_int(int_config, dev);
break;
case BMI160_ACC_HIGH_G_INT:
rslt = set_accel_high_g_int(int_config, dev);
break;
case BMI160_ACC_GYRO_DATA_RDY_INT:
rslt = set_accel_gyro_data_ready_int(int_config, dev);
break;
case BMI160_ACC_GYRO_FIFO_FULL_INT:
rslt = set_fifo_full_int(int_config, dev);
break;
case BMI160_ACC_GYRO_FIFO_WATERMARK_INT:
rslt = set_fifo_watermark_int(int_config, dev);
break;
case BMI160_FIFO_TAG_INT_PIN:

rslt = set_intr_pin_config(int_config, dev);
break;
default:
break;
 } return rslt;
}

#endif


#ifdef BMI_AUX_TYPE
int8_t bmi160_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK;
 uint16_t map_len = 0; 
 if ((dev == NULL) || (dev->read == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
if (dev->aux_cfg.aux_sensor_enable == BMI160_ENABLE)
{
rslt = map_read_len(&map_len, dev);
if (rslt == BMI160_OK)
{
 rslt = extract_aux_read(map_len, reg_addr, aux_data, len, dev);
}
}
else
{
rslt = BMI160_E_INVALID_INPUT;
}
 } return rslt;
}
int8_t bmi160_aux_write(uint8_t reg_addr, uint8_t *aux_data, uint16_t len, const struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK;
 uint8_t count = 0; 
 if ((dev == NULL) || (dev->write == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
for (; count < len; count++)
{

rslt = bmi160_set_regs(BMI160_AUX_IF_4_ADDR, aux_data, 1, dev);
dev->delay_ms(BMI160_AUX_COM_DELAY);
if (rslt == BMI160_OK)
{
 
 rslt = bmi160_set_regs(BMI160_AUX_IF_3_ADDR, &reg_addr, 1, dev);
 dev->delay_ms(BMI160_AUX_COM_DELAY);
 if (rslt == BMI160_OK && (count < len - 1))
 {
aux_data++;
reg_addr++;
 }
}
}
 } return rslt;
}
int8_t bmi160_aux_init(const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
if (dev->aux_cfg.aux_sensor_enable == BMI160_ENABLE)
{

rslt = config_aux_settg(dev);
}
else
{
rslt = BMI160_E_INVALID_INPUT;
}
 } return rslt;
}
int8_t bmi160_set_aux_auto_mode(uint8_t *data_addr, struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
if (dev->aux_cfg.aux_sensor_enable == BMI160_ENABLE)
{

rslt = bmi160_set_regs(BMI160_AUX_IF_2_ADDR, data_addr, 1, dev);
dev->delay_ms(BMI160_AUX_COM_DELAY);
if (rslt == BMI160_OK)
{
 
 rslt = config_aux_odr(dev);
 if (rslt == BMI160_OK)
 {

dev->aux_cfg.manual_enable = BMI160_DISABLE;
rslt = bmi160_config_aux_mode(dev);
 }
}
}
else
{
rslt = BMI160_E_INVALID_INPUT;
}
 } return rslt;
}
int8_t bmi160_config_aux_mode(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t aux_if[2] = { (uint8_t)(dev->aux_cfg.aux_i2c_addr * 2), 0 }; rslt = bmi160_get_regs(BMI160_AUX_IF_1_ADDR, &aux_if[1], 1, dev);
 if (rslt == BMI160_OK)
 {

aux_if[1] = BMI160_SET_BITS(aux_if[1], BMI160_MANUAL_MODE_EN, dev->aux_cfg.manual_enable);
aux_if[1] = BMI160_SET_BITS_POS_0(aux_if[1], BMI160_AUX_READ_BURST, dev->aux_cfg.aux_rd_burst_len);
rslt = bmi160_set_regs(BMI160_AUX_IF_0_ADDR, &aux_if[0], 2, dev);
dev->delay_ms(BMI160_AUX_COM_DELAY);
 } return rslt;
}
int8_t bmi160_read_aux_data_auto_mode(uint8_t *aux_data, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
if ((dev->aux_cfg.aux_sensor_enable == BMI160_ENABLE) && (dev->aux_cfg.manual_enable == BMI160_DISABLE))
{

rslt = bmi160_get_regs(BMI160_AUX_DATA_ADDR, aux_data, 8, dev);
}
else
{
rslt = BMI160_E_INVALID_INPUT;
}
 } return rslt;
}

#endif


#ifdef BMI_AUX_TYPE
int8_t bmi160_extract_aux(struct bmi160_aux_data *aux_data, uint8_t *aux_len, struct bmi160_dev const *dev)
{
 int8_t rslt = 0;
 uint16_t data_index = 0;
 uint16_t data_read_length = 0;
 uint8_t aux_index = 0;
 uint8_t fifo_data_enable = 0; if ((dev == NULL) || (dev->fifo->data == NULL) || (aux_data == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

if (dev->fifo->fifo_header_enable == 0)
{

get_aux_len_to_parse(&data_index, &data_read_length, aux_len, dev);
for (; data_index < data_read_length;)
{
 
 check_frame_validity(&data_index, dev);
 fifo_data_enable = dev->fifo->fifo_data_enable;
 unpack_aux_frame(aux_data, &data_index, &aux_index, fifo_data_enable, dev);
}
*aux_len = aux_index;
dev->fifo->aux_byte_start_idx = data_index;
}
else
{

extract_aux_header_mode(aux_data, aux_len, dev);
}
 } return rslt;
}
#endif


#ifdef BMI_INT_TYPE
int8_t bmi160_get_int_status(enum bmi160_int_status_sel int_status_sel,union bmi160_int_status *int_status,struct bmi160_dev const *dev)
{
 int8_t rslt = 0; 
 if (int_status_sel == BMI160_INT_STATUS_ALL)
 {
rslt = bmi160_get_regs(BMI160_INT_STATUS_ADDR, &int_status->data[0], 4, dev);
 }
 else
 {
if (int_status_sel & BMI160_INT_STATUS_0)
{
rslt = bmi160_get_regs(BMI160_INT_STATUS_ADDR, &int_status->data[0], 1, dev);
}if (int_status_sel & BMI160_INT_STATUS_1)
{
rslt = bmi160_get_regs(BMI160_INT_STATUS_ADDR + 1, &int_status->data[1], 1, dev);
}if (int_status_sel & BMI160_INT_STATUS_2)
{
rslt = bmi160_get_regs(BMI160_INT_STATUS_ADDR + 2, &int_status->data[2], 1, dev);
}if (int_status_sel & BMI160_INT_STATUS_3)
{
rslt = bmi160_get_regs(BMI160_INT_STATUS_ADDR + 3, &int_status->data[3], 1, dev);
}
 } return rslt;
}


static int8_t set_accel_any_motion_int(struct bmi160_int_settg *int_config, struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg = &(int_config->int_type_cfg.acc_any_motion_int);
rslt = enable_accel_any_motion_int(any_motion_int_cfg, dev);
if (rslt == BMI160_OK)
{
rslt = config_any_motion_int_settg(int_config, any_motion_int_cfg, dev);
}
 } return rslt;
}
static int8_t set_accel_tap_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_tap_int_cfg *tap_int_cfg = &(int_config->int_type_cfg.acc_tap_int);
rslt = enable_tap_int(int_config, tap_int_cfg, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 rslt = config_tap_int_settg(int_config, tap_int_cfg, dev);
}
}
 } return rslt;
}
static int8_t set_accel_gyro_data_ready_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
rslt = enable_data_ready_int(dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 rslt = map_hardware_interrupt(int_config, dev);
}
}
 } return rslt;
}
static int8_t set_accel_sig_motion_int(struct bmi160_int_settg *int_config, struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg = &(int_config->int_type_cfg.acc_sig_motion_int);
rslt = enable_sig_motion_int(sig_mot_int_cfg, dev);
if (rslt == BMI160_OK)
{
rslt = config_sig_motion_int_settg(int_config, sig_mot_int_cfg, dev);
}
 } return rslt;
}
static int8_t set_accel_no_motion_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg = &(int_config->int_type_cfg.acc_no_motion_int);
rslt = enable_no_motion_int(no_mot_int_cfg, dev);
if (rslt == BMI160_OK)
{

rslt = config_no_motion_int_settg(int_config, no_mot_int_cfg, dev);
}
 } return rslt;
}
static int8_t set_accel_step_detect_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_step_detect_int_cfg *step_detect_int_cfg = &(int_config->int_type_cfg.acc_step_detect_int);
rslt = enable_step_detect_int(step_detect_int_cfg, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 rslt = map_feature_interrupt(int_config, dev);
 if (rslt == BMI160_OK)
 {
rslt = config_step_detect(step_detect_int_cfg, dev);
 }
}
}
 } return rslt;
}
static int8_t set_accel_orientation_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_orient_int_cfg *orient_int_cfg = &(int_config->int_type_cfg.acc_orient_int);
rslt = enable_orient_int(orient_int_cfg, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 
 rslt = map_feature_interrupt(int_config, dev);
 if (rslt == BMI160_OK)
 {

rslt = config_orient_int_settg(orient_int_cfg, dev);
 }
}
}
 } return rslt;
}
static int8_t set_accel_flat_detect_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_flat_detect_int_cfg *flat_detect_int = &(int_config->int_type_cfg.acc_flat_int);
rslt = enable_flat_int(flat_detect_int, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 
 rslt = map_feature_interrupt(int_config, dev);
 if (rslt == BMI160_OK)
 {

rslt = config_flat_int_settg(flat_detect_int, dev);
 }
}
}
 } return rslt;
}
static int8_t set_accel_low_g_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_low_g_int_cfg *low_g_int = &(int_config->int_type_cfg.acc_low_g_int);
rslt = enable_low_g_int(low_g_int, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 
 rslt = map_feature_interrupt(int_config, dev);
 if (rslt == BMI160_OK)
 {

rslt = config_low_g_data_src(low_g_int, dev);
if (rslt == BMI160_OK)
{
rslt = config_low_g_int_settg(low_g_int, dev);
}
 }
}
}
 } return rslt;
}
static int8_t set_accel_high_g_int(struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = null_ptr_check(dev);
 if ((rslt != BMI160_OK) || (int_config == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

struct bmi160_acc_high_g_int_cfg *high_g_int_cfg = &(int_config->int_type_cfg.acc_high_g_int);
rslt = enable_high_g_int(high_g_int_cfg, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 
 rslt = map_feature_interrupt(int_config, dev);
 if (rslt == BMI160_OK)
 {

rslt = config_high_g_data_src(high_g_int_cfg, dev);
if (rslt == BMI160_OK)
{
rslt = config_high_g_int_settg(high_g_int_cfg, dev);
}
 }
}
}
 } return rslt;
}
static int8_t set_intr_pin_config(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = config_int_out_ctrl(int_config, dev);
 if (rslt == BMI160_OK)
 {
rslt = config_int_latch(int_config, dev);
 } return rslt;
}
#endif

#ifdef BMI_INT_TYPE
static int8_t enable_accel_any_motion_int(const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
if (any_motion_int_cfg->anymotion_en == BMI160_ENABLE)
{
temp = data & ~BMI160_ANY_MOTION_X_INT_EN_MASK;
data = temp | (any_motion_int_cfg->anymotion_x & BMI160_ANY_MOTION_X_INT_EN_MASK);
temp = data & ~BMI160_ANY_MOTION_Y_INT_EN_MASK;
data = temp | ((any_motion_int_cfg->anymotion_y << 1) & BMI160_ANY_MOTION_Y_INT_EN_MASK);
temp = data & ~BMI160_ANY_MOTION_Z_INT_EN_MASK;
data = temp | ((any_motion_int_cfg->anymotion_z << 2) & BMI160_ANY_MOTION_Z_INT_EN_MASK);
dev->any_sig_sel = BMI160_ANY_MOTION_ENABLED;
}
else
{
data = data & ~BMI160_ANY_MOTION_ALL_INT_EN_MASK;
dev->any_sig_sel = BMI160_BOTH_ANY_SIG_MOTION_DISABLED;
}
rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t disable_sig_motion_int(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = (data & BMI160_SIG_MOTION_SEL_MASK);
if (temp)
{
temp = data & ~BMI160_SIG_MOTION_SEL_MASK;
data = temp;
rslt = bmi160_set_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
}
 } return rslt;
}
static int8_t map_feature_interrupt(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data[3] = { 0, 0, 0 };
 uint8_t temp[3] = { 0, 0, 0 }; rslt = bmi160_get_regs(BMI160_INT_MAP_0_ADDR, data, 3, dev);
 if (rslt == BMI160_OK)
 {
temp[0] = data[0] & ~int_mask_lookup_table[int_config->int_type];
temp[2] = data[2] & ~int_mask_lookup_table[int_config->int_type];
switch (int_config->int_channel)
{
case BMI160_INT_CHANNEL_NONE:
 data[0] = temp[0];
 data[2] = temp[2];
 break;
case BMI160_INT_CHANNEL_1:
 data[0] = temp[0] | int_mask_lookup_table[int_config->int_type];
 data[2] = temp[2];
 break;
case BMI160_INT_CHANNEL_2:
 data[2] = temp[2] | int_mask_lookup_table[int_config->int_type];
 data[0] = temp[0];
 break;
case BMI160_INT_CHANNEL_BOTH:
 data[0] = temp[0] | int_mask_lookup_table[int_config->int_type];
 data[2] = temp[2] | int_mask_lookup_table[int_config->int_type];
 break;
default:
 rslt = BMI160_E_OUT_OF_RANGE;
}
if (rslt == BMI160_OK)
{
rslt = bmi160_set_regs(BMI160_INT_MAP_0_ADDR, data, 3, dev);
}
 } return rslt;
}
static int8_t map_hardware_interrupt(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; rslt = bmi160_get_regs(BMI160_INT_MAP_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~int_mask_lookup_table[int_config->int_type];
temp = temp & ~((uint8_t)(int_mask_lookup_table[int_config->int_type] << 4));
switch (int_config->int_channel)
{
case BMI160_INT_CHANNEL_NONE:
 data = temp;
 break;
case BMI160_INT_CHANNEL_1:
 data = temp | (uint8_t)((int_mask_lookup_table[int_config->int_type]) << 4);
 break;
case BMI160_INT_CHANNEL_2:
 data = temp | int_mask_lookup_table[int_config->int_type];
 break;
case BMI160_INT_CHANNEL_BOTH:
 data = temp | int_mask_lookup_table[int_config->int_type];
 data = data | (uint8_t)((int_mask_lookup_table[int_config->int_type]) << 4);
 break;
default:
 rslt = BMI160_E_OUT_OF_RANGE;
}
if (rslt == BMI160_OK)
{
rslt = bmi160_set_regs(BMI160_INT_MAP_1_ADDR, &data, 1, dev);
}
 } return rslt;
}
static int8_t config_any_motion_src(const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_MOTION_SRC_INT_MASK;
data = temp | ((any_motion_int_cfg->anymotion_data_src << 7) & BMI160_MOTION_SRC_INT_MASK);
rslt = bmi160_set_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_any_dur_threshold(const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0;
 uint8_t data_array[2] = { 0 };
 uint8_t dur; 
 rslt = bmi160_get_regs(BMI160_INT_MOTION_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {

dur = (uint8_t)any_motion_int_cfg->anymotion_dur;
temp = data & ~BMI160_SLOPE_INT_DUR_MASK;
data = temp | (dur & BMI160_MOTION_SRC_INT_MASK);
data_array[0] = data;
data_array[1] = any_motion_int_cfg->anymotion_thr; 
rslt = bmi160_set_regs(BMI160_INT_MOTION_0_ADDR, data_array, 2, dev);
 } return rslt;
}

static int8_t config_any_motion_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_any_mot_int_cfg *any_motion_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = set_intr_pin_config(int_config, dev);
 if (rslt == BMI160_OK)
 {
rslt = disable_sig_motion_int(dev);
if (rslt == BMI160_OK)
{
rslt = map_feature_interrupt(int_config, dev);
if (rslt == BMI160_OK)
{
 rslt = config_any_motion_src(any_motion_int_cfg, dev);
 if (rslt == BMI160_OK)
 {
rslt = config_any_dur_threshold(any_motion_int_cfg, dev);
 }
}
}
 } return rslt;
}
static int8_t enable_data_ready_int(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_DATA_RDY_INT_EN_MASK;
data = temp | ((1 << 4) & BMI160_DATA_RDY_INT_EN_MASK);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t enable_no_motion_int(const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
if (no_mot_int_cfg->no_motion_x == 1)
{
temp = data & ~BMI160_NO_MOTION_X_INT_EN_MASK;
data = temp | (1 & BMI160_NO_MOTION_X_INT_EN_MASK);
}if (no_mot_int_cfg->no_motion_y == 1)
{
temp = data & ~BMI160_NO_MOTION_Y_INT_EN_MASK;
data = temp | ((1 << 1) & BMI160_NO_MOTION_Y_INT_EN_MASK);
}if (no_mot_int_cfg->no_motion_z == 1)
{
temp = data & ~BMI160_NO_MOTION_Z_INT_EN_MASK;
data = temp | ((1 << 2) & BMI160_NO_MOTION_Z_INT_EN_MASK);
}
rslt = bmi160_set_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_no_motion_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = set_intr_pin_config(int_config, dev);
 if (rslt == BMI160_OK)
 {
rslt = map_feature_interrupt(int_config, dev);
if (rslt == BMI160_OK)
{
rslt = config_no_motion_data_src(no_mot_int_cfg, dev);
if (rslt == BMI160_OK)
{
 rslt = config_no_motion_dur_thr(no_mot_int_cfg, dev);
}
}
 } return rslt;
}
static int8_t config_no_motion_data_src(const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_MOTION_SRC_INT_MASK;
data = temp | ((no_mot_int_cfg->no_motion_src << 7) & BMI160_MOTION_SRC_INT_MASK);
rslt = bmi160_set_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_no_motion_dur_thr(const struct bmi160_acc_no_motion_int_cfg *no_mot_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0;
 uint8_t temp_1 = 0;
 uint8_t reg_addr;
 uint8_t data_array[2] = { 0 }; 
 reg_addr = BMI160_INT_MOTION_0_ADDR;
 rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_NO_MOTION_INT_DUR_MASK;
data = temp | ((no_mot_int_cfg->no_motion_dur << 2) & BMI160_NO_MOTION_INT_DUR_MASK);
rslt = bmi160_set_regs(reg_addr, &data, 1, dev);
if (rslt == BMI160_OK)
{
reg_addr = BMI160_INT_MOTION_3_ADDR;
rslt = bmi160_get_regs(reg_addr, &data, 1, dev);
if (rslt == BMI160_OK)
{
 temp = data & ~BMI160_NO_MOTION_SEL_BIT_MASK; 
 temp_1 = (no_mot_int_cfg->no_motion_sel & BMI160_NO_MOTION_SEL_BIT_MASK);
 data = (temp | temp_1);
 data_array[1] = data; 
 data_array[0] = no_mot_int_cfg->no_motion_thres;
 reg_addr = BMI160_INT_MOTION_2_ADDR; 
 rslt = bmi160_set_regs(reg_addr, data_array, 2, dev);
}
}
 } return rslt;
}
static int8_t enable_sig_motion_int(const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg, struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
if (sig_mot_int_cfg->sig_en == BMI160_ENABLE)
{
temp = data & ~BMI160_SIG_MOTION_INT_EN_MASK;
data = temp | (7 & BMI160_SIG_MOTION_INT_EN_MASK);
dev->any_sig_sel = BMI160_SIG_MOTION_ENABLED;
}
else
{
data = data & ~BMI160_SIG_MOTION_INT_EN_MASK;
dev->any_sig_sel = BMI160_BOTH_ANY_SIG_MOTION_DISABLED;
}
rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_sig_motion_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = set_intr_pin_config(int_config, dev);
 if (rslt == BMI160_OK)
 {
rslt = map_feature_interrupt(int_config, dev);
if (rslt == BMI160_OK)
{
rslt = config_sig_motion_data_src(sig_mot_int_cfg, dev);
if (rslt == BMI160_OK)
{
 rslt = config_sig_dur_threshold(sig_mot_int_cfg, dev);
}
}
 } return rslt;
}
static int8_t config_sig_motion_data_src(const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_MOTION_SRC_INT_MASK;
data = temp | ((sig_mot_int_cfg->sig_data_src << 7) & BMI160_MOTION_SRC_INT_MASK);
rslt = bmi160_set_regs(BMI160_INT_DATA_1_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_sig_dur_threshold(const struct bmi160_acc_sig_mot_int_cfg *sig_mot_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data;
 uint8_t temp = 0;
 data = sig_mot_int_cfg->sig_mot_thres; 
 rslt = bmi160_set_regs(BMI160_INT_MOTION_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
rslt = bmi160_get_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
if (rslt == BMI160_OK)
{
temp = data & ~BMI160_SIG_MOTION_SKIP_MASK;
data = temp | ((sig_mot_int_cfg->sig_mot_skip << 2) & BMI160_SIG_MOTION_SKIP_MASK);
temp = data & ~BMI160_SIG_MOTION_PROOF_MASK;
data = temp | ((sig_mot_int_cfg->sig_mot_proof << 4) & BMI160_SIG_MOTION_PROOF_MASK);
temp = data & ~BMI160_SIG_MOTION_SEL_MASK;
data = temp | ((sig_mot_int_cfg->sig_en << 1) & BMI160_SIG_MOTION_SEL_MASK);
rslt = bmi160_set_regs(BMI160_INT_MOTION_3_ADDR, &data, 1, dev);
}
 } return rslt;
}
static int8_t enable_step_detect_int(const struct bmi160_acc_step_detect_int_cfg *step_detect_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_STEP_DETECT_INT_EN_MASK;
data = temp | ((step_detect_int_cfg->step_detector_en << 3) & BMI160_STEP_DETECT_INT_EN_MASK);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_2_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_step_detect(const struct bmi160_acc_step_detect_int_cfg *step_detect_int_cfg, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t temp = 0;
 uint8_t data_array[2] = { 0 }; if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_NORMAL)
 {

data_array[0] = 0x15;
data_array[1] = 0x03;
 }
 else if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_SENSITIVE)
 {

data_array[0] = 0x2D;
data_array[1] = 0x00;
 }
 else if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_ROBUST)
 {

data_array[0] = 0x1D;
data_array[1] = 0x07;
 }
 else if (step_detect_int_cfg->step_detector_mode == BMI160_STEP_DETECT_USER_DEFINE)
 {


rslt = bmi160_get_regs(BMI160_INT_STEP_CONFIG_0_ADDR, &data_array[0], 2, dev);
if (rslt == BMI160_OK)
{
temp = data_array[0] & ~BMI160_STEP_DETECT_MIN_THRES_MASK;
data_array[0] = temp | ((step_detect_int_cfg->min_threshold << 3) & BMI160_STEP_DETECT_MIN_THRES_MASK);
temp = data_array[0] & ~BMI160_STEP_DETECT_STEPTIME_MIN_MASK;
data_array[0] = temp | ((step_detect_int_cfg->steptime_min) & BMI160_STEP_DETECT_STEPTIME_MIN_MASK);
temp = data_array[1] & ~BMI160_STEP_MIN_BUF_MASK;
data_array[1] = temp | ((step_detect_int_cfg->step_min_buf) & BMI160_STEP_MIN_BUF_MASK);
}
 } 
 rslt = bmi160_set_regs(BMI160_INT_STEP_CONFIG_0_ADDR, data_array, 2, dev); return rslt;
}
static int8_t enable_tap_int(const struct bmi160_int_settg *int_config,const struct bmi160_acc_tap_int_cfg *tap_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
if (int_config->int_type == BMI160_ACC_SINGLE_TAP_INT)
{
temp = data & ~BMI160_SINGLE_TAP_INT_EN_MASK;
data = temp | ((tap_int_cfg->tap_en << 5) & BMI160_SINGLE_TAP_INT_EN_MASK);
}
else
{
temp = data & ~BMI160_DOUBLE_TAP_INT_EN_MASK;
data = temp | ((tap_int_cfg->tap_en << 4) & BMI160_DOUBLE_TAP_INT_EN_MASK);
}
rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_tap_int_settg(const struct bmi160_int_settg *int_config,const struct bmi160_acc_tap_int_cfg *tap_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = set_intr_pin_config(int_config, dev);
 if (rslt == BMI160_OK)
 {
rslt = map_feature_interrupt(int_config, dev);
if (rslt == BMI160_OK)
{
rslt = config_tap_data_src(tap_int_cfg, dev);
if (rslt == BMI160_OK)
{
 rslt = config_tap_param(int_config, tap_int_cfg, dev);
}
}
 } return rslt;
}
static int8_t config_tap_data_src(const struct bmi160_acc_tap_int_cfg *tap_int_cfg, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_TAP_SRC_INT_MASK;
data = temp | ((tap_int_cfg->tap_data_src << 3) & BMI160_TAP_SRC_INT_MASK);
rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_tap_param(const struct bmi160_int_settg *int_config,const struct bmi160_acc_tap_int_cfg *tap_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t temp = 0;
 uint8_t data = 0;
 uint8_t data_array[2] = { 0 };
 uint8_t count = 0;
 uint8_t dur, shock, quiet, thres; 
 rslt = bmi160_get_regs(BMI160_INT_TAP_0_ADDR, data_array, 2, dev);
 if (rslt == BMI160_OK)
 {
data = data_array[count];
if (int_config->int_type == BMI160_ACC_DOUBLE_TAP_INT)
{
dur = (uint8_t)tap_int_cfg->tap_dur;
temp = (data & ~BMI160_TAP_DUR_MASK);
data = temp | (dur & BMI160_TAP_DUR_MASK);
}shock = (uint8_t)tap_int_cfg->tap_shock;
temp = data & ~BMI160_TAP_SHOCK_DUR_MASK;
data = temp | ((shock << 6) & BMI160_TAP_SHOCK_DUR_MASK);
quiet = (uint8_t)tap_int_cfg->tap_quiet;
temp = data & ~BMI160_TAP_QUIET_DUR_MASK;
data = temp | ((quiet << 7) & BMI160_TAP_QUIET_DUR_MASK);
data_array[count++] = data;
data = data_array[count];
thres = (uint8_t)tap_int_cfg->tap_thr;
temp = data & ~BMI160_TAP_THRES_MASK;
data = temp | (thres & BMI160_TAP_THRES_MASK);
data_array[count++] = data; 
rslt = bmi160_set_regs(BMI160_INT_TAP_0_ADDR, data_array, count, dev);
 } return rslt;
}


static int8_t config_sec_if(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t if_conf = 0;
 uint8_t cmd = BMI160_AUX_NORMAL_MODE; 
 rslt = bmi160_set_regs(BMI160_COMMAND_REG_ADDR, &cmd, 1, dev);
 if (rslt == BMI160_OK)
 {

dev->delay_ms(1);
rslt = bmi160_get_regs(BMI160_IF_CONF_ADDR, &if_conf, 1, dev);
if_conf |= (uint8_t)(1 << 5);
if (rslt == BMI160_OK)
{

rslt = bmi160_set_regs(BMI160_IF_CONF_ADDR, &if_conf, 1, dev);
}
 } return rslt;
}
#endif

#ifdef BMI_AUX_TYPE
static int8_t config_aux_odr(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t aux_odr; rslt = bmi160_get_regs(BMI160_AUX_ODR_ADDR, &aux_odr, 1, dev);
 if (rslt == BMI160_OK)
 {
aux_odr = (uint8_t)(dev->aux_cfg.aux_odr);
rslt = bmi160_set_regs(BMI160_AUX_ODR_ADDR, &aux_odr, 1, dev);
dev->delay_ms(BMI160_AUX_COM_DELAY);
 } return rslt;
}
static int8_t map_read_len(uint16_t *len, const struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK; switch (dev->aux_cfg.aux_rd_burst_len)
 {
case BMI160_AUX_READ_LEN_0:
*len = 1;
break;
case BMI160_AUX_READ_LEN_1:
*len = 2;
break;
case BMI160_AUX_READ_LEN_2:
*len = 6;
break;
case BMI160_AUX_READ_LEN_3:
*len = 8;
break;
default:
rslt = BMI160_E_INVALID_INPUT;
break;
 } return rslt;
}
static int8_t config_aux_settg(const struct bmi160_dev *dev)
{
 int8_t rslt; rslt = config_sec_if(dev);
 if (rslt == BMI160_OK)
 {

rslt = bmi160_config_aux_mode(dev);
 } return rslt;
}
static int8_t extract_aux_read(uint16_t map_len,uint8_t reg_addr,uint8_t *aux_data,uint16_t len,const struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK;
 uint8_t data[8] = { 0, };
 uint8_t read_addr = BMI160_AUX_DATA_ADDR;
 uint8_t count = 0;
 uint8_t read_count;
 uint8_t read_len = (uint8_t)map_len; for (; count < len;)
 {

rslt = bmi160_set_regs(BMI160_AUX_IF_2_ADDR, &reg_addr, 1, dev);
dev->delay_ms(BMI160_AUX_COM_DELAY);
if (rslt == BMI160_OK)
{
rslt = bmi160_get_regs(read_addr, data, map_len, dev);
if (rslt == BMI160_OK)
{
 read_count = 0; 
 if (len < map_len)
 {
read_len = (uint8_t)len;
 }
 else if ((len - count) < map_len)
 {
read_len = (uint8_t)(len - count);
 } for (; read_count < read_len; read_count++)
 {
aux_data[count + read_count] = data[read_count];
 } reg_addr += (uint8_t)map_len;
 count += (uint8_t)map_len;
}
else
{
 rslt = BMI160_E_COM_FAIL;
 break;
}
}
 } return rslt;
}
#endif
#ifdef BMI_INT_TYPE
static int8_t enable_orient_int(const struct bmi160_acc_orient_int_cfg *orient_int_cfg, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_ORIENT_INT_EN_MASK;
data = temp | ((orient_int_cfg->orient_en << 6) & BMI160_ORIENT_INT_EN_MASK);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_orient_int_settg(const struct bmi160_acc_orient_int_cfg *orient_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0;
 uint8_t data_array[2] = { 0, 0 }; 
 rslt = bmi160_get_regs(BMI160_INT_ORIENT_0_ADDR, data_array, 2, dev);
 if (rslt == BMI160_OK)
 {
data = data_array[0];
temp = data & ~BMI160_ORIENT_MODE_MASK;
data = temp | ((orient_int_cfg->orient_mode) & BMI160_ORIENT_MODE_MASK);
temp = data & ~BMI160_ORIENT_BLOCK_MASK;
data = temp | ((orient_int_cfg->orient_blocking << 2) & BMI160_ORIENT_BLOCK_MASK);
temp = data & ~BMI160_ORIENT_HYST_MASK;
data = temp | ((orient_int_cfg->orient_hyst << 4) & BMI160_ORIENT_HYST_MASK);
data_array[0] = data;
data = data_array[1];
temp = data & ~BMI160_ORIENT_THETA_MASK;
data = temp | ((orient_int_cfg->orient_theta) & BMI160_ORIENT_THETA_MASK);
temp = data & ~BMI160_ORIENT_UD_ENABLE;
data = temp | ((orient_int_cfg->orient_ud_en << 6) & BMI160_ORIENT_UD_ENABLE);
temp = data & ~BMI160_AXES_EN_MASK;
data = temp | ((orient_int_cfg->axes_ex << 7) & BMI160_AXES_EN_MASK);
data_array[1] = data;
rslt = bmi160_set_regs(BMI160_INT_ORIENT_0_ADDR, data_array, 2, dev);
 } return rslt;
}
static int8_t enable_flat_int(const struct bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_FLAT_INT_EN_MASK;
data = temp | ((flat_int->flat_en << 7) & BMI160_FLAT_INT_EN_MASK);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_flat_int_settg(const struct bmi160_acc_flat_detect_int_cfg *flat_int, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0;
 uint8_t data_array[2] = { 0, 0 }; 
 rslt = bmi160_get_regs(BMI160_INT_FLAT_0_ADDR, data_array, 2, dev);
 if (rslt == BMI160_OK)
 {
data = data_array[0];
temp = data & ~BMI160_FLAT_THRES_MASK;
data = temp | ((flat_int->flat_theta) & BMI160_FLAT_THRES_MASK);
data_array[0] = data;
data = data_array[1];
temp = data & ~BMI160_FLAT_HOLD_TIME_MASK;
data = temp | ((flat_int->flat_hold_time << 4) & BMI160_FLAT_HOLD_TIME_MASK);
temp = data & ~BMI160_FLAT_HYST_MASK;
data = temp | ((flat_int->flat_hy) & BMI160_FLAT_HYST_MASK);
data_array[1] = data;
rslt = bmi160_set_regs(BMI160_INT_FLAT_0_ADDR, data_array, 2, dev);
 } return rslt;
}
static int8_t enable_low_g_int(const struct bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_LOW_G_INT_EN_MASK;
data = temp | ((low_g_int->low_en << 3) & BMI160_LOW_G_INT_EN_MASK);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_low_g_data_src(const struct bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_LOW_HIGH_SRC_INT_MASK;
data = temp | ((low_g_int->low_data_src << 7) & BMI160_LOW_HIGH_SRC_INT_MASK);
rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_low_g_int_settg(const struct bmi160_acc_low_g_int_cfg *low_g_int, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t temp = 0;
 uint8_t data_array[3] = { 0, 0, 0 }; 
 rslt = bmi160_get_regs(BMI160_INT_LOWHIGH_2_ADDR, &data_array[2], 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data_array[2] & ~BMI160_LOW_G_HYST_MASK;
data_array[2] = temp | (low_g_int->low_hyst & BMI160_LOW_G_HYST_MASK);
temp = data_array[2] & ~BMI160_LOW_G_LOW_MODE_MASK;
data_array[2] = temp | ((low_g_int->low_mode << 2) & BMI160_LOW_G_LOW_MODE_MASK);
data_array[1] = low_g_int->low_thres;
data_array[0] = low_g_int->low_dur;
rslt = bmi160_set_regs(BMI160_INT_LOWHIGH_0_ADDR, data_array, 3, dev);
 } return rslt;
}
static int8_t enable_high_g_int(const struct bmi160_acc_high_g_int_cfg *high_g_int_cfg, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {

temp = data & ~BMI160_HIGH_G_X_INT_EN_MASK;
data = temp | (high_g_int_cfg->high_g_x & BMI160_HIGH_G_X_INT_EN_MASK);
temp = data & ~BMI160_HIGH_G_Y_INT_EN_MASK;
data = temp | ((high_g_int_cfg->high_g_y << 1) & BMI160_HIGH_G_Y_INT_EN_MASK);
temp = data & ~BMI160_HIGH_G_Z_INT_EN_MASK;
data = temp | ((high_g_int_cfg->high_g_z << 2) & BMI160_HIGH_G_Z_INT_EN_MASK);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_high_g_data_src(const struct bmi160_acc_high_g_int_cfg *high_g_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0;
 uint8_t temp = 0; 
 rslt = bmi160_get_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data & ~BMI160_LOW_HIGH_SRC_INT_MASK;
data = temp | ((high_g_int_cfg->high_data_src << 7) & BMI160_LOW_HIGH_SRC_INT_MASK);
rslt = bmi160_set_regs(BMI160_INT_DATA_0_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_high_g_int_settg(const struct bmi160_acc_high_g_int_cfg *high_g_int_cfg,const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t temp = 0;
 uint8_t data_array[3] = { 0, 0, 0 }; rslt = bmi160_get_regs(BMI160_INT_LOWHIGH_2_ADDR, &data_array[0], 1, dev);
 if (rslt == BMI160_OK)
 {
temp = data_array[0] & ~BMI160_HIGH_G_HYST_MASK;
data_array[0] = temp | ((high_g_int_cfg->high_hy << 6) & BMI160_HIGH_G_HYST_MASK);
data_array[1] = high_g_int_cfg->high_dur;
data_array[2] = high_g_int_cfg->high_thres;
rslt = bmi160_set_regs(BMI160_INT_LOWHIGH_2_ADDR, data_array, 3, dev);
 } return rslt;
}
static int8_t config_int_out_ctrl(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t temp = 0;
 uint8_t data = 0; 
 rslt = bmi160_get_regs(BMI160_INT_OUT_CTRL_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {

const struct bmi160_int_pin_settg *intr_pin_sett = &(int_config->int_pin_settg);
if (int_config->int_channel == BMI160_INT_CHANNEL_1)
{

temp = data & ~BMI160_INT1_OUTPUT_EN_MASK;
data = temp | ((intr_pin_sett->output_en << 3) & BMI160_INT1_OUTPUT_EN_MASK);
temp = data & ~BMI160_INT1_OUTPUT_MODE_MASK;
data = temp | ((intr_pin_sett->output_mode << 2) & BMI160_INT1_OUTPUT_MODE_MASK);
temp = data & ~BMI160_INT1_OUTPUT_TYPE_MASK;
data = temp | ((intr_pin_sett->output_type << 1) & BMI160_INT1_OUTPUT_TYPE_MASK);
temp = data & ~BMI160_INT1_EDGE_CTRL_MASK;
data = temp | ((intr_pin_sett->edge_ctrl) & BMI160_INT1_EDGE_CTRL_MASK);
}
else
{


temp = data & ~BMI160_INT2_OUTPUT_EN_MASK;
data = temp | ((intr_pin_sett->output_en << 7) & BMI160_INT2_OUTPUT_EN_MASK);
temp = data & ~BMI160_INT2_OUTPUT_MODE_MASK;
data = temp | ((intr_pin_sett->output_mode << 6) & BMI160_INT2_OUTPUT_MODE_MASK);
temp = data & ~BMI160_INT2_OUTPUT_TYPE_MASK;
data = temp | ((intr_pin_sett->output_type << 5) & BMI160_INT2_OUTPUT_TYPE_MASK);
temp = data & ~BMI160_INT2_EDGE_CTRL_MASK;
data = temp | ((intr_pin_sett->edge_ctrl << 4) & BMI160_INT2_EDGE_CTRL_MASK);
}rslt = bmi160_set_regs(BMI160_INT_OUT_CTRL_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t config_int_latch(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t temp = 0;
 uint8_t data = 0; 
 rslt = bmi160_get_regs(BMI160_INT_LATCH_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {

const struct bmi160_int_pin_settg *intr_pin_sett = &(int_config->int_pin_settg);
if (int_config->int_channel == BMI160_INT_CHANNEL_1)
{


temp = data & ~BMI160_INT1_INPUT_EN_MASK;
data = temp | ((intr_pin_sett->input_en << 4) & BMI160_INT1_INPUT_EN_MASK);
}
else
{


temp = data & ~BMI160_INT2_INPUT_EN_MASK;
data = temp | ((intr_pin_sett->input_en << 5) & BMI160_INT2_INPUT_EN_MASK);
} 
temp = data & ~BMI160_INT_LATCH_MASK;
data = temp | (intr_pin_sett->latch_dur & BMI160_INT_LATCH_MASK);
rslt = bmi160_set_regs(BMI160_INT_LATCH_ADDR, &data, 1, dev);
 } return rslt;
}

#endif

#ifdef BMI_TEST_TYPE


int8_t bmi160_perform_self_test(uint8_t select_sensor, struct bmi160_dev *dev)
{
 int8_t rslt;
 int8_t self_test_rslt = 0; 
 rslt = null_ptr_check(dev);
 if (rslt != BMI160_OK)
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {
switch (select_sensor)
{
case BMI160_ACCEL_ONLY:
 rslt = perform_accel_self_test(dev);
 break;
case BMI160_GYRO_ONLY: 
 dev->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
 rslt = bmi160_set_power_mode(dev); 
 if (rslt == BMI160_OK)
 {

rslt = perform_gyro_self_test(dev);
 } break;
default:
 rslt = BMI160_E_INVALID_INPUT;
 break;
}
if (rslt >= BMI160_OK)
{

self_test_rslt = rslt;
rslt = bmi160_soft_reset(dev);}
if (rslt == BMI160_OK)
{

rslt = self_test_rslt;
}
 } return rslt;
}
static int8_t perform_accel_self_test(struct bmi160_dev *dev)
{
 int8_t rslt;
 struct bmi160_sensor_data accel_pos, accel_neg; 
 rslt = enable_accel_self_test(dev);
 if (rslt == BMI160_OK)
 {

rslt = accel_self_test_positive_excitation(&accel_pos, dev);
if (rslt == BMI160_OK)
{

rslt = accel_self_test_negative_excitation(&accel_neg, dev);
if (rslt == BMI160_OK)
{
 
 rslt = validate_accel_self_test(&accel_pos, &accel_neg);
}
}
 } return rslt;
}
static int8_t enable_accel_self_test(struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t reg_data; 
 dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE; 
 dev->accel_cfg.range = BMI160_ACCEL_RANGE_8G;
 rslt = bmi160_set_sens_conf(dev);
 if (rslt == BMI160_OK)
 {

reg_data = BMI160_ACCEL_SELF_TEST_CONFIG;
rslt = bmi160_set_regs(BMI160_ACCEL_CONFIG_ADDR, &reg_data, 1, dev);
 } return rslt;
}
static int8_t accel_self_test_positive_excitation(struct bmi160_sensor_data *accel_pos, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t reg_data; 
 reg_data = BMI160_ACCEL_SELF_TEST_POSITIVE_EN;
 rslt = bmi160_set_regs(BMI160_SELF_TEST_ADDR, &reg_data, 1, dev);
 if (rslt == BMI160_OK)
 {

dev->delay_ms(BMI160_ACCEL_SELF_TEST_DELAY);
rslt = bmi160_get_sensor_data(BMI160_ACCEL_ONLY, accel_pos, NULL, dev);
 } return rslt;
}
static int8_t accel_self_test_negative_excitation(struct bmi160_sensor_data *accel_neg, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t reg_data; 
 reg_data = BMI160_ACCEL_SELF_TEST_NEGATIVE_EN;
 rslt = bmi160_set_regs(BMI160_SELF_TEST_ADDR, &reg_data, 1, dev);
 if (rslt == BMI160_OK)
 {

dev->delay_ms(BMI160_ACCEL_SELF_TEST_DELAY);
rslt = bmi160_get_sensor_data(BMI160_ACCEL_ONLY, accel_neg, NULL, dev);
 } return rslt;
}
static int8_t validate_accel_self_test(const struct bmi160_sensor_data *accel_pos,const struct bmi160_sensor_data *accel_neg)
{
 int8_t rslt; 
 if (((accel_neg->x - accel_pos->x) > BMI160_ACCEL_SELF_TEST_LIMIT) &&
((accel_neg->y - accel_pos->y) > BMI160_ACCEL_SELF_TEST_LIMIT) &&
((accel_neg->z - accel_pos->z) > BMI160_ACCEL_SELF_TEST_LIMIT))
 {

rslt = BMI160_OK;
 }
 else
 {
rslt = BMI160_W_ACCEl_SELF_TEST_FAIL;
 } return rslt;
}
static int8_t perform_gyro_self_test(const struct bmi160_dev *dev)
{
 int8_t rslt; 
 rslt = enable_gyro_self_test(dev);
 if (rslt == BMI160_OK)
 {

dev->delay_ms(50);
rslt = validate_gyro_self_test(dev);
 } return rslt;
}
static int8_t enable_gyro_self_test(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t reg_data; 
 rslt = bmi160_get_regs(BMI160_SELF_TEST_ADDR, &reg_data, 1, dev);
 if (rslt == BMI160_OK)
 {
reg_data = BMI160_SET_BITS(reg_data, BMI160_GYRO_SELF_TEST, 1);
rslt = bmi160_set_regs(BMI160_SELF_TEST_ADDR, &reg_data, 1, dev);
if (rslt == BMI160_OK)
{

dev->delay_ms(15);
}
 } return rslt;
}
static int8_t validate_gyro_self_test(const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t reg_data; 
 rslt = bmi160_get_regs(BMI160_STATUS_ADDR, &reg_data, 1, dev);
 if (rslt == BMI160_OK)
 {reg_data = BMI160_GET_BITS(reg_data, BMI160_GYRO_SELF_TEST_STATUS);
if (reg_data == BMI160_ENABLE)
{

rslt = BMI160_OK;
}
else
{
rslt = BMI160_W_GYRO_SELF_TEST_FAIL;
}
 } return rslt;
}

#endif
#ifdef BMI_INT_TYPE
static int8_t set_fifo_full_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK; 
 if ((dev == NULL) || (dev->delay_ms == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

rslt = enable_fifo_full_int(int_config, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 rslt = map_hardware_interrupt(int_config, dev);
}
}
 } return rslt;
}
static int8_t enable_fifo_full_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0; rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
data = BMI160_SET_BITS(data, BMI160_FIFO_FULL_INT, int_config->fifo_full_int_en);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 } return rslt;
}
static int8_t set_fifo_watermark_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt = BMI160_OK; if ((dev == NULL) || (dev->delay_ms == NULL))
 {
rslt = BMI160_E_NULL_PTR;
 }
 else
 {

rslt = enable_fifo_wtm_int(int_config, dev);
if (rslt == BMI160_OK)
{

rslt = set_intr_pin_config(int_config, dev);
if (rslt == BMI160_OK)
{
 rslt = map_hardware_interrupt(int_config, dev);
}
}
 } return rslt;
}
static int8_t enable_fifo_wtm_int(const struct bmi160_int_settg *int_config, const struct bmi160_dev *dev)
{
 int8_t rslt;
 uint8_t data = 0; rslt = bmi160_get_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 if (rslt == BMI160_OK)
 {
data = BMI160_SET_BITS(data, BMI160_FIFO_WTM_INT, int_config->fifo_wtm_int_en);
rslt = bmi160_set_regs(BMI160_INT_ENABLE_1_ADDR, &data, 1, dev);
 } return rslt;
}

#endif

#ifdef BMI_AUX_TYPE
static void get_aux_len_to_parse(uint16_t *data_index, uint16_t *data_read_length, const uint8_t *aux_frame_count, const struct bmi160_dev *dev)
{
 
 *data_index = dev->fifo->gyro_byte_start_idx;
 if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_ENABLE)
 {
*data_read_length = (*aux_frame_count) * BMI160_FIFO_M_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_A_ENABLE)
 {
*data_read_length = (*aux_frame_count) * BMI160_FIFO_MA_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_ENABLE)
 {
*data_read_length = (*aux_frame_count) * BMI160_FIFO_MG_LENGTH;
 }
 else if (dev->fifo->fifo_data_enable == BMI160_FIFO_M_G_A_ENABLE)
 {
*data_read_length = (*aux_frame_count) * BMI160_FIFO_MGA_LENGTH;
 }
 else
 {

*data_index = dev->fifo->length;
 } if (*data_read_length > dev->fifo->length)
 {

*data_read_length = dev->fifo->length;
 }
}
static void unpack_aux_frame(struct bmi160_aux_data *aux_data,uint16_t *idx,uint8_t *aux_index,uint8_t frame_info,const struct bmi160_dev *dev)
{
 switch (frame_info)
 {
case BMI160_FIFO_HEAD_M:
case BMI160_FIFO_M_ENABLE:
if ((*idx + BMI160_FIFO_M_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_aux_data(&aux_data[*aux_index], *idx, dev);
*idx = *idx + BMI160_FIFO_M_LENGTH;
(*aux_index)++;
break;
case BMI160_FIFO_HEAD_M_A:
case BMI160_FIFO_M_A_ENABLE:
if ((*idx + BMI160_FIFO_MA_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_aux_data(&aux_data[*aux_index], *idx, dev);
*idx = *idx + BMI160_FIFO_MA_LENGTH;
(*aux_index)++;
break;
case BMI160_FIFO_HEAD_M_G:
case BMI160_FIFO_M_G_ENABLE:
if ((*idx + BMI160_FIFO_MG_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_aux_data(&aux_data[*aux_index], *idx, dev);
(*idx) = (*idx) + BMI160_FIFO_MG_LENGTH;
(*aux_index)++;
break;
case BMI160_FIFO_HEAD_M_G_A:
case BMI160_FIFO_M_G_A_ENABLE:
if ((*idx + BMI160_FIFO_MGA_LENGTH) > dev->fifo->length)
{
 
 *idx = dev->fifo->length;
 break;
}
unpack_aux_data(&aux_data[*aux_index], *idx, dev);
*idx = *idx + BMI160_FIFO_MGA_LENGTH;
(*aux_index)++;
break;
case BMI160_FIFO_HEAD_G:
case BMI160_FIFO_G_ENABLE:
(*idx) = (*idx) + BMI160_FIFO_G_LENGTH;
break;
case BMI160_FIFO_HEAD_G_A:
case BMI160_FIFO_G_A_ENABLE:
*idx = *idx + BMI160_FIFO_GA_LENGTH;
break;
case BMI160_FIFO_HEAD_A:
case BMI160_FIFO_A_ENABLE:
*idx = *idx + BMI160_FIFO_A_LENGTH;
break;
default:
break;
 }
}
static void unpack_aux_data(struct bmi160_aux_data *aux_data, uint16_t data_start_index, const struct bmi160_dev *dev)
{
 
 aux_data->data[0] = dev->fifo->data[data_start_index++];
 aux_data->data[1] = dev->fifo->data[data_start_index++];
 aux_data->data[2] = dev->fifo->data[data_start_index++];
 aux_data->data[3] = dev->fifo->data[data_start_index++];
 aux_data->data[4] = dev->fifo->data[data_start_index++];
 aux_data->data[5] = dev->fifo->data[data_start_index++];
 aux_data->data[6] = dev->fifo->data[data_start_index++];
 aux_data->data[7] = dev->fifo->data[data_start_index++];
}
static void extract_aux_header_mode(struct bmi160_aux_data *aux_data, uint8_t *aux_length, const struct bmi160_dev *dev)
{
 uint8_t frame_header = 0;
 uint16_t data_index;
 uint8_t aux_index = 0; for (data_index = dev->fifo->aux_byte_start_idx; data_index < dev->fifo->length;)
 {

frame_header = (dev->fifo->data[data_index] & BMI160_FIFO_TAG_INTR_MASK);
data_index++;
switch (frame_header)
{

case BMI160_FIFO_HEAD_M:
case BMI160_FIFO_HEAD_M_A:
case BMI160_FIFO_HEAD_M_G:
case BMI160_FIFO_HEAD_M_G_A:
 unpack_aux_frame(aux_data, &data_index, &aux_index, frame_header, dev);
 break;
case BMI160_FIFO_HEAD_G:
 move_next_frame(&data_index, BMI160_FIFO_G_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_G_A:
 move_next_frame(&data_index, BMI160_FIFO_GA_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_A:
 move_next_frame(&data_index, BMI160_FIFO_A_LENGTH, dev);
 break;
case BMI160_FIFO_HEAD_SENSOR_TIME:
 unpack_sensortime_frame(&data_index, dev);
 break;
case BMI160_FIFO_HEAD_SKIP_FRAME:
 unpack_skipped_frame(&data_index, dev);
 break;
case BMI160_FIFO_HEAD_INPUT_CONFIG:
 move_next_frame(&data_index, 1, dev);
 break;
case BMI160_FIFO_HEAD_OVER_READ: 
 data_index = dev->fifo->length;
 break;
default: 
 data_index = dev->fifo->length;
 break;
}
if (*aux_length == aux_index)
{

break;
}
 } 
 *aux_length = aux_index; 
 dev->fifo->aux_byte_start_idx = data_index;
}
#endif


