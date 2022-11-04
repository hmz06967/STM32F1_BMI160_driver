/**
  ******************************************************************************
  * @file      main.c
  * @author    Hamza Özkan
  * @date      25/11/2021
  ******************************************************************************
  */
  
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include "stdbool.h"


/*********************************************************************/
/* own header files */
/*********************************************************************/

#include "bmi160.h"

#define BMI160_INTERFACE_I2C  1
#define BMI160_INTERFACE_SPI  0

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/*
	bmi160 init
*/

/*! bmi160 shuttle id */
#define BMI160_SHUTTLE_ID     0xD8

/*! bmi160 Device address */
#define BMI160_DEV_ADDR       BMI160_I2C_ADDR

/*********************************[global variables]************************************/
struct bmi160_dev bmi160dev;// This structure containing relevant bmi160 info
struct bmi160_sensor_data bmi160_accel;//variable to hold the bmi160 accel data 
struct bmi160_sensor_data bmi160_gyro;//variable to hold the bmi160 gyro data 

/***********************************static function declarations**********************************/
static void init_bmi160(void);//This internal API is used to initialize the bmi160 sensor with default
static void init_bmi160_sensor_driver_interface(void);//This internal API is used to initialize the sensor driver interface
uint8_t buffer[100];
void uprint( const char* format, ...){
    va_list arglist;
    va_start( arglist, format );
    vsprintf((char*)buffer, format, arglist );
    va_end( arglist );
		HAL_UART_Transmit(&huart1, buffer, sizeof(buffer)-1, 10);
		memset(buffer, 0, sizeof(buffer)-1);
}
void i2c_test_bmi(){
		uint8_t data[1];
		HAL_StatusTypeDef stat_i2c;
	
		data[0] = BMI160_SOFT_RESET_CMD;
		stat_i2c = HAL_I2C_Mem_Write(&hi2c1, BMI160_I2C_ADDR<<1, BMI160_COMMAND_REG_ADDR, 1, data, 1, 10);
    uprint("bmi160: Write->Address: 0x%X, Value: 0x%X, Stat:%d\n", BMI160_SOFT_RESET_CMD, data[0], stat_i2c); 
	
		data[0] = 0;
		stat_i2c = HAL_I2C_Mem_Read(&hi2c1, BMI160_I2C_ADDR<<1, BMI160_COMMAND_REG_ADDR, 1, data, 1, 10);
    uprint( "bmi160: Read<-Address: 0x%X, Value: 0x%X, Stat:%d\n", BMI160_COMMAND_REG_ADDR, data[0], stat_i2c);
	
		if(stat_i2c!=HAL_OK)
			uprint("bmi160: test error! \n");
		else
			uprint("bmi160: test ok \n");
}
void bmi_clibration(){
	//get calibre offset
	struct bmi160_offsets offset;
	struct bmi160_foc_conf foc_confg;
	bmi160_get_offsets(&offset, &bmi160dev);
	uprint("bmi160: offset\n");
	uprint("no calibre: ax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\n", offset.off_acc_x, offset.off_acc_y, offset.off_acc_z, offset.off_gyro_x, offset.off_gyro_y, offset.off_gyro_z);
	uprint("bmi160: calibrating..\n");
 
	bmi160_start_foc(&foc_confg, &offset, &bmi160dev);
	
	bmi160_get_offsets(&offset, &bmi160dev);
	bmi160_update_nvm(&bmi160dev);
	uprint("calibre offset ax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\n", offset.off_acc_x, offset.off_acc_y, offset.off_acc_z, offset.off_gyro_x, offset.off_gyro_y, offset.off_gyro_z);
	uprint("foc confg: ax%d\tay:%d\taz:%d\n", foc_confg.foc_acc_x, foc_confg.foc_acc_y, foc_confg.foc_acc_z);
}
static void init_bmi160(void){
    int8_t rslt;
    rslt = bmi160_init(&bmi160dev);
    if (rslt == BMI160_OK){
			  uprint("bmi160: init addr 0x%X\n",bmi160dev.chip_id);
    }else{
			uprint("bmi160: error %d\n", rslt);
			Error_Handler();
			HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_SET);
    }
		
    /* Select the Output data rate, range of accelerometer sensor */
    bmi160dev.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;
    bmi160dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    bmi160dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi160dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    bmi160dev.gyro_cfg.odr = BMI160_GYRO_ODR_50HZ;
    bmi160dev.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
    bmi160dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi160dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    // Set the sensor configuration 
    rslt = bmi160_set_sens_conf(&bmi160dev);
		
    if (rslt != BMI160_OK){
				uprint("bmi160: config error %d\n", rslt);
				Error_Handler();
    }else{
			bmi_clibration();
		}


}

static void init_bmi160_sensor_driver_interface(void){
    /* I2C setup */
    /* link read/write/delay function of host system to appropriate
     * bmi160 function call prototypes */
	bmi160dev.hi2cx = &hi2c1;
	bmi160dev.huart = &huart1;
	bmi160dev.uart_write = HAL_UART_Transmit;
	bmi160dev.write = HAL_I2C_Mem_Write;
	bmi160dev.read = HAL_I2C_Mem_Read;
	bmi160dev.delay_ms = HAL_Delay; 
	/* set correct i2c address */
	bmi160dev.id = BMI160_DEV_ADDR;
	bmi160dev.intf = BMI160_I2C_INTF;
}


uint8_t read_register(uint8_t register_pointer, uint8_t *return_value){
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(BMI160_DEV_ADDR<<1), (uint16_t)register_pointer, I2C_MEMADD_SIZE_8BIT, return_value, 1, 100);
    return status;
}

uint8_t write_register(uint8_t register_pointer, uint8_t value){
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(BMI160_DEV_ADDR<<1), (uint16_t)register_pointer, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
    return status;
}


int main(void)
{
	
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
	uprint("start mcu..\n");
	//test sensor
	i2c_test_bmi();
	//sensor init
	init_bmi160_sensor_driver_interface();
	init_bmi160();
	uint32_t count = 0;
	struct bmi160_sensor_data bmi160_accel;
	struct bmi160_sensor_data bmi160_gyro;
	
	while (1){
			int8_t rsp = bmi160_get_sensor_data(BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL, &bmi160_accel, &bmi160_gyro, &bmi160dev);
			uprint(
						"%d\tax:%d\tay:%d\taz:%d\tgx:%d\tgy:%d\tgz:%d\n", 
						count, 
						bmi160_accel.x, bmi160_accel.y, bmi160_accel.z, 
						bmi160_gyro.x, bmi160_gyro.y, bmi160_gyro.z
			);
			HAL_Delay(200);
			count++;
	}

}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{


  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }


}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler()
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

	
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
