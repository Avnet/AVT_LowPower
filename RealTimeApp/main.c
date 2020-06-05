/*
 * (C) 2005-2019 MediaTek Inc. All rights reserved.
 *
 * Copyright Statement:
 *
 * This MT3620 driver software/firmware and related documentation
 * ("MediaTek Software") are protected under relevant copyright laws.
 * The information contained herein is confidential and proprietary to
 * MediaTek Inc. ("MediaTek"). You may only use, reproduce, modify, or
 * distribute (as applicable) MediaTek Software if you have agreed to and been
 * bound by this Statement and the applicable license agreement with MediaTek
 * ("License Agreement") and been granted explicit permission to do so within
 * the License Agreement ("Permitted User"). If you are not a Permitted User,
 * please cease any access or use of MediaTek Software immediately.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT MEDIATEK SOFTWARE RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE
 * PROVIDED TO RECEIVER ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS
 * ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH MEDIATEK SOFTWARE, AND RECEIVER AGREES TO
 * LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO MEDIATEK SOFTWARE RELEASED
 * HEREUNDER WILL BE ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY
 * RECEIVER TO MEDIATEK DURING THE PRECEDING TWELVE (12) MONTHS FOR SUCH
 * MEDIATEK SOFTWARE AT ISSUE.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "printf.h"
#include "mt3620.h"
#include <semphr.h>
#include "ctype.h"

#include "os_hal_uart.h"
#include "os_hal_i2c.h"
#include "os_hal_gpio.h"
#include "os_hal_mbox.h"
#include "os_hal_mbox_shared_mem.h"
#include "lsm6dso_reg.h"

/*
 *Additional Note:
 *This code uses ISU2 as I2C Master to communicate with the LSM6DSO sensor.
 *The sensor is configured to get acceleration data and detect a double tap 
 *movement. Some double tap detection parameters can be modified from the A7 core
 *through intercore communication. This core uses the ISU0 as UART to print some
 *data. Also, it uses the GPIO6 to detect a double tap sensor interruption.
*/

/******************************************************************************/
/* Configurations */
/******************************************************************************/

//#define ENABLE_READ_WRITE_DEBUG
#define I2C_MIN_LEN 1
#define I2C_MAX_LEN 8//64 /* For AVNET development board, please change to 8. */
#define I2C_SLAVE_TIMEOUT 10000 /* 10000ms */
#define LSM6DSO_ID         0x6C   /* register value */
#define LSM6DSO_ADDRESS	   0x6A	  /* I2C Address */

#define APP_STACK_SIZE_BYTES (1024 / 4)

static const uint8_t uart_port_num = OS_HAL_UART_ISU0;
static const uint8_t i2c_master_speed = I2C_SCL_1000kHz;
static uint8_t i2c_master_port_num = OS_HAL_I2C_ISU2;

lsm6dso_ctx_t dev_ctx;
static uint8_t whoamI, rst;
const uint8_t lsm6dsOAddress = LSM6DSO_ADDRESS;     /* Addr = 0x6A */

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t raw_angular_rate_calibration;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_dps[3];
static float lsm6dsoTemperature_degC;

SemaphoreHandle_t blockDeqSema;
SemaphoreHandle_t blockFifoSema;
static const u32 pay_load_start_offset = 20; /* UUID 16B, Reserved 4B */

BufferHeader* outbound, * inbound;
u8* mbox_buf;
u32 mbox_buf_len;

u32 shared_buf_size;
u32 allocated_buf_size;


/* Maximum mailbox buffer len.
 *    Maximum message len: 1024B
 *                         1024 is the maximum value when HL_APP invoke send().
 *    Component UUID len : 16B
 *    Reserved data len  : 4B
*/
static const uint32_t mbox_buffer_len_max = 1048;

/* Bitmap for IRQ enable. bit_0 and bit_1 are used to communicate with HL_APP */
static const uint32_t mbox_irq_status = 0x3;

/* This ID must be the same as in your High Level application */
u8 core_id[] = { 0x2c, 0x5d, 0x02, 0x25, 0xda, 0x66, 0x48, 0x44, 0xBA, 0xE1, 0xAC, 0x26, 0xFC, 0xDD, 0x36, 0x27 };


/******************************************************************************/
/* Function prototypes */
/******************************************************************************/

void i2c_master_task(void* pParameters);
static void gpio_task(void* pParameters);
void MBOXTask_A(void* pParameters);
static int32_t platform_write(int* fD, uint8_t reg, uint8_t* bufp,
	uint16_t len);
static int32_t platform_read(int* fD, uint8_t reg, uint8_t* bufp,
	uint16_t len);
void Log_Debug(char* message, ...);



/******************************************************************************/
/* Application Hooks */
/******************************************************************************/
/* Hook for "stack over flow". */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	printf("%s: %s\n", __func__, pcTaskName);
}

/* Hook for "memory allocation failed". */
void vApplicationMallocFailedHook(void)
{
	printf("%s\n", __func__);
}

/* Hook for "printf". */
void _putchar(char character)
{
	mtk_os_hal_uart_put_char(uart_port_num, character);
	if (character == '\n')
		mtk_os_hal_uart_put_char(uart_port_num, '\r');
}


/******************************************************************************/
/* Functions */
/******************************************************************************/

_Noreturn void RTCoreMain(void)
{
	/* Setup Vector Table */
	NVIC_SetupVectorTable();

	/* Init UART */
	mtk_os_hal_uart_ctlr_init(uart_port_num);
	printf("\nFreeRTOS LSM6DSO demo\n");

	/* Init I2C Master/Slave */
	mtk_os_hal_i2c_ctrl_init(i2c_master_port_num);

	/* Create I2C Master */
	xTaskCreate(i2c_master_task, "I2C Master Task", APP_STACK_SIZE_BYTES,
		NULL, 2, NULL);
	/* Create GPIO Task */
	xTaskCreate(gpio_task, "GPIO Task",
		APP_STACK_SIZE_BYTES, NULL, 4, NULL);

	/* Open the MBOX channel of A7 <-> M4 */
	mtk_os_hal_mbox_open_channel(OS_HAL_MBOX_CH0);

	/* Create MBOX Task */
	xTaskCreate(MBOXTask_A, "MBOX_A Task", APP_STACK_SIZE_BYTES, NULL, 5,
		NULL);

	vTaskStartScheduler();

	for (;;)
		__asm__("wfi");
}

void i2c_master_task(void* pParameters)
{
	uint8_t reg;
	uint8_t data_cfg;
	int32_t ret;
	printf("[I2C Demo]I2C Master Task Started. (ISU%d)\n",
		i2c_master_port_num);
	mtk_os_hal_i2c_speed_init(i2c_master_port_num, i2c_master_speed);

	
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = (int*)&i2c_master_port_num;
	
	/* Check device ID */
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != LSM6DSO_ID)
	{
		//Log_Debug("LSM6DSO not found!\n");
		printf("LSM6DSO not found!\n");
	}

	/* Restore default configuration */
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do
	{
		lsm6dso_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Disable I3C interface */
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);

	/* Enable Block Data Update */
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_417Hz);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_417Hz);

	/* Set full scale */
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);

	/* Configure filtering chain(No aux interface) */
	/* Accelerometer - LPF1 + LPF2 path	*/
	lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

	
	do
	{
		/* Read the calibration values */
		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			/* Read angular rate field data to use for calibration offsets */
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, raw_angular_rate_calibration.u8bit);
		}

		/* Read the angular data rate again and verify that after applying the calibration, we have 0 angular rate in all directions */
		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			/* Read angular rate field data */
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

			/* Before we store the mdps values subtract the calibration data we captured at startup. */
			angular_rate_dps[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0] - raw_angular_rate_calibration.i16bit[0]);
			angular_rate_dps[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1] - raw_angular_rate_calibration.i16bit[1]);
			angular_rate_dps[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2] - raw_angular_rate_calibration.i16bit[2]);
		}

		/* If the angular values after applying the offset are not 0.7 or less, then do it again! */
	} while ((angular_rate_dps[0] >= 0.7f) && (angular_rate_dps[1] >= 0.7f) && (angular_rate_dps[2] >= 0.7f));


	//ret = lsm6dso_read_reg(&dev_ctx, LSM6DSO_ALL_INT_SRC, (uint8_t*)&reg, 1);
	/* Configure registers for double tap detection */
	data_cfg = 0x03;
	ret = lsm6dso_write_reg(&dev_ctx, LSM6DSO_TAP_CFG0, &data_cfg, 1);
	data_cfg = 0x0c;
	ret = lsm6dso_write_reg(&dev_ctx, LSM6DSO_TAP_CFG1, &data_cfg, 1);
	data_cfg = 0x8c;
	ret = lsm6dso_write_reg(&dev_ctx, LSM6DSO_TAP_CFG2, &data_cfg, 1);
	data_cfg = 0x06;
	ret = lsm6dso_write_reg(&dev_ctx, LSM6DSO_TAP_THS_6D, &data_cfg, 1);
	data_cfg = 0x7f;
	ret = lsm6dso_write_reg(&dev_ctx, LSM6DSO_INT_DUR2, &data_cfg, 1);
	data_cfg = 0x80;
	ret = lsm6dso_write_reg(&dev_ctx, LSM6DSO_WAKE_UP_THS, &data_cfg, 1);
	data_cfg = 0x08;
	ret = lsm6dso_write_reg(&dev_ctx, LSM6DSO_MD1_CFG, &data_cfg, 1);
	
	while (1)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
		/* Read output only if new xl value is available */
		lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			/* Read acceleration field data */
			memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

			acceleration_mg[0] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
			acceleration_mg[1] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
			acceleration_mg[2] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

			printf("\nLSM6DSO: Acceleration [mg]  : %.4lf, %.4lf, %.4lf\n",
				acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		}

		lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			/* Read angular rate field data */
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
			lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

			/* Before we store the mdps values subtract the calibration data we captured at startup. */
			angular_rate_dps[0] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0] - raw_angular_rate_calibration.i16bit[0])) / 1000.0;
			angular_rate_dps[1] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1] - raw_angular_rate_calibration.i16bit[1])) / 1000.0;
			angular_rate_dps[2] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2] - raw_angular_rate_calibration.i16bit[2])) / 1000.0;

			printf("LSM6DSO: Angular rate [dps] : %4.2f, %4.2f, %4.2f\n",
				angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);
		}

		lsm6dso_temp_flag_data_ready_get(&dev_ctx, &reg);
		if (reg)
		{
			/* Read temperature data */
			memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
			lsm6dso_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
			lsm6dsoTemperature_degC = lsm6dso_from_lsb_to_celsius(data_raw_temperature.i16bit);

			printf("LSM6DSO: Temperature [degC] : %.2f\r\n", lsm6dsoTemperature_degC);
		}

		memcpy(mbox_buf, core_id, 16);
		memset(mbox_buf + 16, 0, 4);
		*(mbox_buf + pay_load_start_offset) = 0;
		memcpy(mbox_buf + pay_load_start_offset + 1, &acceleration_mg[0], sizeof(acceleration_mg[0]));
		memcpy(mbox_buf + pay_load_start_offset + 5, &acceleration_mg[1], sizeof(acceleration_mg[1]));
		memcpy(mbox_buf + pay_load_start_offset + 9, &acceleration_mg[2], sizeof(acceleration_mg[2]));
		EnqueueData(inbound, outbound, shared_buf_size, mbox_buf,
			pay_load_start_offset + 13);

	}
}

/*
 * @brief  Reads the GPIO 6 to detect if a double tap detection interruption has been triggered, if so,
 * clears the interrupt and send a message to the A7 core.
 *
 */
static void gpio_task(void* pParameters)
{
	os_hal_gpio_data value = 0;
	uint8_t reg;
	os_hal_gpio_data ledState = OS_HAL_GPIO_DATA_LOW;
	mtk_os_hal_gpio_request(OS_HAL_GPIO_4);
	mtk_os_hal_gpio_set_direction(OS_HAL_GPIO_4, OS_HAL_GPIO_DIR_OUTPUT);

	printf("GPIO Task Started\n");
	while (1)
	{
		/* Get Button_A status and set LED Red. */
		mtk_os_hal_gpio_get_input(OS_HAL_GPIO_6, &value);
		if (value == OS_HAL_GPIO_DATA_HIGH)
		{
			/* Clear interrupt */
			lsm6dso_read_reg(&dev_ctx, LSM6DSO_ALL_INT_SRC, (uint8_t*)&reg, 1);
			if (reg == 0x0a)
			{
				printf("Double-tap detected\n");
				memcpy(mbox_buf, core_id, 16);
				memset(mbox_buf + 16, 0, 4);
				*(mbox_buf + pay_load_start_offset) = 2;
				memcpy(mbox_buf + pay_load_start_offset + 1, "Double-Tap detected", strlen("Double-Tap detected"));
				EnqueueData(inbound, outbound, shared_buf_size, mbox_buf,
					pay_load_start_offset + strlen("Double-Tap detected") + 1);

				// Toggle the App Led to show the user that a double tap was detected
				ledState = (ledState == OS_HAL_GPIO_DATA_LOW) ? OS_HAL_GPIO_DATA_HIGH : OS_HAL_GPIO_DATA_LOW;
				mtk_os_hal_gpio_set_output(OS_HAL_GPIO_4, ledState);
			}
		}

		/* Delay for 100ms */
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void mbox_fifo_cb(struct mtk_os_hal_mbox_cb_data* data)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (data->event.wr_int)
	{
		xSemaphoreGiveFromISR(blockFifoSema,
			&higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

/* Interrupt handler, interrupt is triggered when mailbox shared memory R/W.
 *     data->swint.swint_channel: Channel_0 for A7, Channel_1 for the other M4.
 *     data->swint.swint_sts bit_0: A7 read data from mailbox
 *     data->swint.swint_sts bit_1: A7 write data to mailbox
*/
void mbox_swint_cb(struct mtk_os_hal_mbox_cb_data* data)
{
	BaseType_t higher_priority_task_woken = pdFALSE;

	if (data->swint.swint_sts & (1 << 1))
	{
		xSemaphoreGiveFromISR(blockDeqSema,
			&higher_priority_task_woken);
		portYIELD_FROM_ISR(higher_priority_task_woken);
	}
}

void mbox_print_buf(u8* mbox_buf, u32 mbox_data_len)
{
	u32 payload_len;
	u32 i;

	printf("Received message of %d bytes:\n", mbox_data_len);
	printf("  Component Id (16 bytes): %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n",
		mbox_buf[3], mbox_buf[2], mbox_buf[1], mbox_buf[0],
		mbox_buf[5], mbox_buf[4], mbox_buf[7], mbox_buf[6],
		mbox_buf[8], mbox_buf[9], mbox_buf[10], mbox_buf[11],
		mbox_buf[12], mbox_buf[13], mbox_buf[14], mbox_buf[15]);

	/* Print reserved field as little-endian 4-byte integer. */
	printf("  Reserved (4 bytes): 0x%02X %02X %02X %02X\n",
		mbox_buf[19], mbox_buf[18], mbox_buf[17], mbox_buf[16]);

	/* Print message as hex. */
	payload_len = mbox_data_len - pay_load_start_offset;
	printf("  Payload (%d bytes as hex): ", payload_len);
	for (i = pay_load_start_offset; i < mbox_data_len; ++i)
		printf("0x%02X ", mbox_buf[i]);
	printf("\n");

	/* Print message as text. */
	printf("  Payload (%d bytes as text): ", payload_len);
	for (i = pay_load_start_offset; i < mbox_data_len; ++i)
		printf("%c", mbox_buf[i]);
	printf("\n");

	/* Convert payload, upper to lower, lower to upper.*/
	for (i = pay_load_start_offset; i < mbox_data_len; ++i)
	{
		if (isupper(mbox_buf[i]))
			mbox_buf[i] = tolower(mbox_buf[i]);
		else if (islower(mbox_buf[i]))
			mbox_buf[i] = toupper(mbox_buf[i]);
	}

	printf("  Send back (%d bytes as text):", payload_len);
	for (i = pay_load_start_offset; i < mbox_data_len; ++i)
		printf("%c", mbox_buf[i]);
	printf("\n\n");
}

/*
 * @brief  Reads incomming data from the A7 core and update lsm6dso sensor parameters.
 */
void MBOXTask_A(void* pParameters)
{
	struct mbox_fifo_event mask;

	int result;
	uint8_t data_cfg;
	int32_t ret;

	printf("MBOX_A Task Started\n");

	blockDeqSema = xSemaphoreCreateBinary();
	blockFifoSema = xSemaphoreCreateBinary();

	/* Register interrupt callback */
	mask.channel = OS_HAL_MBOX_CH0;
	mask.ne_sts = 0;	/* FIFO Non-Empty interrupt */
	mask.nf_sts = 0;	/* FIFO Non-Full interrupt */
	mask.rd_int = 0;	/* Read FIFO interrupt */
	mask.wr_int = 1;	/* Write FIFO interrupt */
	mtk_os_hal_mbox_fifo_register_cb(OS_HAL_MBOX_CH0, mbox_fifo_cb, &mask);
	mtk_os_hal_mbox_sw_int_register_cb(OS_HAL_MBOX_CH0, mbox_swint_cb,
		mbox_irq_status);

	/* Get mailbox shared buffer size, defined by Azure Sphere OS. */
	if (GetIntercoreBuffers(&outbound, &inbound, &shared_buf_size) == -1)
	{
		printf("GetIntercoreBuffers failed\n");
		return;
	}

	/* Allocate the M4 buffer for mailbox communication */
	allocated_buf_size = shared_buf_size;
	if (allocated_buf_size > mbox_buffer_len_max)
		allocated_buf_size = mbox_buffer_len_max;
	mbox_buf = pvPortMalloc(allocated_buf_size);
	if (mbox_buf == NULL)
	{
		printf("pvPortMalloc failed\n");
		return;
	}

	printf("shared buf size = %d\n", shared_buf_size);
	printf("allocated buf size = %d\n", allocated_buf_size);

	while (1)
	{
		ret = 0;
		vTaskDelay(pdMS_TO_TICKS(15));

		/* Init buffer */
		mbox_buf_len = allocated_buf_size;
		memset(mbox_buf, 0, allocated_buf_size);

		/* Read from A7, dequeue from mailbox */
		result = DequeueData(outbound, inbound, shared_buf_size,
			mbox_buf, &mbox_buf_len);
		if (result == -1 || mbox_buf_len < pay_load_start_offset)
		{
			xSemaphoreTake(blockDeqSema, portMAX_DELAY);
			continue;
		}

		/* Enable/disable tap detection in Z axis */
		if (*(mbox_buf + pay_load_start_offset) == 0x01)
		{
			data_cfg = 0x03;
			ret |= lsm6dso_write_reg(&dev_ctx, LSM6DSO_TAP_CFG0, &data_cfg, 1);
		}
		else
		{
			data_cfg = 0x01;
			ret |= lsm6dso_write_reg(&dev_ctx, LSM6DSO_TAP_CFG0, &data_cfg, 1);
		}

		/* Z axis double tap detection threshold */
		data_cfg = *(mbox_buf + pay_load_start_offset + 1);
		ret |= lsm6dso_write_reg(&dev_ctx, LSM6DSO_TAP_THS_6D, &data_cfg, 1);

		/* Tap recognition settings */
		/* DUR[3:0] Duration of maximum time gap for double tap recognition. */
		/* QUIET[1:0] Expected quiet time after a tap detection. */
		/* SHOCK[1:0] Maximum duration of overthreshold event. */
		data_cfg = *(mbox_buf + pay_load_start_offset + 2);
		ret |= lsm6dso_write_reg(&dev_ctx, LSM6DSO_INT_DUR2, &data_cfg, 1);

		/* Print received message */
		mbox_print_buf(mbox_buf, mbox_buf_len);

		*(mbox_buf + pay_load_start_offset) = 1;

		if (ret == 0)
		{
			memcpy(mbox_buf + pay_load_start_offset + 1, "Parameters updated", sizeof("Parameters updated"));

			mbox_buf_len = pay_load_start_offset + sizeof("Parameters updated");
		}
		else
		{
			memcpy(mbox_buf + pay_load_start_offset + 1, "ERROR: parameters not updated", sizeof("ERROR: parameters not updated"));

			mbox_buf_len = pay_load_start_offset + sizeof("ERROR: parameters not updated");
		}

		/* Write to A7, enqueue to mailbox */
		EnqueueData(inbound, outbound, shared_buf_size, mbox_buf,
			mbox_buf_len);
	}
}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  fD        file descriptor used to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */

static int32_t platform_write(int* fD, uint8_t reg, uint8_t* bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_write()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %0x\n", len);
	Log_Debug("bufp contents: ");
	for (int i = 0; i < len; i++)
	{

		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n");
#endif 

	/* Construct a new command buffer that contains the register to write to, then the data to write */
	uint8_t cmdBuffer[len + 1];
	cmdBuffer[0] = reg;
	for (int i = 0; i < len; i++)
	{
		cmdBuffer[i + 1] = bufp[i];
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("cmdBuffer contents: ");
	for (int i = 0; i < len + 1; i++)
	{

		Log_Debug("%0x: ", cmdBuffer[i]);
	}
	Log_Debug("\n");
#endif

	/* Write the data to the device */
	int32_t retVal = mtk_os_hal_i2c_write(*fD, lsm6dsOAddress, cmdBuffer, (size_t)len + 1);
	if (retVal < 0)
	{
		//Log_Debug("ERROR: platform_write: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}
#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Wrote %d bytes to device.\n\n", retVal);
#endif
	return 0;
}


/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  fD        file descriptor used to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(int* fD, uint8_t reg, uint8_t* bufp,
	uint16_t len)
{

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("platform_read()\n");
	Log_Debug("reg: %0x\n", reg);
	Log_Debug("len: %d\n", len);
	;
#endif

	/* Set the register address to read */
	int32_t retVal = mtk_os_hal_i2c_write(*fD, lsm6dsOAddress, &reg, 1);
	if (retVal < 0)
	{
		//Log_Debug("ERROR: platform_read(write step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	/* Read the data into the provided buffer */
	retVal = mtk_os_hal_i2c_read(*fD, lsm6dsOAddress, bufp, len);
	if (retVal < 0)
	{
		//Log_Debug("ERROR: platform_read(read step): errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

#ifdef ENABLE_READ_WRITE_DEBUG
	Log_Debug("Read returned: ");
	for (int i = 0; i < len; i++)
	{
		Log_Debug("%0x: ", bufp[i]);
	}
	Log_Debug("\n\n");
#endif 	   

	return 0;
}


void Log_Debug(char* message, ...)
{
	va_list args;
	va_start(args, message);
	vprintf(message, args);
	va_end(args);
}

