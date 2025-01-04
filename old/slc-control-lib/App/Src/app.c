#include "app.h"
#include "usbd_cdc_if.h"
#include "oled.h"
#include "display.h"
#include "bitmap.h"
#include "font.h"
#include "eeprom.h"
#include "memory.h"
#include "button.h"
#include "gps.h"
#include "nmea.h"
#include "dac.h"
#include "tach.h"
#include "lps22hh.h"
#include "qmc5883.h"
#include "icm42688.h"
#include "algebra.h"
#include "string.h"
#include "math.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac3;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp3;

static float slip = 0;
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Memory_Variable var_active = {.reset = 0.0f};
static struct Memory_Variable var_trigger_prim = {.min = 0.0f, .max = 100.0f, .reset = 15.0f, .minDigit = -1};
static struct Memory_Variable var_trigger_aux = {.min = 0.0f, .max = 10000.0f, .reset = 1000.0f, .minDigit = 0};
static struct Memory_Variable var_trigger_min = {.min = 0.0f, .max = 500.0f, .reset = 10.0f, .minDigit = 0};

static struct Memory_Variable var_sensor1_scale = {.min = 1.0f, .max = 80.0f, .reset = 30.0f, .minDigit = 0};
static struct Memory_Variable var_sensor1_max = {.min = 1.0f, .max = 50000.0f, .reset = 10000.0f, .minDigit = 0};

static struct Memory_Variable var_sensor2_scale = {.min = 1.0f, .max = 80.0f, .reset = 30.0f, .minDigit = 0};
static struct Memory_Variable var_sensor2_max = {.min = 1.0f, .max = 50000.0f, .reset = 10000.0f, .minDigit = 0};

static struct Memory_Variable var_sensor3_scale = {.min = 1.0f, .max = 80.0f, .reset = 30.0f, .minDigit = 0};
static struct Memory_Variable var_sensor3_max = {.min = 1.0f, .max = 50000.0f, .reset = 10000.0f, .minDigit = 0};

static struct Memory_Variable var_gps_circ = {.min = 1.0f, .max = 200.0f, .reset = 120.0f, .minDigit = -1};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Memory_Variable* memory_vars[11] = {
		&var_active,
		&var_trigger_prim,
		&var_trigger_aux,
		&var_trigger_min,
		&var_sensor1_scale,
		&var_sensor1_max,
		&var_sensor2_scale,
		&var_sensor2_max,
		&var_sensor3_scale,
		&var_sensor1_max,
};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Eeprom_Handle eeprom = {.hi2c = &hi2c1, .address = 0xA0, .pages = 512, .pageSize = 64};
static struct Memory_Handle memory = {.eeprom = &eeprom, .hash = 1, .vars = memory_vars, .count = 11};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Dac_Handle dac1 = {.hdac = &hdac3, .channel = DAC_CHANNEL_1};
static struct Dac_Handle dac2 = {.hdac = &hdac1, .channel = DAC_CHANNEL_1};
static struct Dac_Handle dac3 = {.hdac = &hdac3, .channel = DAC_CHANNEL_2};
/////////////////////////////////////////////////////////////////////////////////////////////
static GPIO_TypeDef* button_ports[4] = { BTN1_GPIO_Port, BTN2_GPIO_Port, BTN3_GPIO_Port, BTN4_GPIO_Port};
static uint16_t button_pins[4] = {BTN1_Pin, BTN2_Pin, BTN3_Pin, BTN4_Pin};
static struct Button_Handle buttons = {.htim = &htim3, .ports = button_ports, .pins = button_pins, .count = 4};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Gps_Handle gps = {.huart = &huart3, .hdma = &hdma_usart3_rx, .rxBufSize = 1000, .txBufSize = 1000};
static struct Nmea_Handle nmea = {.intPin = PPS_Pin, .timepulse = 0.1f};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Lps22hh_Handle pressure = {.hspi = &hspi1, .csPort = CSP_GPIO_Port, .csPin = CSP_Pin, .intPin = DRDYP_Pin};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Qmc5883_Handle magnet = {.hi2c = &hi2c2, .intPin = DRDYM_Pin, .readTemp = 1};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Icm42688_Handle imu = {.hspi = &hspi1, .csPort = CSG_GPIO_Port, .csPin = CSG_Pin, .intPin = INTG_Pin};
/////////////////////////////////////////////////////////////////////////////////////////////
static uint8_t tach_spokes[3] = {3, 3, 3};
static uint16_t tach_max_rpm[3] = {7000, 7000, 7000};
static struct Tach_Handle tachs = {.htimCount = &htim1, .htimCalc = &htim2, .countFreq = 50000, .channelCount = 3, .spokes = tach_spokes, .maxRpm = tach_max_rpm};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Oled_Handle oled = {.hspi = &hspi2, .csPort = CSD_GPIO_Port, .csPin = CSD_Pin, .dcPort = DC_GPIO_Port, .dcPin = DC_Pin, .width = 128, .height = 64};
static struct Display_Handle display;

static void app_memory_reset(void) {
	Memory_Reset(&memory);
}

static void app_trigger_live() {
	Oled_ClearRectangle(&oled, 44, 34, 86, 45);
	Oled_SetCursor(&oled, 51, 34);
//	sprintf(display.charBuf, "%06.0f", Control_Slip()*1000.0);
	Oled_DrawString(&oled, display.charBuf, &Font_7x10);
	Oled_DrawBitmap(&oled, 70, 42, Bitmap_Decimal, 3, 3);
}

static void app_rpm_live(uint8_t chan) {
	Oled_ClearRectangle(&oled, 48, 34, 75, 44);
	Oled_SetCursor(&oled, 48, 34);
	sprintf(display.charBuf, "%4d", tachs.rpm[chan]);
	Oled_DrawString(&oled, display.charBuf, &Font_7x10);
}

static void app_sensor1_live(void) {
	app_rpm_live(0);
}

static void app_sensor2_live(void) {
	app_rpm_live(1);
}

static void app_sensor3_live(void) {
	app_rpm_live(2);
}

static void app_gps_live(void) {
	Oled_ClearRectangle(&oled, 44, 34, 86, 45);
	if (nmea.fix) {
		Oled_SetCursor(&oled, 51, 34);
		sprintf(display.charBuf, "%4.0f", nmea.speed*10.0);
		Oled_DrawString(&oled, display.charBuf, &Font_7x10);
		Oled_DrawBitmap(&oled, 70, 42, Bitmap_Decimal, 3, 3);
	}else{
		Oled_SetCursor(&oled, 43, 34);
		Oled_DrawString(&oled, "No Fix", &Font_7x10);
	}
}

static void app_pressure_live(void) {

}

static void app_magnet_live(void) {

}

static void app_imu_live(void) {

}

static void app_home_live(void) {

}
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen trigger_live = {.update = &app_trigger_live};
static struct Display_Screen trigger_prim = {.var = &var_trigger_prim};
static struct Display_Screen trigger_aux = {.var = &var_trigger_aux};
static struct Display_Screen trigger_min = {.var = &var_trigger_min};
static struct Display_Option trigger_options[4] = {
		{.text = "Live", .redirect = &trigger_live},
		{.text = "Primary", .redirect = &trigger_prim},
		{.text = "Auxiliary", .redirect = &trigger_aux},
		{.text = "Minimum", .redirect = &trigger_min},
};
static struct Display_Screen trigger_screen = {.optionCount = 4, .options = trigger_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen sensor1_live = {.update = &app_sensor1_live};
static struct Display_Screen sensor1_scale = {.var = &var_sensor1_scale};
static struct Display_Screen sensor1_max = {.var = &var_sensor1_max};
static struct Display_Option sensor1_options[3] = {
		{.text = "Live", .redirect = &sensor1_live},
		{.text = "Scale", .redirect = &sensor1_scale},
		{.text = "Maximum", .redirect = &sensor1_max},
};
static struct Display_Screen sensor1_screen = {.optionCount = 3, .options = sensor1_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen sensor2_live = {.update = &app_sensor2_live};
static struct Display_Screen sensor2_scale = {.var = &var_sensor2_scale};
static struct Display_Screen sensor2_max = {.var = &var_sensor2_max};
static struct Display_Option sensor2_options[3] = {
		{.text = "Live", .redirect = &sensor2_live},
		{.text = "Scale", .redirect = &sensor2_scale},
		{.text = "Maximum", .redirect = &sensor2_max},
};
static struct Display_Screen sensor2_screen = {.optionCount = 3, .options = sensor2_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen sensor3_live = {.update = &app_sensor3_live};
static struct Display_Screen sensor3_scale = {.var = &var_sensor3_scale};
static struct Display_Screen sensor3_max = {.var = &var_sensor3_max};
static struct Display_Option sensor3_options[3] = {
		{.text = "Live", .redirect = &sensor3_live},
		{.text = "Scale", .redirect = &sensor3_scale},
		{.text = "Maximum", .redirect = &sensor3_max},
};
static struct Display_Screen sensor3_screen = {.optionCount = 3, .options = sensor3_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen gps_live = {.update = &app_gps_live};
static struct Display_Screen gps_circ = {.var = &var_gps_circ};
static struct Display_Option gps_options[2] = {
		{.text = "Live", .redirect = &gps_live},
		{.text = "Circumference", .redirect = &gps_circ},
};
static struct Display_Screen gps_screen = {.optionCount = 2, .options = gps_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen pressure_screen = {.update = &app_pressure_live};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen magnet_screen = {.update = &app_magnet_live};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen imu_screen = {.update = &app_imu_live};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Option menu_options[10] = {
		{.text = "Enable", .on = "[ON]", .off = "[OFF]", .var = &var_active},
		{.text = "Trigger", .redirect = &trigger_screen},
		{.text = "Sensor1", .redirect = &sensor1_screen},
		{.text = "Sensor2", .redirect = &sensor2_screen},
		{.text = "Sensor3", .redirect = &sensor3_screen},
		{.text = "GPS", .redirect = &gps_screen},
		{.text = "Pressure", .redirect = &pressure_screen},
		{.text = "Magnometer", .redirect = &magnet_screen},
		{.text = "Accel/Gyro", .redirect = &imu_screen},
		{.text = "Reset", .action = &app_memory_reset},
};
static struct Display_Screen menu_screen = {.optionCount = 10, .options = menu_options};
static struct Display_Screen home_screen = {.update = &app_home_live, .redirect = &menu_screen};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Handle display = {.oled = &oled, .buttons = &buttons, .memory = &memory, .top = &home_screen, .depth = 4, .chars = 12};

//12, 12.5, 3.6
//static struct Vector3f mag_bias = {.x = 1400.0f, .y = 1500.0f, .z = 430.0f};
//static struct Transform3f mag_transform = {.v1 = {1.012f, 0.0f, -0.004f}, .v2 = {0.0f, 1.002f, 0.014f}, .v3 = {-0.004f, 0.014f, 0.987f}};

//static void app_mag_cal(void) {
//	// sending milligauss
//	// bias values should be multiplied by 120
//	char buf[100];
//	sprintf(buf, "Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r", 0, 0, 0, 0, 0, 0, (int16_t)(magnet.vec->x)/12, (int16_t)(magnet.vec->y)/12, (int16_t)(magnet.vec->z)/12);
//	CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
//}

static uint16_t app_rpm_max(uint16_t a, uint16_t b) {
	if (a > b) {
		return a;
	}
	return b;
}

void App_Init(void) {
	// good
	Dac_Set(&dac1, 2048);
	Dac_Set(&dac2, 2048);
	Dac_Set(&dac3, 2048);

	HAL_Delay(100);
	Oled_Init(&oled); // good
	Memory_Init(&memory); // good
	Button_Init(&buttons); // good
	Gps_Init(&gps); // good
	Nmea_Init(&nmea); // good
	Lps22hh_Init(&pressure); // good
	Qmc5883_Init(&magnet); // good
	Icm42688_Init(&imu); // good
	Tach_Init(&tachs); // good
	Display_Init(&display);

	// good
	HAL_COMP_Start(&hcomp1);
	HAL_COMP_Start(&hcomp2);
	HAL_COMP_Start(&hcomp3);

	// good
	HAL_TIM_Base_Start_IT(&htim4);
}

void App_Update(void) {
	const uint16_t vehicle_rpm = app_rpm_max(Tach_GetRpm(&tachs, 0), var_trigger_min.value);
	const uint16_t left_tire_rpm = Tach_GetRpm(&tachs, 1);
	const uint16_t right_tire_rpm = Tach_GetRpm(&tachs, 2);
	const uint16_t avg_tire_rpm = (left_tire_rpm + right_tire_rpm) / 2;
	const uint16_t max_tire_rpm = app_rpm_max(app_rpm_max(left_tire_rpm, right_tire_rpm), 1);

	slip = 1.0f - vehicle_rpm / max_tire_rpm;
	HAL_GPIO_WritePin(DRV1_GPIO_Port, DRV1_Pin, var_active.value || (slip > var_trigger_prim.value));
	HAL_GPIO_WritePin(DRV2_GPIO_Port, DRV2_Pin, var_active.value || (avg_tire_rpm < var_trigger_aux.value));

	__HAL_TIM_SET_AUTORELOAD(&htim4, 40000/vehicle_rpm);
	//20000 = 1rpm
	//1 = 20000rpm

	Display_Update(&display);
	HAL_Delay(20);
//	char buf[20];
//	sprintf(buf, "%d", Tach_GetRpm(&tachs, 0));
//	Oled_ClearRectangle(&oled, 0, 31, 128, 50);
//	Oled_SetCursor(&oled, 4, 32);
//	Oled_DrawString(&oled, buf, &Font_7x10);
//	Oled_Update(&oled);
//
//	HAL_Delay(100);

//	const struct Vector3f vec = *magnet.vec;
//	const float ang = atan2f(vec.y, vec.z);
//	CDC_Transmit_FS((uint8_t*)&ang, 4);
//	app_accel_cal();
//	app_mag_cal();
//	app_accel_cal();
//	uint8_t data[20];
//	memcpy(data, imu.data, 20);
//	CDC_Transmit_FS(data, 20);
}

void App_UsbHandler(uint8_t* data, uint32_t len) {
//	Gps_Transmit(&gps, data, len);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (Tach_TimFlagCount(&tachs, htim)) {
		Tach_TimHandlerCount(&tachs);
	} else if (Tach_TimFlagCalc(&tachs, htim)) {
		Tach_TimHandlerCalc(&tachs);
	} else if (Button_TimFlag(&buttons, htim)) {
		Button_TimHandler(&buttons);
	} else if (htim == &htim4) {
		HAL_GPIO_TogglePin(REPL_GPIO_Port, REPL_Pin);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (Nmea_ExtFlag(&nmea, GPIO_Pin)) {
//		Nmea_ExtHandler(&nmea);
	} else if (Lps22hh_ExtFlag(&pressure, GPIO_Pin)) {
		Lps22hh_ExtHandler(&pressure);
	} else if (Qmc5883_ExtFlag(&magnet, GPIO_Pin)) {
		Qmc5883_ExtHandler(&magnet);
	} else if (Icm42688_ExtFlag(&imu, GPIO_Pin)) {
		Icm42688_ExtHandler(&imu);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	if (Gps_UartFlag(&gps, huart)) {
		Gps_UartHandler(&gps, size);
//		CDC_Transmit_FS(gps.readBuf, size);
		Nmea_Parse(&nmea, gps.readBuf, size);
	}
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
	if (hcomp == &hcomp2) {
		Tach_Trigger(&tachs, 0);
	}else if (hcomp == &hcomp1) {
		Tach_Trigger(&tachs, 1);
	}else if (hcomp == &hcomp3){
		Tach_Trigger(&tachs, 2);
	}
}
