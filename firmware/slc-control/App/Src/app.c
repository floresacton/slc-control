#include "app.h"
#include "math.h"
#include "str.h"
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
#include "tach.h"
#include "lps22hh.h"
#include "qmc5883.h"
#include "icm42688.h"

#define GPS_TO_USB
#define USB_TO_GPS

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1; // 50khz tach count
extern TIM_HandleTypeDef htim2; // 50hz tach update
extern TIM_HandleTypeDef htim3; // 50hz button update
extern TIM_HandleTypeDef htim17; // replicator

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac3;

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
static struct Memory_Handle memory = {.eeprom = &eeprom, .hash = 49, .vars = memory_vars, .count = 11};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Button_Handle button1 = {.port = BTN1_GPIO_Port, .pin = BTN1_Pin};
static struct Button_Handle button2 = {.port = BTN2_GPIO_Port, .pin = BTN2_Pin};
static struct Button_Handle button3 = {.port = BTN3_GPIO_Port, .pin = BTN3_Pin};
static struct Button_Handle button4 = {.port = BTN4_GPIO_Port, .pin = BTN4_Pin};
static struct Button_Handle* buttons[4] = {&button1, &button2, &button3, &button4};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Nmea_Handle nmea = {.intPin = PPS_Pin, .timepulse = 0.1f};
static struct Gps_Handle gps = {.huart = &huart3, .hdma = &hdma_usart3_rx, .rxBufSize = 1000, .txBufSize = 1000};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Lps22hh_Handle pressure = {.hspi = &hspi1, .csPort = CSP_GPIO_Port, .csPin = CSP_Pin, .intPin = DRDYP_Pin};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Qmc5883_Handle magnet = {.hi2c = &hi2c2, .intPin = DRDYM_Pin, .readTemp = 1};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Icm42688_Handle imu = {.hspi = &hspi1, .csPort = CSG_GPIO_Port, .csPin = CSG_Pin, .intPin = INTG_Pin};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Tach_Handle tach1 = {.countFreq = 50000, .spokes = 3, .maxRpm = 7000};
static struct Tach_Handle tach2 = {.countFreq = 50000, .spokes = 3, .maxRpm = 7000};
static struct Tach_Handle tach3 = {.countFreq = 50000, .spokes = 3, .maxRpm = 7000};
static struct Tach_Handle* tachs[3] = {&tach1, &tach2, &tach3};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Oled_Handle oled = {.hspi = &hspi2, .csPort = CSD_GPIO_Port, .csPin = CSD_Pin, .dcPort = DC_GPIO_Port, .dcPin = DC_Pin, .width = 128, .height = 64};
static struct Display_Handle display;

static void app_memory_reset(void) {
    Memory_Reset(&memory);
}

static uint8_t app_trigger_live() {
    Oled_ClearRectangle(&oled, 44, 34, 86, 45);
    Oled_SetCursor(&oled, 51, 34);
//  sprintf(display.charBuf, "%06.0f", Control_Slip()*1000.0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 70, 42, Bitmap_Decimal, 3, 3);
    return 1;
}

static uint8_t app_rpm_live(uint8_t chan) {
    Oled_ClearRectangle(&oled, 48, 34, 75, 44);
    Oled_SetCursor(&oled, 48, 34);
    sprintf(display.charBuf, "%4d", tachs[chan]->rpm);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    return 1;
}

static uint8_t app_sensor1_live(void) {
    app_rpm_live(0);
    return 1;
}

static uint8_t app_sensor2_live(void) {
    app_rpm_live(1);
    return 1;
}

static uint8_t app_sensor3_live(void) {
    app_rpm_live(2);
    return 1;
}

static uint8_t app_gps_live(void) {
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
    return 1;
}

static uint8_t app_pressure_live(void) {
    Oled_Fill(&oled, Oled_ColorBlack);
    //Oled_ClearRectangle(&oled, 44, 34, 86, 45);
    Oled_SetCursor(&oled, 28, 16);
    Oled_DrawString(&oled, "Kpa:", &Font_7x10);
    const uint8_t len1 = Str_PrintFloat(display.charBuf, pressure.pressure, 1);
    Oled_SetCursor(&oled, 98 - 7*len1, 16);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    
    Oled_SetCursor(&oled, 28, 32);
    Oled_DrawString(&oled, "Temp: ", &Font_7x10);
    const uint8_t len2 = Str_PrintFloat(display.charBuf, pressure.temperature, 1);
    Oled_SetCursor(&oled, 98 - 7*len2, 32);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);

    return 1;
}

static uint8_t app_magnet_live(void) {
    Oled_Fill(&oled, Oled_ColorBlack);
    //Oled_ClearRectangle(&oled, 44, 34, 86, 45);
    Oled_SetCursor(&oled, 56, 0);
    Oled_DrawString(&oled, "X:", &Font_7x10);
    const uint8_t len1 = Str_PrintFloat(display.charBuf, magnet.x, 3);
    Oled_SetCursor(&oled, 119 - 7*len1, 0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);

    Oled_SetCursor(&oled, 56, 16);
    Oled_DrawString(&oled, "Y:", &Font_7x10);
    const uint8_t len2 = Str_PrintFloat(display.charBuf, magnet.y, 3);
    Oled_SetCursor(&oled, 119 - 7*len2, 16);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    
    Oled_SetCursor(&oled, 56, 32);
    Oled_DrawString(&oled, "Z:", &Font_7x10);
    const uint8_t len3 = Str_PrintFloat(display.charBuf, magnet.z, 3);
    Oled_SetCursor(&oled, 119 - 7*len3, 32);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10); 

    Oled_SetCursor(&oled, 49, 48);
    Oled_DrawString(&oled, "Temp: ", &Font_7x10);
    const uint8_t len4 = Str_PrintFloat(display.charBuf, magnet.temperature, 1);
    Oled_SetCursor(&oled, 119 - 7*len4, 48);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    
    float dx = 16.0f * (float)cos(magnet.angle);
    float dy = 16.0f * (float)sin(magnet.angle);
    
    Oled_DrawLine(&oled, 16, 32, 16 + (int16_t)dx, 32 + (int16_t)dy);

    return 1;
}

static uint8_t app_imu_live(void) {
    return 1;
}

static uint8_t app_home_live(void) {
    Oled_SetCursor(&oled, 20, 16);
    Oled_DrawString(&oled, "homescreen", &Font_7x10);
    return 1;
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
        {.text = "Enable", .var = &var_active},
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
static struct Display_Handle display = {.oled = &oled, .buttons = buttons, .memory = &memory, .top = &home_screen, .depth = 4, .chars = 12};

//12, 12.5, 3.6
//static struct Vector3f mag_bias = {.x = 1400.0f, .y = 1500.0f, .z = 430.0f};
//static struct Transform3f mag_transform = {.v1 = {1.012f, 0.0f, -0.004f}, .v2 = {0.0f, 1.002f, 0.014f}, .v3 = {-0.004f, 0.014f, 0.987f}};

//static void app_mag_cal(void) {
//  // sending milligauss
//  // bias values should be multiplied by 120
//  char buf[100];
//  sprintf(buf, "Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r", 0, 0, 0, 0, 0, 0, (int16_t)(magnet.vec->x)/12, (int16_t)(magnet.vec->y)/12, (int16_t)(magnet.vec->z)/12);
//  CDC_Transmit_FS((uint8_t*)buf, strlen(buf));
//}

static uint16_t app_rpm_max(uint16_t a, uint16_t b) {
    if (a > b) {
        return a;
    }
    return b;
}

void App_Init(void) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    //HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
    //HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

    Oled_Init(&oled);
    // eeprom has no init
    Memory_Init(&memory);
    Button_Init(&button1);
    Button_Init(&button2);
    Button_Init(&button3);
    Button_Init(&button4);
    
    Nmea_Init(&nmea);
    Gps_Init(&gps);
    Lps22hh_Init(&pressure);
    Qmc5883_Init(&magnet);
    Icm42688_Init(&imu);
    Tach_Init(&tach1);
    // Tach_Init(&tach2);
    // Tach_Init(&tach3);
    Display_Init(&display);

    HAL_COMP_Start(&hcomp1);
    //HAL_COMP_Start(&hcomp2);
    //HAL_COMP_Start(&hcomp3);

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    //HAL_TIM_Base_Start_IT(&htim17);
}

void App_Update(void) {
    // const uint16_t vehicle_rpm = app_rpm_max(tach1.rpm, var_trigger_min.value);
    // const uint16_t left_tire_rpm = tach2.rpm;
    // const uint16_t right_tire_rpm = tach3.rpm;
    // const uint16_t avg_tire_rpm = (left_tire_rpm + right_tire_rpm) / 2;
    // const uint16_t max_tire_rpm = app_rpm_max(app_rpm_max(left_tire_rpm, right_tire_rpm), 1);

    // slip = 1.0f - vehicle_rpm / max_tire_rpm;
    // HAL_GPIO_WritePin(DRV1_GPIO_Port, DRV1_Pin, var_active.value || (slip > var_trigger_prim.value));
    // HAL_GPIO_WritePin(DRV2_GPIO_Port, DRV2_Pin, var_active.value || (avg_tire_rpm < var_trigger_aux.value));

    // __HAL_TIM_SET_AUTORELOAD(&htim4, 40000/vehicle_rpm);
    //20000 = 1rpm
    //1 = 20000rpm

    Display_Update(&display);
    HAL_Delay(20);
//  char buf[20] = "bob";
// //   sprintf(buf, "%d", Tach_GetRpm(&tachs, 0));
//  Oled_ClearRectangle(&oled, 0, 31, 128, 50);
//  Oled_SetCursor(&oled, 4, 32);
//  Oled_DrawString(&oled, buf, &Font_7x10);
//  Oled_Update(&oled);
//
//  HAL_Delay(100);

//  const struct Vector3f vec = *magnet.vec;
//  const float ang = atan2f(vec.y, vec.z);
//  CDC_Transmit_FS((uint8_t*)&ang, 4);
//  app_accel_cal();
//  app_mag_cal();
//  app_accel_cal();
//  uint8_t data[20];
//  memcpy(data, imu.data, 20);
//  CDC_Transmit_FS(data, 20);
}

void App_UsbHandler(uint8_t* data, uint32_t len) {
#ifdef USB_TO_GPS
    Gps_Transmit(&gps, data, len);
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) {
        Tach_Count(&tach1);
        Tach_Count(&tach2);
        Tach_Count(&tach3);
    } else if (htim == &htim2) {
        Tach_Update(&tach1);
        Tach_Update(&tach2);
        Tach_Update(&tach3);
    } else if (htim == &htim3) {
        Button_Update(&button1);
        Button_Update(&button2);
        Button_Update(&button3);
        Button_Update(&button4);
    } else if (htim == &htim17) {
        // HAL_GPIO_TogglePin(REPL_GPIO_Port, REPL_Pin);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (Nmea_ExtFlag(&nmea, GPIO_Pin)) {
        //Nmea_ExtHandler(&nmea);
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
#ifdef GPS_TO_USB
        CDC_Transmit_FS(gps.readBuf, size);
#endif
        Nmea_Parse(&nmea, gps.readBuf, size);
    }
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
    if (hcomp == &hcomp2) {
        Tach_Trigger(&tach1);
    } else if (hcomp == &hcomp1) {
        Tach_Trigger(&tach2);
    } else if (hcomp == &hcomp3){
        Tach_Trigger(&tach3);
    }
}
