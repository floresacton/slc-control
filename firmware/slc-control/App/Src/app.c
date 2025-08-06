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

//#define GPS_TO_USB
//#define USB_TO_GPS

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1; // 10khz tach tick
extern TIM_HandleTypeDef htim2; // 10hz tach update
extern TIM_HandleTypeDef htim3; // 50hz button update
extern TIM_HandleTypeDef htim8; // replicator

extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac3;

extern COMP_HandleTypeDef hcomp1;
extern COMP_HandleTypeDef hcomp2;
extern COMP_HandleTypeDef hcomp3;

/////////////////////////////////////////////////////////////////////////////////////////////
static struct Memory_Variable var_active = {.reset = 0.0f};
// percent slip trigger
static struct Memory_Variable var_trigger_prim = {.min = 0.0f, .max = 99.9f, .reset = 10.0f, .decimals = 0};
// minimumm rear wheel speed to trigger
static struct Memory_Variable var_trigger_min = {.min = 0.0f, .max = 500.0f, .reset = 10.0f, .decimals = 0};
// rear wheel speed trigger aux
static struct Memory_Variable var_trigger_aux = {.min = 0.0f, .max = 500.0f, .reset = 100.0f, .decimals = 0};

static struct Memory_Variable var_sensor1_pulses = {.min = 1.0f, .max = 99.0f, .reset = 30.0f, .decimals = 0};
static struct Memory_Variable var_sensor1_circ = {.min = 0.0f, .max = 5000.0f, .reset = 2000.0f, .decimals = 0};
static struct Memory_Variable var_sensor1_max = {.min = 0.0f, .max = 500.0f, .reset = 300.0f, .decimals = 0};

static struct Memory_Variable var_sensor2_pulses = {.min = 1.0f, .max = 99.0f, .reset = 30.0f, .decimals = 0};
static struct Memory_Variable var_sensor2_circ = {.min = 0.0f, .max = 5000.0f, .reset = 2000.0f, .decimals = 0};
static struct Memory_Variable var_sensor2_max = {.min = 0.0f, .max = 500.0f, .reset = 300.0f, .decimals = 0};

static struct Memory_Variable var_sensor3_pulses = {.min = 1.0f, .max = 99.0f, .reset = 30.0f, .decimals = 0};
static struct Memory_Variable var_sensor3_circ = {.min = 0.0f, .max = 5000.0f, .reset = 2000.0f, .decimals = 0};
static struct Memory_Variable var_sensor3_max = {.min = 0.0f, .max = 500.0f, .reset = 300.0f, .decimals = 0};

static struct Memory_Variable var_replicator_pulses = {.min = 1.0f, .max = 99.0f, .reset = 50.0f, .decimals = 0};
static struct Memory_Variable var_replicator_circ = {.min = 0.0f, .max = 500.0f, .reset = 100.0f, .decimals = 0};
static struct Memory_Variable var_replicator_reload = {.min = 1.0f, .max = 50000.0f, .reset = 1000.0f, .decimals = 0};
static struct Memory_Variable var_replicator_duty = {.min = 1.0f, .max = 5000.0f, .reset = 500.0f, .decimals = 0};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Memory_Variable* memory_vars[17] = {
        &var_active,
        &var_trigger_prim,
        &var_trigger_min,
        &var_trigger_aux,
        &var_sensor1_pulses,
        &var_sensor1_circ,
        &var_sensor1_max,
        &var_sensor2_pulses,
        &var_sensor2_circ,
        &var_sensor2_max,
        &var_sensor3_pulses,
        &var_sensor3_circ,
        &var_sensor3_max,
        &var_replicator_pulses,
        &var_replicator_circ,
        &var_replicator_reload,
        &var_replicator_duty,
};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Eeprom_Handle eeprom = {.hi2c = &hi2c1, .address = 0xA0, .pages = 512, .pageSize = 64};
static struct Memory_Handle memory = {.eeprom = &eeprom, .hash = 49, .vars = memory_vars, .count = 17};
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
static struct Tach_Handle tach1 = {.tick_freq = 50000};
static struct Tach_Handle tach2 = {.tick_freq = 50000};
static struct Tach_Handle tach3 = {.tick_freq = 50000};
static struct Tach_Handle* tachs[3] = {&tach1, &tach2, &tach3};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Oled_Handle oled = {.hspi = &hspi2, .csPort = CSD_GPIO_Port, .csPin = CSD_Pin, .dcPort = DC_GPIO_Port, .dcPin = DC_Pin, .width = 128, .height = 64};
static struct Display_Handle display;

static void app_memory_reset(void) {
    Memory_Reset(&memory);
}

static void app_values_update(void) {
    const uint32_t mm_per_hour1 = (uint16_t)var_sensor1_max.value * 1609347;
    tach1.ppr = var_sensor1_pulses.value;
    tach1.max_rpm = mm_per_hour1 / ((uint32_t)var_sensor1_circ.value * 60);

    const uint32_t mm_per_hour2 = (uint16_t)var_sensor2_max.value * 1609347;
    tach2.ppr = var_sensor2_pulses.value;
    tach2.max_rpm = mm_per_hour2 / ((uint32_t)var_sensor2_circ.value * 60);

    const uint32_t mm_per_hour3 = (uint16_t)var_sensor3_max.value * 1609347;
    tach3.ppr = var_sensor3_pulses.value;
    tach3.max_rpm = mm_per_hour3 / ((uint32_t)var_sensor3_circ.value * 60);

    __HAL_TIM_SET_AUTORELOAD(&htim8, (uint16_t)var_replicator_reload.value-1);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint16_t)var_replicator_duty.value-1);
}

static uint8_t app_trigger_live() {
    /*
    Oled_ClearRectangle(&oled, 44, 34, 86, 45);
    Oled_SetCursor(&oled, 51, 34);
//  sprintf(display.charBuf, "%06.0f", Control_Slip()*1000.0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 70, 42, Bitmap_Decimal, 3, 3);
    */
    return 1;
}

static uint8_t app_sensor_live(uint8_t chan) {
    Oled_Fill(&oled, Oled_ColorBlack);
    //Oled_ClearRectangle(&oled, 48, 34, 75, 44);
    
    const uint16_t rpm = tachs[chan]->rpm;
    sprintf(display.charBuf, "RPM: %4d", rpm);
    Oled_SetCursor(&oled, 32, 20);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    
    const uint32_t mm_per_hour = tachs[chan]->rpm * ((uint32_t)var_sensor3_circ.value * 60);
    const uint16_t mph = mm_per_hour / 1609347; 
    sprintf(display.charBuf, "MPH:  %3d", mph);
    Oled_SetCursor(&oled, 32, 36);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    
    return 1;
}

static uint8_t app_sensor1_live(void) {
    return app_sensor_live(0);
}

static uint8_t app_sensor2_live(void) {
    return app_sensor_live(1);
}

static uint8_t app_sensor3_live(void) {
    return app_sensor_live(2);
}

static uint8_t app_gps_live(void) {
    Oled_Fill(&oled, Oled_ColorBlack);
    
    Oled_SetCursor(&oled, 0, 0);
    Oled_DrawString(&oled, nmea.fix ? "FIX" : "NO FIX", &Font_7x10);

    sprintf(display.charBuf, "%2dSAT", nmea.satCount);
    Oled_SetCursor(&oled, 91, 0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    
    sprintf(display.charBuf, "%02d/%02d/%02d", nmea.month, nmea.day, nmea.year);
    Oled_SetCursor(&oled, 0, 16);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);

    sprintf(display.charBuf, "%02d:%02d:%02d", nmea.hour, nmea.minute, (uint16_t)nmea.second);
    Oled_SetCursor(&oled, 70, 16);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);

    if (nmea.fix) {
        Str_PrintFloat(display.charBuf, 7, 5, 0, nmea.latitude);
        Oled_SetCursor(&oled, 0, 32);
        Oled_DrawString(&oled, display.charBuf, &Font_7x10);
        Oled_DrawChar(&oled, nmea.latHem, &Font_7x10);
        Oled_DrawBitmap(&oled, 12, 39, Bitmap_Decimal, 3, 3);

        Str_PrintFloat(display.charBuf, 8, 5, false, nmea.longitude);
        Oled_SetCursor(&oled, 63, 32);
        Oled_DrawString(&oled, display.charBuf, &Font_7x10);
        Oled_DrawChar(&oled, nmea.lonHem, &Font_7x10);
        Oled_DrawBitmap(&oled, 82, 39, Bitmap_Decimal, 3, 3);

        sprintf(display.charBuf, "%5d", (uint16_t)nmea.altitude);
        Oled_SetCursor(&oled, 0, 48);
        Oled_DrawString(&oled, display.charBuf, &Font_7x10);
        Oled_SetCursor(&oled, 37, 48);
        Oled_DrawString(&oled, "FT", &Font_7x10);

        sprintf(display.charBuf, "%3dMPH", (uint16_t)nmea.speed);
        Oled_SetCursor(&oled, 84, 48);
        Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    }
    return 1;
}

static uint8_t app_pressure_live(void) {
    Oled_Fill(&oled, Oled_ColorBlack);
    //Oled_ClearRectangle(&oled, 44, 34, 86, 45);
    Oled_SetCursor(&oled, 35, 16);
    Oled_DrawString(&oled, "Kpa:", &Font_7x10);

    Str_PrintFloat(display.charBuf, 4, 1, false, pressure.pressure);
    Oled_SetCursor(&oled, 70, 16);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 89, 23, Bitmap_Decimal, 3, 3);
    
    Oled_SetCursor(&oled, 28, 32);
    Oled_DrawString(&oled, "Temp: ", &Font_7x10);

    Str_PrintFloat(display.charBuf, 4, 1, false, pressure.temperature);
    Oled_SetCursor(&oled, 70, 32);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 89, 39, Bitmap_Decimal, 3, 3);

    return 1;
}

static uint8_t app_magnet_live(void) {
    Oled_Fill(&oled, Oled_ColorBlack);
    //Oled_ClearRectangle(&oled, 44, 34, 86, 45);
    Oled_SetCursor(&oled, 63, 0);
    Oled_DrawString(&oled, "X:", &Font_7x10);
    Str_PrintFloat(display.charBuf, 5, 3, false, magnet.x);
    Oled_SetCursor(&oled, 77, 0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 89, 7, Bitmap_Decimal, 3, 3);

    Oled_SetCursor(&oled, 63, 16);
    Oled_DrawString(&oled, "Y:", &Font_7x10);
    Str_PrintFloat(display.charBuf, 5, 3, false, magnet.y);
    Oled_SetCursor(&oled, 77, 16);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 89, 23, Bitmap_Decimal, 3, 3);
    
    Oled_SetCursor(&oled, 63, 32);
    Oled_DrawString(&oled, "Z:", &Font_7x10);
    Str_PrintFloat(display.charBuf, 5, 3, false, magnet.z);
    Oled_SetCursor(&oled, 77, 32);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10); 
    Oled_DrawBitmap(&oled, 89, 39, Bitmap_Decimal, 3, 3);

    Oled_SetCursor(&oled, 42, 48);
    Oled_DrawString(&oled, "Temp: ", &Font_7x10);
    Str_PrintFloat(display.charBuf, 4, 1, false, magnet.temperature);
    Oled_SetCursor(&oled, 84, 48);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 103, 55, Bitmap_Decimal, 3, 3);
    
    float dx = 16.0f * (float)cos(magnet.angle);
    float dy = 16.0f * (float)sin(magnet.angle);
    
    Oled_DrawLine(&oled, 16, 24, 16 + (int16_t)dx, 32 + (int16_t)dy);
    return 1;
}

static uint8_t app_imu_live(void) {
    Oled_Fill(&oled, Oled_ColorBlack);
    //Oled_ClearRectangle(&oled, 44, 34, 86, 45);
    Oled_SetCursor(&oled, 56, 0);
    Oled_DrawString(&oled, "X:", &Font_7x10);
    Str_PrintFloat(display.charBuf, 5, 2, false, imu.accelx);
    Oled_SetCursor(&oled, 77, 0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 96, 7, Bitmap_Decimal, 3, 3);

    Oled_SetCursor(&oled, 56, 16);
    Oled_DrawString(&oled, "Y:", &Font_7x10);
    Str_PrintFloat(display.charBuf, 5, 2, false, imu.accely);
    Oled_SetCursor(&oled, 77, 16);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 96, 23, Bitmap_Decimal, 3, 3);
    
    Oled_SetCursor(&oled, 56, 32);
    Oled_DrawString(&oled, "Z:", &Font_7x10);
    Str_PrintFloat(display.charBuf, 5, 2, false, imu.accelz);
    Oled_SetCursor(&oled, 77, 32);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10); 
    Oled_DrawBitmap(&oled, 96, 39, Bitmap_Decimal, 3, 3);

    Oled_SetCursor(&oled, 35, 48);
    Oled_DrawString(&oled, "Temp: ", &Font_7x10);
    Str_PrintFloat(display.charBuf, 4, 1, false, imu.temperature);
    Oled_SetCursor(&oled, 84, 48);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 103, 55, Bitmap_Decimal, 3, 3);
    return 1;
}

static float app_gabs(float accel) {
    return accel >= 0 ? accel : -accel;
}
static uint8_t app_home_live(void) {
    Oled_Fill(&oled, Oled_ColorBlack);
   
    sprintf(display.charBuf, "%3d", (uint16_t)pressure.temperature);
    Oled_SetCursor(&oled, 7, 0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 29, 0, Bitmap_Decimal, 3, 3);
    Oled_SetCursor(&oled, 32, 0);
    Oled_DrawString(&oled, "F", &Font_7x10);

    sprintf(display.charBuf, "%02d:%02d", nmea.hour, nmea.minute);
    Oled_SetCursor(&oled, 84, 0);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    
    if (nmea.fix) {
        sprintf(display.charBuf, "%3d", (uint16_t)nmea.speed);
    } else {
        display.charBuf[0] = '-';
        display.charBuf[1] = 0;
    }
    Oled_SetCursor(&oled, 77, 21);
    Oled_DrawString(&oled, display.charBuf, &Font_14x20);

    Oled_DrawBitmap(&oled, 7, 20, Bitmap_Forward, 7, 8);
    Str_PrintFloat(display.charBuf, 4, 1, false, app_gabs(imu.accelz));
    Oled_SetCursor(&oled, 14, 20);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 33, 27, Bitmap_Decimal, 3, 3);
    
    Oled_DrawBitmap(&oled, 7, 36, Bitmap_Lateral, 8, 7);
    Str_PrintFloat(display.charBuf, 4, 1, false, app_gabs(imu.accelx));
    Oled_SetCursor(&oled, 14, 36);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_DrawBitmap(&oled, 33, 43, Bitmap_Decimal, 3, 3);

    Str_PrintFloat(display.charBuf, 4, 1, false, pressure.pressure);
    Oled_SetCursor(&oled, 7, 54);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_SetCursor(&oled, 37, 54);
    Oled_DrawString(&oled, "kPa", &Font_7x10);

    sprintf(display.charBuf, "%5d", (uint16_t)nmea.altitude);
    Oled_SetCursor(&oled, 70, 54);
    Oled_DrawString(&oled, display.charBuf, &Font_7x10);
    Oled_SetCursor(&oled, 107, 54);
    Oled_DrawString(&oled, "FT", &Font_7x10);
    return 1;
}
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen trigger_live = {.update = &app_trigger_live};
static struct Display_Screen trigger_prim = {.var = &var_trigger_prim};
static struct Display_Screen trigger_min = {.var = &var_trigger_min};
static struct Display_Screen trigger_aux = {.var = &var_trigger_aux};
static struct Display_Option trigger_options[4] = {
        {.text = "Live", .redirect = &trigger_live},
        {.text = "Primary", .redirect = &trigger_prim},
        {.text = "Minimum", .redirect = &trigger_min},
        {.text = "Auxilary", .redirect = &trigger_aux},
};
static struct Display_Screen trigger_screen = {.optionCount = 4, .options = trigger_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen sensor1_live = {.update = &app_sensor1_live};
static struct Display_Screen sensor1_pulses = {.var = &var_sensor1_pulses};
static struct Display_Screen sensor1_circ = {.var = &var_sensor1_circ};
static struct Display_Screen sensor1_max = {.var = &var_sensor1_max};
static struct Display_Option sensor1_options[4] = {
        {.text = "Live", .redirect = &sensor1_live},
        {.text = "Pulses", .redirect = &sensor1_pulses},
        {.text = "Circ MM", .redirect = &sensor1_circ},
        {.text = "Max MPH", .redirect = &sensor1_max},
};
static struct Display_Screen sensor1_screen = {.optionCount = 4, .options = sensor1_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen sensor2_live = {.update = &app_sensor2_live};
static struct Display_Screen sensor2_pulses = {.var = &var_sensor2_pulses};
static struct Display_Screen sensor2_circ = {.var = &var_sensor2_circ};
static struct Display_Screen sensor2_max = {.var = &var_sensor2_max};
static struct Display_Option sensor2_options[4] = {
        {.text = "Live", .redirect = &sensor2_live},
        {.text = "Pulses", .redirect = &sensor2_pulses},
        {.text = "Circ MM", .redirect = &sensor2_circ},
        {.text = "Max MPH", .redirect = &sensor2_max},
};
static struct Display_Screen sensor2_screen = {.optionCount = 4, .options = sensor2_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen sensor3_live = {.update = &app_sensor3_live};
static struct Display_Screen sensor3_pulses = {.var = &var_sensor3_pulses};
static struct Display_Screen sensor3_circ = {.var = &var_sensor3_circ};
static struct Display_Screen sensor3_max = {.var = &var_sensor3_max};
static struct Display_Option sensor3_options[4] = {
        {.text = "Live", .redirect = &sensor3_live},
        {.text = "Pulses", .redirect = &sensor3_pulses},
        {.text = "Circ MM", .redirect = &sensor3_circ},
        {.text = "Max MPH", .redirect = &sensor3_max},
};
static struct Display_Screen sensor3_screen = {.optionCount = 4, .options = sensor3_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen replicator_pulses = {.var = &var_replicator_pulses};
static struct Display_Screen replicator_circ = {.var = &var_replicator_circ};
static struct Display_Screen replicator_reload = {.var = &var_replicator_reload};
static struct Display_Screen replicator_duty = {.var = &var_replicator_duty};
static struct Display_Option replicator_options[4] = {
        {.text = "Pulses", .redirect = &replicator_pulses},
        {.text = "Circ IN", .redirect = &replicator_circ},
        {.text = "Auto Reload", .redirect = &replicator_reload},
        {.text = "Duty Cycle", .redirect = &replicator_duty},
};
static struct Display_Screen replicator_screen = {.optionCount = 4, .options = replicator_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen gps_live = {.update = &app_gps_live};
//static struct Display_Option gps_options[1] = {
//        {.text = "Live", .redirect = &gps_live},
//};
//static struct Display_Screen gps_screen = {.optionCount = 2, .options = gps_options};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen pressure_screen = {.update = &app_pressure_live};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen magnet_screen = {.update = &app_magnet_live};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Screen imu_screen = {.update = &app_imu_live};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Option menu_options[9] = {
//        {.text = "Enable", .var = &var_active},
//        {.text = "Trigger", .redirect = &trigger_screen},
        {.text = "Sensor1", .redirect = &sensor1_screen},
        {.text = "Sensor2", .redirect = &sensor2_screen},
        {.text = "Sensor3", .redirect = &sensor3_screen},
        {.text = "Replicator", .redirect = &replicator_screen},
        {.text = "GPS", .redirect = &gps_live},
        {.text = "Accel", .redirect = &imu_screen},
        {.text = "Magnometer", .redirect = &magnet_screen},
        {.text = "Pressure", .redirect = &pressure_screen},
        {.text = "Reset", .action = &app_memory_reset},
};
static struct Display_Screen menu_screen = {.optionCount = 9, .options = menu_options};
static struct Display_Screen home_screen = {.update = &app_home_live, .redirect = &menu_screen};
/////////////////////////////////////////////////////////////////////////////////////////////
static struct Display_Handle display = {.oled = &oled, .buttons = buttons, .memory = &memory, .top = &home_screen, .depth = 4, .chars = 12, .values_update = &app_values_update};

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

void App_Init(void) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
    HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

    HAL_Delay(100);

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

    app_values_update();

    Tach_Init(&tach1);
    Tach_Init(&tach2);
    Tach_Init(&tach3);

    Display_Init(&display);

    HAL_COMP_Start(&hcomp1);
    HAL_COMP_Start(&hcomp2);
    HAL_COMP_Start(&hcomp3);

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
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
    HAL_Delay(50);

    //HAL_GPIO_WritePin(REPH_GPIO_Port, REPH_Pin, 1);
    //HAL_Delay(30);
    //HAL_GPIO_WritePin(REPH_GPIO_Port, REPH_Pin, 0);
    //HAL_Delay(30);
    const uint32_t mm_per_hour = tach1.rpm * ((uint32_t)var_sensor3_circ.value * 60);
    const uint16_t mph = mm_per_hour / 1609347;

    const uint32_t pps = ((uint32_t)var_replicator_pulses.value * mph * 1056) / (60 * (uint32_t)var_replicator_circ.value);
    const uint32_t clock_div = 144000000 / (uint16_t)var_replicator_reload.value; // 144000
    uint16_t prescale = 65535;
    if (pps != 0) {
        uint32_t temp_scale = clock_div / pps;
        if (temp_scale < 65535) {
            prescale = temp_scale;
        }
    }
    htim8.Instance->PSC = prescale-1;

    Lps22hh_ExtHandler(&pressure);
    Qmc5883_ExtHandler(&magnet);
    Icm42688_ExtHandler(&imu);

    //8.8 pps
    //2000 pps
    // for good resolution
    // 200khz
    // 100->30,000

    // HAL_GPIO_TogglePin(REPL_GPIO_Port, REPL_Pin);


    //ARR = 1000-1
    //CCR = 100-1
    //144,000,000
    //


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
        Tach_Tick(&tach1);
        Tach_Tick(&tach2);
        Tach_Tick(&tach3);
    } else if (htim == &htim2) {
        Tach_Update(&tach1);
        Tach_Update(&tach2);
        Tach_Update(&tach3);
    } else if (htim == &htim3) {
        Button_Update(&button1);
        Button_Update(&button2);
        Button_Update(&button3);
        Button_Update(&button4);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (Nmea_ExtFlag(&nmea, GPIO_Pin)) {
        //Nmea_ExtHandler(&nmea);
    } else if (Lps22hh_ExtFlag(&pressure, GPIO_Pin)) {
        //Lps22hh_ExtHandler(&pressure);
    } else if (Qmc5883_ExtFlag(&magnet, GPIO_Pin)) {
        //Qmc5883_ExtHandler(&magnet);
    } else if (Icm42688_ExtFlag(&imu, GPIO_Pin)) {
        //Icm42688_ExtHandler(&imu);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (Gps_UartFlag(&gps, huart)) {
        Gps_UartHandler(&gps, size);
#ifdef GPS_TO_USB
        CDC_Transmit_FS(gps.readBuf, size);
#endif
        Nmea_Parse(&nmea, (char*)gps.readBuf, size);
    }
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
    if (hcomp == &hcomp2) {
        Tach_Pulse(&tach1);
    } else if (hcomp == &hcomp1) {
        Tach_Pulse(&tach2);
    } else if (hcomp == &hcomp3){
        Tach_Pulse(&tach3);
    }
}
