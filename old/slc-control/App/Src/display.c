#include "display.h"
#include "string.h"
#include "math.h"
#include "oled.h"
#include "stdio.h"
#include "nmea.h"
#include "tach.h"
#include "control.h"
#include "lps22hh.h"
#include "qmc5883.h"
#include "button.h"
#include "bitmap.h"

static void control_live_update() {
	Oled_ClearRectangle(44, 34, 86, 45);
	Oled_SetCursor(51, 34);
	char buf[DISPLAY_MAX_CHARACTERS];
	sprintf(buf, "%04.0f", Control_GetSlip()*10.0);
	Oled_DrawString(buf, &Font_7x10);
	Oled_DrawBitmap(70, 42, Bitmap_Decimal, 3, 3);

//	Oled_ClearRectangle(44, 34, 86, 45);
//		Oled_SetCursor(51, 34);
//		char buf[DISPLAY_MAX_CHARACTERS];
//		sprintf(buf, "%04.0f", Control_Ratio()*1000.0);
//		Oled_DrawString(buf, &Font_7x10);
//		Oled_DrawBitmap(56, 42, Bitmap_Decimal, 3, 3);
}

static void rpm_live_update(uint8_t chan) {
	Oled_ClearRectangle(48, 34, 75, 44);
	Oled_SetCursor(48, 34);
	char buf[DISPLAY_MAX_CHARACTERS];
//	sprintf(buf, "%4d", Tach_RPM(chan));
	///////
//	float pressure = 100.0*Lps22hh_Pressure(0)/4096.0-200;
//	float temperature = Lps22hh_Temperature(0)/100.0-4;
//	Oled_Fill(0);
//	Oled_SetCursor(48, 24);
//	sprintf(buf, "%5.1f", temperature);
//	Oled_DrawString(buf, &font_7x10);
//	Oled_SetCursor(48, 34);
//	sprintf(buf, "%5.1f", pressure);
//	Oled_DrawString(buf, &font_7x10);
//	Oled_SetCursor(48, 44);
//
//	float kelvin = temperature+273.15;
//
////	    var d = - 0.0065;
////	    var j = Math.pow((pressure/sea level, (8.31432 * -0.0065) / (9.80665 * 0.0289644));
////	    return (((kelvin) * ((1 / j) - 1)) / d)
//
//	float height = kelvin/0.0065*(1-pow(pressure/101325, 0.190263));
//	sprintf(buf, "%5.1f", height);
	//////
//	uint8_t status = Qmc5883_Status();
//	Qmc5883_Update();
	int16_t x = Qmc5883_GetX();
	int16_t y = Qmc5883_GetY();
	int16_t z = Qmc5883_GetZ();
	int16_t t = Qmc5883_GetTemperature();
	Oled_Fill(0);
	Oled_SetCursor(28, 14);
	sprintf(buf, "%06hd", t);
	Oled_DrawString(buf, &Font_7x10);

	Oled_SetCursor(28, 24);
	sprintf(buf, "%06hd", x);
//	sprintf(buf, "%5.1f", x);
	Oled_DrawString(buf, &Font_7x10);
	Oled_SetCursor(28, 34);
	sprintf(buf, "%06hd", y);
//	sprintf(buf, "%5.1f", y);
	Oled_DrawString(buf, &Font_7x10);
	Oled_SetCursor(28, 44);
	sprintf(buf, "%06hd", z);
	Oled_DrawString(buf, &Font_7x10);
}

static void tach1_live_update(void) {
	rpm_live_update(Tach1);
}

static void tach2_live_update(void) {
	rpm_live_update(Tach2);
}

static void tach3_live_update(void) {
	rpm_live_update(Tach3);
}

static void gps_live_update(void) {
//	Oled_ClearRectangle(44, 34, 86, 45);
	Oled_Fill(OledBlack);
	Oled_SetCursor(51, 24);
	char buf[DISPLAY_MAX_CHARACTERS];
	sprintf(buf, "%4.0f", Nmea_GetSpeed()*10.0);
	Oled_DrawString(buf, &Font_7x10);
	Oled_DrawBitmap(70, 32, Bitmap_Decimal, 3, 3);
	Oled_SetCursor(11, 26);
	sprintf(buf, "%2d", Nmea_GetSatelliteCount());
	Oled_DrawString(buf, &Font_7x10);
	Oled_SetCursor(21, 38);
	struct Time* time = Nmea_GetTime();
	sprintf(buf, "%02u:%02u:%04.1f", (time->hour+20)%24, time->min, time->second);
	Oled_DrawString(buf, &Font_7x10);
	Oled_SetCursor(21, 48);
	sprintf(buf, "%02u/%02u/%02u", time->month, time->day, time->year);
	Oled_DrawString(buf, &Font_7x10);

	struct Position* position = Nmea_GetPosition();
	Oled_SetCursor(0, 0);
	sprintf(buf, "%6.3f %c, %6.2f %c", position->latitude, position->latHem, position->longitude, position->lonHem);
	Oled_DrawString(buf, &Font_7x10);
	Oled_SetCursor(40, 11);
	sprintf(buf, "%.1f", position->altitude);
	Oled_DrawString(buf, &Font_7x10);
}

void compass_live_update(void) {

}

void barometer_live_update(void) {
	Oled_Fill(OledBlack);
	Oled_SetCursor(51, 24);
	char buf[DISPLAY_MAX_CHARACTERS];
	sprintf(buf, "%4.2f", Lps22hh_GetPressure(Lps22hh1)/409.6);
	Oled_DrawString(buf, &Font_7x10);
}

void imu_live_update(void) {

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen control_live = {.type = ScreenLive, .title = "Control Live", .update = &control_live_update};
static struct Screen control_slip = {.type = ScreenEdit, .title = "Unit %", .memType = MemFloat, .loc = MemSlipThresh};
static struct Screen control_aux = {.type = ScreenEdit, .title = "Unit RPM", .memType = MemShort, .loc = MemAuxThresh};
static struct Screen control_min = {.type = ScreenEdit, .title = "Unit RPM", .memType = MemShort, .loc = MemMinRpm};

static struct Option control_options[4] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &control_live},
		{.type = OptionRedirect, .text = "Slip", .redirect = &control_slip},
		{.type = OptionRedirect, .text = "Aux", .redirect = &control_aux},
		{.type = OptionRedirect, .text = "Min", .redirect = &control_min}};
static struct Screen control = {.type = ScreenScroll, .title = "Control", .optionCount = 4, .options = control_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen tach1_live = {.type = ScreenLive, .title = "Tach 1 Live", .update = &tach1_live_update};
static struct Screen tach1_spokes = {.type = ScreenEdit, .title = "Unit #", .memType = MemByte, .loc = MemTach1Spokes};
static struct Screen tach1_thresh = {.type = ScreenEdit, .title = "0-4096", .memType = MemShort, .loc = MemTach1Thresh};
static struct Screen tach1_max = {.type = ScreenEdit, .title = "Unit RPM", .memType = MemShort, .loc = MemTach1Max};

static struct Option tach1_options[4] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &tach1_live},
		{.type = OptionRedirect, .text = "Spokes", .redirect = &tach1_spokes},
		{.type = OptionRedirect, .text = "Thresh", .redirect = &tach1_thresh},
		{.type = OptionRedirect, .text = "Max", .redirect = &tach1_max}};
static struct Screen tach1 = {.type = ScreenScroll, .title = "Tachometer 1", .optionCount = 4, .options = tach1_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen tach2_live = {.type = ScreenLive, .title = "Tach 2 Live", .update = &tach2_live_update};
static struct Screen tach2_spokes = {.type = ScreenEdit, .title = "Unit #", .memType = MemByte, .loc = MemTach2Spokes};
static struct Screen tach2_thresh = {.type = ScreenEdit, .title = "0-4096", .memType = MemShort, .loc = MemTach2Thresh};
static struct Screen tach2_max = {.type = ScreenEdit, .title = "Unit RPM", .memType = MemShort, .loc = MemTach2Max};

static struct Option tach2_options[4] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &tach2_live},
		{.type = OptionRedirect, .text = "Spokes", .redirect = &tach2_spokes},
		{.type = OptionRedirect, .text = "Thresh", .redirect = &tach2_thresh},
		{.type = OptionRedirect, .text = "Max", .redirect = &tach2_max}};
static struct Screen tach2 = {.type = ScreenScroll, .title = "Tachometer 2", .optionCount = 4, .options = tach2_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen tach3_live = {.type = ScreenLive, .title = "Tach 3 Live", .update = &tach3_live_update};
static struct Screen tach3_spokes = {.type = ScreenEdit, .title = "Unit #", .memType = MemByte, .loc = MemTach3Spokes};
static struct Screen tach3_thresh = {.type = ScreenEdit, .title = "0-4096", .memType = MemShort, .loc = MemTach3Thresh};
static struct Screen tach3_max = {.type = ScreenEdit, .title = "Unit RPM", .memType = MemShort, .loc = MemTach3Max};

static struct Option tach3_options[4] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &tach3_live},
		{.type = OptionRedirect, .text = "Spokes", .redirect = &tach3_spokes},
		{.type = OptionRedirect, .text = "Thresh", .redirect = &tach3_thresh},
		{.type = OptionRedirect, .text = "Max", .redirect = &tach3_max}};
static struct Screen tach3 = {.type = ScreenScroll, .title = "Tachometer 3", .optionCount = 4, .options = tach3_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen gps_live = {.type = ScreenLive, .title = "GPS Live", .update = &gps_live_update};
static struct Screen gps_tire = {.type = ScreenEdit, .title = "Diameter(Inch)", .memType = MemFloat, .loc = MemGpsTire};

static struct Option gps_options[2] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &gps_live},
		{.type = OptionRedirect, .text = "Tire", .redirect = &gps_tire}};
static struct Screen gps = {.type = ScreenScroll, .title = "GPS", .optionCount = 2, .options = gps_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen compass_live = {.type = ScreenLive, .title = "Compass Live", .update = &compass_live_update};

static struct Option compass_options[1] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &compass_live}};
static struct Screen compass = {.type = ScreenScroll, .title = "Compass", .optionCount = 1, .options = compass_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen barometer_live = {.type = ScreenLive, .title = "Barometer Live", .update = &barometer_live_update};

static struct Option barometer_options[1] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &barometer_live}};
static struct Screen barometer = {.type = ScreenScroll, .title = "Barometer", .optionCount = 1, .options = barometer_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Screen imu_live = {.type = ScreenLive, .title = "IMU Live", .update = &imu_live_update};

static struct Option imu_options[1] = {
		{.type = OptionRedirect, .text = "Live", .redirect = &imu_live}};
static struct Screen imu = {.type = ScreenScroll, .title = "IMU", .optionCount = 1, .options = imu_options};
/////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct Option menu_options[9] = {
		{.type = OptionRedirect, .text = "Control", .redirect = &control},
		{.type = OptionRedirect, .text = "Tach 1", .redirect = &tach1},
		{.type = OptionRedirect, .text = "Tach 2", .redirect = &tach2},
		{.type = OptionRedirect, .text = "Tach 3", .redirect = &tach3},
		{.type = OptionRedirect, .text = "GPS", .redirect = &gps},
		{.type = OptionRedirect, .text = "Compass", .redirect = &compass},
		{.type = OptionRedirect, .text = "Barometer", .redirect = &barometer},
		{.type = OptionRedirect, .text = "IMU", .redirect = &imu},
		{.type = OptionAction, .text = "Reset", .action = &Memory_Reset}};
static struct Screen menu = {.type = ScreenScroll, .title = "Menu", .optionCount = 9, .options =
		menu_options, };
/////////////////////////////////////////////////////////////////////////////////////////////////////////

static struct Screen *stack[DISPLAY_MAX_STACK] = {&menu};
static uint8_t stackIndex = 0;

//edit variables
static int8_t incrementIndex;
static float increment;
static float valBuf;

static uint8_t int_max(uint8_t a, uint8_t b) {
	if (a > b) {
		return a;
	}
	return b;
}

static void display_update_title(void) {
	Oled_ClearRectangle(0, 0, 128, 16);
	const uint8_t len = strlen(stack[stackIndex]->title);
	const uint16_t hwidth = (7*len)/2;
	Oled_SetCursor(64-hwidth, 0);
	Oled_DrawString(stack[stackIndex]->title, &Font_7x10);
	Oled_DrawHorizontalLine(8, 120, 11);
}

static void display_update_cursor(void) {
	Oled_ClearRectangle(10, 16, 18, 64);
	Oled_SetCursor(10, 16 * (stack[stackIndex]->select - stack[stackIndex]->scroll + 1) + 6);
	Oled_DrawString(">", &Font_7x10);
}

static void display_update_options(void) {
	for (uint8_t i = 0; i < 3; i++) {
		Oled_ClearRectangle(20, 16 * (i + 1), 122, 16 * (i + 1) + 16);
		if (i < stack[stackIndex]->optionCount) {
			Oled_SetCursor(20, 16 * (i + 1) + 6);
			Oled_DrawString(stack[stackIndex]->options[stack[stackIndex]->scroll+i].text, &Font_7x10);
			if (stack[stackIndex]->options[stack[stackIndex]->scroll+i].type == OptionToggle) {
				if (Memory_ReadByte(stack[stackIndex]->options[stack[stackIndex]->scroll+i].toggleLoc)) {
					Oled_DrawString(stack[stackIndex]->options[stack[stackIndex]->scroll+i].on, &Font_7x10);
				}else{
					Oled_DrawString(stack[stackIndex]->options[stack[stackIndex]->scroll+i].off, &Font_7x10);
				}
			}
		}
	}
	if (stack[stackIndex]->optionCount > 3) {
		Oled_ClearRectangle(124, 18, 126, 62);
		Oled_DrawBitmap(122, 16, Bitmap_Scroll, 6, 48);
		//unit = 44/option_count
		//start = 18 + unit*scroll
		//end = 18 + unit*(scroll+3) - 1
		const uint8_t start = 18 + 44*stack[stackIndex]->scroll/stack[stackIndex]->optionCount;
		const uint8_t end = 18 + 44*(stack[stackIndex]->scroll+3)/stack[stackIndex]->optionCount;
		Oled_FillRectangle(124, start, 126, end);
	}else{
		Oled_ClearRectangle(122, 16, 128, 64);
	}
}

static void display_load_variable(void) {
	if (stack[stackIndex]->memType == MemByte) {
		valBuf = Memory_ReadByte(stack[stackIndex]->loc);
	}else if (stack[stackIndex]->memType == MemShort) {
		valBuf = Memory_ReadShort(stack[stackIndex]->loc);
	}else if (stack[stackIndex]->memType == MemFloat) {
		valBuf = Memory_ReadFloat(stack[stackIndex]->loc);
	}
}

static void display_save_variable(void) {
	if (stack[stackIndex]->memType == MemByte) {
		Memory_WriteByte(stack[stackIndex]->loc, valBuf);
	}else if (stack[stackIndex]->memType == MemShort) {
		Memory_WriteShort(stack[stackIndex]->loc, valBuf);
	}else if (stack[stackIndex]->memType == MemFloat) {
		Memory_WriteFloat(stack[stackIndex]->loc, valBuf);
	}
}

static void display_update_increment(void) {
	Oled_ClearRectangle(44, 31, 88, 33);
	Oled_DrawBitmap(72-7*incrementIndex, 31, Bitmap_Increment, 7, 2);
}

static void display_update_variable(void) {
	Oled_ClearRectangle(44, 34, 86, 44);
	Oled_SetCursor(44, 34);
	char buf[DISPLAY_MAX_CHARACTERS];
	if (stack[stackIndex]->memType == MemFloat) {
		sprintf(buf, "  %04.0f", valBuf*10.0);
		Oled_DrawString(buf, &Font_7x10);
		Oled_DrawBitmap(78, 42, Bitmap_Decimal, 3, 3);
	}else if (stack[stackIndex]->memType == MemShort) {
		sprintf(buf, "%05.0f", valBuf);
		Oled_DrawString(buf, &Font_7x10);
	}else{
		sprintf(buf, "  %03.0f", valBuf);
		Oled_DrawString(buf, &Font_7x10);
	}
}

static void display_init_screen(void) {
	Oled_Fill(OledBlack);
	display_update_title();
	if (stack[stackIndex]->type == ScreenScroll) {
		display_update_options();
		display_update_cursor();
	}else if (stack[stackIndex]->type == ScreenEdit) {
		incrementIndex = 0;
		increment = 1;
		display_load_variable();
		display_update_increment();
		display_update_variable();
	}else if (stack[stackIndex]->type == ScreenLive) {
		//DO NOTHING
	}
}

void Display_Init(void) {
//	Oled_DrawBitmap(0, 0, Bitmap_Logo, 128, 16);
//	Oled_DrawBitmap(120, 0, Bitmap_NoSignal, 8, 8);
	display_init_screen();
	Oled_Update();
}

void Display_Update(void) {
	if (stack[stackIndex]->type == ScreenScroll) {
		if (Button_GetPressed(ButtonUp)) {
			if (stack[stackIndex]->select == stack[stackIndex]->scroll && stack[stackIndex]->optionCount > 3) {
				if (!stack[stackIndex]->select) {
					stack[stackIndex]->select = stack[stackIndex]->optionCount - 1;
					stack[stackIndex]->scroll = int_max(2, stack[stackIndex]->select) - 2;
					display_update_cursor();
				} else {
					stack[stackIndex]->select--;
					stack[stackIndex]->scroll--;
				}
				display_update_options();
			} else {
				if (!stack[stackIndex]->select) {
					stack[stackIndex]->select = stack[stackIndex]->optionCount - 1;
				} else {
					stack[stackIndex]->select--;
				}
				display_update_cursor();
			}
			Oled_Update();
		}else if (Button_GetPressed(ButtonDown)) {
			if (stack[stackIndex]->select == stack[stackIndex]->scroll + 2 && stack[stackIndex]->optionCount > 3) {
				if (stack[stackIndex]->select == stack[stackIndex]->optionCount - 1) {
					stack[stackIndex]->select = 0;
					stack[stackIndex]->scroll = 0;
					display_update_cursor();
				} else {
					stack[stackIndex]->select++;
					stack[stackIndex]->scroll++;
				}
				display_update_options();
			} else {
				if (stack[stackIndex]->select == stack[stackIndex]->optionCount-1) {
					stack[stackIndex]->select = 0;
				}else{
					stack[stackIndex]->select++;
				}
				display_update_cursor();
			}
			Oled_Update();
		}else if (Button_GetPressed(ButtonEnter)) {
			if (stack[stackIndex]->options[stack[stackIndex]->select].type == OptionRedirect) {
				stack[stackIndex+1] = stack[stackIndex]->options[stack[stackIndex]->select].redirect;
				stackIndex++;
				display_init_screen();
			}else if (stack[stackIndex]->options[stack[stackIndex]->select].type == OptionToggle) {
				const uint8_t loc = stack[stackIndex]->options[stack[stackIndex]->select].toggleLoc;
				Memory_WriteByte(loc, !Memory_ReadByte(loc));
				display_update_options();
			}else if (stack[stackIndex]->options[stack[stackIndex]->select].type == OptionAction) {
				(*stack[stackIndex]->options[stack[stackIndex]->select].action)();
			}
			Oled_Update();
		}else if (Button_GetPressed(ButtonBack)) {
			stack[stackIndex]->scroll = 0;
			stack[stackIndex]->select = 0;
			if (stackIndex > 0) {
				stackIndex--;
			}
			display_init_screen();
			Oled_Update();
		}
	}else if (stack[stackIndex]->type == ScreenEdit) {
		if (Button_GetPressed(ButtonUp)) {
			valBuf += increment;
			if (stack[stackIndex]->memType == MemByte && valBuf > 255) {
				valBuf = 255;
			}
			if (stack[stackIndex]->memType == MemShort && valBuf > 65535) {
				valBuf = 65535;
			}
			display_update_variable();
			Oled_Update();
		}else if (Button_GetPressed(ButtonDown)) {
			valBuf -= increment;
			if (valBuf < 0) {
				valBuf = 0;
			}
			display_update_variable();
			Oled_Update();
		}else if (Button_GetPressed(ButtonEnter)) {
			incrementIndex++;
			if (stack[stackIndex]->memType == MemFloat) {
				if (incrementIndex > 2) {
					incrementIndex = -1;
				}
			}else if (stack[stackIndex]->memType == MemByte){
				if (incrementIndex > 2) {
					incrementIndex = 0;
				}
			}else{
				if (incrementIndex > 4) {
					incrementIndex = 0;
				}
			}
			increment = 1;
			for (uint8_t i = 0; i < incrementIndex; i++) {
				increment *= 10;
			}
			for (int8_t i = incrementIndex; i < 0; i++) {
				increment /= 10;
			}
			display_update_increment();
			Oled_Update();
		}else if (Button_GetPressed(ButtonBack)) {
			display_save_variable();
			stackIndex--;
			display_init_screen();
			Oled_Update();
		}
	}else if (stack[stackIndex]->type == ScreenLive) {
		(*stack[stackIndex]->update)();
		if (Button_GetPressed(ButtonBack)) {
			stackIndex--;
			display_init_screen();
		}
		Oled_Update();
	}
}

//void Display_UpdateGPS(uint8_t fix) {
//	Oled_ClearRectangle(120, 0, 128, 8);
//	if (fix) {
//		Oled_DrawBitmap(120, 0, Bitmap_Signal, 8, 8);
//	}else{
//		Oled_DrawBitmap(120, 0, Bitmap_NoSignal, 8, 8);
//	}
//	Oled_Update();
//}
