#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "stdint.h"
#include "memory.h"

#define DISPLAY_MAX_STACK 4
#define DISPLAY_MAX_CHARACTERS 22

enum ScreenType {
	ScreenEdit,
	ScreenLive,
	ScreenScroll,
};

enum OptionType {
	OptionRedirect,
	OptionToggle,
	OptionAction,
};

struct Screen;

struct Option {
	char* text;
	enum OptionType type;
	//redirect
	struct Screen* redirect;
	//toggle
	uint8_t toggleLoc;
	char* on;
	char* off;
	//action
	void (*action)(void);
};

struct Screen {
	char* title;
	//
	enum ScreenType type;
	//scroll
	uint8_t optionCount;
	struct Option* options;
	uint8_t select;
	uint8_t scroll;
	//edit
	enum MemoryType memType;
	uint8_t loc;
	//live
	void (*update)(void);
};

void Display_Init(void);
void Display_Update(void);

#endif
