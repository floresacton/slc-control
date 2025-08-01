#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include "memory.h"
#include "main.h"

struct Display_Screen;

struct Display_Option {
    // configuration
    char* text;
    // redirect
    struct Display_Screen* redirect;
    // toggle
    struct Memory_Variable* var;
    // action
    void (*action)(void);
};

struct Display_Screen {
    // configuration
    // scroll
    uint8_t optionCount;
    struct Display_Option* options;
    // edit
    struct Memory_Variable* var;
    // live
    uint8_t (*update)(void);
    struct Display_Screen* redirect;

    // internal
    uint8_t select;
    uint8_t scroll;
};

struct Display_Handle {
    // configuration
    struct Oled_Handle* oled;
    struct Button_Handle** buttons;
    struct Memory_Handle* memory;

    struct Display_Screen* top;
    uint8_t depth;

    uint8_t chars;

    void (*values_update)(void);

    // internal
    uint8_t stackIndex;
    struct Display_Screen **stack;
    struct Display_Screen *current;

    char* charBuf;

    int8_t editDigit;
    float editValue;
};

void Display_Init(struct Display_Handle* handle);
void Display_Update(struct Display_Handle* handle);

#endif
