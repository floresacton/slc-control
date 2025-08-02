#include "display.h"
#include "stdlib.h"
#include "stdio.h"
#include "oled.h"
#include "memory.h"
#include "button.h"
#include "bitmap.h"
#include "font.h"

static uint8_t display_int_max(uint8_t a, uint8_t b) {
    if (a > b) {
        return a;
    }
    return b;
}

static void display_update_cursor(struct Display_Handle* handle) {
    Oled_ClearRectangle(handle->oled, 10, 16, 18, 64);
    Oled_SetCursor(handle->oled, 10, 16 * (handle->current->select - handle->current->scroll + 1) + 6);
    Oled_DrawString(handle->oled, ">", &Font_7x10);
}

static void display_update_options(struct Display_Handle* handle) {
    for (uint8_t i = 0; i < 3; i++) {
        Oled_ClearRectangle(handle->oled, 21, 16 * (i + 1), 122, 16 * (i + 1) + 16);
        if (i < handle->current->optionCount) {
            Oled_SetCursor(handle->oled, 21, 16 * (i + 1) + 6);
            Oled_DrawString(handle->oled, handle->current->options[handle->current->scroll+i].text, &Font_7x10);
            if (handle->current->options[handle->current->scroll+i].var) {
                if (handle->current->options[handle->current->scroll+i].var->value) {
                    Oled_DrawString(handle->oled, "[ON]", &Font_7x10);
                }else{
                    Oled_DrawString(handle->oled, "[OFF]", &Font_7x10);
                }
            }
        }
    }
    if (handle->current->optionCount > 3) {
        Oled_ClearRectangle(handle->oled, 124, 18, 126, 62);
        Oled_DrawBitmap(handle->oled, 122, 16, Bitmap_Scroll, 6, 48);
        //unit = 44/option_count
        //start = 18 + unit*scroll
        //end = 18 + unit*(scroll+3) - 1
        const uint8_t start = 18 + 44*handle->current->scroll/handle->current->optionCount;
        const uint8_t end = 18 + 44*(handle->current->scroll+3)/handle->current->optionCount;
        Oled_FillRectangle(handle->oled, 124, start, 126, end);
    }else{
        Oled_ClearRectangle(handle->oled ,122, 16, 128, 64);
    }
}

static void display_update_edit(struct Display_Handle* handle) {
    Oled_ClearRectangle(handle->oled, 44, 31, 88, 33);
    Oled_DrawBitmap(handle->oled, 72 - 7 * handle->editDigit, 31, Bitmap_Edit, 7, 2);
}

static void display_update_variable(struct Display_Handle* handle) {
    Oled_ClearRectangle(handle->oled, 44, 34, 86, 44);

    const uint8_t len = Memory_Print(handle->charBuf, handle->current->var);
    
    Oled_SetCursor(handle->oled, 79 - 7*len, 34);
    Oled_DrawString(handle->oled, handle->charBuf, &Font_7x10);

}

static void display_init_screen(struct Display_Handle* handle) {
    handle->current = handle->stack[handle->stackIndex];
    Oled_Fill(handle->oled, Oled_ColorBlack);
    if (handle->current->optionCount) {
        Oled_DrawBitmap(handle->oled, 14, 0, Bitmap_Logo, 89, 16);

        display_update_options(handle);
        display_update_cursor(handle);
    } else if (handle->current->var) {
        handle->editDigit = 0;
        handle->editValue = 1;
        //draw decimal if applicaple
        display_update_edit(handle);
        display_update_variable(handle);
    } else if (handle->current->update) {

    }
}

void Display_Init(struct Display_Handle* handle) {
    handle->stack = malloc(handle->depth * sizeof(struct Display_Screen *));
    handle->stack[0] = handle->top;
    handle->charBuf = malloc(handle->chars);

    display_init_screen(handle);
    Oled_Update(handle->oled);
}

// button order is up, down, enter, back
void Display_Update(struct Display_Handle* handle) {
    if (handle->current->optionCount) {
        if (handle->buttons[0]->pressed) {
            if (handle->current->select == handle->current->scroll && handle->current->optionCount > 3) {
                if (!handle->current->select) {
                    handle->current->select = handle->current->optionCount - 1;
                    handle->current->scroll = display_int_max(2, handle->current->select) - 2;
                    display_update_cursor(handle);
                } else {
                    handle->current->select--;
                    handle->current->scroll--;
                }
                display_update_options(handle);
            } else {
                if (!handle->current->select) {
                    handle->current->select = handle->current->optionCount - 1;
                } else {
                    handle->current->select--;
                }
                display_update_cursor(handle);
            }
        }else if (handle->buttons[1]->pressed) {
            if (handle->current->select == handle->current->scroll + 2 && handle->current->optionCount > 3) {
                if (handle->current->select == handle->current->optionCount - 1) {
                    handle->current->select = 0;
                    handle->current->scroll = 0;
                    display_update_cursor(handle);
                } else {
                    handle->current->select++;
                    handle->current->scroll++;
                }
                display_update_options(handle);
            } else {
                if (handle->current->select == handle->current->optionCount-1) {
                    handle->current->select = 0;
                }else{
                    handle->current->select++;
                }
                display_update_cursor(handle);
            }
        }else if (handle->buttons[2]->pressed) {
            if (handle->current->options[handle->current->select].redirect) {
                handle->stackIndex++;
                handle->stack[handle->stackIndex] = handle->current->options[handle->current->select].redirect;
                display_init_screen(handle);
            }else if (handle->current->options[handle->current->select].var) {
                struct Memory_Variable* var = handle->current->options[handle->current->select].var;
                var->value = !var->value;
                Memory_Save(handle->memory, var);
                display_update_options(handle);
            }else if (handle->current->options[handle->current->select].action) {
                (*handle->current->options[handle->current->select].action)();
            }
        }else if (handle->buttons[3]->pressed) {
            handle->current->scroll = 0;
            handle->current->select = 0;
            handle->stackIndex--;
            display_init_screen(handle);
        }else{
            return;
        }
        Oled_Update(handle->oled);
    } else if (handle->current->var) {
        if (handle->buttons[0]->pressed) {
            handle->current->var->value += handle->editValue;
            if (handle->current->var->value > handle->current->var->max) {
                handle->current->var->value = handle->current->var->max;
            }
            display_update_variable(handle);
        }else if (handle->buttons[1]->pressed) {
            handle->current->var->value -= handle->editValue;
            if (handle->current->var->value < handle->current->var->min) {
                handle->current->var->value = handle->current->var->min;
            }
            display_update_variable(handle);
        }else if (handle->buttons[2]->pressed) {
            handle->editDigit++;
            if (handle->editDigit > handle->current->var->maxDigit) {
                handle->editDigit = -handle->current->var->decimals;
            }
            handle->editValue = 1.0f;
            for (uint8_t i = 0; i < handle->editDigit; i++) {
                handle->editValue *= 10.0;
            }
            for (int8_t i = handle->editDigit; i < 0; i++) {
                handle->editValue /= 10.0;
            }
            display_update_edit(handle);
        }else if (handle->buttons[3]->pressed) {
            Memory_Save(handle->memory, handle->current->var);
            (*handle->values_update)();
            handle->stackIndex--;
            display_init_screen(handle);
        }else{
            return;
        }
        Oled_Update(handle->oled);
    } else if (handle->current->update) {
        if (handle->buttons[2]->pressed && handle->current->redirect) {
            handle->stackIndex++;
            handle->stack[handle->stackIndex] = handle->current->redirect;
            display_init_screen(handle);
        } else if (handle->buttons[3]->pressed) {
            if (handle->stackIndex) {
                handle->stackIndex--;
            }
            display_init_screen(handle);
        } else if (!(*handle->current->update)()) {
            return;
        }
        Oled_Update(handle->oled);
    }
    for (uint8_t i = 0; i < 4; i++) {
        handle->buttons[i]->pressed = 0;
    }
}
