#include <assert.h>
#include <string.h>

#include "fonts.h"
#include "../schichor_program/Core/Src/fonts.c"

int main(void) {
    char *str = "Hello";
    FontDef_t Font = { .FontWidth = 5, .FontHeight = 7, .data = NULL };
    FONTS_SIZE_t Size;
    char *ret = FONTS_GetStringSize(str, &Size, &Font);
    assert(ret == str);
    assert(Size.Length == strlen(str) * Font.FontWidth);
    assert(Size.Height == Font.FontHeight);
    return 0;
}
