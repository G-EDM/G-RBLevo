#include "ili9341_config.h"

TFT_eSPI tft         = TFT_eSPI();
TFT_eSprite canvas   = TFT_eSprite(&tft);
TFT_eSprite canvas_b = TFT_eSprite(&tft);