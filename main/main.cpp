#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp32LGFXLib.hpp"

extern "C" void app_main(void)
{
  lcd.init();
  setupLGFX(16, ROT0);
  int16_t rssi = 0;
  float htltf_amp = -1.0, stbchtltf_amp = -1.0;
  uint32_t sz = _w * _h;

  printf("w=%ld h=%ld\n",
       lcd.width(),
       lcd.height());

  spr.setColorDepth(DEPTH_8BIT);
  if (!spr.createSprite(_w, _h)) {
    printf("ERROR: malloc error (tmpspr:%ldByte)\n", sz);
    while(1) {
      sleep(1);
    };
  }
  printf("init done\n");
  

  while(1) {
    spr.fillSprite(TFT_BLACK);
    spr.drawLine(0, _hh, _w, _hh, TFT_DARKGRAY);spr.drawLine(_hw, 0, _hw, _h, TFT_DARKGRAY);
    spr.setCursor(0,0);
    spr.setTextColor(TFT_WHITE);spr.setTextSize(1);spr.printf("RSSI=%d\n", rssi);
    spr.printf("AMP_HT-LTF = %2.2f\n", htltf_amp);
    spr.printf("AMP_HT-LTF2= %2.2f\n\n", stbchtltf_amp);
    printf("spr prepared\n");
    lcd.printf("spr prepared\n");
    spr.pushSprite(&lcd, 0, 0);
    printf("spr pushed\n");

    vTaskDelay(100);
  }
}
