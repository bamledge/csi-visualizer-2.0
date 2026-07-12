//  name          : esp32LGFXLib.hpp
//  date/author   : 2025/08/12 Takeshi
//  update/author : 2025/08/12 Takeshi
//

#ifndef __ESP32LGFX_HPP__
#define __ESP32LGFX_HPP__

// For ESP32-C5(Waveshare)
#define VSPI_CLK  (24)
#define VSPI_MISO  (8)
#define VSPI_MOSI  (9)
#define VSPI_CS    (26)
#define VSPI_DC    (23)
#define VSPI_RST   (25)
#define VSPI_CS2   (27)

// For ESP32 WROOM
/*
#define VSPI_CLK  (18)
#define VSPI_MISO  (19)
#define VSPI_MOSI  (23)
#define VSPI_CS    (5)
#define VSPI_DC    (13)
#define VSPI_RST   (14)
#define VSPI_CS2   (4)
*/

#define ROT0   (0)
#define ROT90  (1)
#define ROT180 (2)
#define ROT270 (3)
#define DEPTH_8BIT (8)
#define DEPTH_16BIT (16)

#define MEGA (1000 * 1000)
#include <LovyanGFX.hpp>

class MyLGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ILI9341  _panel_instance; // Panel setting instance
  lgfx::Bus_SPI        _bus_instance;   // SPI Bus setting instance
  lgfx::Touch_XPT2046  _touch_instance; // Touch screen setting instance

public:
  MyLGFX(void)
  {
    { // Bus settings part
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;     // SPI selection (ESP32 : VSPI_HOST or HSPI_HOST)
      // ※ ESP-IDFバージョンアップに伴い、VSPI_HOST , HSPI_HOSTの記述は非推奨になるため、エラーが出る場合は代わりにSPI2_HOST , SPI3_HOSTを使用してください。
      cfg.spi_mode = 0;
      cfg.freq_write = 40 * MEGA;    // Tx SPI clock 40MHz
      cfg.freq_read  = 16 * MEGA;    // Tx SPI clock 16MHz
      //cfg.dma_channel = SPI_DMA_CH_AUTO; // DMA channel to be used (0=disabled / 1=1ch / 2=ch / SPI_DMA_CH_AUTO)
      cfg.dma_channel = 0; // DMA channel to be used (0=disabled / 1=1ch / 2=ch / SPI_DMA_CH_AUTO)
      cfg.pin_sclk = VSPI_CLK;            // ESP32 VSPI default
      cfg.pin_mosi = VSPI_MOSI;            // ESP32 VSPI default
      cfg.pin_miso = VSPI_MISO;            // ESP32 VSPI default
      cfg.pin_dc   = VSPI_DC;            // Assigned to GPIO21
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    { // Panel settings part
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = VSPI_CS;  // ESP32 VSPI default
      cfg.pin_rst          = VSPI_RST;  // Assigned to GPIO22
      cfg.pin_busy         =    -1;  // Disabled
      _panel_instance.config(cfg);
    }

    { // Touch screen settings part
      auto cfg = _touch_instance.config();
      cfg.bus_shared = true; // Shared with panel
      cfg.offset_rotation = 7;
      cfg.spi_host = SPI2_HOST;
      cfg.freq = 1000000;
      cfg.pin_sclk = VSPI_CLK;
      cfg.pin_mosi = VSPI_MOSI;
      cfg.pin_miso = VSPI_MISO;
      cfg.pin_cs   = VSPI_CS2;     // Assigned to GPIO4
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

static MyLGFX lcd;
static LGFX_Sprite spr;
static bool useGraphics = false;
static uint16_t _w, _h, _hw, _hh;

int16_t setupLGFX(int16_t cdep, int16_t rot) {
  if (!lcd.init()) {
    printf("init failed\n");
    return -1;
  }
  lcd.setRotation(rot);
  spr.setColorDepth(cdep);
  spr.createSprite(lcd.width(), lcd.height());
  _w = spr.width(); _h = spr.height();
  _hw = _w >> 1; _hh = _h >> 1;
  useGraphics = true;
  return 0;
}

//==================================
// getfps
//==================================
uint32_t getfps(void) {

  static uint32_t psec = 0;
  static uint32_t cnt = 0;
  static uint32_t fps = 0;
  uint32_t sec = 0;

  //sec = lgfx::v1::millis() / 1000;
  ++cnt;
  if (psec != sec) {
    psec = sec;
    fps = cnt;
    cnt = 0;
  }
  return fps;
}

//==================================
// drawAst
//==================================
void drawAst(LGFX_Sprite *dst, int gx, int gy, int fgcol, int bgcol) {
  static int cnt = 0;
  char ast[5] = "|/-\\";
	dst->setTextColor(fgcol, bgcol);
  dst->setCursor(gx, gy);
  dst->printf("[%c]%3ldfps\n", ast[cnt], getfps());
  cnt = (cnt + 1) % 4;
}

//==================================
// drawFparam
//==================================
void drawFparam(LGFX_Sprite *spr, int gx, int gy, char *fmt, float fdt, int fgcol, int bgcol) {
	spr->setTextColor(fgcol, bgcol);
  spr->setCursor(gx, gy);
  spr->printf(fmt, fdt);
}

//==================================
// drawIparam
//==================================
void drawIparam(LGFX_Sprite *spr, int gx, int gy, char *fmt, uint32_t dt, int fgcol, int bgcol) {
	spr->setTextColor(fgcol, bgcol);
  spr->setCursor(gx, gy);
  spr->printf(fmt, dt);
}

//==================================
// drawSparam
//==================================
void drawSparam(LGFX_Sprite *spr, int gx, int gy, char *fmt, int fgcol, int bgcol) {
	spr->setTextColor(fgcol, bgcol);
  spr->setCursor(gx, gy);
  spr->print(fmt);
}

//==============================
// drawCorner
//==============================
void drawCorner(LGFX_Sprite *dst, int rectX, int rectY, int rectWidth, int rectHeight, int col) {
    int cSize = 16;
    dst->drawLine(rectX, rectY, rectX + cSize, rectY, col);
    dst->drawLine(rectX, rectY, rectX, rectY + cSize, col);
    dst->drawLine(rectX + rectWidth - cSize, rectY, rectX + rectWidth, rectY, col);
    dst->drawLine(rectX + rectWidth, rectY, rectX + rectWidth, rectY + cSize, col);
    dst->drawLine(rectX, rectY + rectHeight - cSize, rectX, rectY + rectHeight, col);
    dst->drawLine(rectX, rectY + rectHeight, rectX + cSize, rectY + rectHeight, col);
    dst->drawLine(rectX + rectWidth - cSize, rectY + rectHeight, rectX + rectWidth, rectY + rectHeight, col);
    dst->drawLine(rectX + rectWidth, rectY + rectHeight - cSize, rectX + rectWidth, rectY + rectHeight, col);
}

#endif
