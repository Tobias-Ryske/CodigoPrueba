#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

#define BL_ACTIVE_HIGH 1

// Pines oficiales Waveshare ESP32-C6-LCD-1.47 (ST7789)
#define PIN_LCD_MOSI   6
#define PIN_LCD_SCLK   7
#define PIN_LCD_CS    14
#define PIN_LCD_DC    15
#define PIN_LCD_RST   21
#define PIN_LCD_BL    22

#define LCD_HRES 172
#define LCD_VRES 320

static const char *TAG = "LCD";

// ----------------- Fuente 5x7 (solo lo necesario) -----------------
typedef struct { char c; uint8_t col[5]; } glyph_t;
static const glyph_t FONT[] = {
  {'H', {0x7F,0x08,0x08,0x08,0x7F}},
  {'M', {0x7F,0x06,0x18,0x06,0x7F}},
  {'N', {0x7F,0x04,0x08,0x10,0x7F}},
  {'D', {0x7F,0x41,0x41,0x22,0x1C}},
  {'O', {0x3E,0x41,0x41,0x41,0x3E}},
  {'a', {0x20,0x54,0x54,0x54,0x78}},
  {'d', {0x1C,0x22,0x41,0x41,0x7F}},
  {'l', {0x00,0x41,0x7F,0x40,0x00}},
  {'n', {0x7E,0x08,0x04,0x04,0x78}},
  {'o', {0x38,0x44,0x44,0x44,0x38}},
  {'u', {0x3C,0x40,0x40,0x20,0x7C}},
  {' ', {0x00,0x00,0x00,0x00,0x00}},
};

static inline void bl_set(int on){
#if BL_ACTIVE_HIGH
  gpio_set_level(PIN_LCD_BL, on ? 1 : 0);
#else
  gpio_set_level(PIN_LCD_BL, on ? 0 : 1);
#endif
}

static const uint8_t* get_glyph(char c){
  for (size_t i=0;i<sizeof(FONT)/sizeof(FONT[0]);++i)
    if (FONT[i].c==c) return FONT[i].col;
  return FONT[sizeof(FONT)/sizeof(FONT[0])-1].col; // espacio
}

static void draw_char_scaled(uint16_t *fb, int x, int y, char c, uint16_t color, int scale){
  const uint8_t* g = get_glyph(c);
  for(int col=0; col<5; col++){
    uint8_t bits = g[col];
    for(int row=0; row<7; row++){
      if (bits & (1 << (7-1-row))) { // MSB = fila superior
        for(int dy=0; dy<scale; ++dy)
          for(int dx=0; dx<scale; ++dx){
            int px = x + col*scale + dx;
            int py = y + row*scale + dy;
            if (px>=0 && px<LCD_HRES && py>=0 && py<LCD_VRES)
              fb[py*LCD_HRES + px] = color;
          }
      }
    }
  }
}

static void draw_text_center(uint16_t *fb, const char *txt, uint16_t color, int scale){
  int len=0; while (txt[len]) len++;
  int cw = 5*scale, cs = 1*scale;   // ancho char + espacio
  int w = len*cw + (len-1)*cs;
  int h = 7*scale;
  int x = (LCD_HRES - w)/2;
  int y = (LCD_VRES - h)/2;
  for (int i=0;i<len;i++){
    draw_char_scaled(fb, x, y, txt[i], color, scale);
    x += cw + cs;
  }
}

void app_main(void)
{
  // Backlight
  gpio_config_t blcfg = {.mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << PIN_LCD_BL};
  gpio_config(&blcfg);
  bl_set(1);

  // SPI
  spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_LCD_MOSI,
    .miso_io_num = -1,
    .sclk_io_num = PIN_LCD_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = LCD_HRES * LCD_VRES * 2 + 8,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // IO LCD
  esp_lcd_panel_io_handle_t io = NULL;
  esp_lcd_panel_io_spi_config_t io_cfg = {
    .dc_gpio_num = PIN_LCD_DC,
    .cs_gpio_num = PIN_LCD_CS,
    .pclk_hz = 10 * 1000 * 1000,   // luego podés subir a 20-40 MHz
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
    .spi_mode = 0,
    .trans_queue_depth = 10,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_cfg, &io));

  // Panel ST7789
  esp_lcd_panel_handle_t panel = NULL;
  esp_lcd_panel_dev_config_t panel_cfg = {
    .reset_gpio_num = PIN_LCD_RST,
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .bits_per_pixel = 16,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &panel));

  // Init + ajustes para 1.47" (172x320)
  esp_lcd_panel_reset(panel);
  vTaskDelay(pdMS_TO_TICKS(50));
  esp_lcd_panel_init(panel);
  esp_lcd_panel_invert_color(panel, true);      // corrige colores
  esp_lcd_panel_set_gap(panel, 34, 0);          // corrige franja lateral
  esp_lcd_panel_swap_xy(panel, false);
  esp_lcd_panel_mirror(panel, false, true);     // invierte eje Y
  esp_lcd_panel_disp_on_off(panel, true);

  // Framebuffer
  size_t npix = LCD_HRES * LCD_VRES;
  uint16_t *fb = (uint16_t *)heap_caps_malloc(npix * 2, MALLOC_CAP_DMA);
  if (!fb) { ESP_LOGE(TAG,"Sin RAM framebuffer"); while(1) vTaskDelay(1000/portTICK_PERIOD_MS); }

  // Fondo negro
  for (size_t i=0;i<npix;i++) fb[i]=0x0000;

  // Texto centrado
  draw_text_center(fb, "Hola Mundo", 0xFFFF, 2);

  // Enviar al panel
  esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_HRES, LCD_VRES, fb);
  ESP_LOGI(TAG, "Mostrado 'Hola Mundo' centrado sobre negro.");
  while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
