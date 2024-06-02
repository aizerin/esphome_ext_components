#pragma once

#include "esphome/core/component.h"
#include <WiFi.h>
#include <SPI.h>
#include <GxEPD2_3C.h>

namespace esphome
{
    namespace eink_bmp
    {
        class EinkBmp : public PollingComponent
        {
        public:
            void setup() override;
            void update() override;

        protected:
            const uint32_t EPD_SCK_PIN = 13;
            const uint32_t EPD_MOSI_PIN = 14;
            const uint32_t EPD_CS_PIN = 15;
            const uint32_t EPD_RST_PIN = 26;
            const uint32_t EPD_DC_PIN = 27;
            const uint32_t EPD_BUSY_PIN = 25;
            static const uint16_t input_buffer_pixels = 800; // may affect performance
            static const uint16_t max_row_width = 1872;      // for up to 7.8" display 1872x1404
            static const uint16_t max_palette_pixels = 256;  // for depth <= 8

            uint8_t input_buffer[3 * input_buffer_pixels];        // up to depth 24
            uint8_t output_row_mono_buffer[max_row_width / 8];    // buffer for at least one row of b/w bits
            uint8_t output_row_color_buffer[max_row_width / 8];   // buffer for at least one row of color bits
            uint8_t mono_palette_buffer[max_palette_pixels / 8];  // palette buffer for depth <= 8 b/w
            uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w

            GxEPD2_3C<GxEPD2_750c_Z08, GxEPD2_750c_Z08::HEIGHT / 2> *display;

            uint16_t read16(WiFiClient &client);
            uint32_t read32(WiFiClient &client);
            uint32_t read8n(WiFiClient &client, uint8_t *buffer, int32_t bytes);
            uint32_t skip(WiFiClient &client, int32_t bytes);
            void showBitmapFrom_HTTP(const char *host, int16_t port, const char *path, const char *filename, int16_t x, int16_t y, bool with_color);
        };
    }
}