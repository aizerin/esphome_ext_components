
#include "spi_test.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include <GxEPD2_3C.h>

namespace esphome
{
    namespace spi_test
    {

        static const char *TAG = "spi_test";

        // const uint32_t EPD_SCK_PIN = 13;
        // const uint32_t EPD_MOSI_PIN = 14;
        // const uint32_t EPD_CS_PIN = 15;
        // const uint32_t EPD_RST_PIN = 26;
        // const uint32_t EPD_DC_PIN = 27;
        // const uint32_t EPD_BUSY_PIN = 25;

        GxEPD2_3C<GxEPD2_750c_Z08,
                  GxEPD2_750c_Z08::HEIGHT / 2>
            display2(
                GxEPD2_750c_Z08(15,
                                27,
                                26,
                                25));

        void SpiTest::setup()
        {
            ESP_LOGD(TAG, "Setting up");
            this->spi_setup();
            display2.epd2.selectSPI(this->parent_->interface_, SPISettings(4000000, MSBFIRST, SPI_MODE0));
            display2.init(115200);
        }

        void SpiTest::dump_config()
        {
            ESP_LOGCONFIG(TAG, "SPIDevice");
            LOG_PIN("  CS pin: ", this->cs_);
            ESP_LOGCONFIG(TAG, "  Mode: %d", this->mode_);
            if (this->data_rate_ < 1000000)
            {
                ESP_LOGCONFIG(TAG, "  Data rate: %" PRId32 "kHz", this->data_rate_ / 1000);
            }
            else
            {
                ESP_LOGCONFIG(TAG, "  Data rate: %" PRId32 "MHz", this->data_rate_ / 1000000);
            }
        }

        void SpiTest::update()
        {
            if (network::is_connected())
            {
                ESP_LOGD(TAG, "NETWORK CONNECTED");
                display2.clearScreen();
            }
            else
            {
                ESP_LOGD(TAG, "NETWORK NOT CONNECTED");
            }
        }

    }
}