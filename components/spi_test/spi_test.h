#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"

namespace esphome
{
    namespace spi_test
    {
        class SpiTest : public PollingComponent,
                        public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_HIGH,
                                              spi::CLOCK_PHASE_TRAILING, spi::DATA_RATE_4MHZ>
        {

        public:
            using SPIDevice::SPIDevice;

            SPIClass *get_spi_interface() const { return this->parent_->interface_; }

            void setup() override;
            void update() override;
            void dump_config() override;
        };
    }
}