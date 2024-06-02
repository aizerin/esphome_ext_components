#pragma once

#include "esphome/core/component.h"

namespace esphome
{
    namespace dummy
    {
        class Dummy : public Component
        {
        public:
            void setup() override;
            // void update() override;
            void loop() override;
        };
    }
}