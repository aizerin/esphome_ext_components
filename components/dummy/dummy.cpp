#include "dummy.h"
#include "esphome/core/log.h"

namespace esphome
{
    namespace dummy
    {

        static const char *TAG = "dummy";

        void Dummy::setup()
        {
            ESP_LOGCONFIG(TAG, "SETUP KUA");
            ESP_LOGD(TAG, "SETUP");
        }

        // void Dummy::update()
        // {
        //     ESP_LOGD(TAG, "UPDATE");
        // }

        void Dummy::loop()
        {
            // ESP_LOGD(TAG, "loop");
        }

    }
}