// Copyright 2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include "i2s_bus.h"

#define I2S_SCLK        (GPIO_NUM_18)
#define I2S_LCLK        (GPIO_NUM_17)
#define I2S_DOUT        (GPIO_NUM_12)
#define I2S_DSIN        (GPIO_NUM_46)

esp_err_t i2s_bus_init(void)
{
    int res = 0;
    /*!<  for 36Khz sample rates, we create 100Hz sine wave, every cycle need 36000/100 = 360 samples (4-bytes or 8-bytes each sample) */
    /*!<  depend on bits_per_sample */
    /*!<  using 6 buffers, we need 60-samples per buffer */
    /*!<  if 2-channels, 16-bit each channel, total buffer is 360*4 = 1440 bytes */
    /*!<  if 2-channels, 24/32-bit each channel, total buffer is 360*8 = 2880 bytes */
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,                                  /*!<  Only TX */
        .sample_rate = 16000,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                           /*!< 1-channels */
        .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,    //与该选项无关
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .use_apll = true,
        .tx_desc_auto_clear = true,     /*!< I2S auto clear tx descriptor if there is underflow condition (helps in avoiding noise in case of data unavailability) */
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCLK,
        .ws_io_num = I2S_LCLK,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_DSIN                                          /*!< Not used */
    };

    res |= i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    res |= i2s_set_pin(I2S_NUM, &pin_config);
    return res ? ESP_FAIL: ESP_OK;
}
