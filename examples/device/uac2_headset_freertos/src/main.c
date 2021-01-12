/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Jerzy Kasenberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"

#include "driver/gpio.h"
#include "driver/i2s.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "i2s_bus.h"
#include "es8311.h"

#define SAMPLE_RATE     (44100)
#define I2S_NUM         (0)

static const char *TAG = "USB_AUDIO";

#define CFG_USB_AUDIO_TASK_CREAT_STATIC 0

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+

/* Blink pattern
 * - 25 ms   : streaming data
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
  BLINK_STREAMING = 25,
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

enum
{
  VOLUME_CTRL_0_DB = 0,
  VOLUME_CTRL_10_DB = 2560,
  VOLUME_CTRL_20_DB = 5120,
  VOLUME_CTRL_30_DB = 7680,
  VOLUME_CTRL_40_DB = 10240,
  VOLUME_CTRL_50_DB = 12800,
  VOLUME_CTRL_60_DB = 15360,
  VOLUME_CTRL_70_DB = 17920,
  VOLUME_CTRL_80_DB = 20480,
  VOLUME_CTRL_90_DB = 23040,
  VOLUME_CTRL_100_DB = 25600,
  VOLUME_CTRL_SILENCE = 0x8000,
};

// queue
QueueHandle_t spk_data_size;

// static timer
StaticTimer_t blinky_tmdef;
TimerHandle_t blinky_tm;

// Audio controls
// Current states
int8_t mute[CFG_TUD_AUDIO_N_CHANNELS_TX + 1];       // +1 for master channel 0
int16_t volume[CFG_TUD_AUDIO_N_CHANNELS_TX + 1];    // +1 for master channel 0

// Buffer for microphone data
int16_t mic_buf[1000];
// Buffer for speaker data
int16_t spk_buf[1000];
// // Speaker data size received in the last frame
// int spk_data_size;

// static task for usbd
// Increase stack size when debug log is enabled
#if CFG_TUSB_DEBUG
  #define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE)
#else
  #define USBD_STACK_SIZE     (3*configMINIMAL_STACK_SIZE/2)
#endif

#if CFG_USB_AUDIO_TASK_CREAT_STATIC
StackType_t  usb_device_stack[USBD_STACK_SIZE];
StaticTask_t usb_device_taskdef;

StackType_t  usb_audio_stack[USBD_STACK_SIZE];
StaticTask_t usb_audio_taskdef;
#else 
#define USB_DEVICE_TASK_STACK_SIZE (20*configMINIMAL_STACK_SIZE)
#define USB_AUDIO_TASK_STACK_SIZE (20*configMINIMAL_STACK_SIZE)
#endif

void led_blinky_cb(TimerHandle_t xTimer);
void audio_task(void);

void usb_device_task(void* param);

/*------------- MAIN -------------*/
int main(void)
{
  board_init();

  // soft timer for blinky
  blinky_tm = xTimerCreateStatic(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb, &blinky_tmdef);
  xTimerStart(blinky_tm, 0);

  // creat queue
  spk_data_size = xQueueCreate(2, sizeof(int32_t));

  // i2c
  ESP_ERROR_CHECK(i2c_bus_init());

  // es8311
  ESP_ERROR_CHECK(es8311_init(SAMPLE_RATE));
  ESP_ERROR_CHECK(es8311_set_voice_volume(100));

  // i2s
  ESP_ERROR_CHECK(i2s_bus_init());
  

  TU_LOG1("Headset running\r\n");

#if CFG_USB_AUDIO_TASK_CREAT_STATIC
  // Create a task for tinyusb device stack
  (void) xTaskCreateStatic( usb_device_task, "usbd", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_device_stack, &usb_device_taskdef);


  // Create a task for tinyusb device stack
  (void) xTaskCreateStatic( audio_task, "audio", USBD_STACK_SIZE, NULL, configMAX_PRIORITIES-1, usb_audio_stack, &usb_audio_taskdef);
#else
  // Create a task for tinyusb device stack
  xTaskCreate(usb_device_task, "usbd", USB_DEVICE_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);


  // Create a task for tinyusb device stack
  xTaskCreate(audio_task, "audio", USB_AUDIO_TASK_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
#endif

  return 0;
}

#if 0
static void audio_test_task(void *arg)
{
    while(1) {
        size_t bytes_written = 0;
        ESP_LOGI(TAG, "Start to play music");
        // i2s_write(I2S_NUM, mp3_1_pcm_start, mp3_1_pcm_end - mp3_1_pcm_start, &bytes_written, portMAX_DELAY);
        vTaskDelay(500);
    }
    ESP_LOGW(TAG, "Reach the end of music, release all resource");
    i2s_stop(I2S_NUM);
    i2s_driver_uninstall(I2S_NUM);
}
#endif


#if CFG_TUSB_MCU == OPT_MCU_ESP32S2
void app_main(void)
{
  main();
}
#endif

// USB Device Driver task
// This top level thread process all usb events and invoke callbacks
void usb_device_task(void* param)
{
  (void) param;

  // This should be called after scheduler/kernel is started.
  // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS queue API.
  tusb_init();

  // RTOS forever loop
  while (1)
  {
    // tinyusb device task
    tud_task();
  }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), 0);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_SUSPENDED), 0);
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);
}

typedef struct TU_ATTR_PACKED
{
  union
  {
    struct TU_ATTR_PACKED
    {
      uint8_t recipient :  5; ///< Recipient type tusb_request_recipient_t.
      uint8_t type      :  2; ///< Request type tusb_request_type_t.
      uint8_t direction :  1; ///< Direction type. tusb_dir_t
    } bmRequestType_bit;

    uint8_t bmRequestType;
  };

  uint8_t bRequest;
  uint8_t bChannelNumber;
  uint8_t bControlSelector;
  union
  {
    uint8_t bInterface;
    uint8_t bEndpoint;
  };
  uint8_t bEntityID;
  uint16_t wLength;
} audio_control_request_t;

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

  // Example supports only single frequency, same value will be used for current value and range
  if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
  {
    if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      TU_LOG2("Clock get current freq %u\r\n", AUDIO_SAMPLE_RATE);

      audio_control_cur_4_t curf = { tu_htole32(AUDIO_SAMPLE_RATE) };
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
    }
    else if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_4_n_t(1) rangef =
      {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { tu_htole32(AUDIO_SAMPLE_RATE), tu_htole32(AUDIO_SAMPLE_RATE), 0}
      };
      TU_LOG2("Clock get freq range (%d, %d, %d)\r\n", (int)rangef.subrange[0].bMin, (int)rangef.subrange[0].bMax, (int)rangef.subrange[0].bRes);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
    }
  }
  else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
           request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t cur_valid = { .bCur = 1 };
    TU_LOG2("Clock get is valid %u\r\n", cur_valid.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
  }
  TU_LOG1("Clock get request not supported, entity = %u, selector = %u, request = %u\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);
  return false;
}

// Helper for feature unit get requests
static bool tud_audio_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && request->bRequest == AUDIO_CS_REQ_CUR)
  {
    audio_control_cur_1_t mute1 = { .bCur = mute[request->bChannelNumber] };
    TU_LOG2("Get channel %u mute %d\r\n", request->bChannelNumber, mute1.bCur);
    return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
  }
  else if (UAC2_ENTITY_SPK_FEATURE_UNIT && request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    if (request->bRequest == AUDIO_CS_REQ_RANGE)
    {
      audio_control_range_2_n_t(1) range_vol = {
        .wNumSubRanges = tu_htole16(1),
        .subrange[0] = { .bMin = tu_htole16(-VOLUME_CTRL_50_DB), tu_htole16(VOLUME_CTRL_0_DB), tu_htole16(256) }
      };
      TU_LOG2("Get channel %u volume range (%d, %d, %u) dB\r\n", request->bChannelNumber,
              range_vol.subrange[0].bMin / 256, range_vol.subrange[0].bMax / 256, range_vol.subrange[0].bRes / 256);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
    }
    else if (request->bRequest == AUDIO_CS_REQ_CUR)
    {
      audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(volume[request->bChannelNumber]) };
      TU_LOG2("Get channel %u volume %u dB\r\n", request->bChannelNumber, cur_vol.bCur);
      return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
    }
  }
  TU_LOG1("Feature unit get request not supported, entity = %u, selector = %u, request = %u\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

// Helper for feature unit set requests
static bool tud_audio_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
  (void)rhport;

  TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
  TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

  if (request->bControlSelector == AUDIO_FU_CTRL_MUTE)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));

    mute[request->bChannelNumber] = ((audio_control_cur_1_t *)buf)->bCur;

    TU_LOG2("Set channel %d Mute: %d\r\n", request->bChannelNumber, mute[request->bChannelNumber]);

    return true;
  }
  else if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME)
  {
    TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));

    volume[request->bChannelNumber] = ((audio_control_cur_2_t const *)buf)->bCur;

    TU_LOG2("Set channel %d volume: %d dB\r\n", request->bChannelNumber, volume[request->bChannelNumber] / 256);

    return true;
  }
  else
  {
    TU_LOG1("Feature unit set request not supported, entity = %u, selector = %u, request = %u\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
  }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
  audio_control_request_t *request = (audio_control_request_t *)p_request;
  uint8_t *a = (uint8_t *)request;
  if (request->bEntityID == UAC2_ENTITY_CLOCK)
    return tud_audio_clock_get_request(rhport, request);
  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_get_request(rhport, request);
  else
  {
    TU_LOG1("Get request not handled, entity =%d, selector = %d, request = %d\r\n",
            request->bEntityID, request->bControlSelector, request->bRequest);
  }
  return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;

  if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT)
    return tud_audio_feature_unit_set_request(rhport, request, buf);

  TU_LOG1("Set request not handled, entity = %d, selector = %d, request = %d\r\n",
          request->bEntityID, request->bControlSelector, request->bRequest);

  return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;

  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0)
      xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_MOUNTED), 0);

  return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  TU_LOG2("Set interface %d alt %d\r\n", itf, alt);
  if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0)
      xTimerChangePeriod(blinky_tm, pdMS_TO_TICKS(BLINK_STREAMING), 0);

  return true;
}

bool tud_audio_rx_done_cb(uint8_t rhport, uint8_t *buffer, uint16_t buf_size)
{
  (void)rhport;

  int data_size = buf_size;
  BaseType_t xStatus = xQueueSend(spk_data_size, &data_size, portMAX_DELAY);
  if(xStatus != pdPASS)
  {
    printf( "Could not send to the queue.\r\n" );
  }
  memcpy(spk_buf, buffer, buf_size);

  // size_t bytes_written = 0;
  // ESP_LOGI(TAG, "Start to play music");
  // i2s_write(I2S_NUM, buffer, data_size, &bytes_written, portMAX_DELAY);

  // for(int i = 0; i < data_size; i++) {
  //   printf("%x ", buffer[i]);
  //   if(!((i + 1) % 8)) {
  //     printf(" ");
  //   }
  //   if(!((i + 1) % 16)) {
  //     printf("     data_size:%d\r\n", data_size);
  //   }
  // }
  // printf("\r\n\r\n\r\n end \r\n\r\n\r\n");

  for(int i = 0; i < data_size; i++) {
    uint8_t temp1 = spk_buf[i] >> 8;
    uint8_t temp2 = spk_buf[i];
    printf("%x %x ", temp1, temp2);
    if(!((i + 1) % 4)) {
      printf(" ");
    }
    if(!((i + 1) % 8)) {
      printf("     data_size:%d\r\n", data_size*2);
    }
  }
  printf("\r\n\r\n\r\n end \r\n\r\n\r\n");

  return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
  (void)rhport;
  (void)itf;
  (void)ep_in;
  (void)cur_alt_setting;

  // This callback could be used to fill microphone data separately
  return true;
}

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void audio_task(void)
{
  while(1) {
    size_t bytes_written = 0;
    // When new data arrived, copy data from speaker buffer, to microphone buffer
    // and send it over
    int data_size;
    BaseType_t xStatus = xQueueReceive(spk_data_size, &data_size, portMAX_DELAY);
		if(xStatus == pdFAIL){
			printf( "Could not receive from the queue.\r\n" );
		}

    if (data_size)
    {
      int16_t *src = spk_buf;
      int16_t *limit = spk_buf + data_size / 2;
      int16_t *dst = mic_buf;
      while (src < limit)
      {
        // Combine two channels into one
        int32_t left = *src++;
        int32_t right = *src++;
        *dst++ = (int16_t)((left + right) / 2);
      }
      // tud_audio_write((uint8_t *)mic_buf, data_size / 2);
      // i2s_write(I2S_NUM, dst, data_size / 2, &bytes_written, portMAX_DELAY);

      // for(int i = 0; i < data_size; i++) {
      //   uint8_t temp1 = src[i] >> 8;
      //   uint8_t temp2 = src[i];
      //   printf("%x %x ", temp1, temp2);
      //   if(!((i + 1) % 4)) {
      //     printf(" ");
      //   }
      //   if(!((i + 1) % 8)) {
      //     printf("\r\n");
      //   }
      // }
      // printf("\r\n\r\nbuffer_data_size:%d", data_size);
      // printf("\r\n\r\n end \r\n\r\n\r\n");
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinky_cb(TimerHandle_t xTimer)
{
  (void) xTimer;
  static bool led_state = false;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
