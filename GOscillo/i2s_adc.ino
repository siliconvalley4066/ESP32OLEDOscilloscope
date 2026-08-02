#ifndef ESP32_C3
// Sample from the ADC continuously at a particular sample rate
// Copyright (c) 2022, Siliconvalley4066

#include "driver/i2s.h"

#define I2S_NUM                     I2S_NUM_0
#define ADC_UNIT                    ADC_UNIT_1          // ADC1 or ADC2
#define I2S_BUFFER_COUNT            4
#define I2S_BUFFER_SIZE             256

void sample_dma() {
  byte ch;
  uint16_t *p;
  size_t bytes_read;

  if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    ch = ad_ch1;
    p = cap_buf1;
  } else {
    ch = ad_ch0;
    p = cap_buf;
  }
  i2s_set_adc_mode(ADC_UNIT, (adc1_channel_t) ch);

// for I2S_CHANNEL_FMT_ALL_LEFT
//  i2s_read(I2S_NUM, p, NSAMP * 2, &bytes_read, 20);
//  for (int i=0; i < NSAMP/2; i++) {
//    p[i] = p[i+i] & 0xfff;  // pick up LEFT data and mask MSBs
//  }

// for I2S_CHANNEL_FMT_ONLY_LEFT
  i2s_read(I2S_NUM, p, NSAMP, &bytes_read, 20);
// Swap word order to fix ESP32 bug in packing 16bits into 32bits
  int16_t tmp;
  for (int i=0; i < NSAMP/2; i++) {
    if (i&1) {
      p[i] = tmp;
    } else {
      tmp = p[i] & 0xfff;
      p[i] = p[i+1] & 0xfff;
    }
  }
  delay(1);
  scaleDataArray(ch, trigger_point());
  delay(1);
}

static const uint32_t sample_rate[6] = {
  250000, // 4us sampling (250ksps) x10
  250000, // 4us sampling (250ksps) x5
  250000, // 4us sampling (250ksps) x2
  250000, // 4us sampling (250ksps)
  100000, // 10us sampling (100ksps)
  50000}; // 20us sampling (50ksps)

void i2sInit() {
  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate          = sample_rate[rate],
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = I2S_BUFFER_COUNT,
    .dma_buf_len          = I2S_BUFFER_SIZE,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    i2s_set_adc_mode(ADC_UNIT, ADC1_CHANNEL_7);
  } else {
    i2s_set_adc_mode(ADC_UNIT, ADC1_CHANNEL_6);
  }
  i2s_adc_enable(I2S_NUM);
}

void rate_dma_mode_config(void) {
  if (orate > RATE_DMA && rate <= RATE_DMA) {
    i2sInit();                        // initialize I2S ADC
  } else if (orate <= RATE_DMA && rate > RATE_DMA) {
    i2s_adc_disable(I2S_NUM);
    i2s_driver_uninstall(I2S_NUM);    //stop & destroy i2s driver
    if (dds_mode)
      dac_output_enable(DAC_CHANNEL_1); // fix the problem why?
  }
  if (rate <= RATE_DMA) {
    i2s_set_sample_rates(I2S_NUM, sample_rate[rate]);
  }
}

//void i2s_adc_stop(void) {
//  i2s_stop(I2S_NUM);
//}

#else
// Sample the ADC continuous mode at a particular sample rate
// Copyright (c) 2026, Siliconvalley4066

#include <driver/adc.h>
#include <esp_adc/adc_continuous.h>

adc_continuous_handle_t adc_handle = NULL;
static adc_channel_t channel[2] = { ADC_CHANNEL_1, ADC_CHANNEL_3 };

void sample_dma() {
  adc_digi_output_data_t *data = (adc_digi_output_data_t *)dma_buf;
  uint32_t num_bytes_read = 0;

  adc_continuous_read(adc_handle, (uint8_t *)dma_buf, sizeof(dma_buf),
                      &num_bytes_read, 100);  // 100ms timeout
  int num_samples = num_bytes_read / sizeof(adc_digi_output_data_t);
  // Serial.println(num_samples);

  int i1 = 0, i2 = 0;
  for (size_t i = 0; i < num_samples; ++i) {
    adc_digi_output_data_t *p = &data[i];
    uint16_t chan = p->type2.channel;
    if (chan == channel[0])
      cap_buf[i1++] = p->type2.data;
    else if (chan == channel[1])
      cap_buf1[i2++] = p->type2.data;
  }

  // byte ch;
  // if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
  //   ch = ad_ch1;
  // } else {
  //   ch = ad_ch0;
  // }
  vTaskDelay(1);
  int t = trigger_point();
  if (ch0_mode != MODE_OFF)
    scaleDataArray(ad_ch0, t);
  if (ch1_mode != MODE_OFF)
    scaleDataArray(ad_ch1, t);
  vTaskDelay(1);
}

static const uint32_t sample_rate[8] = {
  83333,  // 12us sampling (83.3ksps) x10
  83333,  // 12us sampling (83.3ksps) x5
  83333,  // 12us sampling (83.3ksps) x2
  83333,  // 12us sampling (83.3ksps)
  50000,  // 20us sampling (50ksps)
  80000,  // 25us sampling (40ksps) dual
  40000,  // 50us sampling (20ksps) dual
  20000}; // 100us sampling (10ksps) dual

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num) {
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = NSAMP * sizeof(adc_digi_output_data_t),
    .conv_frame_size = NSAMP * sizeof(uint16_t),
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adc_handle));

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = { 0 };
  for (int i = 0; i < channel_num; i++) {
    adc_pattern[i].atten = ADC_ATTEN_DB_11;
    adc_pattern[i].channel = channel[i] & 0x7;
    adc_pattern[i].unit = ADC_UNIT_1;
    adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  }

  adc_continuous_config_t dig_cfg = {
    .pattern_num = channel_num,
    .adc_pattern = adc_pattern,
    .sample_freq_hz = sample_rate[rate],
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &dig_cfg));
  adc_continuous_start(adc_handle);
}

void rate_dma_mode_config(void) {
  adc_channel_t *ch = channel;
  if (ch0_mode == MODE_OFF && ch1_mode != MODE_OFF) {
    ch = &channel[1];
  } else {
    ch = channel;
  }
  if (rate <= RATE_DMA) {
    if (orate <= RATE_DMA)
      dma_adc_stop();
    if (rate >= RATE_DUAL)
      continuous_adc_init(channel, 2);  // initialize DMA ADC
    else
      continuous_adc_init(ch, 1);       // initialize DMA ADC
  } else if (orate <= RATE_DMA) {
    dma_adc_stop();
  }
}

void dma_adc_stop(void) {
  adc_continuous_stop(adc_handle);
  adc_continuous_deinit(adc_handle);  //stop & destroy continuous driver
}
#endif

int trigger_point() {
  int trigger_ad, i;
  uint16_t *cap;

  if (trig_ch == ad_ch1) {
    trigger_ad = advalue(trig_lv, VREF[range1], ch1_mode, ch1_off);
    cap = cap_buf1;
  } else {
    trigger_ad = advalue(trig_lv, VREF[range0], ch0_mode, ch0_off);
    cap = cap_buf;
  }
  for (i = 0; i < (NSAMP/2 - SAMPLES - 1); ++i) {
    if (trig_edge == TRIG_E_UP) {
      if (cap[i] < trigger_ad && cap[i+1] > trigger_ad)
        break;
    } else {
      if (cap[i] > trigger_ad && cap[i+1] < trigger_ad)
        break;
    }
  }
  return i;
}
