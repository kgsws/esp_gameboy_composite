#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/lldesc.h"
#include "soc/gpio_periph.h"
#include "soc/timer_group_struct.h"
#include "hal/dac_ll.h"
#include "hal/i2s_ll.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"

// composite output is on GPIO25

// select output signal hack
// 0 - close to NTSC 240p with extra incomplete line
// 1 - extra line, shorter sync lines
// 2 - longer lines, shorted sync lines
// 3 - experimental 144p (long blank)
// 4 - experimental 144p (slower clock)
// 5 - experimental 288p (pixels are doubled)
#define MODE_HACK	3

// input pins
#define GBLCD_INPUT_S	19 // vsync
#define GBLCD_INPUT_CP_ST	18 // clock | hsync (combine these two inputs with OR gate)
#define GBLCD_INPUT_LD0	12 // DATA 0
#define GBLCD_INPUT_LD1	14 // DATA 1

#define DAC_BLACK	23
#define DAC_WHITE	85

// NOTE: all line-related values must be even due to ESP DMA
#define START_X	126	// only non-experimental modes
#define START_Y	74

#if MODE_HACK == 1
#define LINE_WIDTH	364
#define LINE_COUNT	263
#define SYNC_HACK	-22
#define CLK_DIV_NUM	7
#elif MODE_HACK == 2
#define LINE_WIDTH	366
#define LINE_COUNT	262
#define SYNC_HACK	-74
#define CLK_DIV_NUM	7
#elif MODE_HACK == 3
#define LINE_WIDTH	240
#define LINE_SYNC	2
#define LINE_BLANK	130
#define LINE_COUNT	(144 + LINE_SYNC + LINE_BLANK)
#define SYNC_HACK	0
#define SYNC_PULSE	12
#define CLK_DIV_NUM	10
#undef START_X
#define START_X	46
#elif MODE_HACK == 4
#define LINE_WIDTH	230
#define LINE_SYNC	2
#define LINE_BLANK	60
#define LINE_COUNT	(144 + LINE_SYNC + LINE_BLANK)
#define SYNC_HACK	0
#define SYNC_PULSE	10
#define CLK_DIV_NUM	14
#undef START_X
#define START_X	40
#elif MODE_HACK == 5
#define LINE_WIDTH	210
#define LINE_SYNC	2
#define LINE_BLANK	30
#define LINE_COUNT	(288 + LINE_SYNC + LINE_BLANK)
#define SYNC_HACK	0
#define CLK_DIV_NUM	10
#undef START_X
#define START_X	40
#else
#define LINE_WIDTH	364
#define LINE_COUNT	262
#define SYNC_HACK	0
#define CLK_DIV_NUM	7
#endif

#define GAMEBOY_WIDTH	160
#define GAMEBOY_HEIGHT	144

#define GBLCD_SIZE	(GAMEBOY_WIDTH * GAMEBOY_HEIGHT)
#define GBLCD_SIZE_DMA	(GBLCD_SIZE * 2)
#define GB_MAX_DMA	4092
#define GB_DMA_COUNT	((GBLCD_SIZE_DMA + (GB_MAX_DMA-1)) / GB_MAX_DMA)

#define FB_STRIDE	(LINE_WIDTH * 2)

//

static lldesc_t dma_out[LINE_COUNT];
static lldesc_t dma_in[GB_DMA_COUNT];

static uint8_t gbframe[GBLCD_SIZE_DMA];

static uint16_t ldat_empty[LINE_WIDTH];
static uint16_t ldat_sync[LINE_WIDTH + SYNC_HACK];

static uint16_t ldat_screen[LINE_WIDTH * GAMEBOY_HEIGHT];

static uint8_t *const framebuf = (uint8_t*)&ldat_screen[START_X] + 1;

static uint8_t *dst_ptr = framebuf +  GAMEBOY_HEIGHT * FB_STRIDE;
static uint8_t *src_ptr;

static const uint8_t remap[] = {DAC_WHITE, 64, 44, DAC_BLACK};

//
// OUTPUT

static void IRAM_ATTR prepare_line_isr(void *arg)
{
	I2S0.int_clr.out_eof = 1;

	if(dst_ptr >= framebuf +  GAMEBOY_HEIGHT * FB_STRIDE)
		// should never happen
		return;

	for(uint32_t i = 0; i < GAMEBOY_WIDTH * 2; i += 2)
		dst_ptr[i] = remap[src_ptr[i]];

	src_ptr += GAMEBOY_WIDTH * 2;
	dst_ptr += FB_STRIDE;
}

static void dac_init(void)
{
	uint16_t *dst = ldat_screen;

#if MODE_HACK == 3 || MODE_HACK == 4
	uint32_t li = 0;

	for(uint32_t i = SYNC_PULSE; i < LINE_WIDTH; i++)
		ldat_empty[i] = DAC_BLACK << 8;

	for(uint32_t i = LINE_WIDTH+SYNC_HACK-SYNC_PULSE; i < sizeof(ldat_sync)/sizeof(uint16_t); i++)
		ldat_sync[i] = DAC_BLACK << 8;

	for( ; li < LINE_SYNC; li++)
	{
		dma_out[li].offset = 0;
		dma_out[li].size = sizeof(ldat_sync);
		dma_out[li].length = sizeof(ldat_sync);
		dma_out[li].sosf = 0;
		dma_out[li].eof = 0;
		dma_out[li].owner = 1;
		dma_out[li].buf = (uint8_t*)ldat_sync;
		dma_out[li].empty = (uint32_t)&dma_out[li+1];
	}

	for( ; li < LINE_BLANK; li++)
	{
		dma_out[li].offset = 0;
		dma_out[li].size = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].length = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].sosf = 0;
		dma_out[li].eof = 0;
		dma_out[li].owner = 1;
		dma_out[li].buf = (uint8_t*)ldat_empty;
		dma_out[li].empty = (uint32_t)&dma_out[li+1];
	}

	for(uint32_t i = 0; i < GAMEBOY_HEIGHT; li++, i++)
	{
		dma_out[li].offset = 0;
		dma_out[li].size = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].length = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].sosf = 0;
		dma_out[li].eof = 0;
		dma_out[li].owner = 1;
		dma_out[li].buf = (uint8_t*)dst;
		dma_out[li].empty = (uint32_t)&dma_out[li+1];

		dma_out[li - 1].eof = 1;

		memcpy(dst, ldat_empty, sizeof(ldat_empty));
		dst += LINE_WIDTH;
	}

	dma_out[LINE_COUNT-1].size -= 22 * sizeof(uint16_t);
	dma_out[LINE_COUNT-1].empty = 0;
#elif MODE_HACK == 5
	uint32_t li = 0;

	for(uint32_t i = 12; i < LINE_WIDTH; i++)
		ldat_empty[i] = DAC_BLACK << 8;

	for(uint32_t i = LINE_WIDTH+SYNC_HACK-12; i < sizeof(ldat_sync)/sizeof(uint16_t); i++)
		ldat_sync[i] = DAC_BLACK << 8;

	for( ; li < LINE_SYNC; li++)
	{
		dma_out[li].offset = 0;
		dma_out[li].size = sizeof(ldat_sync);
		dma_out[li].length = sizeof(ldat_sync);
		dma_out[li].sosf = 0;
		dma_out[li].eof = 0;
		dma_out[li].owner = 1;
		dma_out[li].buf = (uint8_t*)ldat_sync;
		dma_out[li].empty = (uint32_t)&dma_out[li+1];
	}

	for( ; li < LINE_BLANK; li++)
	{
		dma_out[li].offset = 0;
		dma_out[li].size = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].length = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].sosf = 0;
		dma_out[li].eof = 0;
		dma_out[li].owner = 1;
		dma_out[li].buf = (uint8_t*)ldat_empty;
		dma_out[li].empty = (uint32_t)&dma_out[li+1];
	}

	for(uint32_t i = 0; i < GAMEBOY_HEIGHT * 2; li++, i++)
	{
		dma_out[li].offset = 0;
		dma_out[li].size = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].length = LINE_WIDTH * sizeof(uint16_t);
		dma_out[li].sosf = 0;
		dma_out[li].eof = 0;
		dma_out[li].owner = 1;
		dma_out[li].buf = (uint8_t*)dst;
		dma_out[li].empty = (uint32_t)&dma_out[li+1];

		if(!(i & 1))
			continue;

		dma_out[li - 2].eof = 1;

		memcpy(dst, ldat_empty, sizeof(ldat_empty));
		dst += LINE_WIDTH;
	}

	dma_out[LINE_COUNT-1].size -= 22 * sizeof(uint16_t);
	dma_out[LINE_COUNT-1].empty = 0;
#else
	for(uint32_t i = 0; i < LINE_COUNT; i++)
	{
		dma_out[i].offset = 0;
		dma_out[i].size = LINE_WIDTH * sizeof(uint16_t);
		dma_out[i].length = LINE_WIDTH * sizeof(uint16_t);
		dma_out[i].sosf = 0;
		dma_out[i].eof = 0;
		dma_out[i].owner = 1;
		dma_out[i].buf = (uint8_t*)ldat_empty;
		dma_out[i].empty = (uint32_t)&dma_out[i+1];
	}

	for(uint32_t i = LINE_COUNT-3; i < LINE_COUNT; i++)
	{
		dma_out[i].size = sizeof(ldat_sync);
		dma_out[i].length = sizeof(ldat_sync);
		dma_out[i].buf = (uint8_t*)ldat_sync;
	}

	dma_out[LINE_COUNT-1].size -= 22 * sizeof(uint16_t);
	dma_out[LINE_COUNT-1].empty = 0;

	for(uint32_t i = 26; i < LINE_WIDTH; i++)
		ldat_empty[i] = DAC_BLACK << 8;

	for(uint32_t i = LINE_WIDTH+SYNC_HACK-24; i < sizeof(ldat_sync)/sizeof(uint16_t); i++)
		ldat_sync[i] = DAC_BLACK << 8;

	for(uint32_t i = START_Y; i < START_Y + GAMEBOY_HEIGHT; i++)
	{
		dma_out[i-1].eof = 1;
		dma_out[i].buf = (uint8_t*)dst;
		memcpy(dst, ldat_empty, sizeof(ldat_empty));
		dst += LINE_WIDTH;
	}
#endif

	// I2S

	periph_module_enable(PERIPH_I2S0_MODULE);

	I2S0.conf.tx_reset = 1;
	I2S0.conf.tx_reset = 0;
	I2S0.conf.tx_fifo_reset = 1;
	I2S0.conf.tx_fifo_reset = 0;

	I2S0.conf2.val = 0;
	I2S0.conf2.lcd_en = 1;
	I2S0.conf2.lcd_tx_wrx2_en = 1;

	I2S0.sample_rate_conf.val = 0;
	I2S0.sample_rate_conf.rx_bits_mod = 16;
	I2S0.sample_rate_conf.tx_bits_mod = 16;
	I2S0.sample_rate_conf.rx_bck_div_num = 2;
	I2S0.sample_rate_conf.tx_bck_div_num = 2;

	I2S0.fifo_conf.tx_fifo_mod = 2;

	I2S0.clkm_conf.val = 0;
	I2S0.clkm_conf.clka_en = 0;
	I2S0.clkm_conf.clkm_div_a = 0;
	I2S0.clkm_conf.clkm_div_b = 0;
	I2S0.clkm_conf.clkm_div_num = CLK_DIV_NUM; // 40000000 is base

	I2S0.conf1.val = 0;
	I2S0.conf1.tx_pcm_bypass = 1;

	I2S0.conf_chan.tx_chan_mod = 1;
	I2S0.conf_chan.rx_chan_mod = 1;

	I2S0.lc_conf.in_rst = 1;
	I2S0.lc_conf.out_rst = 1;
	I2S0.lc_conf.ahbm_rst = 1;
	I2S0.lc_conf.ahbm_fifo_rst = 1;
	I2S0.lc_conf.in_rst = 0;
	I2S0.lc_conf.out_rst = 0;
	I2S0.lc_conf.ahbm_rst = 0;
	I2S0.lc_conf.ahbm_fifo_rst = 0;

	I2S0.conf.tx_reset = 1;
	I2S0.conf.tx_fifo_reset = 1;
	I2S0.conf.rx_fifo_reset = 1;
	I2S0.conf.tx_reset = 0;
	I2S0.conf.tx_fifo_reset = 0;
	I2S0.conf.rx_fifo_reset = 0;

	I2S0.timing.val = 0;
	I2S0.int_ena.val = 0;

	I2S0.lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;

	esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, prepare_line_isr, NULL, NULL);

	// DAC

	dac_ll_power_on(DAC_CHAN_0);
	dac_ll_digi_enable_dma(1);
}

static void IRAM_ATTR dac_frame(void)
{
	dst_ptr = framebuf;
	src_ptr = gbframe;

	I2S0.int_clr.out_eof = 1;
	I2S0.int_ena.out_eof = 1;
	I2S0.out_link.addr = (uint32_t)dma_out & 0xfffff;
	I2S0.out_link.start = 1;
	I2S0.conf.tx_start = 1;
}

//
// INPUT

static void IRAM_ATTR finish_capture_isr(void *arg)
{
	typeof(TIMERG1.int_st_timers) status = TIMERG1.int_st_timers;

	if(status.val == 0)
		return;

	TIMERG1.int_clr_timers.val = status.val;

	if(status.t0_int_st)
	{
		// one extra clock cycle has to be faked here
		// seems like DMA is one uint32_t behind clock pulse
		gpio_matrix_in(0x38, I2S1I_WS_IN_IDX, false);
		gpio_matrix_in(GBLCD_INPUT_CP_ST, I2S1I_WS_IN_IDX, false);

		I2S1.conf.rx_start = 0;
		I2S1.in_link.stop = 1;
		I2S1.conf.rx_reset = 1;
		I2S1.conf.rx_reset = 0;

		TIMERG1.hw_timer[0].config.tx_en = 0;

		gpio_intr_enable(GBLCD_INPUT_S);
	}
}

static void IRAM_ATTR capture_vsync_isr(void *arg)
{
	dac_frame();

	gpio_intr_disable(GBLCD_INPUT_S);

	I2S1.conf.rx_start = 0;
	I2S1.conf.rx_reset = 1;
	I2S1.conf.rx_reset = 0;
	I2S1.conf.rx_fifo_reset = 1;
	I2S1.conf.rx_fifo_reset = 0;
	I2S1.lc_conf.in_rst = 1;
	I2S1.lc_conf.in_rst = 0;
	I2S1.lc_conf.ahbm_fifo_rst = 1;
	I2S1.lc_conf.ahbm_fifo_rst = 0;
	I2S1.lc_conf.ahbm_rst = 1;
	I2S1.lc_conf.ahbm_rst = 0;

	I2S1.rx_eof_num = (GBLCD_SIZE_DMA / sizeof(uint32_t)) + 1;
	I2S1.in_link.addr = (uint32_t)dma_in & 0xfffff;
	I2S1.in_link.start = 1;
	I2S1.conf.rx_start = 1;

	TIMERG1.hw_timer[0].load.tx_load = 1;
	TIMERG1.hw_timer[0].config.tx_alarm_en = 1;
	TIMERG1.hw_timer[0].config.tx_en = 1;
}

static void gblcd_init()
{
	uint32_t tmp;
	gpio_config_t io_conf = {0};

	/// I2S (camera capture mode)

	tmp = GBLCD_SIZE_DMA;
	for(uint32_t i = 0; i < GB_DMA_COUNT; i++)
	{
		dma_in[i].size = tmp > GB_MAX_DMA ? GB_MAX_DMA : tmp;
		dma_in[i].length = 0;
		dma_in[i].sosf = 0;
		dma_in[i].eof = 0;
		dma_in[i].owner = 1;
		dma_in[i].buf = gbframe + GB_MAX_DMA * i;
		dma_in[i].empty = (uint32_t)&dma_in[i + 1];
		tmp -= GB_MAX_DMA;
	}
	dma_in[GB_DMA_COUNT-1].empty = 0;

	//
	periph_module_enable(PERIPH_I2S1_MODULE);
	periph_module_enable(PERIPH_TIMG1_MODULE);

	// init

	I2S1.conf.rx_reset = 1;
	I2S1.conf.rx_reset = 0;
	I2S1.conf.rx_fifo_reset = 1;
	I2S1.conf.rx_fifo_reset = 0;
	I2S1.lc_conf.in_rst = 1;
	I2S1.lc_conf.in_rst = 0;
	I2S1.lc_conf.ahbm_fifo_rst = 1;
	I2S1.lc_conf.ahbm_fifo_rst = 0;
	I2S1.lc_conf.ahbm_rst = 1;
	I2S1.lc_conf.ahbm_rst = 0;

	I2S1.conf.rx_slave_mod = 1;
	I2S1.conf.rx_right_first = 0;
	I2S1.conf.rx_msb_right = 0;
	I2S1.conf.rx_msb_shift = 0;
	I2S1.conf.rx_mono = 0;
	I2S1.conf.rx_short_sync = 0;

	I2S1.conf2.lcd_en = 1;
	I2S1.conf2.camera_en = 1;

	// configure clock divider
	I2S1.clkm_conf.clkm_div_a = 0;
	I2S1.clkm_conf.clkm_div_b = 0;
	I2S1.clkm_conf.clkm_div_num = 2;

	I2S1.fifo_conf.dscr_en = 1;
	I2S1.fifo_conf.rx_fifo_mod = 1;
	I2S1.fifo_conf.rx_fifo_mod_force_en = 1;

	I2S1.conf_chan.rx_chan_mod = 1;
	I2S1.sample_rate_conf.rx_bits_mod = 0;
	I2S1.timing.val = 0;
	I2S1.timing.rx_dsync_sw = 1;

	I2S1.int_ena.val = 0;

	// config

	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	io_conf.pin_bit_mask = 1 << GBLCD_INPUT_S;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
	gpio_config(&io_conf);
	gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
	gpio_isr_handler_add(GBLCD_INPUT_S, capture_vsync_isr, NULL);
	gpio_intr_disable(GBLCD_INPUT_S);

	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GBLCD_INPUT_S], PIN_FUNC_GPIO);
	gpio_set_direction(GBLCD_INPUT_S, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GBLCD_INPUT_S, GPIO_FLOATING);

	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GBLCD_INPUT_CP_ST], PIN_FUNC_GPIO);
	gpio_set_direction(GBLCD_INPUT_CP_ST, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GBLCD_INPUT_CP_ST, GPIO_FLOATING);
	gpio_matrix_in(GBLCD_INPUT_CP_ST, I2S1I_WS_IN_IDX, false);

	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GBLCD_INPUT_LD0], PIN_FUNC_GPIO);
	gpio_set_direction(GBLCD_INPUT_LD0, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GBLCD_INPUT_LD0, GPIO_FLOATING);
	gpio_matrix_in(GBLCD_INPUT_LD0, I2S1I_DATA_IN0_IDX, false);

	PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[GBLCD_INPUT_LD1], PIN_FUNC_GPIO);
	gpio_set_direction(GBLCD_INPUT_LD1, GPIO_MODE_INPUT);
	gpio_set_pull_mode(GBLCD_INPUT_LD1, GPIO_FLOATING);
	gpio_matrix_in(GBLCD_INPUT_LD1, I2S1I_DATA_IN1_IDX, false);

	gpio_matrix_in(0x38, I2S1I_H_ENABLE_IDX, false);
	gpio_matrix_in(0x38, I2S1I_H_SYNC_IDX, false);
	gpio_matrix_in(0x38, I2S1I_V_SYNC_IDX, false);

	gpio_matrix_in(0x30, I2S1I_DATA_IN2_IDX, false);
	gpio_matrix_in(0x30, I2S1I_DATA_IN3_IDX, false);
	gpio_matrix_in(0x30, I2S1I_DATA_IN4_IDX, false);
	gpio_matrix_in(0x30, I2S1I_DATA_IN5_IDX, false);
	gpio_matrix_in(0x30, I2S1I_DATA_IN6_IDX, false);
	gpio_matrix_in(0x30, I2S1I_DATA_IN7_IDX, false);

	// frame timer
	TIMERG1.int_ena_timers.val = 0;
	TIMERG1.int_clr_timers.val = 0xFFFFFFFF;
	TIMERG1.hw_timer[0].config.tx_autoreload = 0;
	TIMERG1.hw_timer[0].config.tx_divider = 80;
	TIMERG1.hw_timer[0].config.tx_en = 0;
	TIMERG1.hw_timer[0].config.tx_increase = 1;
	TIMERG1.hw_timer[0].config.tx_alarm_en = 0;
	TIMERG1.hw_timer[0].config.tx_level_int_en = 1;
	TIMERG1.hw_timer[0].config.tx_edge_int_en = 0;
	TIMERG1.hw_timer[0].loadhi.tx_load_hi = 0;
	TIMERG1.hw_timer[0].loadlo.tx_load_lo = 0;
	TIMERG1.hw_timer[0].alarmhi.tx_alarm_hi = 0;
	TIMERG1.hw_timer[0].alarmlo.tx_alarm_lo = 16000;
	TIMERG1.int_ena_timers.t0_int_ena = 1;

	esp_intr_alloc(ETS_TG1_T0_LEVEL_INTR_SOURCE, ESP_INTR_FLAG_IRAM, finish_capture_isr, NULL, NULL);

	// enable
	gpio_intr_enable(GBLCD_INPUT_S);
}

//
// MAIN

void app_main(void)
{
	printf("kgsws' GameBoy to Composite\n");

	dac_init();
	gblcd_init();
}
