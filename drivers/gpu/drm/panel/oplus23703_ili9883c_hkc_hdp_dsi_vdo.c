// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>

#include <soc/oplus/device_info.h>
#include <soc/oplus/system/boot_mode.h>
#include <mt-plat/mtk_boot_common.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif
#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/msm_drm_notify.h>
#include <drm/drm_panel.h>
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif
//#include "../mediatek/mediatek_v2/mtk_corner_pattern/oplus_23662_tianma_mtk_data_hw_roundedpattern.h"

#include "../../../misc/mediatek/gate_ic/gate_i2c.h"

#include "ktz8866.h"
#define CHANGE_FPS_EN 1

#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern enum boot_mode_t get_boot_mode(void);
#else
extern int get_boot_mode(void);
#endif
#else
/* extern unsigned int silence_mode; */
extern int get_boot_mode(void);
#endif
#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif
#ifdef LCD_LOAD_TP_FW
extern void lcd_queue_load_tp_fw(void);
#endif

#define MAX_NORMAL_BRIGHTNESS 3210
#define MULTIPLE_BRIGHTNESS   1070

static int esd_brightness;
extern unsigned long esd_flag;
extern unsigned int g_shutdown_flag;
//static int current_esd_fps;

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
//#include "lcm_i2c.h"
#include "../bias/ocp2130_drv.h"

static char bl_tb0[] = { 0x51, 0x0F, 0xFF};
//static char bl_open[] = { 0x53, 0x2C };
/*esd check*/
//static char fps_open[] = 	{ 0xB9, 0x83, 0x10, 0x21, 0x55, 0x00 };
//static char fps_BD[] =     	{ 0xBD, 0x00 };
//static char fps120_e2[] =   { 0xE2, 0x00 };
//static char fps60_e2[] =    { 0xE2, 0x50 };
//static char fps40_e2[] =    { 0xE2, 0x60 };
//static char fps_close[] =    { 0xB9, 0x00, 0x00, 0x00, 0x00, 0x00 };
//TO DO: You have to do that remove macro BYPASSI2C and solve build error
//otherwise voltage will be unstable
#ifdef HIMAX_WAKE_UP
extern uint8_t wake_flag_drm;
#endif
extern unsigned int oplus_max_normal_brightness;

static int first_set_bl;
static int cabc_status = 3;
static int esd_last_level;

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	bool prepared;
	bool enabled;

	unsigned int gate_ic;

	int error;
};


#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                           \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void lcm_panel_init(struct lcm *ctx)
{
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5001);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(5 * 1000, 5001);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(5 * 1000, 5001);
	gpiod_set_value(ctx->reset_gpio, 1);

#ifdef LCD_LOAD_TP_FW
	lcd_queue_load_tp_fw();
#endif
	usleep_range(15*1000, 15001);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	first_set_bl = 1;

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x01 );
	lcm_dcs_write_seq_static(ctx, 0x00, 0x4D);
	lcm_dcs_write_seq_static(ctx, 0x01, 0x35);
	lcm_dcs_write_seq_static(ctx, 0x02, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x03, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x04, 0xC9);
	lcm_dcs_write_seq_static(ctx, 0x05, 0x15);
	lcm_dcs_write_seq_static(ctx, 0x06, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x07, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x08, 0x89);
	lcm_dcs_write_seq_static(ctx, 0x09, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x0A, 0xB4);
	lcm_dcs_write_seq_static(ctx, 0x0B, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x0C, 0x53);
	lcm_dcs_write_seq_static(ctx, 0x0D, 0x53);
	lcm_dcs_write_seq_static(ctx, 0x0E, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x0F, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x16, 0x89);
	lcm_dcs_write_seq_static(ctx, 0x17, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x18, 0x34);
	lcm_dcs_write_seq_static(ctx, 0x19, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x1A, 0x53);
	lcm_dcs_write_seq_static(ctx, 0x1B, 0x53);
	lcm_dcs_write_seq_static(ctx, 0x1C, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x1D, 0x00);

	lcm_dcs_write_seq_static(ctx, 0x28, 0x4F);
	lcm_dcs_write_seq_static(ctx, 0x2A, 0x4F);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x91);
	lcm_dcs_write_seq_static(ctx, 0x2B, 0x91);

	lcm_dcs_write_seq_static(ctx, 0x31, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x32, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x33, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x34, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x36, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x37, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x38, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x39, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x3A, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x3B, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x3C, 0x10);
	lcm_dcs_write_seq_static(ctx, 0x3D, 0x12);
	lcm_dcs_write_seq_static(ctx, 0x3E, 0x14);
	lcm_dcs_write_seq_static(ctx, 0x3F, 0x16);
	lcm_dcs_write_seq_static(ctx, 0x40, 0x18);
	lcm_dcs_write_seq_static(ctx, 0x41, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0x42, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0x43, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0x44, 0x08);
	lcm_dcs_write_seq_static(ctx, 0x45, 0x07);
	lcm_dcs_write_seq_static(ctx, 0x46, 0x07);

	lcm_dcs_write_seq_static(ctx, 0x47, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x48, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x49, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x4A, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x4B, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x4C, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x4D, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x4E, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x4F, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x50, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x52, 0x11);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x13);
	lcm_dcs_write_seq_static(ctx, 0x54, 0x15);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x17);
	lcm_dcs_write_seq_static(ctx, 0x56, 0x19);
	lcm_dcs_write_seq_static(ctx, 0x57, 0x1B);
	lcm_dcs_write_seq_static(ctx, 0x58, 0x0D);
	lcm_dcs_write_seq_static(ctx, 0x59, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0x5A, 0x09);
	lcm_dcs_write_seq_static(ctx, 0x5B, 0x07);
	lcm_dcs_write_seq_static(ctx, 0x5C, 0x07);

	lcm_dcs_write_seq_static(ctx, 0x61, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x62, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x63, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x64, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x65, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x66, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x67, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x68, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x69, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x6A, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x6B, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x6C, 0x17);
	lcm_dcs_write_seq_static(ctx, 0x6D, 0x15);
	lcm_dcs_write_seq_static(ctx, 0x6E, 0x13);
	lcm_dcs_write_seq_static(ctx, 0x6F, 0x11);
	lcm_dcs_write_seq_static(ctx, 0x70, 0x1B);
	lcm_dcs_write_seq_static(ctx, 0x71, 0x19);
	lcm_dcs_write_seq_static(ctx, 0x72, 0x0D);
	lcm_dcs_write_seq_static(ctx, 0x73, 0x09);
	lcm_dcs_write_seq_static(ctx, 0x74, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0x75, 0x07);
	lcm_dcs_write_seq_static(ctx, 0x76, 0x07);

	lcm_dcs_write_seq_static(ctx, 0x77, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x78, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x79, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x7A, 0x22);
	lcm_dcs_write_seq_static(ctx, 0x7B, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x7C, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x7D, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x7E, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x7F, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x80, 0x2A);
	lcm_dcs_write_seq_static(ctx, 0x81, 0x23);
	lcm_dcs_write_seq_static(ctx, 0x82, 0x16);
	lcm_dcs_write_seq_static(ctx, 0x83, 0x14);
	lcm_dcs_write_seq_static(ctx, 0x84, 0x12);
	lcm_dcs_write_seq_static(ctx, 0x85, 0x10);
	lcm_dcs_write_seq_static(ctx, 0x86, 0x1A);
	lcm_dcs_write_seq_static(ctx, 0x87, 0x18);
	lcm_dcs_write_seq_static(ctx, 0x88, 0x0C);
	lcm_dcs_write_seq_static(ctx, 0x89, 0x08);
	lcm_dcs_write_seq_static(ctx, 0x8A, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0x8B, 0x07);
	lcm_dcs_write_seq_static(ctx, 0x8C, 0x07);

	lcm_dcs_write_seq_static(ctx, 0xD1, 0x10);
	lcm_dcs_write_seq_static(ctx, 0xD5, 0x5D);
	lcm_dcs_write_seq_static(ctx, 0xD6, 0x91);
	lcm_dcs_write_seq_static(ctx, 0xE6, 0x22);
	lcm_dcs_write_seq_static(ctx, 0xE7, 0x54);

	lcm_dcs_write_seq_static(ctx, 0xC3, 0x00);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x02);
	lcm_dcs_write_seq_static(ctx, 0x06, 0x58);
	lcm_dcs_write_seq_static(ctx, 0x0A, 0x5B);
	lcm_dcs_write_seq_static(ctx, 0x0C, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x0D, 0x14);
	lcm_dcs_write_seq_static(ctx, 0x0E, 0x68);
	lcm_dcs_write_seq_static(ctx, 0x39, 0x05);
	lcm_dcs_write_seq_static(ctx, 0x3A, 0x14);
	lcm_dcs_write_seq_static(ctx, 0x3B, 0x68);
	lcm_dcs_write_seq_static(ctx, 0x3C, 0xAB);
	lcm_dcs_write_seq_static(ctx, 0xF0, 0x05);
	lcm_dcs_write_seq_static(ctx, 0xF1, 0x47);
	lcm_dcs_write_seq_static(ctx, 0x48, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x44, 0x68);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x03);
	lcm_dcs_write_seq_static(ctx, 0x84, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x20, 0x01);
	lcm_dcs_write_seq_static(ctx, 0x22, 0xFA);
	lcm_dcs_write_seq_static(ctx, 0x80, 0x04);
	lcm_dcs_write_seq_static(ctx, 0x81, 0x04);
	lcm_dcs_write_seq_static(ctx, 0x82, 0x04);
	lcm_dcs_write_seq_static(ctx, 0xAF, 0x18);
	lcm_dcs_write_seq_static(ctx, 0xAD, 0xDD);
	lcm_dcs_write_seq_static(ctx, 0x8C, 0x59);
	lcm_dcs_write_seq_static(ctx, 0x90, 0xA9);
	lcm_dcs_write_seq_static(ctx, 0x92, 0xB1);
	lcm_dcs_write_seq_static(ctx, 0x93, 0xB2);
	lcm_dcs_write_seq_static(ctx, 0xB5, 0xD3);

	/*3% MODE1*/
	lcm_dcs_write_seq_static(ctx, 0xB6, 0x84);
	lcm_dcs_write_seq_static(ctx, 0x88, 0xEE);
	lcm_dcs_write_seq_static(ctx, 0x89, 0xEE);
	lcm_dcs_write_seq_static(ctx, 0x8A, 0xEF);
	lcm_dcs_write_seq_static(ctx, 0x8B, 0xF1);
	lcm_dcs_write_seq_static(ctx, 0xA6, 0xEF);
	lcm_dcs_write_seq_static(ctx, 0xAC, 0xEF);

	/*7% MODE2*/
	lcm_dcs_write_seq_static(ctx, 0xB7, 0x84);
	lcm_dcs_write_seq_static(ctx, 0x8C, 0xE4);
	lcm_dcs_write_seq_static(ctx, 0x8D, 0xE4);
	lcm_dcs_write_seq_static(ctx, 0x8E, 0xE5);
	lcm_dcs_write_seq_static(ctx, 0x8F, 0xE5);
	lcm_dcs_write_seq_static(ctx, 0x90, 0xE5);
	lcm_dcs_write_seq_static(ctx, 0x91, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0x92, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0x93, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0x94, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0x95, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0xA7, 0xE6);
	lcm_dcs_write_seq_static(ctx, 0xAD, 0xE6);

	/*11.5% MODE3*/
	lcm_dcs_write_seq_static(ctx, 0xB8, 0x84);
	lcm_dcs_write_seq_static(ctx, 0x96, 0x60);
	lcm_dcs_write_seq_static(ctx, 0x97, 0x9B);
	lcm_dcs_write_seq_static(ctx, 0x98, 0x9C);
	lcm_dcs_write_seq_static(ctx, 0x99, 0x9F);
	lcm_dcs_write_seq_static(ctx, 0x9A, 0xC6);
	lcm_dcs_write_seq_static(ctx, 0x9B, 0xC9);
	lcm_dcs_write_seq_static(ctx, 0x9C, 0xCE);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0xCE);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0xCF);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0xE1);
	lcm_dcs_write_seq_static(ctx, 0xA8, 0xF0);
	lcm_dcs_write_seq_static(ctx, 0xAE, 0xD9);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x05);
	lcm_dcs_write_seq_static(ctx, 0x69, 0x9C);
	lcm_dcs_write_seq_static(ctx, 0x6A, 0x9C);
	lcm_dcs_write_seq_static(ctx, 0x6D, 0x8D);
	lcm_dcs_write_seq_static(ctx, 0x73, 0x93);
	lcm_dcs_write_seq_static(ctx, 0x79, 0x8D);
	lcm_dcs_write_seq_static(ctx, 0x7F, 0x7F);
	lcm_dcs_write_seq_static(ctx, 0x68, 0x3E);
	lcm_dcs_write_seq_static(ctx, 0x66, 0x33);
	lcm_dcs_write_seq_static(ctx, 0x61, 0x11);
	lcm_dcs_write_seq_static(ctx, 0x5D, 0x5D);

	lcm_dcs_write_seq_static(ctx, 0xA5, 0x7B);
	lcm_dcs_write_seq_static(ctx, 0x4F, 0x0A);

	lcm_dcs_write_seq_static(ctx, 0x6E, 0xC9);
	lcm_dcs_write_seq_static(ctx, 0x6F, 0x8D);
	lcm_dcs_write_seq_static(ctx, 0x70, 0x8D);
	lcm_dcs_write_seq_static(ctx, 0x71, 0x8D);
	lcm_dcs_write_seq_static(ctx, 0x74, 0xCF);
	lcm_dcs_write_seq_static(ctx, 0x75, 0x93);
	lcm_dcs_write_seq_static(ctx, 0x76, 0x93);
	lcm_dcs_write_seq_static(ctx, 0x77, 0x93);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x06);
	lcm_dcs_write_seq_static(ctx, 0x06, 0xA4);
	lcm_dcs_write_seq_static(ctx, 0xD9, 0x1F);
	lcm_dcs_write_seq_static(ctx, 0xC0, 0x44);
	lcm_dcs_write_seq_static(ctx, 0xC1, 0x16);
	lcm_dcs_write_seq_static(ctx, 0x0A, 0x50);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x07);
	lcm_dcs_write_seq_static(ctx, 0xC0, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xCB, 0xB5);
	lcm_dcs_write_seq_static(ctx, 0xCC, 0xAC);
	lcm_dcs_write_seq_static(ctx, 0xCD, 0x9D);
	lcm_dcs_write_seq_static(ctx, 0xCE, 0x7E);
	lcm_dcs_write_seq_static(ctx, 0xD1, 0x20);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x0A);
	lcm_dcs_write_seq_static(ctx, 0xE0, 0x01);
	lcm_dcs_write_seq_static(ctx, 0xE1, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0xE2, 0x01);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x0B);
	lcm_dcs_write_seq_static(ctx, 0x9A, 0x85);
	lcm_dcs_write_seq_static(ctx, 0x9B, 0x9A);
	lcm_dcs_write_seq_static(ctx, 0x9C, 0x04);
	lcm_dcs_write_seq_static(ctx, 0x9D, 0x04);
	lcm_dcs_write_seq_static(ctx, 0x9E, 0x8C);
	lcm_dcs_write_seq_static(ctx, 0x9F, 0x8C);
	lcm_dcs_write_seq_static(ctx, 0xAA, 0x22);
	lcm_dcs_write_seq_static(ctx, 0xAB, 0xE0);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x0E);
	lcm_dcs_write_seq_static(ctx, 0x11, 0x4B);
	lcm_dcs_write_seq_static(ctx, 0x12, 0x02);
	lcm_dcs_write_seq_static(ctx, 0x13, 0x14);
	lcm_dcs_write_seq_static(ctx, 0x00, 0xA0);

	lcm_dcs_write_seq_static(ctx, 0xFF, 0x98, 0x83, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x35, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x53, 0x2c);
	lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
	lcm_dcs_write_seq_static(ctx, 0x68, 0x05);
	lcm_dcs_write_seq_static(ctx, 0x11, 0x00);
	usleep_range(65 * 1000, 65 * 1010);
	lcm_dcs_write_seq_static(ctx, 0x29, 0x00);
	usleep_range(5 * 1000, 5 * 1010);

	pr_info("%s-\n", __func__);
}

static void lcm_init_set_cabc(struct lcm *ctx) {

  	if (cabc_status == 1) {
  		lcm_dcs_write_seq_static(ctx, 0x55, 0x01);
  	} else if (cabc_status == 2) {
  		lcm_dcs_write_seq_static(ctx, 0x55, 0x02);
  	} else if (cabc_status == 3) {
  		lcm_dcs_write_seq_static(ctx, 0x55, 0x03);
  	} else if (cabc_status == 0) {
  		lcm_dcs_write_seq_static(ctx, 0x55, 0x00);
  	}

	pr_info("%s- cabc_init_mode=%d\n", __func__,cabc_status);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	pr_info("%s line = %d\n", __func__,__LINE__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int mode = 0;
	//int blank = 0;
	int flag_poweroff = 1;

	pr_info("%s+\n", __func__);
	if (!ctx->prepared)
		return 0;

	mode = get_boot_mode();
	if ((mode != MSM_BOOT_MODE__FACTORY) && (mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
		if(tp_gesture_enable_notifier && tp_gesture_enable_notifier(0) && (g_shutdown_flag == 0) && (esd_flag == 0)) {
			flag_poweroff = 0;
		} else {
			flag_poweroff = 1;
		}
	}

	msleep(8);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	msleep(20);

	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(90);
	pr_err("[TP]flag_poweroff = %d\n",flag_poweroff);
	if (flag_poweroff == 1) {
		/*ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);*/

		ctx->bias_neg =
			devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		usleep_range(5000, 5001);

		ctx->bias_pos =
			devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);
	}
	ctx->error = 0;
	ctx->prepared = false;
	pr_info("%s-\n", __func__);
	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;
	//int blank = 0;

	pr_info("%s+\n", __func__);
	if (ctx->prepared)
		return 0;

	// lcd reset H -> L -> L
	//ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	//gpiod_set_value(ctx->reset_gpio, 1);
	//usleep_range(10000, 10001);
	//gpiod_set_value(ctx->reset_gpio, 0);
	//msleep(20);
	//gpiod_set_value(ctx->reset_gpio, 1);
	//devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	// end
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(2*1000, 2*1000+100);// 2ms

	_bias_ic_i2c_panel_bias_enable(1);

	ctx->bias_pos =
		devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_pos, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_pos);

	usleep_range(5000, 5010);// 5ms
	ctx->bias_neg =
		devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->bias_neg, 1);
	devm_gpiod_put(ctx->dev, ctx->bias_neg);
	usleep_range(2 * 1000, 2 * 1000+100);// 2ms

	/*#define LCD_CTL_TP_LOAD_FW 0x10
	#define LCD_CTL_CS_ON  0x19
	blank = LCD_CTL_CS_ON;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	usleep_range(5000, 5100);
	blank = LCD_CTL_TP_LOAD_FW;
	mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
	usleep_range(5000, 5100);*/
#ifdef HIMAX_WAKE_UP
	//if (wake_flag_drm == 0)
		//lcd_set_bl_bias_reg(ctx->dev, 1);
#else
		//lcd_set_bl_bias_reg(ctx->dev, 1);
#endif
//	_lcm_i2c_write_bytes(0x0, 0xf);
//	_lcm_i2c_write_bytes(0x1, 0xf);
	lcm_panel_init(ctx);
	lcm_init_set_cabc(ctx);
	ret = ctx->error;
	if (ret < 0) {
		lcm_unprepare(panel);
		pr_info("%s11111-\n", __func__);
	}
	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif
/*
#ifdef LCD_LOAD_TP_FW
	lcd_queue_load_tp_fw();
#endif
*/
	pr_info("%s-\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	pr_info("%s line = %d\n", __func__,__LINE__);
	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 156696,
	.hdisplay = 720,
	.hsync_start = 720 + 150,//HFP
	.hsync_end = 720 + 150 + 8,//HSA
	.htotal = 720 + 150 + 8 + 4,//HBP
	.vdisplay = 1604,
	.vsync_start = 1604 + 350,/* 23703 90FPS */
	.vsync_end = 1604 + 350 + 2,//VSA
	.vtotal = 1604 + 350 + 2 + 18,//VBP
};


#if CHANGE_FPS_EN
static const struct drm_display_mode performance_mode_50hz = {
	.clock = 155408,
	.hdisplay = 720,
	.hsync_start = 720 + 150,//HFP
	.hsync_end = 720 + 150 + 8,//HSA
	.htotal = 720 + 150 + 8 + 4,//HBP
	.vdisplay = 1604,
	.vsync_start = 1604 + 1900,/* 23703 50FPS */
	.vsync_end = 1604 + 1900 + 2,//VSA
	.vtotal = 1604 + 1900 + 2 + 18,//VBP
};

static const struct drm_display_mode performance_mode_60hz = {
	.clock = 156061,
	.hdisplay = 720,
	.hsync_start = 720 + 150,//HFP
	.hsync_end = 720 + 150 + 8,//HSA
	.htotal = 720 + 150 + 8 + 4,//HBP
	.vdisplay = 1604,
	.vsync_start = 1604 + 1325,/* 23703 60FPS */
	.vsync_end = 1604 + 1325 + 2,//VSA
	.vtotal = 1604 + 1325 + 2 + 18,//VBP
};
#endif


#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params = {
	//.change_fps_by_vfp_send_cmd = 1,
	//.vfp_low_power = 148,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x09, .count = 3, .para_list[0] = 0x80, .para_list[1] = 0x03, .para_list[2] = 0x06,
	},


	.data_rate = 1030, /* 943 */
	//.data_rate_khz = 1030000, /* 943307 */

	.oplus_display_global_dre = 1,
};

#if CHANGE_FPS_EN
static struct mtk_panel_params ext_params_50hz = {
	//.change_fps_by_vfp_send_cmd = 1,
	//.vfp_low_power = 4484,
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x09, .count = 3, .para_list[0] = 0x80, .para_list[1] = 0x03, .para_list[2] = 0x06,
	},

	.data_rate = 1030, /* 943 */
	//.data_rate_khz = 920190, /* 943307 */

#if 0
	.lfr_enable = 0,
	.lfr_minimum_fps = 50,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 40,
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.vfp = 1900,
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.rotate = 1,
	/*.phy_timcon = {
		.hs_trail = 0x0B,
		.clk_hs_post = 0x0F,
	},*/
#endif
	.oplus_display_global_dre = 1,
};

static struct mtk_panel_params ext_params_60hz = {
	//.change_fps_by_vfp_send_cmd = 1,
	//.vfp_low_power = 2316, /* 60 FPS */
	.cust_esd_check = 0,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0x09, .count = 3, .para_list[0] = 0x80, .para_list[1] = 0x03, .para_list[2] = 0x06,
	},

	.data_rate = 1030, /* 943 */
#if 0
	//.data_rate_khz = 920190, /* 943307  待定*/
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.vfp = 1325,  /* 60 FPS */
	},
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
        .round_corner_en = 1,
        .corner_pattern_height = ROUND_CORNER_H_TOP,
        .corner_pattern_height_bot = ROUND_CORNER_H_BOT,
        .corner_pattern_tp_size = sizeof(top_rc_pattern),
        .corner_pattern_lt_addr = (void *)top_rc_pattern,
#endif
	.rotate = 1,
	//.phy_timcon = {
	//	.hs_trail = 0x0B,
	//	.clk_hs_post = 0x0F,
	//},
#endif
	.oplus_display_global_dre = 1,
};
#endif

static void cabc_switch(void *dsi, dcs_write_gce cb,void *handle, unsigned int cabc_mode)
{
    char bl_tb1[] = {0x55, 0x03}; /* no cabc ui pictures videoes*/

    pr_err("%s cabc = %d\n", __func__, cabc_mode);

    if (cabc_mode == 1) {
        bl_tb1[1] = 0x01;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 2) {
        bl_tb1[1] = 0x02;
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    } else if (cabc_mode == 3) {
        if(cabc_status == 0){
            bl_tb1[1] = 0x01;
            usleep_range(3000, 4000);
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));

            bl_tb1[1] = 0x02;
            usleep_range(3000, 4000);
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));

            bl_tb1[1] = 0x03;
            usleep_range(3000, 4000);
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        }else{
            bl_tb1[1] = 0x03;
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        }
    } else if (cabc_mode == 0) {
        if(cabc_status == 3){

            bl_tb1[1] = 0x02;
            usleep_range(3000, 4000);
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));

            bl_tb1[1] = 0x01;
            usleep_range(3000, 4000);
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));

            bl_tb1[1] = 0x00; /* cabc off */
            usleep_range(3000, 4000);
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        }else{
            bl_tb1[1] = 0x00; /* cabc off */
            cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        }
    }else {
        bl_tb1[1] = 0x03; /* default */
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
    }
	cabc_status = cabc_mode;
    /* cabc_lastlevel = cabc_mode; */
}


static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int map_exp[4096] = {0};

static void init_global_exp_backlight(void)
{
        int lut_index[41] = {0, 64, 144, 154, 187, 227, 264, 300, 334, 366, 397, 427, 456, 484, 511, 537, 563, 587, 611, 635, 658, 680, 702,
                                                 723, 744, 764, 784, 804, 823, 842, 861, 879, 897, 915, 933, 950, 967, 984, 1000, 1016, 1070};
        int lut_value1[41] = {0, 3,  11,  13,  21,  32,  45, 59,  76,  92, 111, 130,  150, 172, 194, 219, 244, 268, 293, 320, 347, 373, 402,
                                                430, 459, 487, 517, 549, 579, 611, 642, 675, 707, 740, 775, 808, 843, 878, 911, 947, 1070};
        int index_start = 0, index_end = 0;
        int value1_start = 0, value1_end = 0;
        int i,j;
        int index_len = sizeof(lut_index) / sizeof(int);
        int value_len = sizeof(lut_value1) / sizeof(int);
        if (index_len == value_len) {
                for (i = 0; i < index_len - 1; i++) {
                        index_start = lut_index[i] * oplus_max_normal_brightness / MULTIPLE_BRIGHTNESS;
                        index_end = lut_index[i+1] * oplus_max_normal_brightness / MULTIPLE_BRIGHTNESS;
                        value1_start = lut_value1[i] * oplus_max_normal_brightness / MULTIPLE_BRIGHTNESS;
                        value1_end = lut_value1[i+1] * oplus_max_normal_brightness / MULTIPLE_BRIGHTNESS;
                        for (j = index_start; j <= index_end; j++) {
                                map_exp[j] = value1_start + (value1_end - value1_start) * (j - index_start) / (index_end - index_start);
                        }
                }
        }
}


static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	unsigned int bl_level = level;
	/* bl_level = map_exp[level]; */
	unsigned int mode;

	mode = get_boot_mode();
	/*
	if (mode == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		level = 2047;
		if (silence_mode == 1)
			level = 0; */

	/*if ((level > 0) && (level < oplus_max_normal_brightness)) {
		bl_level = map_exp[level];
	}
	*/
	if (mode == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0)
		bl_level = 2047;
	else
		bl_level = level;

	esd_last_level = level;

	pr_info("%s bl_level: %d,%d mode = %d %d \n", __func__, bl_level,level,mode);

	//lcd cabc backlight
	if (bl_level > 4095)
		bl_level = 4095;
	bl_tb0[1] = (bl_level >> 8)& 0x0f;
	bl_tb0[2] = bl_level & 0xFF;
	esd_brightness = level;

	if (first_set_bl) {
		msleep(12);
		first_set_bl = 0;
	}

	pr_info("%s level = %d,backlight = %d,bl_tb0[1] = 0x%x,bl_tb0[2] = 0x%x\n",
		__func__, level, bl_level, bl_tb0[1], bl_tb0[2]);

	if (!cb)
		return -1;

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x0F, 0xFF};

	pr_err("%s esd_backlight = %d\n", __func__, esd_brightness);
	bl_tb0[1] =  (esd_brightness >> 8) & 0x0f;
	bl_tb0[2] = esd_brightness & 0xff;
	if (first_set_bl) {
		msleep(12);
		first_set_bl = 0;
	}
	if (!cb)
		return -1;


	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}
static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, mode);

	pr_info("drm_mode_vrefresh(m) =%d", drm_mode_vrefresh(m));

	if (ext && m && drm_mode_vrefresh(m) == 90){
		ext->params = &ext_params;
	}

#if CHANGE_FPS_EN
	else if (ext && m && drm_mode_vrefresh(m) == 50){
		ext->params = &ext_params_50hz;
	}
	else if (ext && m && drm_mode_vrefresh(m) == 60){
		ext->params = &ext_params_60hz;
	}
#endif

	else
		ret = 1;

	return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.ext_param_set = mtk_panel_ext_param_set,
	//.mode_switch = mode_switch,
	.ata_check = panel_ata_check,
	.cabc_switch = cabc_switch,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *	   become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *	  display the first valid frame after starting to receive
	 *	  video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *	   turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *		 to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;

#if CHANGE_FPS_EN
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;
#endif


	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 default_mode.hdisplay, default_mode.vdisplay,
			 drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	printk("lcm_pack_modes:mode->name[%s] mode->type[%u] htotal=%u vtotal =%u\n",
                mode->name, mode->type, mode->htotal, mode->vtotal);
	drm_mode_probed_add(connector, mode);

#if CHANGE_FPS_EN
	mode2 = drm_mode_duplicate(connector->dev, &performance_mode_50hz);
	if (!mode2) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_50hz.hdisplay, performance_mode_50hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_50hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &performance_mode_60hz);
	if (!mode3) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_60hz.hdisplay, performance_mode_60hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_60hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);
#endif
	connector->display_info.width_mm = 152;
	connector->display_info.height_mm = 248;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	unsigned int value;
	int ret;

	pr_info("%s+ lcm ,ili9883c_hkc\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET |
			MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = of_property_read_u32(dev->of_node, "gate-ic", &value);
	if (ret < 0)
		value = 0;
	else
		ctx->gate_ic = value;

	pr_info(" %d  %s,ctx->ctx->gate_ic = %d \n", __LINE__, __func__,ctx->gate_ic);

	value = 0;
	ret = of_property_read_u32(dev->of_node, "rc-enable", &value);
	if (ret < 0)
		value = 0;
	else {
		ext_params.round_corner_en = value;
#if 0
#if CHANGE_FPS_EN
		ext_params_40hz.round_corner_en = value;
		ext_params_60hz.round_corner_en = value;
#endif
#endif
	}

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}

	pr_info(" %d  %s,ctx->reset_gpio = %d \n", __LINE__, __func__,ctx->reset_gpio);

	devm_gpiod_put(dev, ctx->reset_gpio);
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_info(dev, "cannot get bias-gpios 0 %ld\n",
			 PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	pr_info(" %d  %s,ctx->bias_pos = %x \n", __LINE__, __func__,ctx->bias_pos);
	devm_gpiod_put(dev, ctx->bias_pos);

	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_info(dev, "cannot get bias-gpios 1 %ld\n",
			 PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	pr_info(" %d  %s,ctx->bias_neg = %x \n", __LINE__, __func__,ctx->bias_neg);
	devm_gpiod_put(dev, ctx->bias_neg);


	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	pr_info(" %d  %s\n", __LINE__, __func__);
	if (ret < 0)
		return ret;

#endif

	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	init_global_exp_backlight();

	/* wanhang */
	register_device_proc("lcd", "ili9883c", "truly_hkc");
	pr_info("%s- lcm,ili9883c_truly_hkc,vdo,90hz\n", __func__);

	return ret;
}

static int lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "oplus23703_ili9883c_hkc_hdp_dsi_vdo",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus23703_ili9883c_hkc_hdp_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("zhangxian <xian.zhang@tinno.com>");
MODULE_DESCRIPTION("ICETRON lcm HX83102J VDO 120HZ LCD Panel Driver");
MODULE_LICENSE("GPL v2");

