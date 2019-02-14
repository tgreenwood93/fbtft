/*
 * FB driver for the ILI9341 LCD display controller
 *
 * This display uses 9-bit SPI: Data/Command bit + 8 data bits
 * For platforms that doesn't support 9-bit, the driver is capable
 * of emulating this using 8-bit transfer.
 * This is done by transfering eight 9-bit words in 9 bytes.
 *
 * Copyright (C) 2013 Christian Vogelgsang
 * Based on adafruit22fb.c by Noralf Tronnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * Copyright and GPL statement
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>

#include "fbtft.h"

#define DRVNAME		"fb_ili9488"
#define WIDTH		320
#define HEIGHT		480

/* Level 1 Commands (from the display Datasheet) */
#define ILI9488_CMD_NOP                             0x00
#define ILI9488_CMD_SOFTWARE_RESET                  0x01
#define ILI9488_CMD_READ_DISP_ID                    0x04
#define ILI9488_CMD_READ_DSI_ERRORS                 0x05
#define ILI9488_CMD_READ_DISP_STATUS                0x09
#define ILI9488_CMD_READ_DISP_POWER_MODE            0x0A
#define ILI9488_CMD_READ_DISP_MADCTRL               0x0B
#define ILI9488_CMD_READ_DISP_PIXEL_FORMAT          0x0C
#define ILI9488_CMD_READ_DISP_IMAGE_MODE            0x0D
#define ILI9488_CMD_READ_DISP_SIGNAL_MODE           0x0E
#define ILI9488_CMD_READ_DISP_SELF_DIAGNOSTIC       0x0F
#define ILI9488_CMD_SLEEP_IN                        0x10
#define ILI9488_CMD_SLEEP_OUT                       0x11
#define ILI9488_CMD_PARTIAL_MODE_ON                 0x12
#define ILI9488_CMD_NORMAL_DISP_MODE_ON             0x13
#define ILI9488_CMD_DISP_INVERSION_OFF              0x20
#define ILI9488_CMD_DISP_INVERSION_ON               0x21
#define ILI9488_CMD_ALL_PIXELS_OFF                  0x22
#define ILI9488_CMD_ALL_PIXELS_ON                   0x23
#define ILI9488_CMD_DISPLAY_OFF                     0x28
#define ILI9488_CMD_DISPLAY_ON                      0x29
#define ILI9488_CMD_COLUMN_ADDRESS_SET              0x2A
#define ILI9488_CMD_PAGE_ADDRESS_SET                0x2B
#define ILI9488_CMD_MEMORY_WRITE                    0x2C
#define ILI9488_CMD_MEMORY_READ                     0x2E
#define ILI9488_CMD_PARTIAL_AREA                    0x30
#define ILI9488_CMD_VERT_SCROLL_DEFINITION          0x33
#define ILI9488_CMD_TEARING_EFFECT_LINE_OFF         0x34
#define ILI9488_CMD_TEARING_EFFECT_LINE_ON          0x35
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL           0x36
#define ILI9488_CMD_VERT_SCROLL_START_ADDRESS       0x37
#define ILI9488_CMD_IDLE_MODE_OFF                   0x38
#define ILI9488_CMD_IDLE_MODE_ON                    0x39
#define ILI9488_CMD_INTERFACE_PIXEL_FORMAT          0x3A
#define ILI9488_CMD_WRITE_MEMORY_CONTINUE           0x3C
#define ILI9488_CMD_READ_MEMORY_CONTINUE            0x3E
#define ILI9488_CMD_WRITE_TEAR_SCANLINE             0x44
#define ILI9488_CMD_READ_TEAR_SCANLINE              0x45
#define ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS        0x51
#define ILI9488_CMD_READ_DISPLAY_BRIGHTNESS         0x52
#define ILI9488_CMD_WRITE_CTRL_DISPLAY              0x53
#define ILI9488_CMD_READ_CTRL_DISPLAY               0x54
#define ILI9488_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS  0x55
#define ILI9488_CMD_READ_CONTENT_ADAPT_BRIGHTNESS   0x56
#define ILI9488_CMD_WRITE_MIN_CAB_LEVEL             0x5E
#define ILI9488_CMD_READ_MIN_CAB_LEVEL              0x5F
#define ILI9488_CMD_READ_BRIGHTNESS_DIAG_RESULT     0x68
#define ILI9488_CMD_READ_ID1                        0xDA
#define ILI9488_CMD_READ_ID2                        0xDB
#define ILI9488_CMD_READ_ID3                        0xDC

/* Level 2 Commands (from the display Datasheet) */
#define ILI9488_CMD_INTERFACE_MODE_CONTROL          0xB0
#define ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL       0xB1
#define ILI9488_CMD_FRAME_RATE_CONTROL_IDLE_8COLOR  0xB2
#define ILI9488_CMD_FRAME_RATE_CONTROL_PARTIAL      0xB3
#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL       0xB4
#define ILI9488_CMD_BLANKING_PORCH_CONTROL          0xB5
#define ILI9488_CMD_DISPLAY_FUNCTION_CONTROL        0xB6
#define ILI9488_CMD_ENTRY_MODE_SET                  0xB7
#define ILI9488_CMD_COLOR_ENHANCEMENT_CTRL_1        0xB9
#define ILI9488_CMD_COLOR_ENHANCEMENT_CTRL_2        0xBA
#define ILI9488_CMD_HS_LANES_CONTROL                0xBE
#define ILI9488_CMD_POWER_CONTROL_1                 0xC0
#define ILI9488_CMD_POWER_CONTROL_2                 0xC1
#define ILI9488_CMD_POWER_CONTROL_3                 0xC2
#define ILI9488_CMD_POWER_CONTROL_4                 0xC3
#define ILI9488_CMD_POWER_CONTROL_5                 0xC4
#define ILI9488_CMD_VCOM_CONTROL_1                  0xC5
#define ILI9488_CMD_CABC_CONTROL_1                  0xC6
#define ILI9488_CMD_CABC_CONTROL_2                  0xC8
#define ILI9488_CMD_CABC_CONTROL_3                  0xC9
#define ILI9488_CMD_CABC_CONTROL_4                  0xCA
#define ILI9488_CMD_CABC_CONTROL_5                  0xCB
#define ILI9488_CMD_CABC_CONTROL_6                  0xCC
#define ILI9488_CMD_CABC_CONTROL_7                  0xCD
#define ILI9488_CMD_CABC_CONTROL_8                  0xCE
#define ILI9488_CMD_CABC_CONTROL_9                  0xCF
#define ILI9488_CMD_NVMEM_WRITE                     0xD0
#define ILI9488_CMD_NVMEM_PROTECTION_KEY            0xD1
#define ILI9488_CMD_NVMEM_STATUS_READ               0xD2
#define ILI9488_CMD_READ_ID4                        0xD3
#define ILI9488_CMD_ADJUST_CONTROL_1                0xD7
#define ILI9488_CMD_READ_ID4_CHECK                  0xD8
#define ILI9488_CMD_POSITIVE_GAMMA_CTRL             0xE0
#define ILI9488_CMD_NEGATIVE_GAMMA_CTRL             0xE1
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_1         0xE2
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_2         0xE3
#define ILI9488_CMD_SET_IMAGE_FUNCTION              0xE9
#define ILI9488_CMD_ADJUST_CONTROL_2                0xF2
#define ILI9488_CMD_ADJUST_CONTROL_3                0xF7
#define ILI9488_CMD_ADJUST_CONTROL_4                0xF8
#define ILI9488_CMD_ADJUST_CONTROL_5                0xF9
#define ILI9488_CMD_SPI_READ_CMD_SETTING            0xFB
#define ILI9488_CMD_ADJUST_CONTROL_6                0xFC
#define ILI9488_CMD_ADJUST_CONTROL_7                0xFF

/*Communication Commands*/
#define ILI9488_TFTWIDTH  320
#define ILI9488_TFTHEIGHT 480

#define ILI9488_NOP     0x00
#define ILI9488_SWRESET 0x01
#define ILI9488_RDDID   0x04
#define ILI9488_RDDST   0x09

#define ILI9488_SLPIN   0x10
#define ILI9488_SLPOUT  0x11
#define ILI9488_PTLON   0x12
#define ILI9488_NORON   0x13

#define ILI9488_RDMODE  0x0A
#define ILI9488_RDMADCTL  0x0B
#define ILI9488_RDPIXFMT  0x0C
#define ILI9488_RDIMGFMT  0x0D
#define ILI9488_RDSELFDIAG  0x0F

#define ILI9488_INVOFF  0x20
#define ILI9488_INVON   0x21
#define ILI9488_GAMMASET 0x26
#define ILI9488_DISPOFF 0x28
#define ILI9488_DISPON  0x29

#define ILI9488_CASET   0x2A
#define ILI9488_PASET   0x2B
#define ILI9488_RAMWR   0x2C
#define ILI9488_RAMRD   0x2E

#define ILI9488_PTLAR   0x30
#define ILI9488_MADCTL  0x36
#define ILI9488_PIXFMT  0x3A

#define ILI9488_FRMCTR1 0xB1
#define ILI9488_FRMCTR2 0xB2
#define ILI9488_FRMCTR3 0xB3
#define ILI9488_INVCTR  0xB4
#define ILI9488_DFUNCTR 0xB6

#define ILI9488_PWCTR1  0xC0
#define ILI9488_PWCTR2  0xC1
#define ILI9488_PWCTR3  0xC2
#define ILI9488_PWCTR4  0xC3
#define ILI9488_PWCTR5  0xC4
#define ILI9488_VMCTR1  0xC5
#define ILI9488_VMCTR2  0xC7

#define ILI9488_RDID1   0xDA
#define ILI9488_RDID2   0xDB
#define ILI9488_RDID3   0xDC
#define ILI9488_RDID4   0xDD

#define ILI9488_GMCTRP1 0xE0
#define ILI9488_GMCTRN1 0xE1

static int default_init_sequence[] = {
	-1, ILI9488_CMD_POSITIVE_GAMMA_CTRL, 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A,
	0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F,
    -1, ILI9488_CMD_NEGATIVE_GAMMA_CTRL, 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05,
	0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F,
    -1, ILI9488_CMD_POWER_CONTROL_1, 0x17, 0x15,
	-1, ILI9488_CMD_POWER_CONTROL_2, 0x41,
	-1, ILI9488_CMD_VCOM_CONTROL_1, 0x00, 0x12, 0x80,
	-1, ILI9488_CMD_MEMORY_ACCESS_CONTROL, 0x48,
	-1, ILI9488_CMD_INTERFACE_PIXEL_FORMAT, 0x66,
	-1, ILI9488_CMD_INTERFACE_MODE_CONTROL, 0x80,
	-1, ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, 0xA0,
	-1, ILI9488_CMD_DISPLAY_INVERSION_CONTROL, 0x02,
	-1, ILI9488_CMD_DISPLAY_FUNCTION_CONTROL, 0x02, 0x02,
	-1, ILI9488_CMD_SET_IMAGE_FUNCTION, 0x00,
	-1, ILI9488_CMD_ADJUST_CONTROL_3, 0xA9, 0x51, 0x2C, 0x82,
	-1, ILI9488_SLPOUT,
	-1, 150,
	-1, ILI9488_DISPON,
	-2, 20,
	-3
};

static int init_display(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	printk("initalizing ili9488");

	return 0;
}

static void set_addr_win(struct fbtft_par *par, int xs, int ys, int xe, int ye)
{
	fbtft_par_dbg(DEBUG_SET_ADDR_WIN, par,
		"%s(xs=%d, ys=%d, xe=%d, ye=%d)\n", __func__, xs, ys, xe, ye);

	/* Column address */
	write_reg(par, 0x2A, xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF);

	/* Row adress */
	write_reg(par, 0x2B, ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF);

	/* Memory write */
	write_reg(par, 0x2C);
}

static int set_var(struct fbtft_par *par)
{
	fbtft_par_dbg(DEBUG_INIT_DISPLAY, par, "%s()\n", __func__);

	printk("setting window orientation");

	switch (par->info->var.rotate) {
	case 0:
		write_reg(par, 0x36, 0x80 | (par->bgr << 3));
		break;
	case 90:
		write_reg(par, 0x36, 0x20 | (par->bgr << 3));
		break;
	case 180:
		write_reg(par, 0x36, 0x40 | (par->bgr << 3));
		break;
	case 270:
		write_reg(par, 0x36, 0xE0 | (par->bgr << 3));
		break;
	default:
		break;
	}

	return 0;
}

static struct fbtft_display display = {
	.regwidth = 8,
	.width = WIDTH,
	.height = HEIGHT,
	.init_sequence = default_init_sequence,
	.fbtftops = {
		.init_display = init_display,
		.set_addr_win = set_addr_win,
		.set_var = set_var,
	},
};

/*
    This makes the driver autoload if the device is present
    modprobe will load it from /lib/modules and not the build directory
    If your driver changes doesn't come into effect, this could be why
*/
FBTFT_REGISTER_DRIVER(DRVNAME, "ilitek,ili488", &display);

MODULE_ALIAS("spi:" DRVNAME);
MODULE_ALIAS("platform:" DRVNAME);
MODULE_ALIAS("spi:ili9488");
MODULE_ALIAS("platform:ili9488");

MODULE_DESCRIPTION("FB driver for the ILI9488 LCD display controller");
MODULE_AUTHOR("Tom Greenwood");
MODULE_LICENSE("GPL");