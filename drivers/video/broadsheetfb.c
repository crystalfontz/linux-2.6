/*
 * broadsheetfb.c -- FB driver for E-Ink Broadsheet controller
 *
 * Copyright (C) 2008, Jaya Kumar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 * Layout is based on skeletonfb.c by James Simmons and Geert Uytterhoeven.
 *
 * This driver is written to be used with the Broadsheet display controller.
 *
 * It is intended to be architecture independent. A board specific driver
 * must be used to perform all the physical IO interactions.
 *
 */

#define DEBUG 1

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/firmware.h>
#include <linux/uaccess.h>

#include <video/broadsheetfb.h>

static int rot_mode = 0;
static unsigned int skip_collision_processing = 0;
static unsigned int skip_rework_damagelist = 0;
static unsigned int skip_2nd_buf = 0;
static unsigned int waveform_mode = 2;
static unsigned int do_frend = 1;
static unsigned int do_dqstuff = 1;
static unsigned int update_mode = 1;
static unsigned int panel_type = 37;
static unsigned int panel_index = 1;
static unsigned int do_test = 1;

static int wfm_timing[] = { 750, 250, 750, 750, 5 };

static void broadsheetfb_dpy_update(struct broadsheetfb_par *par);
static void broadsheetfb_upd_part(struct broadsheetfb_par *par);
static void broadsheetfb_load_image_area(struct broadsheetfb_par *par, u16 x, u16 y, u16 w, u16 h);
static void broadsheetfb_upd_full(struct broadsheetfb_par *par);

struct panel_info {
	int w;
	int h;
	u16 sdcfg;
	u16 gdcfg;
	u16 lutfmt;
	u16 fsynclen;
	u16 fendfbegin;
	u16 lsynclen;
	u16 lendlbegin;
	u16 pixclk;
	int cmaplen;
};

static struct panel_info panel_table[] = {
	{
		.w = 800,
		.h = 600,
		.sdcfg = (100 | (1 << 8) | (1 << 9)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 4,
		.fendfbegin = (10 << 8) | 4,
		.lsynclen = 10,
		.lendlbegin = (100 << 8) | 4,
		.pixclk = 6,
		.cmaplen = 16,
	},
	{
		.w = 320,
		.h = 240,
		.sdcfg = (67 | (0 << 8) | (0 << 9) | (0 << 10) | (0 << 12) ),
		.gdcfg = 3,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 0,
		.fendfbegin = (80 << 8) | 4,
		.lsynclen = 10,
		.lendlbegin = (80 << 8) | 20,
		.pixclk = 14,
		.cmaplen = 16,
	},
	{
		.w = 1200,
		.h = 825,
		.sdcfg = (100 | (1 << 8) | (1 << 9) | (0 << 10) | (0 << 12) ),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 0,
		.fendfbegin = (4 << 8) | 4,
		.lsynclen = 4,
		.lendlbegin = (60 << 8) | 10,
		.pixclk = 3,
		.cmaplen = 16,
	},
};

/* Display specific information */
#define DPY_W 320
#define DPY_H 240
#define MAX_OLAP 800 /* number of maximum overlaping pixels we can ignore */

static struct fb_fix_screeninfo broadsheetfb_fix __devinitdata = {
	.id =		"broadsheetfb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_STATIC_PSEUDOCOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.line_length =	DPY_W,
	.accel =	FB_ACCEL_NONE,
};

static struct fb_var_screeninfo broadsheetfb_var = {
	.xres		= DPY_W,
	.yres		= DPY_H,
	.xres_virtual	= DPY_W,
	.yres_virtual	= DPY_H,
	.bits_per_pixel	= 8,
	.grayscale	= 1,
	.red =		{ 0, 4, 0 },
	.green =	{ 0, 0, 0 },
	.blue =		{ 0, 0, 0 },
	.transp =	{ 0, 0, 0 },
};

static struct fb_var_screeninfo broadsheetfb_var_565 = {
	.xres		= DPY_W,
	.yres		= DPY_H,
	.xres_virtual	= DPY_W,
	.yres_virtual	= DPY_H,
	.bits_per_pixel	= 16,
	.grayscale	= 0,
	.red =		{ 11, 5, 0 },
	.green =	{  5, 6, 0 },
	.blue =		{  0, 5, 0 },
	.transp =	{ 0, 0, 0 },
};


/* main broadsheetfb functions */
static void broadsheet_gpio_issue_data(struct broadsheetfb_par *par, u16 data)
{
	par->board->set_ctl(par, BS_WR, 0);
	par->board->set_hdb(par, data);
	par->board->set_ctl(par, BS_WR, 1);
}

static void broadsheet_gpio_issue_data_strict(struct broadsheetfb_par *par, u16 data)
{
	par->board->set_ctl(par, BS_DC, 1);
	par->board->set_ctl(par, BS_CS, 0);
	par->board->set_ctl(par, BS_WR, 0);
	par->board->set_hdb(par, data);
	par->board->set_ctl(par, BS_WR, 1);
	par->board->set_ctl(par, BS_CS, 1);
}

static void broadsheet_issue_data(struct broadsheetfb_par *par, u16 data)
{
	if (par->board->mmio_write)
		par->board->mmio_write(par, BS_MMIO_DATA, data);
	else
		broadsheet_gpio_issue_data(par, data);
}

static void broadsheet_issue_data_strict(struct broadsheetfb_par *par, u16 data)
{
	if (par->board->mmio_write)
		par->board->mmio_write(par, BS_MMIO_DATA, data);
	else
		broadsheet_gpio_issue_data_strict(par, data);
}


static void broadsheet_gpio_issue_cmd(struct broadsheetfb_par *par, u16 data)
{
	par->board->set_ctl(par, BS_DC, 0);
	broadsheet_gpio_issue_data(par, data);
}

static void broadsheet_gpio_send_command(struct broadsheetfb_par *par, u16 data)
{
	par->board->wait_for_rdy(par);

	par->board->set_ctl(par, BS_CS, 0);
	broadsheet_gpio_issue_cmd(par, data);
	par->board->set_ctl(par, BS_DC, 1);
	par->board->set_ctl(par, BS_CS, 1);
}

static void broadsheet_gpio_send_cmdargs(struct broadsheetfb_par *par, u16 cmd,
					int argc, u16 *argv)
{
	int i;

	par->board->wait_for_rdy(par);

	par->board->set_ctl(par, BS_CS, 0);
	broadsheet_gpio_issue_cmd(par, cmd);
	par->board->set_ctl(par, BS_DC, 1);

	for (i = 0; i < argc; i++)
		broadsheet_gpio_issue_data(par, argv[i]);
	par->board->set_ctl(par, BS_CS, 1);
}

static void broadsheet_mmio_send_cmdargs(struct broadsheetfb_par *par, u16 cmd,
				    int argc, u16 *argv)
{
	int i;

	par->board->mmio_write(par, BS_MMIO_CMD, cmd);

	for (i = 0; i < argc; i++)
		par->board->mmio_write(par, BS_MMIO_DATA, argv[i]);
}

static void broadsheet_send_command(struct broadsheetfb_par *par, u16 data)
{
	if (par->board->mmio_write)
		par->board->mmio_write(par, BS_MMIO_CMD, data);
	else
		broadsheet_gpio_send_command(par, data);
}

static void broadsheet_send_cmdargs(struct broadsheetfb_par *par, u16 cmd,
				    int argc, u16 *argv)
{
	if (par->board->mmio_write)
		broadsheet_mmio_send_cmdargs(par, cmd, argc, argv);
	else
		broadsheet_gpio_send_cmdargs(par, cmd, argc, argv);
}

static void broadsheet_gpio_burst_write(struct broadsheetfb_par *par, int size,
					u16 *data)
{
	int i;
	u16 tmp;
	u16 srcval;
	u16 r, g, b;

	par->board->set_ctl(par, BS_CS, 0);
	par->board->set_ctl(par, BS_DC, 1);
	if (par->bypp == 2) {
		for (i = 0; i < size; i+=2) {
			par->board->set_ctl(par, BS_WR, 0);
			switch (waveform_mode) {
			case 3:
			case 2:
				srcval = data[i];
				r = (srcval & 0xF800) >> 11;
				g = (srcval & 0x07E0) >> 5;
				b = (srcval & 0x001F) >> 0;
				tmp = (((r*11) + (g*8) + (b*5)) >> 2 ) & 0x00F0;
				srcval = data[i+1];
				r = (srcval & 0xF800) >> 11;
				g = (srcval & 0x07E0) >> 5;
				b = (srcval & 0x001F) >> 0;
				tmp |= ((((r*11) + (g*8) + (b*5)) >> 2 ) & 0x00F0) << 8;
	#if 0
				if ((srcval != 0) && (srcval != 0xFFFF)) {
				if (!(count++ % 1024)) {
					printk("src=0x%x,r=%d,g=%d,b=%d,tmp=0x%x\n", srcval, r, g, b, tmp);
				} 
				}
	#endif
				break;
			case 1:
			case 4:
				srcval = data[i];
				r = (srcval & 0xF800) >> 11;
				g = (srcval & 0x07E0) >> 5;
				b = (srcval & 0x001F) >> 0;
				tmp = (((((r*11) + (g*8) + (b*5)) >> 2 ) & 0x00F0) >= 8) ? 0xF0 : 0x00;
				srcval = data[i+1];
				r = (srcval & 0xF800) >> 11;
				g = (srcval & 0x07E0) >> 5;
				b = (srcval & 0x001F) >> 0;
				tmp |= ((((((r*11) + (g*8) + (b*5)) >> 2 ) & 0x00F0) >= 8) ? 0xF0 : 0x00) << 8;
				break;
			}
			par->board->set_hdb(par, tmp);
			par->board->set_ctl(par, BS_WR, 1);
		}
	} else {
		for (i = 0; i < size/2; i++) {
			par->board->set_ctl(par, BS_WR, 0);
			switch (waveform_mode) {
			case 3:
			case 2:
				tmp = (data[i] & 0x0F) << 4;
				tmp |= (data[i] & 0x0F00) << 4;
				break;
			case 1:
			case 4:
				tmp = (((data[i] & 0x0F) > 8)) ? 0xF : 0x0 << 4;
				tmp |= ((data[i] & 0x0F00) > 0x0800) ? 0x0F00 : 0x0000;
				break;
			}
			par->board->set_hdb(par, tmp);
			par->board->set_ctl(par, BS_WR, 1);
		}
	}
	par->board->set_ctl(par, BS_CS, 1);
}

static void broadsheet_mmio_burst_write(struct broadsheetfb_par *par, int size,
				   u16 *data)
{
	int i;
	u16 tmp;

	for (i = 0; i < size; i++) {
		tmp = (data[i] & 0x0F) << 4;
		tmp |= (data[i] & 0x0F00) << 4;
		par->board->mmio_write(par, BS_MMIO_DATA, tmp);
	}

}

static void broadsheet_burst_write(struct broadsheetfb_par *par, int size,
				   u16 *data)
{
	if (par->board->mmio_write)
		broadsheet_mmio_burst_write(par, size, data);
	else
		broadsheet_gpio_burst_write(par, size, data);
}

static u16 broadsheet_gpio_get_data(struct broadsheetfb_par *par)
{
	u16 res;
	/* wait for ready to go hi. (lo is busy) */
	par->board->wait_for_rdy(par);

	/* cs lo, dc lo for cmd, we lo for each data, db as usual */
	par->board->set_ctl(par, BS_DC, 1);
	par->board->set_ctl(par, BS_CS, 0);
	par->board->set_ctl(par, BS_WR, 0);

	res = par->board->get_hdb(par);

	/* strobe wr */
	par->board->set_ctl(par, BS_WR, 1);
	par->board->set_ctl(par, BS_CS, 1);

	return res;
}


static u16 broadsheet_get_data(struct broadsheetfb_par *par)
{
	if (par->board->mmio_read)
		return par->board->mmio_read(par);
	else
		return broadsheet_gpio_get_data(par);
}

static void broadsheet_gpio_write_reg(struct broadsheetfb_par *par, u16 reg,
					u16 data)
{
	/* wait for ready to go hi. (lo is busy) */
	par->board->wait_for_rdy(par);

	/* cs lo, dc lo for cmd, we lo for each data, db as usual */
	par->board->set_ctl(par, BS_CS, 0);

	broadsheet_gpio_issue_cmd(par, BS_CMD_WR_REG);

	par->board->set_ctl(par, BS_DC, 1);

	broadsheet_gpio_issue_data_strict(par, reg);
	broadsheet_gpio_issue_data_strict(par, data);

	par->board->set_ctl(par, BS_CS, 1);
}

static void broadsheet_mmio_write_reg(struct broadsheetfb_par *par, u16 reg,
				 u16 data)
{
	par->board->mmio_write(par, BS_MMIO_CMD, BS_CMD_WR_REG);
	par->board->mmio_write(par, BS_MMIO_DATA, reg);
	par->board->mmio_write(par, BS_MMIO_DATA, data);

}

static void broadsheet_write_reg(struct broadsheetfb_par *par, u16 reg,
					u16 data)
{
	if (par->board->mmio_write)
		broadsheet_mmio_write_reg(par, reg, data);
	else
		broadsheet_gpio_write_reg(par, reg, data);
}

static void broadsheet_write_reg32(struct broadsheetfb_par *par, u16 reg,
					u32 data)
{
	broadsheet_write_reg(par, reg, cpu_to_le32(data) & 0xFFFF);
	broadsheet_write_reg(par, reg + 2, (cpu_to_le32(data) >> 16) & 0xFFFF);
}

static u16 broadsheet_read_reg(struct broadsheetfb_par *par, u16 reg)
{
	broadsheet_send_cmdargs(par, BS_CMD_RD_REG, 1, &reg);
	par->board->wait_for_rdy(par);
	return broadsheet_get_data(par);
}

/* functions for waveform manipulation */
static int is_broadsheet_pll_locked(struct broadsheetfb_par *par)
{
	return broadsheet_read_reg(par, 0x000A) & 0x0001;
}

static int broadsheet_setup_plls(struct broadsheetfb_par *par)
{
	int retry_count = 0;
	u16 tmp;

	/* disable arral saemipu mode */
	broadsheet_write_reg(par, 0x0006, 0x0000);

	broadsheet_write_reg(par, 0x0010, 0x0004);
	broadsheet_write_reg(par, 0x0012, 0x5949);
	broadsheet_write_reg(par, 0x0014, 0x0040);
	broadsheet_write_reg(par, 0x0016, 0x0000);

	do {
		if (retry_count++ > 100)
			return -ETIMEDOUT;
		mdelay(1);
	} while (!is_broadsheet_pll_locked(par));

	tmp = broadsheet_read_reg(par, 0x0006);
	tmp &= ~0x1;
	broadsheet_write_reg(par, 0x0006, tmp);

	return 0;
}

static int broadsheet_setup_spi(struct broadsheetfb_par *par)
{

	broadsheet_write_reg(par, 0x0204, ((3 << 3) | 1));
	broadsheet_write_reg(par, 0x0208, 0x0001);

	return 0;
}

static int broadsheet_setup_spiflash(struct broadsheetfb_par *par,
						u16 *orig_sfmcd)
{

	*orig_sfmcd = broadsheet_read_reg(par, 0x0204);
	broadsheet_write_reg(par, 0x0208, 0);
	broadsheet_write_reg(par, 0x0204, 0);
	broadsheet_write_reg(par, 0x0204, ((3 << 3) | 1));

	return 0;
}

static int broadsheet_wait_for_bit(struct broadsheetfb_par *par,
						u16 reg, int bitnum, int val,
						int timeout)
{
	u16 tmp;

	do {
		tmp = broadsheet_read_reg(par, reg);
		if (((tmp >> bitnum) & 1) == val)
			return 0;
		mdelay(1);
	} while (timeout--);

	return -ETIMEDOUT;
}

static int broadsheet_spiflash_write_byte(struct broadsheetfb_par *par, u8 data)
{
	broadsheet_write_reg(par, 0x0202, (data | 0x100));

	return broadsheet_wait_for_bit(par, 0x0206, 3, 0, 100);
}

static int broadsheet_spiflash_read_byte(struct broadsheetfb_par *par, u8 *data)
{
	int err;
	u16 tmp;

	broadsheet_write_reg(par, 0x0202, 0);

	err = broadsheet_wait_for_bit(par, 0x0206, 3, 0, 100);
	if (err)
		return err;

	tmp = broadsheet_read_reg(par, 0x200);

	*data = tmp & 0xFF;

	return 0;
}

static int broadsheet_spiflash_wait_for_status(struct broadsheetfb_par *par,
								int timeout)
{
	u8 tmp;
	int err;

	do {
		broadsheet_write_reg(par, 0x0208, 1);

		err = broadsheet_spiflash_write_byte(par, 0x05);
		if (err)
			goto failout;

		err = broadsheet_spiflash_read_byte(par, &tmp);
		if (err)
			goto failout;

		broadsheet_write_reg(par, 0x0208, 0);

		if (!(tmp & 0x1))
			return 0;

		mdelay(5);
	} while (timeout--);

	dev_err(par->info->device, "Timed out waiting for spiflash status\n");
	return -ETIMEDOUT;

failout:
	broadsheet_write_reg(par, 0x0208, 0);
	return err;
}

static int broadsheet_spiflash_op_on_address(struct broadsheetfb_par *par,
							u8 op, u32 addr)
{
	int i;
	u8 tmp;
	int err;

	broadsheet_write_reg(par, 0x0208, 1);

	err = broadsheet_spiflash_write_byte(par, op);
	if (err)
		return err;

	for (i = 2; i >= 0; i--) {
		tmp = ((addr >> (i * 8)) & 0xFF);
		err = broadsheet_spiflash_write_byte(par, tmp);
		if (err)
			return err;
	}

	return err;
}

static int broadsheet_verify_spiflash(struct broadsheetfb_par *par,
						int *flash_type)
{
	int err = 0;
	u8 sig;

	err = broadsheet_spiflash_op_on_address(par, 0xAB, 0x00000000);
	if (err)
		goto failout;

	err = broadsheet_spiflash_read_byte(par, &sig);
	if (err)
		goto failout;

	if ((sig != 0x10) && (sig != 0x11)) {
		dev_err(par->info->device, "Unexpected flash type\n");
		err = -EINVAL;
		goto failout;
	}

	*flash_type = sig;

failout:
	broadsheet_write_reg(par, 0x0208, 0);
	return err;
}

static int broadsheet_setup_for_wfm_write(struct broadsheetfb_par *par,
					u16 *initial_sfmcd, int *flash_type)

{
	int err;

	err = broadsheet_setup_plls(par);
	if (err)
		return err;

	broadsheet_write_reg(par, 0x0106, 0x0203);

	err = broadsheet_setup_spi(par);
	if (err)
		return err;

	err = broadsheet_setup_spiflash(par, initial_sfmcd);
	if (err)
		return err;

	return broadsheet_verify_spiflash(par, flash_type);
}

static int broadsheet_spiflash_write_control(struct broadsheetfb_par *par,
						int mode)
{
	int err;

	broadsheet_write_reg(par, 0x0208, 1);
	if (mode)
		err = broadsheet_spiflash_write_byte(par, 0x06);
	else
		err = broadsheet_spiflash_write_byte(par, 0x04);

	broadsheet_write_reg(par, 0x0208, 0);
	return err;
}

static int broadsheet_spiflash_erase_sector(struct broadsheetfb_par *par,
						int addr)
{
	int err;

	broadsheet_spiflash_write_control(par, 1);

	err = broadsheet_spiflash_op_on_address(par, 0xD8, addr);

	broadsheet_write_reg(par, 0x0208, 0);

	if (err)
		return err;

	err = broadsheet_spiflash_wait_for_status(par, 1000);

	return err;
}

static int broadsheet_spiflash_read_range(struct broadsheetfb_par *par,
						int addr, int size, char *data)
{
	int err;
	int i;

	err = broadsheet_spiflash_op_on_address(par, 0x03, addr);
	if (err)
		goto failout;

	for (i = 0; i < size; i++) {
		err = broadsheet_spiflash_read_byte(par, &data[i]);
		if (err)
			goto failout;
	}

failout:
	broadsheet_write_reg(par, 0x0208, 0);
	return err;
}

#define BS_SPIFLASH_PAGE_SIZE 256
static int broadsheet_spiflash_write_page(struct broadsheetfb_par *par,
						int addr, const char *data)
{
	int err;
	int i;

	broadsheet_spiflash_write_control(par, 1);

	err = broadsheet_spiflash_op_on_address(par, 0x02, addr);
	if (err)
		goto failout;

	for (i = 0; i < BS_SPIFLASH_PAGE_SIZE; i++) {
		err = broadsheet_spiflash_write_byte(par, data[i]);
		if (err)
			goto failout;
	}

	broadsheet_write_reg(par, 0x0208, 0);

	err = broadsheet_spiflash_wait_for_status(par, 100);

failout:
	return err;
}

static int broadsheet_spiflash_write_sector(struct broadsheetfb_par *par,
				int addr, const char *data, int sector_size)
{
	int i;
	int err;

	for (i = 0; i < sector_size; i += BS_SPIFLASH_PAGE_SIZE) {
		err = broadsheet_spiflash_write_page(par, addr + i, &data[i]);
		if (err)
			return err;
	}
	return 0;
}

/*
 * The caller must guarantee that the data to be rewritten is entirely
 * contained within this sector. That is, data_start_addr + data_len
 * must be less than sector_start_addr + sector_size.
 */
static int broadsheet_spiflash_rewrite_sector(struct broadsheetfb_par *par,
					int sector_size, int data_start_addr,
					int data_len, const char *data)
{
	int err;
	char *sector_buffer;
	int tail_start_addr;
	int start_sector_addr;

	sector_buffer = kzalloc(sizeof(char)*sector_size, GFP_KERNEL);
	if (!sector_buffer)
		return -ENOMEM;

	/* the start address of the sector is the 0th byte of that sector */
	start_sector_addr = (data_start_addr / sector_size) * sector_size;

	/*
	 * check if there is head data that we need to readback into our sector
	 * buffer first
	 */
	if (data_start_addr != start_sector_addr) {
		/*
		 * we need to read every byte up till the start address of our
		 * data and we put it into our sector buffer.
		 */
		err = broadsheet_spiflash_read_range(par, start_sector_addr,
						data_start_addr, sector_buffer);
		if (err)
			return err;
	}

	/* now we copy our data into the right place in the sector buffer */
	memcpy(sector_buffer + data_start_addr, data, data_len);

	/*
	 * now we check if there is a tail section of the sector that we need to
	 * readback.
	 */
	tail_start_addr = (data_start_addr + data_len) % sector_size;

	if (tail_start_addr) {
		int tail_len;

		tail_len = sector_size - tail_start_addr;

		/* now we read this tail into our sector buffer */
		err = broadsheet_spiflash_read_range(par, tail_start_addr,
			tail_len, sector_buffer + tail_start_addr);
		if (err)
			return err;
	}

	/* if we got here we have the full sector that we want to rewrite. */

	/* first erase the sector */
	err = broadsheet_spiflash_erase_sector(par, start_sector_addr);
	if (err)
		return err;

	/* now write it */
	err = broadsheet_spiflash_write_sector(par, start_sector_addr,
					sector_buffer, sector_size);
	return err;
}

static int broadsheet_write_spiflash(struct broadsheetfb_par *par, u32 wfm_addr,
				const u8 *wfm, int bytecount, int flash_type)
{
	int sector_size;
	int err;
	int cur_addr;
	int writecount;
	int maxlen;
	int offset = 0;

	switch (flash_type) {
	case 0x10:
		sector_size = 32*1024;
		break;
	case 0x11:
	default:
		sector_size = 64*1024;
		break;
	}

	while (bytecount) {
		cur_addr = wfm_addr + offset;
		maxlen = roundup(cur_addr, sector_size) - cur_addr;
		writecount = min(bytecount, maxlen);

		err = broadsheet_spiflash_rewrite_sector(par, sector_size,
				cur_addr, writecount, wfm + offset);
		if (err)
			return err;

		offset += writecount;
		bytecount -= writecount;
	}

	return 0;
}

static int broadsheet_store_waveform_to_spiflash(struct broadsheetfb_par *par,
						const u8 *wfm, size_t wfm_size)
{
	int err = 0;
	u16 initial_sfmcd = 0;
	int flash_type = 0;

	err = broadsheet_setup_for_wfm_write(par, &initial_sfmcd, &flash_type);
	if (err)
		goto failout;

	err = broadsheet_write_spiflash(par, 0x886, wfm, wfm_size, flash_type);

failout:
	broadsheet_write_reg(par, 0x0204, initial_sfmcd);
	return err;
}

static ssize_t broadsheet_loadstore_waveform(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t len)
{
	int err;
	struct fb_info *info = dev_get_drvdata(dev);
	struct broadsheetfb_par *par = info->par;
	const struct firmware *fw_entry;

	if (len < 1)
		return -EINVAL;

	err = request_firmware(&fw_entry, "broadsheet.wbf", dev);
	if (err < 0) {
		dev_err(dev, "Failed to get broadsheet waveform\n");
		goto err_failed;
	}

	/* try to enforce reasonable min max on waveform */
	if ((fw_entry->size < 8*1024) || (fw_entry->size > 64*1024)) {
		dev_err(dev, "Invalid waveform\n");
		err = -EINVAL;
		goto err_failed;
	}

	mutex_lock(&(par->io_lock));
	err = broadsheet_store_waveform_to_spiflash(par, fw_entry->data,
							fw_entry->size);

	mutex_unlock(&(par->io_lock));
	if (err < 0) {
		dev_err(dev, "Failed to store broadsheet waveform\n");
		goto err_failed;
	}

	dev_info(dev, "Stored broadsheet waveform, size %d\n", fw_entry->size);

	return len;

err_failed:
	return err;
}
static DEVICE_ATTR(loadstore_waveform, S_IWUSR, NULL,
			broadsheet_loadstore_waveform);

static int broadsheet_set_rotmode(struct broadsheetfb_par *par, int mode)
{
	u16 args[1];

	args[0] = mode << 8;
	broadsheet_send_cmdargs(par, BS_CMD_INIT_ROTMODE, 1, args);
	rot_mode = mode;
	return 0;
}

static void (*broadsheet_set_ext_temp)(struct broadsheetfb_par *par);

static void __null_set_ext_temp(struct broadsheetfb_par *par)
{
}

static void __real_set_ext_temp(struct broadsheetfb_par *par)
{
	broadsheet_write_reg(par, 0x322, par->board->get_ext_temp());
}

static void broadsheetfb_set_imgupd_addr(struct broadsheetfb_par *par)
{
	u16 args[5];
	u32 startaddr;

	return;
	if (par->which_dqueue) {
		startaddr=0xEA600;
	} else {
		startaddr=0xEA600 + (par->info->var.xres * par->info->var.yres * 2);
	}
	args[0] = startaddr & 0xFFFF;
	args[1] = startaddr >> 16;
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG_SETADR, 2, args);
	broadsheet_send_cmdargs(par, BS_CMD_UPD_SET_IMGADR, 2, args);
	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
	par->board->wait_for_rdy(par);
}
static void broadsheetfb_set_upd_addr(struct broadsheetfb_par *par)
{
	u16 args[5];
	u32 startaddr;

	return;
	if (par->which_dqueue) {
		startaddr=0xEA600;
	} else {
		startaddr=0xEA600 + (par->info->var.xres * par->info->var.yres* 2);
	}
	args[0] = startaddr & 0xFFFF;
	args[1] = startaddr >> 16;
	broadsheet_send_cmdargs(par, BS_CMD_UPD_SET_IMGADR, 2, args);
	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
	par->board->wait_for_rdy(par);
}
static void __devinit broadsheet_init_display(struct broadsheetfb_par *par)
{
	u16 args[5];

	args[0] = panel_table[par->panel_index].w;
	args[1] = panel_table[par->panel_index].h;
	args[2] = panel_table[par->panel_index].sdcfg;
	args[3] = panel_table[par->panel_index].gdcfg;
	args[4] = panel_table[par->panel_index].lutfmt;
	broadsheet_send_cmdargs(par, BS_CMD_INIT_DSPE_CFG, 5, args);

	/* did the controller really set it? */
	broadsheet_send_cmdargs(par, BS_CMD_INIT_DSPE_CFG, 5, args);

	args[0] = panel_table[par->panel_index].fsynclen;
	args[1] =  panel_table[par->panel_index].fendfbegin;
	args[2] =  panel_table[par->panel_index].lsynclen;
	args[3] =  panel_table[par->panel_index].lendlbegin;
	args[4] =  panel_table[par->panel_index].pixclk;
	broadsheet_send_cmdargs(par, BS_CMD_INIT_DSPE_TMG, 5, args);

	broadsheet_write_reg(par, 0x310, (par->info->var.xres * par->info->var.yres * 2) & 0xFFFF);
	broadsheet_write_reg(par, 0x312, ((par->info->var.xres * par->info->var.yres * 2) >> 16) & 0xFFFF);
	/* setup waveform */
	args[0] = 0x886;
	args[1] = 0;
	broadsheet_send_cmdargs(par, BS_CMD_RD_WFM_INFO, 2, args);

	broadsheet_send_command(par, BS_CMD_UPD_GDRV_CLR);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

	broadsheet_set_rotmode(par, rot_mode);

	broadsheet_write_reg(par, 0x330, 0x84);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
#if 0
	broadsheetfb_set_imgupd_addr(par);
#endif
	args[0] = (0x3 << 4);
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG, 1, args);

	args[0] = 0x154;
	broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);

	broadsheet_burst_write(par, (panel_table[par->panel_index].w * panel_table[par->panel_index].h),
				(u16 *) par->info->screen_base);

	broadsheet_send_command(par, BS_CMD_LD_IMG_END);

	broadsheet_set_ext_temp(par);

	args[0] = 0x4000 | (waveform_mode << 8);
	broadsheet_send_cmdargs(par, BS_CMD_UPD_FULL, 1, args);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);

	par->board->wait_for_rdy(par);
}

static void __devinit broadsheet_identify(struct broadsheetfb_par *par)
{
	u16 rev, prc;
	struct device *dev = par->info->device;

	rev = broadsheet_read_reg(par, BS_REG_REV);
	prc = broadsheet_read_reg(par, BS_REG_PRC);
	dev_info(dev, "Broadsheet Rev 0x%x, Product Code 0x%x\n", rev, prc);

	if (prc != 0x0047)
		dev_warn(dev, "Unrecognized Broadsheet Product Code\n");
	if (rev != 0x0100)
		dev_warn(dev, "Unrecognized Broadsheet Revision\n");
}

static void __devinit broadsheet_init(struct broadsheetfb_par *par)
{
 	int j;
 	int dpyw, dpyh;

	/* pre sys run board init, power supplies, etc */
	if (par->board->init_pre_run) {
		par->board->init_pre_run(par);
	}

	broadsheet_send_command(par, BS_CMD_INIT_SYS_RUN);
	/* the controller needs a second */
	msleep(1000);

	/* external temperate routine */
	if (par->board->get_ext_temp) {
		broadsheet_write_reg(par, 0x320, 0x1);
		broadsheet_set_ext_temp = __real_set_ext_temp;
	} else {
		broadsheet_set_ext_temp = __null_set_ext_temp;
	}

	broadsheet_init_display(par);
 	if (!do_test)
 		return;

 	/* draw white 0xFF */
 	memset(par->info->screen_base, 0xff, par->info->fix.smem_len);
 	broadsheetfb_dpy_update(par);
 	printk(KERN_ERR "white\n");
	msleep(1000);
 	/* draw black 0x00 */
 	memset(par->info->screen_base, 0x00, par->info->fix.smem_len);
 	broadsheetfb_dpy_update(par);
 	printk(KERN_ERR "black\n");
	msleep(1000);
#if 0
 	/* draw checker 0x00 */
	dpyw = panel_table[par->panel_index].w;
	dpyh = panel_table[par->panel_index].h;
 	for (j=0; j < 1; j++) {
 	{
 		int x;
 		for (x=0; x < dpyw*dpyh; x++) {
 			if ((x % 8) > 3) { 
 				*(par->info->screen_base + x) = 0xff;
 			} else {
 				*(par->info->screen_base + x) = 0x00;
 			}
 		}
 	}
 	broadsheetfb_dpy_update(par);
 	printk(KERN_ERR "checker\n");
 	msleep(1000);
 	}
 	for (j=0; j < 1; j++) {
 		msleep(1000);
 	}
 	/* draw checker 0x00 */
 	for (j=0; j < 2; j++) {
 	{
 		int x;
 		int y;
 		for (y=0; y < dpyh; y++) {
 			for (x=0; x < dpyw; x++) {
 				if (x < (32 - 1)) {
 					*(par->info->screen_base + y*dpyw + x) = 0xff;
 				} else if ((x) < (32*2 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xee;
 				} else if ((x) < (32*3 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xdd;
 				} else if ((x) < (32*4 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xcc;
 				} else if ((x) < (32*5 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xaa;
 				} else if ((x) < (32*6 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x88;
 				} else if ((x) < (32*7 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x66;
 				} else if ((x) < (32*8 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x44;
 				} else if ((x) < (32*9 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x22;
 				} else if ((x) < (32*10 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x00;
 				}
 			}
 		}
 	}
 #if 0
 	broadsheetfb_dpy_update(par);
 #endif
 	broadsheetfb_load_image_area(par, 0,0, dpyw, dpyh);
 	broadsheetfb_upd_full(par);
 	printk(KERN_ERR "checker 2\n");
 	msleep(1000);
 	msleep(1000);
 	}
 
 	/* checker 3 */
 	for (j=0; j < 2; j++) {
 	{
 		int x;
 		int y;
 		for (y=0; y < dpyh; y++) {
 			for (x=0; x < dpyw; x++) {
 #if 0
 				if (y < 8) {
 					*(par->info->screen_base + y*dpyw + x) = 0xff;
 				} else {
 					*(par->info->screen_base + y*dpyw + x) = 0x00;
 				}
 #endif
 #if 1
 				if (y < (24 - 1)) {
 					*(par->info->screen_base + y*dpyw + x) = 0xff;
 				} else if ((y) < (24*2 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xee;
 				} else if ((y) < (24*3 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xdd;
 				} else if ((y) < (24*4 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xcc;
 				} else if ((y) < (24*5 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0xaa;
 				} else if ((y) < (24*6 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x88;
 				} else if ((y) < (24*7 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x66;
 				} else if ((y) < (24*8 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x44;
 				} else if ((y) < (24*9 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x22;
 				} else if ((y) < (24*10 - 1)) { 
 					*(par->info->screen_base + y*dpyw + x) = 0x00;
 				}
 #endif
 			}
 		}
 	}
 #if 0
 	broadsheetfb_dpy_update(par);
 #endif
 	broadsheetfb_load_image_area(par, 0,0, dpyw, dpyh);
 	broadsheetfb_upd_full(par);
 	printk(KERN_ERR "checker 3\n");
 	msleep(1000);
 	msleep(1000);
 	msleep(1000);
 	msleep(1000);
 	msleep(1000);
 	}
#endif
 
}

static void broadsheetfb_dpy_update_pages(struct broadsheetfb_par *par,
						u16 y1, u16 y2)
{
	u16 args[5];
	unsigned char *buf = (unsigned char *)par->info->screen_base;
	if (do_frend) {	
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
	}
	mutex_lock(&(par->io_lock));
	/* y1 must be a multiple of 4 so drop the lower bits */
	y1 &= 0xFFFC;
	args[0] = 0x3 << 4;
	args[1] = 0;
	args[2] = y1;
	args[3] = cpu_to_le16(par->info->var.xres);
	args[4] = y2;
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG_AREA, 5, args);

	if (y2 < y1) {
		printk(KERN_ERR "y2 %d less than y1 %d, FAILURE\n", y2, y1);
		mutex_unlock(&(par->io_lock));
		return;
	}
	args[0] = 0x154;
	broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);

	buf += y1 * par->info->var.xres * par->bypp;
	broadsheet_burst_write(par, ((1 + y2 - y1) * par->info->var.xres),
				(u16 *) buf);

	broadsheet_send_command(par, BS_CMD_LD_IMG_END);

	broadsheet_set_ext_temp(par);

	args[0] = 0x4000 | (waveform_mode << 8);
	if (update_mode) {
		broadsheet_send_cmdargs(par, BS_CMD_UPD_PART, 1, args);
	} else {
		broadsheet_send_cmdargs(par, BS_CMD_UPD_FULL, 1, args);
	}

	par->board->wait_for_rdy(par);
	mutex_unlock(&(par->io_lock));

}

static void broadsheetfb_dpy_update(struct broadsheetfb_par *par)
{
	u16 args[5];
#if 0
	broadsheetfb_set_imgupd_addr(par);
#endif
	mutex_lock(&(par->io_lock));
	args[0] = 0x3 << 4;
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG, 1, args);

	args[0] = 0x154;
	broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);
	broadsheet_burst_write(par, (panel_table[par->panel_index].w *
					panel_table[par->panel_index].h),
					(u16 *) par->info->screen_base);

	broadsheet_send_command(par, BS_CMD_LD_IMG_END);

	broadsheet_set_ext_temp(par);

	args[0] = 0x4000 | (waveform_mode << 8);
	if (update_mode) {
		broadsheet_send_cmdargs(par, BS_CMD_UPD_PART, 1, args);
	} else {
		broadsheet_send_cmdargs(par, BS_CMD_UPD_FULL, 1, args);
	}

	if (do_frend) {
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);

		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
	}

	par->board->wait_for_rdy(par);

	mutex_unlock(&(par->io_lock));
}

static void broadsheetfb_upd_full(struct broadsheetfb_par *par)
{
	u16 args[5];

	if (waveform_mode != 1) {
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
		par->board->wait_for_rdy(par);
	}
	args[0] = 0x4000 | (waveform_mode << 8);
	broadsheet_send_cmdargs(par, BS_CMD_UPD_FULL, 1, args);

	if (waveform_mode != 1) {
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
		par->board->wait_for_rdy(par);
	}
}

static void broadsheetfb_upd_part(struct broadsheetfb_par *par)
{
	u16 args[5];

	if (do_frend) {
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
		par->board->wait_for_rdy(par);
	}
	args[0] = 0x4000 | (waveform_mode << 8);
	broadsheet_send_cmdargs(par, BS_CMD_UPD_PART, 1, args);
	if (do_frend) {
		broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
		par->board->wait_for_rdy(par);
	}
}

static void broadsheetfb_load_image(struct broadsheetfb_par *par)
{
	u16 args[5];

	mutex_lock(&(par->io_lock));
	args[0] = 0x3 << 4;
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG, 1, args);

	args[0] = 0x154;
	broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);
	broadsheet_burst_write(par,
				par->info->var.xres * par->info->var.yres,
				(u16 *) par->info->screen_base);

	broadsheet_send_command(par, BS_CMD_LD_IMG_END);
	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
	par->board->wait_for_rdy(par);
	mutex_unlock(&(par->io_lock));
}



static void broadsheetfb_load_image_area(struct broadsheetfb_par *par, u16 x,
						u16 y, u16 w, u16 h)
{
	u16 args[5];
	unsigned char *sbuf = (unsigned char *)par->info->screen_base;
	unsigned char *buf;
	int j;

	/* x must be a multiple of 4 so drop the lower bits */
	w += x - (x & 0xFFFE);
	x &= 0xFFFE;
	w += (w % 2);

	args[0] = 0x3 << 4;
	args[1] = x;
	args[2] = y;
	args[3] = w;
	args[4] = h;
	broadsheet_send_cmdargs(par, BS_CMD_LD_IMG_AREA, 5, args);

//	printk(KERN_INFO "%s:%d %d,%d,%d,%d bypp=%d\n", __func__, __LINE__, x,y,w,h, par->bypp);
	args[0] = 0x154;
	broadsheet_send_cmdargs(par, BS_CMD_WR_REG, 1, args);

	for (j = y; j < y + h; j++) {
		buf = sbuf + ( x * par->bypp) + (j * par->info->var.xres * par->bypp);
		broadsheet_burst_write(par, w, (u16 *) buf);
	}
	broadsheet_send_command(par, BS_CMD_LD_IMG_END);
	broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
	par->board->wait_for_rdy(par);
}

static void insert_dqueue_into_damage(unsigned long *dqueue_map, struct fb_damage_rect *dqueue, struct fb_damage_rect *damagerects, struct broadsheetfb_par *par)
{
	int i;
	int ret = 0;

	/* walk display queue and insert entry */
	for (i = find_first_bit(dqueue_map, ARRAY_SIZE(par->damages)); i < ARRAY_SIZE(par->damages);
			i = find_next_bit(dqueue_map, ARRAY_SIZE(par->damages), i + 1)) {
		if ((par->damage_count + 1) < ARRAY_SIZE(par->damages)) {
			memcpy(&damagerects[par->damage_count], &dqueue[i], sizeof(struct fb_damage_rect));
			par->damage_count++;
		} else {
			ret = 1;
			break;
		}
	}

	if (ret) {	
		printk(KERN_ERR "FAIL: no space in damage for dq\n");
	}
}

/*
 * Remember, set_damage puts damage rects into par->damages array.
 * This takes those damage rects and copies them into a damage queue
 * associated with the image buffer that we are currently using. It
 * is typically called after we have transferred that particular rect
 * into the image buffer. So both process_damage and full_damage will
 * call this.
 */
static void insert_damage_into_dqueue(struct broadsheetfb_par *par, struct fb_damage_rect *rect)
{
	int i;
	int ret = -1;
	unsigned long *dqueue_map;
	u64 *dqueue_expiry;
	struct fb_damage_rect *dqueue_rects;

	if (par->which_dqueue) {
		dqueue_map = par->dqueue_map1;
		dqueue_rects = par->dqueue1;
		dqueue_expiry = par->dqueue_expiry1;
	} else {
		dqueue_map = par->dqueue_map;
		dqueue_rects = par->dqueue;
		dqueue_expiry = par->dqueue_expiry;
	}

	/* walk display queue and insert entry */
	for (i = find_first_zero_bit(dqueue_map, ARRAY_SIZE(par->damages)); i < ARRAY_SIZE(par->damages);
			i = find_next_zero_bit(dqueue_map, ARRAY_SIZE(par->damages), i + 1)) {
		memcpy(&dqueue_rects[i], rect, sizeof(*rect));
		dqueue_expiry[i] = jiffies + msecs_to_jiffies(wfm_timing[waveform_mode]);
		set_bit(i, dqueue_map);
		ret = 0;
		break;
	}
	if (ret) {	
		printk(KERN_ERR "FAIL: no space for damage\n");
	}
}

static int broadsheetfb_process_damage(struct fb_info *info)
{
	struct broadsheetfb_par *par = info->par;
	int ret = -EINVAL;
	int i;
	struct fb_damage_rect *rect;
	u64 oldj;
	u64 j1, j2;

	j1=jiffies;


//	printk(KERN_INFO "%s:%d\n", __func__, __LINE__);
	mutex_lock(&par->dqueue_lock);
	for (i = 0; i < par->damage_count; i++) {
		j2 = jiffies;
		rect = &par->damages[i];
		if (!rect->w) {
			continue;
		}
		oldj = jiffies;
		broadsheetfb_set_imgupd_addr(par);
		broadsheetfb_load_image_area(par, rect->x, rect->y, rect->w, rect->h);
		insert_damage_into_dqueue(par, rect);	
		rect->w = 0;
	}
	par->damage_count = 0;
	mutex_unlock(&par->dqueue_lock);

//	printk(KERN_INFO "%s:%d\n", __func__, __LINE__);

	broadsheetfb_set_upd_addr(par);
	if (update_mode)
		broadsheetfb_upd_part(par);
	else
		broadsheetfb_upd_full(par);

	ret = 0;
	return ret;
}

static void clear_dqueue(struct broadsheetfb_par *par)
{
	mutex_lock(&par->dqueue_lock);
	if (par->which_dqueue) {
		bitmap_zero(par->dqueue_map1, ARRAY_SIZE(par->damages));
	} else {
		bitmap_zero(par->dqueue_map, ARRAY_SIZE(par->damages));
	}
	mutex_unlock(&par->dqueue_lock);
}

static int is_dqueue_empty(struct broadsheetfb_par *par)
{
	int i;
	int ret = 1; /* assume empty until proven otherwise */
	unsigned long *dqueue_map;
	u64 *dqueue_expiry;
	struct fb_damage_rect *dqueue_rects;

	if (par->which_dqueue) {
		dqueue_map = par->dqueue_map1;
		dqueue_rects = par->dqueue1;
		dqueue_expiry = par->dqueue_expiry1;
	} else {
		dqueue_map = par->dqueue_map;
		dqueue_rects = par->dqueue;
		dqueue_expiry = par->dqueue_expiry;
	}


	mutex_lock(&par->dqueue_lock);
	/* walk display queue and clear out expired entries */
	for (i = find_first_bit(dqueue_map, ARRAY_SIZE(par->damages)); i < ARRAY_SIZE(par->damages);
			i = find_next_bit(dqueue_map, ARRAY_SIZE(par->damages), i+1)) {
		if (dqueue_expiry[i] < jiffies) {
			memset(&dqueue_rects[i], 0, sizeof(struct fb_damage_rect));
			dqueue_expiry[i] = 0;
			clear_bit(i, dqueue_map);
		} else {
			ret = 0;
		}
	}
	
	mutex_unlock(&par->dqueue_lock);
	return ret;
}

static int get_area_of_overlap(int x1, int x2, int x3, int x4)
{
	int start = 0, end = 0;
	if (x1 < x3) {
		if (x3 > x2) {
			return 0;
		}
		start = x3;
		if (x4 > x2) {
			end = x2;
		} else {
			end = x4;
		}
	} else {
		if (x1 > x4) {
			return 0;
		}
		start = x1;
		if (x4 > x2) {
			end = x2;
		} else {
			end = x4;
		}
	}
	return (end - start);
}

static int measure_overlap(struct fb_damage_rect *a,
				struct fb_damage_rect *b)
{
	int xolap, yolap;

	xolap = get_area_of_overlap(a->x, a->x + a->w, b->x, b->x + b->w);
	yolap = get_area_of_overlap(a->y, a->y + a->h, b->y, b->y + b->h);

	return (xolap * yolap);
}
static int measure_overlaps(struct fb_damage_rect *a, int alen,
				struct fb_damage_rect *b, unsigned long *dqueue_map, int map_size)
{
	int i, j;
	int total = 0;

	for (i = 0; i < alen; i++) {
		for (j = find_first_bit(dqueue_map, map_size); j < map_size;
			j = find_next_bit(dqueue_map, map_size, j+1)) {
			if ((a[i].w) && (b[j].w)) {
				total += measure_overlap(&a[i], &b[j]);
				if (total > MAX_OLAP)
					return total;
			}
		}
	}
	return total;
}
static int measure_collisions(struct broadsheetfb_par *par)
{
	int ret = 0;
	int totalarea = 0;
	unsigned long *dqueue_map;
	struct fb_damage_rect *dqueue_rects;

	if (par->which_dqueue) {
		dqueue_map = par->dqueue_map1;
		dqueue_rects = par->dqueue1;
	} else {
		dqueue_map = par->dqueue_map;
		dqueue_rects = par->dqueue;
	}


	mutex_lock(&par->dqueue_lock);
	totalarea = measure_overlaps(par->damages, par->damage_count, dqueue_rects,
					dqueue_map, ARRAY_SIZE(par->damages));
	if (totalarea > MAX_OLAP) {
		ret = 1;
		goto finish;
	} else {
	}

finish:
	mutex_unlock(&par->dqueue_lock);
	return ret;
}

static void do_collision_processing(struct broadsheetfb_par *par)
{
	int totalarea;
	u64 oldj;

	totalarea = measure_collisions(par);
	if (totalarea) {
		oldj = jiffies;
		if (do_frend) {
			broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
			broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
			par->board->wait_for_rdy(par);
		}
		clear_dqueue(par);
	}
}

/*
 * complex damage is complex and messy.
 * first, we have to check and clear out any aged display queue entries.
 * if the queue is not empty at that point, then we have to determine if
 * these display queue entries colide with our damage list entries.
 */
static void broadsheetfb_dpy_do_complex_damage(struct broadsheetfb_par *par)
{
	/* if there are pending display items, we must mesaure the amount
	 * of collision and process it.
	 */
	if ((waveform_mode != 4) && !is_dqueue_empty(par)) {
		if (skip_collision_processing) {
			clear_dqueue(par);
		} else {
			do_collision_processing(par);
		}
	} else {
	}

//	printk(KERN_INFO "%s:%d\n", __func__, __LINE__);
	/* once we get here, we are safely collision free. */ 
	/* final damage processing and delivery to hw */
	broadsheetfb_process_damage(par->info);
}


/*
 * simple case. a full damage update so we do it, save the time and
 * mark display queue accordingly. caller must hold the damage lock.
 */
static void broadsheetfb_dpy_do_full_damage(struct broadsheetfb_par *par)
{
	u64 oldj;

	/* all damage can be cleared now */
	par->damage_count = 0;

	/* this is a full update, so we must check if there
	 * are any pending display items. */
	if (!is_dqueue_empty(par)) {
		/* if the dqueue is not empty then we have to wait
		 * for those items to complete
		 */
		oldj = jiffies;
		if (waveform_mode != 4) {
			if (do_frend) {
				broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
				broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
				par->board->wait_for_rdy(par);
			}
		}
		/* clearing of the dqueue is taken care of at the end */
	} else {
//		printk(KERN_ERR "empty dqueue in full\n");
	}

	broadsheetfb_set_imgupd_addr(par);
	/* load the full image and update the display */
	broadsheetfb_load_image(par);

	if (update_mode)
		broadsheetfb_upd_part(par);
	else
		broadsheetfb_upd_full(par);

	/* clear our damage count since we've moved it to the display
	 * queue
	 */
	mutex_lock(&par->dqueue_lock);
	insert_damage_into_dqueue(par, &par->damages[0]);
	par->damages[0].w = 0;
	mutex_unlock(&par->dqueue_lock);
}

/*
 * if the superset of a rect and an oldrect is found to
 * be smaller in area than the 2 rects (indicating sufficient overlap)
 * to justify coalescing, then erase the rect and store the
 * superset into oldrect.
 */
static int bbox_optimizable(struct fb_damage_rect *rect,
				struct fb_damage_rect *oldrect)
			
{
	int area1, area2, area3;
	int xmin, neww, ymin, newh;	

	area1 = rect->w * rect->h;
	area2 = oldrect->w * oldrect->h;

	xmin = min(rect->x, oldrect->x);	
	ymin = min(rect->y, oldrect->y);
	neww = max(rect->x + rect->w, oldrect->x + oldrect->w) - xmin;
	newh = max(rect->y + rect->h, oldrect->y + oldrect->h) - ymin;

	area3 = neww * newh;
	if (area3 <= (area1 + area2 + 100)) {
		rect->w = 0;
		oldrect->x = xmin;
		oldrect->y = ymin;
		oldrect->w = neww;
		oldrect->h = newh;
		return 1;
	}
	return 0;
}

static int compare_and_prep(struct fb_damage_rect *rect,
				struct fb_damage_rect *oldrect,
				int maxw, int maxh)
{
	if ((!oldrect->w) || (!rect->w)) {
		return 0;
	}
	if (!memcmp(rect, oldrect, sizeof(struct fb_damage_rect))) {
		memset(rect, 0, sizeof(struct fb_damage_rect));
	} else if (bbox_optimizable(rect, oldrect)) {
	/* if it has been optimized, then the result is stored in oldrect */
	}

 	if ((oldrect->w == maxw) && (oldrect->h == maxh)) {
		return 1;
	}
	return 0;

}


/* caller must hold the damage mutex */
static int rework_damagelist(struct broadsheetfb_par *par)
{
	int ret = 0;
	int i, j;

	/* test every damage entry with every other damage entry */
	for (i = 0; i < par->damage_count; i++) {
		if ((par->damages[i].w == par->info->var.xres) &&
				(par->damages[i].h == par->info->var.yres)) {
			par->damages[0].w = par->info->var.xres;
			par->damages[0].h = par->info->var.yres;
			par->damages[0].x = 0;
			par->damages[0].y = 0;
			par->damage_count = 1;
			return 1;
		}
		for (j = i + 1; j < par->damage_count; j++) {
			if ((!par->damages[i].w) || (!par->damages[j].w)) {
				continue;
			}
			ret = compare_and_prep(&par->damages[i], &par->damages[j],
					par->info->var.xres,
					par->info->var.yres);
			/* if an optimization is made, damages[i] is 0ed and
			 * damages[j] gets the optimized result.
			 */
			if (ret) {
				par->damages[0].w = par->info->var.xres;
				par->damages[0].h = par->info->var.yres;
				par->damages[0].x = 0;
				par->damages[0].y = 0;
				par->damage_count = 1;
				return 1;
			}
		}
	}
	return 0;
}


/*
 * defio calls handle damage if there is damage to work with.
 */
static void broadsheetfb_dpy_handle_damage(struct fb_info *info, struct broadsheetfb_par *par)
{
	u64 oldj;
	int ret;
	int i, j;

	oldj = jiffies;
	
	/*
	 * first, we process our damage to clean it up for things like
	 * subsets and adjacents.
	 */
	if (skip_rework_damagelist) {
		ret = 0;
	} else {
		ret = rework_damagelist(par);
	}

	/*
	 * now, we can check which kind of damage processing we need to
	 * do. if we detected a full update from our damage list processing
	 * then treat it as a full_damage update which is simpler.
	 */
	if (ret) {
//	printk(KERN_INFO "%s:%d\n", __func__, __LINE__);
		broadsheetfb_dpy_do_full_damage(par);
	} else {
//	printk(KERN_INFO "%s:%d\n", __func__, __LINE__);
		/* handoff to complex damage handler */
		broadsheetfb_dpy_do_complex_damage(par);
	}
}

static void broadsheetfb_dpy_traditional_defio(struct fb_info *info, struct broadsheetfb_par *par)
{
	u16 yres = info->var.yres;
	u16 xres = info->var.xres;
	u16 y1 = 0, h = 0;
	int prev_index = -1;
	int h_inc;
	struct fb_deferred_io *fbdefio = info->fbdefio;
	int y2;
	int i;

	/* height increment is fixed per page */
	h_inc = DIV_ROUND_UP(PAGE_SIZE , xres * par->bypp);

	for (i = find_first_bit(fbdefio->pagemap, fbdefio->pagecount);
		i < fbdefio->pagecount;
		i = find_next_bit(fbdefio->pagemap, fbdefio->pagecount, i+1)) {
		if (prev_index < 0) {
			/* just starting so assign first page */
			y1 = (i << PAGE_SHIFT) / ( xres * par->bypp );
			y2 = DIV_ROUND_UP(((i + 1) << PAGE_SHIFT) , ( xres * par->bypp ));
		} else if ((prev_index + 1) == i) {
			/* this page is consecutive so increase our height */
			y2 = DIV_ROUND_UP(((i + 1) << PAGE_SHIFT) , ( xres * par->bypp ));
		} else {
			/* page not consecutive, issue previous update first */
			broadsheetfb_set_imgupd_addr(par);
			broadsheetfb_dpy_update_pages(par, y1, min(y2 + 10, yres));
			/* start over with our non consecutive page */
			y1 = (i << PAGE_SHIFT) / ( xres * par->bypp);
			y2 = DIV_ROUND_UP(((i + 1) << PAGE_SHIFT) , ( xres * par->bypp ));
		}
		prev_index = i;
	}


	broadsheetfb_set_imgupd_addr(par);
	/* if we still have any pages to update we do so now */
	if (h >= yres) {
		/* its a full screen update, just do it */
		broadsheetfb_dpy_update(par);
	} else {
		broadsheetfb_dpy_update_pages(par, y1, min(y2 + 10, yres));
	}
}

static void copy_dqueue_to_damage(struct broadsheetfb_par *par)
{
	/* if we were previously using dq1, we must copy whatever
	 * damage existed in that queue into our current damage pool
	 * and vice versa */

	if (par->which_dqueue) {
		/* prev was dq1 so use that to feed current damage */
		insert_dqueue_into_damage(par->dqueue_map1, par->dqueue1, par->damages, par);
	} else {
		/* prev was dq0 so use that to feed current damage */
		insert_dqueue_into_damage(par->dqueue_map, par->dqueue, par->damages, par);
	}
}

static void set_which_dqueue(struct broadsheetfb_par *par)
{
	copy_dqueue_to_damage(par);

	/* switch which image buffer we're using */
	if (par->which_dqueue) {
		par->which_dqueue = 0;
	} else {
		par->which_dqueue = 1;
	}
}



/*
 * This is called back from the deferred io workqueue. Remember that
 * set_damage stuck damage into par->damage rects.
 */
static void broadsheetfb_dpy_deferred_io(struct fb_info *info)
{
	struct broadsheetfb_par *par = info->par;
	u64 oldj;
	static u64 defio_finish_time = 0;
	int elapsed_time;
	int defio_delay;
	oldj = jiffies;

	mutex_lock(&par->damage_lock);

	/*
	 * If we're swapping back and forth then swap our internal
	 * notion of which dqueue we're using. So we start of with
	 * which_dqueue = 1, and then we set_which_dqueue which
	 * takes the previous damage pushed to the display, eg:
	 * the full screen in the case of youtube, and then inserts
	 * that into the damage pool. That way, when we go to draw,
	 * we have made sure that we have not skipped pushing something
	 * that had previously been drawn to the other buffer.
	 * Eg: 
	 * 1. draw frame 1 to buffer 1
	 * 2. set damage of frame 1 to dqueue1
	 * 3. draw frame 2 to buffer 0
	 * 4. --> first, copy the damage of frame 1 back to current damages
	 * 5. really draw frame 2 to buffer 0
	 * 6. copy the damage of frame 2 (including the frame 1 damage)
	 * into dqueue2.
	 */
	if (!skip_2nd_buf) 
		set_which_dqueue(par);

	if (!par->damage_count) {
		/* if we don't have damage information, we use page info */

		if (do_frend) {
			broadsheet_send_command(par, BS_CMD_WAIT_DSPE_TRG);
			broadsheet_send_command(par, BS_CMD_WAIT_DSPE_FREND);
			par->board->wait_for_rdy(par);
		}
		
		/* we clear the display queue since we have no more pending
		 * display queue tasks because we did a wait above. 
		 */
		clear_dqueue(par);
		/* since no damage rely on page information */
//	printk(KERN_INFO "%s:%d\n", __func__, __LINE__);
		broadsheetfb_dpy_traditional_defio(info, par);
		
	} else {
		/* we have damage to work with.  */
		broadsheetfb_dpy_handle_damage(info, par);
	}
	defio_finish_time = jiffies;
	mutex_unlock(&par->damage_lock);
}

static void broadsheetfb_fillrect(struct fb_info *info,
				   const struct fb_fillrect *rect)
{
	struct broadsheetfb_par *par = info->par;

	sys_fillrect(info, rect);

	schedule_delayed_work(&par->resched_work, info->fbdefio->delay);
}

static void broadsheetfb_copyarea(struct fb_info *info,
				   const struct fb_copyarea *area)
{
	struct broadsheetfb_par *par = info->par;

	sys_copyarea(info, area);

	schedule_delayed_work(&par->resched_work, info->fbdefio->delay);
}

static void broadsheetfb_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	struct broadsheetfb_par *par = info->par;

	sys_imageblit(info, image);

	schedule_delayed_work(&par->resched_work, info->fbdefio->delay);
}

/*
 * this is the slow path from userspace. they can seek and write to
 * the fb. it's inefficient to do anything less than a full screen draw
 */
static ssize_t broadsheetfb_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct broadsheetfb_par *par = info->par;
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void *)(info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if  (!err)
		*ppos += count;

	schedule_delayed_work(&par->resched_work, info->fbdefio->delay);

	return (err) ? err : count;
}

/*
 * here, we pickup damage information from userspace.
 * if there is already a flag set saying that the whole screen is going
 * to be updated, then we just leave.
 * we prepare a bounding box from the rects provided by userspace. if
 * one of those rects causes a whole screen update, then we flag it and
 * then add the rects to the damage list anyway.
 * in the normal scenario, we just add to the damage list.
 */
static int broadsheetfb_set_damage(struct fb_info *info,
				struct fb_damage_user *udamage)
{
	struct broadsheetfb_par *par = info->par;
	int ret = -EINVAL;
	int size = udamage->len;
	u64 oldj;
	int i;
	struct fb_damage_rect *rect;

	oldj = jiffies;

	/* prepare to pull damage information from userspace */
	if (size > BROADSHEETFB_DAMAGE_COUNT_MAX)
		goto finish;

	mutex_lock(&par->damage_lock);
	if (size + par->damage_count > ARRAY_SIZE(par->damages)) {
		printk(KERN_ERR "excess damage so no space for %d due to %d\n", size, par->damage_count);
		goto unlock_finish;
	}

	if (copy_from_user(&par->damages[par->damage_count],
			udamage->rects, sizeof(struct fb_damage_rect)*size)) {
		ret = -EFAULT;
		goto unlock_finish;
	}
#if 0
	printk(KERN_INFO "set damage with count %d, incoming size %d\n", par->damage_count, size);
	for (i=0; i < size; i++) {
		struct fb_damage_rect *rect = &par->damages[par->damage_count + i];
		printk(KERN_INFO "rect[%d]=%d,%d,%d,%d\n", i, rect->x, rect->y, rect->w, rect->h);
	}
#endif
	par->damage_count += size;
	ret = 0;
unlock_finish:
	mutex_unlock(&par->damage_lock);
finish:
	return ret;
}

static int broadsheetfb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct device *dev = info->device;
	struct broadsheetfb_par *par = info->par;

	switch (var->bits_per_pixel) {
	case 8:
		memcpy(var, &broadsheetfb_var, sizeof(struct fb_var_screeninfo));
		break;
	case 16:
		memcpy(var, &broadsheetfb_var_565, sizeof(struct fb_var_screeninfo));
		break;
	default:
		dev_err(dev, "color depth %d not supported\n",
					var->bits_per_pixel);
		return -EINVAL;
	}

	return 0;
}
static int broadsheetfb_set_par(struct fb_info *info)
{
	struct broadsheetfb_par *par = info->par;

	if (info->var.bits_per_pixel == 8) {
		info->fix.visual = FB_VISUAL_STATIC_PSEUDOCOLOR;
		info->fix.line_length = info->var.xres ;
		info->fix.smem_len = info->var.xres * info->var.yres;
	} else {
		par->bypp = 2;
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		info->fix.line_length = info->var.xres * 2;
		info->fix.smem_len = info->var.xres * info->var.yres * 2;
	}

	return 0;
}
static int broadsheetfb_setcolreg(unsigned regno, unsigned red, unsigned green,
		unsigned blue, unsigned transp, struct fb_info *info)
{
	if (regno >= 16)
		return 1;

	red >>= 8;
	green >>= 8;
	blue >>= 8;
	transp >>= 8;

	((__be32 *)info->pseudo_palette)[regno] = cpu_to_be32(transp << 24 |
		red << 0 | green << 8 | blue << 16);
	return 0;
}


static struct fb_ops broadsheetfb_ops = {
	.owner		= THIS_MODULE,
	.fb_read        = fb_sys_read,
	.fb_write	= broadsheetfb_write,
	.fb_fillrect	= broadsheetfb_fillrect,
	.fb_copyarea	= broadsheetfb_copyarea,
	.fb_imageblit	= broadsheetfb_imageblit,
	.fb_set_damage	= broadsheetfb_set_damage,
	.fb_check_var	= broadsheetfb_check_var,
	.fb_set_par	= broadsheetfb_set_par,
	.fb_setcolreg	= broadsheetfb_setcolreg,
};

static struct fb_deferred_io broadsheetfb_defio = {
	.delay		= HZ/4,
	.deferred_io	= broadsheetfb_dpy_deferred_io,
};

static ssize_t broadsheetfb_set_upd_mode(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	char ** last = NULL;

	update_mode = simple_strtoul(buf, last, 0);
//	broadsheetfb_dpy_update2(fb_info->par);
	return count;
}

static ssize_t broadsheet_show_upd_mode(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	return snprintf(buf, PAGE_SIZE, "%d\n", update_mode);
}

static DEVICE_ATTR(update_mode, 0666, broadsheet_show_upd_mode, broadsheetfb_set_upd_mode);



static ssize_t broadsheetfb_set_wfm_mode(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char ** last = NULL;

	waveform_mode = simple_strtoul(buf, last, 0);
	return count;
}

static ssize_t broadsheet_show_wfm_mode(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	return snprintf(buf, PAGE_SIZE, "%d\n", waveform_mode);
}

static DEVICE_ATTR(waveform_mode, 0666, broadsheet_show_wfm_mode, broadsheetfb_set_wfm_mode);

static ssize_t broadsheetfb_set_defio_delay(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	char ** last = NULL;

	fb_info->fbdefio->delay = simple_strtoul(buf, last, 0);
	return count;
}

static ssize_t broadsheet_show_defio_delay(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	return snprintf(buf, PAGE_SIZE, "%d\n", fb_info->fbdefio->delay);
}

static DEVICE_ATTR(defio_delay, 0666, broadsheet_show_defio_delay, broadsheetfb_set_defio_delay);


static ssize_t broadsheetfb_set_do_frend(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	char ** last = NULL;

	do_frend = simple_strtoul(buf, last, 0);
	return count;
}

static ssize_t broadsheet_show_do_frend(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	return snprintf(buf, PAGE_SIZE, "%d\n", do_frend);
}

static DEVICE_ATTR(do_frend, 0666, broadsheet_show_do_frend, broadsheetfb_set_do_frend);

static ssize_t broadsheetfb_set_do_dqstuff(struct device *device,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct fb_info *info = dev_get_drvdata(device);
	char ** last = NULL;
	struct broadsheetfb_par *par = info->par;

	do_dqstuff = simple_strtoul(buf, last, 0);
	if (do_dqstuff & 0x01) {
		skip_collision_processing = 1;
	} else {
		skip_collision_processing = 0;
	}

	if (do_dqstuff & 0x02) {
		skip_rework_damagelist = 1;
	} else {
		skip_rework_damagelist = 0;
	}
	if (do_dqstuff & 0x04) {
		mutex_lock(&par->damage_lock);
		skip_2nd_buf = 1;
		par->which_dqueue = 1;
		broadsheetfb_set_imgupd_addr(par);
		broadsheetfb_load_image(par);
		broadsheetfb_upd_full(par);
		clear_dqueue(par);
		mutex_unlock(&par->damage_lock);
	} else {
		mutex_lock(&par->damage_lock);
		skip_2nd_buf = 0;
		clear_dqueue(par);
		set_which_dqueue(par);
		broadsheetfb_set_imgupd_addr(par);
		broadsheetfb_load_image(par);
		broadsheetfb_upd_full(par);
		set_which_dqueue(par);
		clear_dqueue(par);
		broadsheetfb_set_imgupd_addr(par);
		broadsheetfb_load_image(par);
		broadsheetfb_upd_full(par);
		mutex_unlock(&par->damage_lock);
	}

	return count;
}

static ssize_t broadsheet_show_do_dqstuff(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct fb_info *fb_info = dev_get_drvdata(device);
	return snprintf(buf, PAGE_SIZE, "%d\n", do_dqstuff);
}

static DEVICE_ATTR(do_dqstuff, 0666, broadsheet_show_do_dqstuff, broadsheetfb_set_do_dqstuff);

static void broadsheetfb_resched_work(struct work_struct *work)
{
	struct broadsheetfb_par *par = container_of(work, struct broadsheetfb_par, resched_work.work);

 	broadsheetfb_dpy_update(par);
}

static int __devinit broadsheetfb_probe(struct platform_device *dev)
{
	struct fb_info *info;
	struct broadsheet_board *board;
	int retval = -ENOMEM;
	int videomemorysize;
	unsigned char *videomemory;
	struct broadsheetfb_par *par;
	int i;
	int dpyw, dpyh;
	int cmaplen;

	/* pick up board specific routines */
	board = dev->dev.platform_data;
	if (!board)
		return -EINVAL;

	/* try to count device specific driver, if can't, platform recalls */
	if (!try_module_get(board->owner))
		return -ENODEV;

	info = framebuffer_alloc(sizeof(struct broadsheetfb_par), &dev->dev);
	if (!info)
		goto err;

	switch (board->get_panel_type()) {
	case 6:
		panel_index = 0;
		break;
	case 37:
		panel_index = 1;
		break;
	case 97:
		panel_index = 2;
		break;
	}

	if (rot_mode & 1) {
		/* this is portrait mode 90 or 270 degrees */
		dpyw = panel_table[panel_index].h;
		dpyh = panel_table[panel_index].w;
	} else {
		dpyw = panel_table[panel_index].w;
		dpyh = panel_table[panel_index].h;
	}

	cmaplen = panel_table[panel_index].cmaplen;

	videomemorysize = roundup((dpyw*dpyh*2), PAGE_SIZE);
	videomemory = vmalloc(videomemorysize);
	if (!videomemory)
		goto err_fb_rel;

	memset(videomemory, 0, videomemorysize);

	info->screen_base = (char *)videomemory;
	info->fbops = &broadsheetfb_ops;

	broadsheetfb_var_565.xres = dpyw;
	broadsheetfb_var_565.yres = dpyh;
	broadsheetfb_var_565.xres_virtual = dpyw;
	broadsheetfb_var_565.yres_virtual = dpyh;

	broadsheetfb_var.xres = dpyw;
	broadsheetfb_var.yres = dpyh;
	broadsheetfb_var.xres_virtual = dpyw;
	broadsheetfb_var.yres_virtual = dpyh;
	info->var = broadsheetfb_var;

	broadsheetfb_fix.line_length = dpyw;
	info->fix = broadsheetfb_fix;
	info->fix.smem_len = videomemorysize;
	info->var.activate |= FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW;
	par = info->par;
	par->panel_index = panel_index;
	par->info = info;
	par->board = board;
	par->write_reg = broadsheet_write_reg;
	par->read_reg = broadsheet_read_reg;
	init_waitqueue_head(&par->waitq);
	memset(par->dqueue_map, 0, sizeof(par->dqueue_map));
	memset(par->dqueue_map1, 0, sizeof(par->dqueue_map1));
	par->which_dqueue = 1;
	par->bypp = 1;
	info->pseudo_palette = par->pseudo_palette;

	mutex_init(&par->io_lock);
	info->flags = FBINFO_FLAG_DEFAULT;

	INIT_DELAYED_WORK(&par->resched_work, broadsheetfb_resched_work);

	info->fbdefio = &broadsheetfb_defio;
	fb_deferred_io_init(info);

	retval = fb_alloc_cmap(&info->cmap, cmaplen, 0);
	if (retval < 0) {
		dev_err(&dev->dev, "Failed to allocate colormap\n");
		goto err_vfree;
	}

	/* set cmap */
	for (i = 0; i < cmaplen; i++) {
		info->cmap.red[i] = (((2*i)+1)*(0xFFFF))/(cmaplen*2);
	}
	info->cmap.red[cmaplen - 1] = 0xFFFF;
	memcpy(info->cmap.green, info->cmap.red, sizeof(u16)*cmaplen);
	memcpy(info->cmap.blue, info->cmap.red, sizeof(u16)*cmaplen);

	retval = par->board->setup_irq(info);
	if (retval < 0)
		goto err_cmap;

	/* this inits the dpy */
	retval = board->init(par);
	if (retval < 0)
		goto err_free_irq;

	broadsheet_init(par);

	mutex_init(&par->damage_lock);
	mutex_init(&par->dqueue_lock);

	retval = register_framebuffer(info);
	if (retval < 0)
		goto err_free_irq;
	platform_set_drvdata(dev, info);

	/* create device files */
	retval = device_create_file(info->dev, &dev_attr_waveform_mode);
	if (retval) {
		printk(KERN_ERR "failed to add debug attribute\n");
	}

	retval = device_create_file(info->dev, &dev_attr_update_mode);
	if (retval) {
		printk(KERN_ERR "failed to add debug attribute\n");
	}

	retval = device_create_file(info->dev, &dev_attr_defio_delay);
	if (retval) {
		printk(KERN_ERR "failed to add debug attribute\n");
	}

	retval = device_create_file(info->dev, &dev_attr_do_frend);
	if (retval) {
		printk(KERN_ERR "failed to add debug attribute\n");
	}

	retval = device_create_file(info->dev, &dev_attr_do_dqstuff);
	if (retval) {
		printk(KERN_ERR "failed to add debug attribute\n");
	}

	retval = device_create_file(info->dev, &dev_attr_loadstore_waveform);
	if (retval) {
		printk(KERN_ERR "failed to add debug attribute\n");
	}


	printk(KERN_INFO
	       "fb%d: Broadsheet frame buffer, using %dK of video memory\n",
	       info->node, videomemorysize >> 10);


	return 0;

err_unreg_fb:
	unregister_framebuffer(info);
err_free_irq:
	board->cleanup(par);
	mutex_destroy(&par->damage_lock);
err_cmap:
	fb_dealloc_cmap(&info->cmap);
	fb_deferred_io_cleanup(info);
err_vfree:
	vfree(videomemory);
err_fb_rel:
	framebuffer_release(info);
err:
	module_put(board->owner);
	return retval;

}

static void __devexit broadsheetfb_cleanup_damage(struct broadsheetfb_par *par)
{
	broadsheetfb_process_damage(par->info);
	mutex_destroy(&par->damage_lock);
}

static int __devexit broadsheetfb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
		struct broadsheetfb_par *par = info->par;

		unregister_framebuffer(info);

		/* cleanup any pending damage */
		broadsheetfb_cleanup_damage(par);

 		device_remove_file(info->dev, &dev_attr_waveform_mode);
 		device_remove_file(info->dev, &dev_attr_defio_delay);
 		device_remove_file(info->dev, &dev_attr_do_frend);
 		device_remove_file(info->dev, &dev_attr_do_dqstuff);
 		device_remove_file(info->dev, &dev_attr_loadstore_waveform);
		par->board->cleanup(par);
		fb_dealloc_cmap(&info->cmap);
		fb_deferred_io_cleanup(info);
		vfree((void *)info->screen_base);
		module_put(par->board->owner);
		framebuffer_release(info);
	}
	return 0;
}

static struct platform_driver broadsheetfb_driver = {
	.probe	= broadsheetfb_probe,
	.remove = broadsheetfb_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "broadsheetfb",
	},
};

static int __init broadsheetfb_init(void)
{
	return platform_driver_register(&broadsheetfb_driver);
}

static void __exit broadsheetfb_exit(void)
{
	platform_driver_unregister(&broadsheetfb_driver);
}

module_init(broadsheetfb_init);
module_exit(broadsheetfb_exit);

module_param(panel_type, uint, 0);
MODULE_PARM_DESC(panel_type, "Select panel type");

module_param(do_test, uint, 0);
MODULE_PARM_DESC(do_test, "Perform test patterns");

module_param(update_mode, uint, 0);
MODULE_PARM_DESC(update_mode, "Update mode");

module_param(rot_mode, uint, 0);
MODULE_PARM_DESC(rot_mode, "Rotate mode 0, 1 = 90, 2=180, 3=270");

MODULE_DESCRIPTION("fbdev driver for Broadsheet controller");
MODULE_AUTHOR("Jaya Kumar");
MODULE_LICENSE("GPL");
