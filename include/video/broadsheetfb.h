/*
 * broadsheetfb.h - definitions for the broadsheet framebuffer driver
 *
 * Copyright (C) 2008 by Jaya Kumar
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#ifndef _LINUX_BROADSHEETFB_H_
#define _LINUX_BROADSHEETFB_H_

/* Broadsheet command defines */
#define BS_CMD_INIT_SYS_RUN	0x06
#define BS_CMD_INIT_DSPE_CFG	0x09
#define BS_CMD_INIT_DSPE_TMG	0x0A
#define BS_CMD_INIT_ROTMODE	0x0B
#define BS_CMD_RD_REG		0x10
#define BS_CMD_WR_REG		0x11
#define BS_CMD_LD_IMG		0x20
#define BS_CMD_LD_IMG_AREA	0x22
#define BS_CMD_LD_IMG_END	0x23
#define BS_CMD_LD_IMG_SETADR	0x25
#define BS_CMD_WAIT_DSPE_TRG	0x28
#define BS_CMD_WAIT_DSPE_FREND	0x29
#define BS_CMD_RD_WFM_INFO	0x30
#define BS_CMD_UPD_INIT		0x32
#define BS_CMD_UPD_FULL		0x33
#define BS_CMD_UPD_PART		0x35
#define BS_CMD_UPD_GDRV_CLR	0x37
#define BS_CMD_UPD_SET_IMGADR	0x38

/* Broadsheet register interface defines */
#define BS_REG_REV		0x00
#define BS_REG_PRC		0x02

/* Broadsheet pin interface specific defines */
#define BS_CS	0x01
#define BS_DC 	0x02
#define BS_WR 	0x03

#define BROADSHEETFB_DAMAGE_COUNT_MAX 511
#define BROADSHEETFB_DAMAGE_SIZE 512

/* Broadsheet IO interface specific defines */
#define BS_MMIO_CMD	0x01
#define BS_MMIO_DATA	0x02

/* struct used by broadsheet. board specific stuff comes from *board */
struct broadsheetfb_par {
	struct fb_info *info;
	struct broadsheet_board *board;
	void (*write_reg)(struct broadsheetfb_par *, u16 reg, u16 val);
	u16 (*read_reg)(struct broadsheetfb_par *, u16 reg);
	wait_queue_head_t waitq;
 	struct mutex io_lock;
	int panel_index;

	/* for collecting userspace damage information */
	struct mutex damage_lock;
	struct fb_damage_rect damages[BROADSHEETFB_DAMAGE_SIZE];
	int damage_count;

	/* for storing pending display items and their finish times */
	struct mutex dqueue_lock;
	struct fb_damage_rect dqueue[BROADSHEETFB_DAMAGE_SIZE];
	struct fb_damage_rect dqueue1[BROADSHEETFB_DAMAGE_SIZE];
	u64 dqueue_expiry[BROADSHEETFB_DAMAGE_SIZE];
	u64 dqueue_expiry1[BROADSHEETFB_DAMAGE_SIZE];
	unsigned long dqueue_map[BROADSHEETFB_DAMAGE_SIZE/32];
	unsigned long dqueue_map1[BROADSHEETFB_DAMAGE_SIZE/32];
	int which_dqueue;
	struct delayed_work resched_work;
	int bypp;
	u32 pseudo_palette[16];
};

/* board specific routines */
struct broadsheet_board {
	struct module *owner;
	int (*init)(struct broadsheetfb_par *);
	int (*init_pre_run)(struct broadsheetfb_par *);
	int (*wait_for_rdy)(struct broadsheetfb_par *);
	void (*cleanup)(struct broadsheetfb_par *);
	int (*get_panel_type)(void);
	int (*setup_irq)(struct fb_info *);
	int (*get_ext_temp)(void);

 	/* Functions for boards that use GPIO */
 	void (*set_ctl)(struct broadsheetfb_par *, unsigned char, u8);
 	void (*set_hdb)(struct broadsheetfb_par *, u16);
 	u16 (*get_hdb)(struct broadsheetfb_par *);
 
 	/* Functions for boards that have specialized MMIO */
 	void (*mmio_write)(struct broadsheetfb_par *, int type, u16);
 	u16 (*mmio_read)(struct broadsheetfb_par *);

};

struct broadsheetfb_damage {
	__u32 len;		/* Number of entries */
	struct fb_damage_rect *rects; /* array of damage rectangles */
	u64 finish_time;
};

#endif
