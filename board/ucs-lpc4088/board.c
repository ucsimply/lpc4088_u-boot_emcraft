/*
 * (C) Copyright 2011, 2012
 *
 * Alexander Potashev, Emcraft Systems, aspotashev@emcraft.com
 *
 * 
 * Adapted for the uCSimply LPC4088 module
 * Vit Mares, Elvoris, mares.vit@elvoris.cz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Board specific code for the uCSimply LPC4088 module.
 */

#include <common.h>
#include <netdev.h>

#include <asm/arch/lpc178x_gpio.h>

/*
 * SDRAM-specific configuration
 */

/* CPU clock = 108 MHz, EMC clock = 1/2 CPU clock = 54 MHz */
#define EMC_FREQ_HZ         54000000
#define EMC_CLK_NS          18.52

#define CLK_DELAY           7

#define SDR_CMD_NOP         0x00000183
#define SDR_CMD_PALL        0x00000103
#define SDR_CMD_MODE        0x00000083
#define SDR_CMD_NORMAL      0x00000000

/* settings for AS4C16M16S-7BCN device
 * - 256Mbit, 16M x 16bit, 4 Banks x 4M words x 16bit
 * - 13 bit row address A[0..12] = 8192 rows
 * - 9 bit column address A[0..8] = 512 columns
 */
#define SDR_ROWS            8192
#define SDR_RAS_CLK         2
#define SDR_CAS_CLK         2
#define SDR_BURST_LEN_8     3
#define SDR_MODEREG         ((SDR_CAS_CLK << 4) | SDR_BURST_LEN_8)
#define SDR_ADDR_MODEREG    (CONFIG_SYS_RAM_BASE | (SDR_MODEREG << (9 + 2 + 1)))

#define SDR_REFRESH_NS  (64000000 / SDR_ROWS) // 7812.5
#define SDR_REFRESH     26                    // (7813 / 18.52) / 16
#define SDR_READ_CFG    1
#define SDR_RAS_CAS     ((SDR_CAS_CLK << 8) | SDR_RAS_CLK )
#define SDR_SIZE_256    (0x03 << 9)
#define SDR_BUS_16      (0x01 << 7)
#define SDR_CONFIG      (SDR_SIZE_256 | SDR_BUS_16)

/* ****************** */
/* 1 cycle = 18.52 ns */
/* ****************** */
/* Precharge Command Period                             21 ns */
#define SDR_tRP         1
/* Active to Precharge Command Period                   49 ns */
#define SDR_tRAS        2
/* Self Refresh Exit Time (use tXSR if not specified) */
#define SDR_tSREX       3
/* Last Data Out to Active Time                         1 CLK */
#define SDR_tAPR        1
/* Data In to Active Command Time                       CAS_CLK + tRP */
#define SDR_tDAL        3
/* Write Recovery Time                                  14 ns */
#define SDR_tWR         0
/* Active to Active Command Period                      63 ns */
#define SDR_tRC         3
/* Auto-refresh Period                                  63 ns */
#define SDR_tRFC        3
/* Exit Self Refresh  (tRC + tIS)                       63 + 1.5 ns */
#define SDR_tXSR        3
/* Active Bank A to Active Bank B Time                  14 ns */
#define SDR_tRRD        0
/* Load Mode register command to Active Command         14 ns*/
#define SDR_tMRD        0


/* EMC data pins (DQ0..DQ15) */
#define LPC178X_EMC_DATA_PINS	16
/* EMC row/column address pins (A0..A12) */
#define LPC178X_EMC_ADDR_PINS	13

/*
 * EMC per-chip registers for DRAM.
 *
 * This structure must be 0x20 bytes in size
 * (for `struct lpc178x_emc_regs` to be correct.)
 */
struct lpc178x_emc_dy_regs {
	u32 cfg;	/* Dynamic Memory Configuration register */
	u32 rascas;	/* Dynamic Memory RAS & CAS Delay registers */
	u32 rsv0[6];
};

/*
 * EMC controls for Static Memory CS. Each block occupies 0x20 bytes.
 */
struct lpc178x_emc_st_regs {
	u32 cfg;	/* Static Memory Configuration register */
	u32 we;		/* CS to WE delay register */
	u32 oe;		/* CS to OE delay register */
	u32 rd;		/* CS to Read delay register */
	u32 page;	/* async page mode access delay */
	u32 wr;		/* CS to Write delay register */
	u32 ta;		/* number of turnaround cycles */
	u32 rsv0[1];
};

/*
 * EMC (External Memory Controller) register map
 * Should be mapped at 0x2009C000.
 */
struct lpc178x_emc_regs {
	/* 0x2009C000 */
	u32 emcctrl;	/* EMC Control register */
	u32 emcsts;	/* EMC Status register */
	u32 emccfg;	/* EMC Configuration register */
	u32 rsv0[5];

	/* 0x2009C020 */
	u32 dy_ctrl;	/* Dynamic Memory Control register */
	u32 dy_rfsh;	/* Dynamic Memory Refresh Timer register */
	u32 dy_rdcfg;	/* Dynamic Memory Read Configuration register */
	u32 rsv1;

	/* 0x2009C030 */
	u32 dy_trp;	/* Dynamic Memory Precharge Command Period register */
	u32 dy_tras;	/* Dynamic Memory Active to Precharge Command
				Period register */
	u32 dy_srex;	/* Dynamic Memory Self-refresh Exit Time register */
	u32 dy_apr;	/* Dynamic Memory Last Data Out to Active
				Time register */
	u32 dy_dal;	/* Dynamic Memory Data-in to Active Command
				Time register */
	u32 dy_wr;	/* Dynamic Memory Write Recovery Time register */
	u32 dy_rc;	/* Dynamic Memory Active to Active Command
				Period register */
	u32 dy_rfc;	/* Dynamic Memory Auto-refresh Period register */
	u32 dy_xsr;	/* Dynamic Memory Exit Self-refresh register */
	u32 dy_rrd;	/* Dynamic Memory Active Bank A to
				Active Bank B Time register */
	u32 dy_mrd;	/* Dynamic Memory Load Mode register to
				Active Command Time */
	/* 0x2009C05C */
	u32 rsv2[41];

	/* 0x2009C100 */
	struct lpc178x_emc_dy_regs dy[4];	/* 4 DRAM chips are possible */
	u32 rsv3[32];
	/* 0x2009C200 */
	struct lpc178x_emc_st_regs st[4];	/* 4 Static RAM devices (flash) */
};

#define LPC178X_EMC_BASE		(LPC178X_AHB_PERIPH_BASE + 0x0001C000)
#define LPC178X_EMC			((volatile struct lpc178x_emc_regs *) \
					LPC178X_EMC_BASE)

DECLARE_GLOBAL_DATA_PTR;

/*
 * GPIO pin configuration table for uCSimply LPC4088 module
 *
 * This table does not list all GPIO pins that will be configured. 
 * See also the code in `gpio_init()`.
 */
static const struct lpc178x_gpio_pin_config ea_lpc1788_gpio[] = {
	/*
	 * GPIO configuration for UART
	 */
#if CONFIG_LPC178X_UART_PORT == 0
	/* P0.2 (D) = UART0 TXD */
	{{0,  2}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P0.3 (D) = UART0 RXD */
	{{0,  3}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
#else /* Not UART0 */
#error This configuration of GPIO pins supports only UART0
#endif

#ifdef CONFIG_NR_DRAM_BANKS
	/*
	 * GPIO configuration for SDRAM
	 */
#define LPC178X_GPIO_EMC_REGVAL \
	(LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0))

	/* Configure EMC bank address select 0 and 1 (BA0, BA1) */
	{{4, 13}, LPC178X_GPIO_EMC_REGVAL},
	{{4, 14}, LPC178X_GPIO_EMC_REGVAL},

	/* Configure EMC column address strobe (CAS) */
	{{2, 16}, LPC178X_GPIO_EMC_REGVAL},
	/* Configure EMC row address strobe (RAS) */
	{{2, 17}, LPC178X_GPIO_EMC_REGVAL},

	/* Configure EMC write enable (WE) */
	{{4, 25}, LPC178X_GPIO_EMC_REGVAL},

	/* Configure EMC clock input (CLK) */
	{{2, 18}, LPC178X_GPIO_EMC_REGVAL},
	/* Configure EMC clock enable (CKE) */
	{{2, 24}, LPC178X_GPIO_EMC_REGVAL},

	/* Configure EMC chip select (DYCS0) */
	{{2, 20}, LPC178X_GPIO_EMC_REGVAL},

	/* Configure EMC I/O mask (DQM0..DQM1) */
	{{2, 28}, LPC178X_GPIO_EMC_REGVAL},
	{{2, 29}, LPC178X_GPIO_EMC_REGVAL},
#endif /* CONFIG_NR_DRAM_BANKS */

#ifdef CONFIG_LPC178X_ETH
	/*
	 * GPIO configuration for Ethernet
	 */
	/* P1.0 (D) = RMII ENET_TXD0 */
	{{1,  0}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.1 (D) = RMII ENET_TXD1 */
	{{1,  1}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.4 (D) = RMII ENET_TX_EN */
	{{1,  4}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.8 (D) = RMII CRS */
	{{1,  8}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.9 (D) = RMII RXD0 */
	{{1,  9}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.10 (D) = RMII RXD1 */
	{{1, 10}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 1, 0)},
	/* P1.14 (D) = RMII RXER */
	{{1, 14}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.15 (D) = RMII CLK */
	{{1, 15}, LPC178X_GPIO_CONFIG_D(1, LPC178X_NO_PULLUP, 0, 0, 0, 0)},
	/* P1.16 (D) = RMII MCD */
	{{1, 16}, LPC178X_GPIO_CONFIG_W(1, LPC178X_NO_PULLUP, 0, 0, 1, 0, 0, 0)},
	/* P1.17 (D) = RMII MDIO */
	{{1, 17}, LPC178X_GPIO_CONFIG_W(1, LPC178X_NO_PULLUP, 0, 0, 1, 0, 0, 0)},
#endif /* CONFIG_LPC178X_ETH */

};

/*
 * Configure all necessary GPIO pins
 */
static void gpio_init(void)
{
	struct lpc178x_gpio_dsc dsc;

	/*
	 * Enable power on GPIO. This is not really necessary, because power
	 * on GPIO is enabled on SoC reset.
	 */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCGPIO_MSK, 1);

    /* reset Ethernet PHY KSZ8081 */
	/* P0.4 - Reset for KSZ8081 PHY */
    dsc.port = 0;
    dsc.pin  = 4;
    lpc178x_gpio_config(&dsc, LPC178X_GPIO_CONFIG_D(0, LPC178X_NO_PULLUP, 0, 0, 0, 0));
    lpc178x_gpio_config_direction(&dsc, 1);
    lpc178x_gpout_set(&dsc, 0);
    udelay(1000);
    lpc178x_gpout_set(&dsc, 1);
    udelay(1000);

	/*
	 * Configure GPIO pins using the `ea_lpc1788_gpio[]` table
	 */
	lpc178x_gpio_config_table(ea_lpc1788_gpio, ARRAY_SIZE(ea_lpc1788_gpio));

#ifdef CONFIG_NR_DRAM_BANKS
	/*
	 * Configure GPIO pins used for the External Memory Controller (EMC)
	 */
	/* Configure EMC data pins (DQ0..DQ15) */
	dsc.port = 3;
	for (dsc.pin = 0; dsc.pin < LPC178X_EMC_DATA_PINS; dsc.pin++)
		lpc178x_gpio_config(&dsc, LPC178X_GPIO_EMC_REGVAL);

	/*
	 * Configure EMC row/column address pins (A0..A12) and
	 * NOR FLash address pins.
	*/
	dsc.port = 4;
	for (dsc.pin = 0; dsc.pin < LPC178X_EMC_ADDR_PINS; dsc.pin++)
		lpc178x_gpio_config(&dsc, LPC178X_GPIO_EMC_REGVAL);
#endif
}

/*
 * Early hardware init.
 */
int board_init(void)
{
    struct lpc178x_gpio_dsc dsc;
    
	/* Enable power on EMC */
	lpc178x_periph_enable(LPC178X_SCC_PCONP_PCEMC_MSK, 1);
	/* Clock delay for EMC */
	LPC178X_SCC->emcdlyctl = (CLK_DELAY) | (CLK_DELAY << 8);
	/* Enable EMC, normal memory map, normal mode */
	LPC178X_EMC->emcctrl = 1;
	/* Little-endian mode */
	LPC178X_EMC->emccfg = 0;
	/* Enable GPIO pins */
	gpio_init();

	return 0;
}

/*
 * Dump pertinent info to the console.
 */
int checkboard(void)
{
	printf("Board: Module LPC4088, www.ucsimply.cz\n");

	return 0;
}

/*
 * Configure board specific parts.
 */
#ifdef CONFIG_MISC_INIT_R
int misc_init_r(void)
{
	/* TBD */
	return 0;
}
#endif /* CONFIG_MISC_INIT_R */

/*
 * Setup external RAM.
 */
int dram_init(void)
{
	volatile struct lpc178x_emc_dy_regs *dy;
	u32 tmp32;

	dy = &LPC178X_EMC->dy[CONFIG_SYS_RAM_CS];

	/* Address mapping - DynMem Configuration */
	dy->cfg = SDR_CONFIG;

    /* RAS and CAS delay */
	dy->rascas = SDR_RAS_CAS;
    
    /* Read strategy - command delayed */
	LPC178X_EMC->dy_rdcfg = SDR_READ_CFG;

	/* Configure DRAM timing */
	LPC178X_EMC->dy_trp   = SDR_tRP;
	LPC178X_EMC->dy_tras  = SDR_tRAS;
	LPC178X_EMC->dy_srex  = SDR_tSREX;
	LPC178X_EMC->dy_apr   = SDR_tAPR;
	LPC178X_EMC->dy_dal   = SDR_tDAL;
	LPC178X_EMC->dy_wr    = SDR_tWR;
	LPC178X_EMC->dy_rc    = SDR_tRC;
	LPC178X_EMC->dy_rfc   = SDR_tRFC;
	LPC178X_EMC->dy_xsr   = SDR_tXSR;
	LPC178X_EMC->dy_rrd   = SDR_tRRD;
	LPC178X_EMC->dy_mrd   = SDR_tMRD;
	udelay(100000);

	/* Issue SDRAM NOP (no operation) command */
	LPC178X_EMC->dy_ctrl = SDR_CMD_NOP;
	udelay(200000);

	/* Pre-charge all with fast refresh */
	LPC178X_EMC->dy_ctrl = SDR_CMD_PALL;
	LPC178X_EMC->dy_rfsh = 2;
	udelay(1000);

	/* Set refresh period */
	LPC178X_EMC->dy_rfsh = SDR_REFRESH;

	/* MODE command */
	LPC178X_EMC->dy_ctrl = SDR_CMD_MODE;
    /* Read from the address to set the SDRAM Mode register */
    tmp32 = *(volatile u32 *)(SDR_ADDR_MODEREG);

	/* Normal mode */
	LPC178X_EMC->dy_ctrl = SDR_CMD_NORMAL;

	/* Enable DRAM buffer */
	dy->cfg |= (1 << 19);
	udelay(1000);

	/*
	 * Fill in global info with description of DRAM configuration
	 */
	gd->bd->bi_dram[0].start = CONFIG_SYS_RAM_BASE;
	gd->bd->bi_dram[0].size  = CONFIG_SYS_RAM_SIZE;

	return 0;
}

#ifdef CONFIG_LPC178X_ETH
/*
 * Register ethernet driver
 */
int board_eth_init(bd_t *bis)
{
	return lpc178x_eth_driver_init(bis);
}
#endif

