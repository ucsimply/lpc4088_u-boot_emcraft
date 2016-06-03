/*
 * (C) Copyright 2016
 *
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

#include <common.h>

/*
 * SPIFI interface register map
 * Should be mapped at 0x20094000.
 * 
 * SPIFI interface is available only
 * on LPC40XX and LPC1773 devices 
 */
struct lpc40xx_spifi_regs {
	u32	ctrl;
	u32	cmd;
	u32	addr;
	u32	idata;
	u32	climit;
	union {
		u8	data8;
		u16	data16;
		u32	data32;
	};
	u32	mcmd;
	u32	stat;
};

/*
 * SPIFI registers base
 */
#define LPC40XX_SPIFI_BASE		0x20094000
#define LPC40XX_SPIFI			((volatile struct lpc40xx_spifi_regs *) \
					LPC40XX_SPIFI_BASE)

/*
 * SPIFI registers bits
 */
#define SPIFI_CTRL_TIMEOUT		(100 << 0)
#define SPIFI_CTRL_CSHIGH		(10 << 16)
#define SPIFI_CTRL_MODE3		(1 << 23)
#define SPIFI_CTRL_RFCLK		(1 << 29)
#define SPIFI_CTRL_FBCLK		(1 << 30)

#define SPIFI_CMD_DATALEN_BIT		0
#define SPIFI_CMD_POLL			(1 << 14)
#define SPIFI_CMD_DOUT			(1 << 15)
#define SPIFI_CMD_INTLEN_1		(1 << 16)
#define SPIFI_CMD_INTLEN_2		(2 << 16)
#define SPIFI_CMD_INTLEN_3		(3 << 16)
#define SPIFI_CMD_INTLEN_4		(4 << 16)
#define SPIFI_CMD_FIELDF_SALL		(0 << 19)
#define SPIFI_CMD_FIELDF_QDTA		(1 << 19)
#define SPIFI_CMD_FIELDF_SOPC		(2 << 19)
#define SPIFI_CMD_FIELDF_QALL		(3 << 19)
#define SPIFI_CMD_FRAMEF_OPNA		(1 << 21)
#define SPIFI_CMD_FRAMEF_OP1A		(2 << 21)
#define SPIFI_CMD_FRAMEF_OP2A		(3 << 21)
#define SPIFI_CMD_FRAMEF_OP3A		(4 << 21)
#define SPIFI_CMD_FRAMEF_OP4A		(5 << 21)
#define SPIFI_CMD_FRAMEF_NOP3A		(6 << 21)
#define SPIFI_CMD_FRAMEF_NOP4A		(7 << 21)
#define SPIFI_CMD_OPCODE_BIT		24

#define SPIFI_STAT_MCINIT		(1 << 0)
#define SPIFI_STAT_CMD			(1 << 1)
#define SPIFI_STAT_RESET		(1 << 4)

/*
 * Flash OP codes
 */
#define OP_READ_JID			0x9F
#define OP_WRITE_ENB			0x06
#define OP_READ_SREG1			0x05
#define OP_READ_SREG2			0x35
#define OP_WRITE_SREG			0x01
#define OP_READN			0x03
#define OP_READF			0x0B
#define OP_READQ			0x6B

#define SREG1_BUSY			(1 << 0)
#define SREG1_WENBL			(1 << 1)
#define SREG2_QENB			(1 << 9)		


int spifi_initialize(void)
{
	u8 buff[3];
	
	/* Reset the SPIFI interface */
	LPC40XX_SPIFI->stat = SPIFI_STAT_RESET;
	while (LPC40XX_SPIFI->stat & SPIFI_STAT_RESET);
	
	LPC40XX_SPIFI->ctrl = 0;
	
	/* Read JEDEC ID */
	LPC40XX_SPIFI->cmd = 
		(OP_READ_JID << SPIFI_CMD_OPCODE_BIT) |
		SPIFI_CMD_FRAMEF_OPNA | 3;
	buff[0] = LPC40XX_SPIFI->data8;
	buff[1] = LPC40XX_SPIFI->data8;
	buff[2] = LPC40XX_SPIFI->data8;
	while (LPC40XX_SPIFI->stat & SPIFI_STAT_CMD);
	
	printf("SPIFI: JEDEC-ID=0x%02X%02X%02X\n", buff[0], buff[1], buff[2]);
	
	/* Read status regs */
	LPC40XX_SPIFI->cmd = 
		(OP_READ_SREG1 << SPIFI_CMD_OPCODE_BIT) |
		SPIFI_CMD_FRAMEF_OPNA | 1;
	buff[0] = LPC40XX_SPIFI->data8;
	while (LPC40XX_SPIFI->stat & SPIFI_STAT_CMD);
	
	LPC40XX_SPIFI->cmd = 
		(OP_READ_SREG2 << SPIFI_CMD_OPCODE_BIT) |
		SPIFI_CMD_FRAMEF_OPNA | 1;
	buff[1] = LPC40XX_SPIFI->data8;
	while (LPC40XX_SPIFI->stat & SPIFI_STAT_CMD);
	
	printf("SPIFI: SREG=0x%02X%02X\n", buff[1], buff[0]);
	
	/* switch to memory mode */
	LPC40XX_SPIFI->ctrl = SPIFI_CTRL_TIMEOUT | SPIFI_CTRL_CSHIGH;
	LPC40XX_SPIFI->idata = 0xA5;
	LPC40XX_SPIFI->mcmd = 
		(OP_READQ << SPIFI_CMD_OPCODE_BIT) |
		SPIFI_CMD_FRAMEF_OP3A |
		SPIFI_CMD_FIELDF_QDTA |
		SPIFI_CMD_INTLEN_1;

	puts("SPIFI: set MEM-mode\n");

	return 0;
}

int spifi_write(ulong offset, const void *buf, ulong len)
{
	puts("SPIFI in MEM-mode, writes not supported.\n");
	return 1;
}

void spifi_cancel_mem_mode(void)
{
}

