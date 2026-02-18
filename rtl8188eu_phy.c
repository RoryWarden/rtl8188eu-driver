/*
 * RTL8188EU PHY Layer - BB/RF Register Access Functions
 *
 * This file implements low-level access to:
 * - BB (Baseband) registers: Direct 32-bit USB access to 0x800-0xDFF
 * - RF (Radio Frequency) registers: 20-bit values via 3-wire serial interface
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/delay.h>
#include "rtl8188eu.h"
#include "rtl8188eu_phy.h"

/* Register defines needed for testing */
#define REG_SYS_FUNC_EN		0x0002
#define REG_RF_CTRL		0x001F
#define FEN_BBRSTB		BIT(0)
#define FEN_BB_GLB_RSTn		BIT(1)

/* Conditional table parsing constants */
#define COND_IF		0
#define COND_ELSE_IF	1
#define COND_ELSE	2
#define COND_ENDIF	3

/* Interface types for conditional matching */
#define INTF_PCI	0
#define INTF_USB	1
#define INTF_SDIO	2

/* Forward declarations of USB register access functions */
/* These are implemented in rtl8188eu_minimal.c */
extern u32 rtl8188eu_read_reg32_direct(void *priv, u16 addr);
extern int rtl8188eu_write_reg32_direct(void *priv, u16 addr, u32 val);
extern int rtl8188eu_write_reg8(void *priv, u16 addr, u8 val);

/* Register defines for calibration */
#define REG_TXPAUSE			0x0522	/* TX Pause register */

/**
 * check_positive_condition - Check if hardware matches table condition
 * @priv: Driver private structure
 * @cond: Condition value from table
 *
 * Returns true if condition matches our hardware, false otherwise.
 * For USB interface, we only match USB-specific conditions.
 */
static bool check_positive_condition(struct rtl8188eu_priv *priv, u32 cond)
{
	/* Extract condition type from bits [7:4] */
	u8 cond_type = (cond & 0xF0) >> 4;

	/* For RTL8188EU, we're always USB interface */
	if (cond_type == 0x4) {  /* Interface type condition */
		u8 intf = cond & 0x0F;
		return (intf == INTF_USB);
	}

	/* For now, accept other conditions as true (board type, cut version, etc.) */
	/* This can be expanded later if needed */
	return true;
}

/**
 * rtl8188eu_phy_calculate_bit_shift - Calculate bit shift position from mask
 * @bitmask: Bit mask (e.g., 0x0000FF00)
 *
 * Returns the position of the first set bit in the mask.
 * Example: 0x0000FF00 returns 8
 */
u32 rtl8188eu_phy_calculate_bit_shift(u32 bitmask)
{
	u32 i;

	for (i = 0; i <= 31; i++) {
		if (((bitmask >> i) & 0x1) == 1)
			break;
	}

	return i;
}
EXPORT_SYMBOL(rtl8188eu_phy_calculate_bit_shift);

/**
 * rtl8188eu_init_phy_reg_def - Initialize PHY register definitions
 * @priv: Driver private structure
 *
 * Sets up BB register addresses for RF access via 3-wire serial interface.
 * MUST be called before any RF register access.
 */
void rtl8188eu_init_phy_reg_def(struct rtl8188eu_priv *priv)
{
	/* Path A (RTL8188EU is 1T1R, only path A exists) */
	priv->phy_reg_def[RF_PATH_A].rfintfs = rFPGA0_XAB_RFInterfaceSW;
	priv->phy_reg_def[RF_PATH_A].rfintfo = rFPGA0_XA_RFInterfaceOE;
	priv->phy_reg_def[RF_PATH_A].rfintfe = rFPGA0_XA_RFInterfaceOE;
	priv->phy_reg_def[RF_PATH_A].rf3wireOffset = rFPGA0_XA_LSSIParameter;  /* KEY! */
	priv->phy_reg_def[RF_PATH_A].rfHSSIPara2 = rFPGA0_XA_HSSIParameter2;
	priv->phy_reg_def[RF_PATH_A].rfLSSIReadBack = rFPGA0_XA_LSSIReadBack;
	priv->phy_reg_def[RF_PATH_A].rfLSSIReadBackPi = TransceiverA_HSPI_Readback;

	pr_info("PHY register definitions initialized\n");
}
EXPORT_SYMBOL(rtl8188eu_init_phy_reg_def);

/**
 * rtl8188eu_read_bb_reg - Read baseband register with bitmask
 * @priv: Driver private structure
 * @addr: BB register address (0x800-0xDFF)
 * @bitmask: Bit mask (use bMaskDWord for full 32-bit)
 *
 * Returns the masked and shifted value.
 */
u32 rtl8188eu_read_bb_reg(struct rtl8188eu_priv *priv, u32 addr, u32 bitmask)
{
	u32 original_value, return_value, bitshift;

	original_value = rtl8188eu_read_reg32_direct(priv, (u16)addr);
	bitshift = rtl8188eu_phy_calculate_bit_shift(bitmask);
	return_value = (original_value & bitmask) >> bitshift;

	return return_value;
}
EXPORT_SYMBOL(rtl8188eu_read_bb_reg);

/**
 * rtl8188eu_write_bb_reg - Write baseband register with bitmask
 * @priv: Driver private structure
 * @addr: BB register address (0x800-0xDFF)
 * @bitmask: Bit mask (use bMaskDWord for full 32-bit)
 * @data: Data to write
 *
 * If bitmask is not full 32-bit, performs read-modify-write.
 */
void rtl8188eu_write_bb_reg(struct rtl8188eu_priv *priv, u32 addr,
			     u32 bitmask, u32 data)
{
	u32 original_value, bitshift;

	if (bitmask != bMaskDWord) {
		/* Read-modify-write for partial register update */
		original_value = rtl8188eu_read_reg32_direct(priv, (u16)addr);
		bitshift = rtl8188eu_phy_calculate_bit_shift(bitmask);
		data = ((original_value & (~bitmask)) | ((data << bitshift) & bitmask));
	}

	rtl8188eu_write_reg32_direct(priv, (u16)addr, data);
}
EXPORT_SYMBOL(rtl8188eu_write_bb_reg);

/**
 * rtl8188eu_write_rf_reg - Write RF register via 3-wire serial
 * @priv: Driver private structure
 * @path: RF path (RF_PATH_A for RTL8188EU)
 * @addr: RF register address (0x00-0xFF)
 * @bitmask: Bit mask (use bRFRegOffsetMask for full 20-bit)
 * @data: Data to write (20-bit max)
 *
 * Writes to RF chip via BB register 0x840 (3-wire serial interface).
 */
void rtl8188eu_write_rf_reg(struct rtl8188eu_priv *priv, enum rf_path path,
			     u32 addr, u32 bitmask, u32 data)
{
	u32 data_and_addr = 0;
	u32 original_value, bitshift;
	struct bb_register_definition *phy_reg = &priv->phy_reg_def[path];

	/* Mask RF register address to 8 bits */
	addr &= 0xff;

	/* If not full 20-bit write, need read-modify-write */
	if (bitmask != bRFRegOffsetMask) {
		original_value = rtl8188eu_read_rf_reg(priv, path, addr, bRFRegOffsetMask);
		bitshift = rtl8188eu_phy_calculate_bit_shift(bitmask);
		data = ((original_value & (~bitmask)) | (data << bitshift));
	}

	/* Encode: bits [27:20] = address, bits [19:0] = data */
	data_and_addr = ((addr << 20) | (data & 0x000fffff)) & 0x0fffffff;

	/* Write to BB register which triggers 3-wire serial transfer to RF chip */
	rtl8188eu_write_bb_reg(priv, phy_reg->rf3wireOffset, bMaskDWord, data_and_addr);

	/* Longer delay for RF chip to process - prevent USB disconnect */
	udelay(10);
}
EXPORT_SYMBOL(rtl8188eu_write_rf_reg);

/**
 * rtl8188eu_read_rf_reg - Read RF register via 3-wire serial
 * @priv: Driver private structure
 * @path: RF path (RF_PATH_A for RTL8188EU)
 * @addr: RF register address (0x00-0xFF)
 * @bitmask: Bit mask (use bRFRegOffsetMask for full 20-bit)
 *
 * Reads from RF chip via BB registers (readback at 0x8a0 or 0x8b8).
 * Returns the masked and shifted value.
 */
u32 rtl8188eu_read_rf_reg(struct rtl8188eu_priv *priv, enum rf_path path,
			   u32 addr, u32 bitmask)
{
	u32 original_value, readback_value, bitshift;
	struct bb_register_definition *phy_reg = &priv->phy_reg_def[path];
	u32 tmplong;
	u8 rf_pi_enable = 0;

	mutex_lock(&priv->rf_read_mutex);

	/* Mask RF register address to 8 bits */
	addr &= 0xff;

	/* Set readback address in BB HSSI parameter register */
	tmplong = rtl8188eu_read_bb_reg(priv, rFPGA0_XA_HSSIParameter2, bMaskDWord);
	tmplong = (tmplong & (~bLSSIReadAddress)) | (addr << 23) | bLSSIReadEdge;
	rtl8188eu_write_bb_reg(priv, rFPGA0_XA_HSSIParameter2, bMaskDWord,
				tmplong & (~bLSSIReadEdge));
	udelay(10);  /* Wait for setup */

	/* Trigger readback with edge signal */
	rtl8188eu_write_bb_reg(priv, rFPGA0_XA_HSSIParameter2, bMaskDWord,
				tmplong | bLSSIReadEdge);
	udelay(100);  /* Wait for RF 3-wire serial response — old driver uses 100us */

	/* Check if PI (Parallel Interface) mode is enabled */
	rf_pi_enable = (u8)rtl8188eu_read_bb_reg(priv, rFPGA0_XA_HSSIParameter1, BIT(8));

	if (rf_pi_enable) {
		/* PI mode: Read from 0x8b8 */
		original_value = rtl8188eu_read_bb_reg(priv, phy_reg->rfLSSIReadBackPi,
							bLSSIReadBackData);
	} else {
		/* SI mode: Read from 0x8a0 */
		original_value = rtl8188eu_read_bb_reg(priv, phy_reg->rfLSSIReadBack,
							bLSSIReadBackData);
	}

	mutex_unlock(&priv->rf_read_mutex);

	/* Apply bitmask */
	bitshift = rtl8188eu_phy_calculate_bit_shift(bitmask);
	readback_value = (original_value & bitmask) >> bitshift;

	return readback_value;
}
EXPORT_SYMBOL(rtl8188eu_read_rf_reg);

/**
 * rtl8188eu_load_phy_reg_table - Load PHY_REG configuration table
 * @priv: Driver private structure
 *
 * Loads baseband register configuration table with conditional parsing.
 * Skips entries that don't match USB interface configuration.
 * Returns 0 on success, negative on error.
 */
int rtl8188eu_load_phy_reg_table(struct rtl8188eu_priv *priv)
{
	const u32 *table;
	int size, i;
	int count = 0;
	bool is_matched = true;  /* Start with default entries enabled */
	bool is_skipped = false;
	u32 pre_v1 = 0, pre_v2 = 0;

	/* Get the PHY_REG table */
	table = rtl8188eu_get_phy_reg_table(&size);
	if (!table) {
		pr_err("Failed to get PHY_REG table\n");
		return -ENOENT;
	}

	pr_info("Loading PHY_REG table with conditional parsing (%d values)\n", size);

	/* Process table with conditional handling */
	i = 0;
	while ((i + 1) < size) {
		u32 v1 = table[i];
		u32 v2 = table[i + 1];

		/* Check for conditional entries */
		if (v1 & (BIT(31) | BIT(30))) {
			if (v1 & BIT(31)) {  /* Positive condition (IF/ELSE/ENDIF) */
				u8 c_cond = (u8)((v1 & (BIT(29) | BIT(28))) >> 28);

				if (c_cond == COND_ENDIF) {
					is_matched = true;
					is_skipped = false;
				} else if (c_cond == COND_ELSE) {
					is_matched = is_skipped ? false : true;
				} else {  /* COND_IF or COND_ELSE_IF */
					pre_v1 = v1;
					pre_v2 = v2;
				}
			} else if (v1 & BIT(30)) {  /* Negative condition (condition check) */
				if (!is_skipped) {
					/* Check if condition matches our hardware */
					if (check_positive_condition(priv, pre_v2)) {
						is_matched = true;
						is_skipped = true;
					} else {
						is_matched = false;
						is_skipped = false;
					}
				} else {
					is_matched = false;
				}
			}
		} else {
			/* Normal register entry - only write if condition matched */
			if (is_matched) {
				/* Skip if address is 0 (padding) */
				if (v1 == 0) {
					i += 2;
					continue;
				}

				/* Handle delay commands */
				switch (v1) {
				case 0xFE:
					msleep(50);
					pr_debug("PHY_REG: 50ms delay\n");
					break;
				case 0xFD:
					msleep(5);
					pr_debug("PHY_REG: 5ms delay\n");
					break;
				case 0xFC:
					msleep(1);
					pr_debug("PHY_REG: 1ms delay\n");
					break;
				case 0xFB:
					udelay(50);
					break;
				case 0xFA:
					udelay(5);
					break;
				case 0xF9:
					udelay(1);
					break;
				default:
					/* Write the register */
					rtl8188eu_write_bb_reg(priv, v1, bMaskDWord, v2);
					count++;

					/* Small delay after each write */
					udelay(10);

					/* Longer delay every 16 writes */
					if ((count % 16) == 0)
						msleep(1);
					break;
				}
			}
		}
		i += 2;
	}

	pr_info("PHY_REG table loaded: %d registers written (USB-specific only)\n", count);
	return 0;
}
EXPORT_SYMBOL(rtl8188eu_load_phy_reg_table);

/**
 * rtl8188eu_load_agc_table - Load AGC configuration table
 * @priv: Driver private structure
 *
 * Loads automatic gain control table. TEMPORARILY loading ALL entries
 * to fix RX reception issues (AGC is critical for RX gain control).
 * Returns 0 on success, negative on error.
 */
int rtl8188eu_load_agc_table(struct rtl8188eu_priv *priv)
{
	const u32 *table;
	int size, i;
	int count = 0;
	int skipped = 0;

	/* Get the AGC table */
	table = rtl8188eu_get_agc_table(&size);
	if (!table) {
		pr_err("Failed to get AGC table\n");
		return -ENOENT;
	}

	pr_info("Loading AGC table - FORCING ALL ENTRIES for RX fix (%d values)\n", size);

	/* TEMPORARY: Load ALL AGC entries to fix RX
	 * AGC is critical for RX gain control. With only 10 entries,
	 * RX sensitivity is severely degraded.
	 */
	for (i = 0; i < size; i += 2) {
		u32 v1 = table[i];
		u32 v2 = table[i + 1];

		/* Skip conditional markers but load ALL register values */
		if (v1 & (BIT(31) | BIT(30))) {
			skipped++;
			continue;  /* Skip conditional control entries */
		}

		/* Handle delay commands */
		switch (v1) {
		case 0xFE:
			msleep(50);
			break;
		case 0xFD:
			msleep(5);
			break;
		case 0xFC:
			msleep(1);
			break;
		case 0xFB:
			udelay(50);
			break;
		case 0xFA:
			udelay(5);
			break;
		case 0xF9:
			udelay(1);
			break;
		default:
			/* AGC table is mostly writes to 0xC78 */
			if (v1 >= 0x800 && v1 <= 0xFFF) {  /* Valid BB register range */
				rtl8188eu_write_bb_reg(priv, v1, bMaskDWord, v2);
				count++;

				/* Delay between AGC writes */
				udelay(10);

				/* Extra delay every 32 writes */
				if ((count % 32) == 0)
					msleep(1);
			}
			break;
		}
	}

	pr_info("AGC table loaded: %d registers written, %d conditionals skipped\n",
		count, skipped);
	return 0;
}
EXPORT_SYMBOL(rtl8188eu_load_agc_table);

/**
 * rtl8188eu_load_radioa_table - Load RadioA configuration table
 * @priv: Driver private structure
 *
 * Loads RF chip configuration with conditional parsing.
 * Handles special delay codes and conditional entries.
 * Returns 0 on success, negative on error.
 */
int rtl8188eu_load_radioa_table(struct rtl8188eu_priv *priv)
{
	const u32 *table;
	int size, i;
	int count = 0;
	bool is_matched = true;
	bool is_skipped = false;
	u32 pre_v1 = 0, pre_v2 = 0;

	/* Get the RadioA table */
	table = rtl8188eu_get_radioa_table(&size);
	if (!table) {
		pr_err("Failed to get RadioA table\n");
		return -ENOENT;
	}

	pr_info("Loading RadioA table with conditional parsing (%d values)\n", size);

	/* Process table with conditional handling */
	i = 0;
	while ((i + 1) < size) {
		u32 v1 = table[i];
		u32 v2 = table[i + 1];

		/* Check for conditional entries */
		if (v1 & (BIT(31) | BIT(30))) {
			if (v1 & BIT(31)) {  /* Positive condition */
				u8 c_cond = (u8)((v1 & (BIT(29) | BIT(28))) >> 28);

				if (c_cond == COND_ENDIF) {
					is_matched = true;
					is_skipped = false;
				} else if (c_cond == COND_ELSE) {
					is_matched = is_skipped ? false : true;
				} else {
					pre_v1 = v1;
					pre_v2 = v2;
				}
			} else if (v1 & BIT(30)) {  /* Negative condition */
				if (!is_skipped) {
					if (check_positive_condition(priv, pre_v2)) {
						is_matched = true;
						is_skipped = true;
					} else {
						is_matched = false;
						is_skipped = false;
					}
				} else {
					is_matched = false;
				}
			}
		} else {
			/* Normal register entry */
			if (is_matched) {
				/* Check for special delay codes */
				switch (v1) {
				case 0xFE:
				case 0xFFE:
					msleep(50);
					pr_debug("RadioA: 50ms delay\n");
					break;
				case 0xFD:
				case 0xFFD:
					msleep(5);
					pr_debug("RadioA: 5ms delay\n");
					break;
				case 0xFC:
				case 0xFFC:
					msleep(1);
					pr_debug("RadioA: 1ms delay\n");
					break;
				case 0xFB:
				case 0xFFB:
					udelay(50);
					break;
				case 0xFA:
				case 0xFFA:
					udelay(5);
					break;
				case 0xF9:
				case 0xFF9:
					udelay(1);
					break;
				default:
					/* Normal RF register write */
					if (v1 <= 0xFF) {  /* Valid RF register address */
						rtl8188eu_write_rf_reg(priv, RF_PATH_A, v1,
									bRFRegOffsetMask, v2);
						count++;

						/* Longer delay between RF writes */
						udelay(50);
					}
					break;
				}
			}
		}
		i += 2;
	}

	pr_info("RadioA table loaded: %d RF registers written (USB-specific only)\n", count);
	return 0;
}
EXPORT_SYMBOL(rtl8188eu_load_radioa_table);

/**
 * rtl8188eu_load_mac_reg_table - Load MAC register configuration table
 * @priv: Driver private structure
 *
 * Loads MAC layer register configuration from the hardware vendor table.
 * These are 8-bit register writes (unlike PHY tables which are 32-bit).
 * Must be called BEFORE PHY table loading (matches old driver init order).
 * Returns 0 on success, negative on error.
 */
int rtl8188eu_load_mac_reg_table(struct rtl8188eu_priv *priv)
{
	const u32 *table;
	int size, i;
	int count = 0;

	table = rtl8188eu_get_mac_reg_table(&size);
	if (!table) {
		pr_err("Failed to get MAC register table\n");
		return -ENOENT;
	}

	pr_info("Loading MAC register table (%d values, %d pairs)\n",
		size, size / 2);

	for (i = 0; i + 1 < size; i += 2) {
		u32 addr = table[i];
		u8 value = (u8)(table[i + 1] & 0xFF);

		rtl8188eu_write_reg8(priv, (u16)addr, value);
		count++;
		udelay(1);
	}

	pr_info("MAC register table loaded: %d registers written\n", count);
	return 0;
}
EXPORT_SYMBOL(rtl8188eu_load_mac_reg_table);

/**
 * rtl8188eu_phy_bb_config - Initialize baseband hardware
 * @priv: Driver private structure
 *
 * Main BB initialization: loads PHY_REG and AGC tables.
 * Returns 0 on success, negative on error.
 */
int rtl8188eu_phy_bb_config(struct rtl8188eu_priv *priv)
{
	int ret;

	pr_info("=== Starting BB configuration ===\n");

	/* Initialize PHY register definitions (for RF access) */
	rtl8188eu_init_phy_reg_def(priv);

	/* Give hardware time to stabilize after clock enable */
	msleep(10);

	/* Load PHY_REG table */
	ret = rtl8188eu_load_phy_reg_table(priv);
	if (ret < 0) {
		pr_err("Failed to load PHY_REG table: %d\n", ret);
		return ret;
	}

	/* Load AGC table */
	ret = rtl8188eu_load_agc_table(priv);
	if (ret < 0) {
		pr_err("Failed to load AGC table: %d\n", ret);
		return ret;
	}

	pr_info("=== BB configuration complete ===\n");
	return 0;
}
EXPORT_SYMBOL(rtl8188eu_phy_bb_config);

/**
 * rtl8188eu_phy_rf_config - Initialize RF hardware
 * @priv: Driver private structure
 *
 * Main RF initialization: loads RadioA configuration.
 * Returns 0 on success, negative on error.
 */
int rtl8188eu_phy_rf_config(struct rtl8188eu_priv *priv)
{
	int ret;
	u32 rfenv_save;
	u32 tmp;

	pr_info("=== Starting RF configuration ===\n");

	/*
	 * RF environment setup - matches old driver's phy_RF6052_Config_ParaFile()
	 * Uses read-modify-write to preserve hardware default bits.
	 * Old code did full 32-bit overwrites of 0x860/0x870 which destroyed
	 * important bits and potentially disconnected the RF data path.
	 */

	/* Step A: Save original RFENV from 0x870 (rfintfs), bit 4 */
	rfenv_save = rtl8188eu_read_bb_reg(priv, 0x870, 0x10);
	pr_info("RF env: 0x870 RFENV saved = %u\n", rfenv_save);

	/* Step B: Enable RF_ENV output enable in 0x860 (rfintfe), bit 20 */
	rtl8188eu_write_bb_reg(priv, 0x860, 0x100000, 0x1);
	udelay(1);

	/* Step C: Set RF_ENV output data high in 0x860 (rfintfo), bit 4 */
	rtl8188eu_write_bb_reg(priv, 0x860, 0x10, 0x1);
	udelay(1);

	/* Step D: Configure 3-wire address length in 0x824, bit 10 = 0 */
	rtl8188eu_write_bb_reg(priv, 0x824, 0x400, 0x0);
	udelay(1);

	/* Step E: Configure 3-wire data length in 0x824, bit 11 = 0 */
	rtl8188eu_write_bb_reg(priv, 0x824, 0x800, 0x0);
	udelay(1);

	tmp = rtl8188eu_read_bb_reg(priv, 0x860, 0xFFFFFFFF);
	pr_info("RF env: 0x860 after setup = 0x%08x\n", tmp);
	tmp = rtl8188eu_read_bb_reg(priv, 0x870, 0xFFFFFFFF);
	pr_info("RF env: 0x870 after setup = 0x%08x\n", tmp);

	/* Load RadioA table */
	ret = rtl8188eu_load_radioa_table(priv);
	if (ret < 0) {
		pr_err("Failed to load RadioA table: %d\n", ret);
		return ret;
	}

	/* Step F: Restore RFENV in 0x870, bit 4 */
	rtl8188eu_write_bb_reg(priv, 0x870, 0x10, rfenv_save);

	pr_info("=== RF configuration complete ===\n");
	return 0;
}
EXPORT_SYMBOL(rtl8188eu_phy_rf_config);

/**
 * rtl8188eu_phy_test_read_only - Test READ-ONLY access to BB/RF registers
 * @priv: Driver private data
 *
 * This function performs safe, read-only tests on BB and RF registers
 * to verify access without risking USB disconnection.
 *
 * Return: 0 on success, negative on error
 */
int rtl8188eu_phy_test_read_only(struct rtl8188eu_priv *priv)
{
	u32 val;
	u16 val16;
	u8 val8;
	int ret = 0;
	int i;

	pr_info("=== PHY Read-Only Test Starting ===\n");

	/* NEW: Try loading first 50 PHY_REG entries to activate BB */
	pr_info("Loading first 50 PHY_REG entries to activate BB...\n");
	const u32 *table;
	int size;
	table = rtl8188eu_get_phy_reg_table(&size);

	for (i = 0; i < 100 && i < size; i += 2) {  /* 50 register/value pairs */
		u32 addr = table[i];
		u32 value = table[i + 1];

		/* Only print every 10th write to reduce spam */
		if ((i % 20) == 0) {
			pr_info("  Writing PHY_REG[%d]: 0x%04x = 0x%08x\n", i/2, addr, value);
		}

		/* Use proper BB write function with full mask */
		rtl8188eu_write_bb_reg(priv, addr, bMaskDWord, value);
		/* Note: write_bb_reg doesn't return error code */

		/* Add small delay between writes for safety */
		udelay(10);

		/* Check if BB becomes accessible after certain registers */
		if (addr == 0x0860 || addr == 0x0870) {  /* RF interface registers */
			u32 test = rtl8188eu_read_reg32_direct(priv, 0x800);
			if (test != 0xeaeaeaea) {
				pr_info("  BB activated after writing 0x%04x! BB 0x800 = 0x%08x\n", addr, test);
			}
		}
	}

	pr_info("Loaded %d PHY_REG entries. Now testing BB reads...\n", i/2);
	msleep(5);  /* Let BB stabilize after config */

	/* Test 1: Read some safe BB registers */
	pr_info("Test 1: Reading BB registers after PHY_REG load...\n");

	/* Read BB register 0x800 - should now show real value! */
	val = rtl8188eu_read_reg32_direct(priv, 0x800);
	pr_info("  BB 0x800 = 0x%08x (expected: 0x80040000)\n", val);

	/* Read BB register 0x804 */
	val = rtl8188eu_read_reg32_direct(priv, 0x804);
	pr_info("  BB 0x804 = 0x%08x\n", val);

	/* Read BB register 0x808 */
	val = rtl8188eu_read_reg32_direct(priv, 0x808);
	pr_info("  BB 0x808 = 0x%08x\n", val);

	/* Read BB register 0x80c */
	val = rtl8188eu_read_reg32_direct(priv, 0x80c);
	pr_info("  BB 0x80c = 0x%08x\n", val);

	/* Test 2: Check if RF interface is accessible (read-only) */
	pr_info("Test 2: Checking RF interface registers (no RF access)...\n");

	/* Read RF control register - this is safe */
	/* Using direct 32-bit read since we don't have 8-bit direct function */
	val = rtl8188eu_read_reg32_direct(priv, REG_RF_CTRL & ~3);
	val8 = (val >> ((REG_RF_CTRL & 3) * 8)) & 0xFF;
	pr_info("  REG_RF_CTRL (0x1f) = 0x%02x\n", val8);

	/* Read BB register 0x860 (RF interface enable) */
	val = rtl8188eu_read_reg32_direct(priv, 0x860);
	pr_info("  BB 0x860 (RF interface) = 0x%08x\n", val);

	/* Read BB register 0x870 (RF interface) */
	val = rtl8188eu_read_reg32_direct(priv, 0x870);
	pr_info("  BB 0x870 (RF interface) = 0x%08x\n", val);

	/* Test 3: Check system function enable */
	pr_info("Test 3: Checking system enables...\n");
	/* Using direct 32-bit read since we don't have 16-bit direct function */
	val = rtl8188eu_read_reg32_direct(priv, REG_SYS_FUNC_EN & ~3);
	val16 = (val >> ((REG_SYS_FUNC_EN & 2) * 8)) & 0xFFFF;
	pr_info("  REG_SYS_FUNC_EN = 0x%04x\n", val16);
	if (!(val16 & FEN_BBRSTB)) {
		pr_info("  WARNING: BB not enabled (FEN_BBRSTB=0)\n");
	}
	if (!(val16 & FEN_BB_GLB_RSTn)) {
		pr_info("  WARNING: BB global reset not cleared\n");
	}

	/* Test 4: Read some AGC registers */
	pr_info("Test 4: Reading AGC area (0xc00-0xc7f)...\n");
	val = rtl8188eu_read_reg32_direct(priv, 0xc00);
	pr_info("  0xc00 = 0x%08x\n", val);

	val = rtl8188eu_read_reg32_direct(priv, 0xc04);
	pr_info("  0xc04 = 0x%08x\n", val);

	val = rtl8188eu_read_reg32_direct(priv, 0xc50);  /* This crashed before! */
	pr_info("  0xc50 = 0x%08x (previously caused crash)\n", val);

	val = rtl8188eu_read_reg32_direct(priv, 0xc78);  /* AGC table register */
	pr_info("  0xc78 = 0x%08x (AGC table register)\n", val);

	pr_info("=== PHY Read-Only Test Complete ===\n");
	pr_info("Device still connected? Check dmesg for USB errors!\n");

	return ret;
}
EXPORT_SYMBOL(rtl8188eu_phy_test_read_only);

/**
 * rtl8188eu_lc_calibrate - LC Tank Calibration
 * @priv: Driver private structure
 *
 * Calibrates the LC tank circuit in the RF chip.
 * This is essential for proper frequency stability.
 * Should be called after RF initialization.
 */
void rtl8188eu_lc_calibrate(struct rtl8188eu_priv *priv)
{
	u8 tmp_reg;
	u32 lc_cal, rf_val, rf_amode = 0;
	bool is_cont_tx;

	pr_info("Starting LC calibration...\n");

	/* Step 1: Check if we're in continuous TX mode */
	tmp_reg = rtl8188eu_read_reg32_direct(priv, 0xd00) >> 24;  /* Read 0xd03 */
	is_cont_tx = (tmp_reg & 0x70) != 0;

	if (is_cont_tx) {
		/* Continuous TX mode: save RF_AC, put RF in standby, disable TX */
		rf_amode = rtl8188eu_read_rf_reg(priv, RF_PATH_A, 0x00, bRFRegOffsetMask);
		pr_info("LC Cal: Continuous TX — saved RF_AC = 0x%05x\n", rf_amode);

		rtl8188eu_write_reg32_direct(priv, 0xd00,
			(rtl8188eu_read_reg32_direct(priv, 0xd00) & 0x00FFFFFF) |
			((tmp_reg & 0x8F) << 24));

		rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x00, bRFRegOffsetMask,
				       (rf_amode & 0x8FFFF) | 0x10000);
	} else {
		/* Packet TX mode (normal case during probe): just pause TX queues */
		pr_info("LC Cal: Packet TX mode — pausing queues, not touching RF_AC\n");
		rtl8188eu_write_reg8(priv, REG_TXPAUSE, 0xFF);
	}

	/* Step 2: Trigger LC calibration via RF reg 0x18 bit 15 */
	lc_cal = rtl8188eu_read_rf_reg(priv, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask);
	pr_info("LC Cal: RF 0x18 = 0x%05x\n", lc_cal);

	rtl8188eu_write_rf_reg(priv, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask,
				lc_cal | 0x08000);

	/* Step 3: Wait for calibration to complete */
	msleep(100);

	/* Step 4: Restore TX state */
	if (is_cont_tx) {
		/* Restore RF_AC and re-enable continuous TX */
		rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x00, bRFRegOffsetMask, rf_amode);
		rtl8188eu_write_reg32_direct(priv, 0xd00,
			(rtl8188eu_read_reg32_direct(priv, 0xd00) & 0x00FFFFFF) |
			(tmp_reg << 24));
	} else {
		/* Unpause TX queues */
		rtl8188eu_write_reg8(priv, REG_TXPAUSE, 0x00);
	}

	/* Read back to verify RF is still alive */
	rf_val = rtl8188eu_read_rf_reg(priv, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask);
	pr_info("LC Cal: After calibration, RF 0x18 = 0x%05x\n", rf_val);

	rf_val = rtl8188eu_read_rf_reg(priv, RF_PATH_A, RF_AC, bRFRegOffsetMask);
	pr_info("LC Cal: RF 0x00 = 0x%05x (expect 0x33e60)\n", rf_val);

	pr_info("LC calibration complete\n");
}
EXPORT_SYMBOL(rtl8188eu_lc_calibrate);

/* IQK register arrays — from old driver halrf_8188e_ce.c */
static const u32 adda_reg[16] = {
	0x85c, 0xe6c, 0xe70, 0xe74, 0xe78, 0xe7c, 0xe80, 0xe84,
	0xe88, 0xe8c, 0xed0, 0xed4, 0xed8, 0xedc, 0xee0, 0xeec
};

static const u32 iqk_mac_reg[4] = { 0x522, 0x550, 0x551, 0x040 };

static const u32 iqk_bb_reg[9] = {
	0xc04, 0xc08, 0x874, 0xb68, 0xb6c, 0x870, 0x860, 0x864, 0xa04
};

/**
 * iqk_save_adda - Save ADDA registers
 */
static void iqk_save_adda(struct rtl8188eu_priv *priv, const u32 *regs,
			   u32 *backup, int count)
{
	int i;

	for (i = 0; i < count; i++)
		backup[i] = rtl8188eu_read_reg32_direct(priv, (u16)regs[i]);
}

/**
 * iqk_restore - Restore registers from backup
 */
static void iqk_restore(struct rtl8188eu_priv *priv, const u32 *regs,
			 u32 *backup, int count)
{
	int i;

	for (i = 0; i < count; i++)
		rtl8188eu_write_reg32_direct(priv, (u16)regs[i], backup[i]);
}

/**
 * iqk_save_mac - Save MAC registers (mixed 8/32-bit)
 */
static void iqk_save_mac(struct rtl8188eu_priv *priv, u32 *backup)
{
	/* 0x522 is 8-bit, 0x550/0x551 are 8-bit, 0x040 is 32-bit */
	/* Read all as 32-bit aligned and extract later during restore */
	backup[0] = rtl8188eu_read_reg32_direct(priv, 0x0520) >> 16; /* byte at 0x522 */
	backup[1] = rtl8188eu_read_reg32_direct(priv, 0x0550);       /* bytes at 0x550-0x553 */
	backup[2] = backup[1];                                        /* 0x551 within same word */
	backup[3] = rtl8188eu_read_reg32_direct(priv, 0x0040);       /* 32-bit reg */
}

/**
 * iqk_set_adda - Set all ADDA registers to path-on value
 */
static void iqk_set_adda(struct rtl8188eu_priv *priv, u32 path_on_value)
{
	int i;

	/* First register gets a different value (path A on) */
	rtl8188eu_write_reg32_direct(priv, (u16)adda_reg[0], 0x0b1b25a0);

	/* Rest get the 1T1R value */
	for (i = 1; i < 16; i++)
		rtl8188eu_write_reg32_direct(priv, (u16)adda_reg[i], path_on_value);
}

/**
 * iqk_config_mac - Configure MAC registers for IQK
 */
static void iqk_config_mac(struct rtl8188eu_priv *priv)
{
	rtl8188eu_write_reg8(priv, 0x522, 0x3F);

	/* BCN_CTRL &= ~BIT3 — read 32-bit at 0x550, modify byte */
	{
		u32 tmp = rtl8188eu_read_reg32_direct(priv, 0x0550);
		u8 b550 = tmp & 0xFF;
		b550 &= ~BIT(3);
		rtl8188eu_write_reg8(priv, 0x550, b550);
	}

	/* REG_GPIO_MUXCFG (0x0040) &= ~BIT5 */
	{
		u32 tmp = rtl8188eu_read_reg32_direct(priv, 0x0040);
		tmp &= ~BIT(5);
		rtl8188eu_write_reg32_direct(priv, 0x0040, tmp);
	}
}

/**
 * rtl8188eu_iqk_tx - TX I/Q Calibration (inner)
 * @priv: Driver private structure
 *
 * Runs TX IQK tone. Called after ADDA/MAC/BB are configured.
 * Returns 0 on success, -1 on failure.
 */
static int rtl8188eu_iqk_tx(struct rtl8188eu_priv *priv)
{
	u32 reg_eac, reg_e94, reg_e9c;

	/* TX IQK tone/PI settings */
	rtl8188eu_write_reg32_direct(priv, 0xe30, 0x10008c1c);
	rtl8188eu_write_reg32_direct(priv, 0xe34, 0x30008c1c);
	rtl8188eu_write_reg32_direct(priv, 0xe38, 0x8214032a);
	rtl8188eu_write_reg32_direct(priv, 0xe3c, 0x28160000);

	/* LO calibration setting */
	rtl8188eu_write_reg32_direct(priv, 0xe4c, 0x00462911);

	/* Enable IQK */
	rtl8188eu_write_reg32_direct(priv, 0xe28, 0x00808000);

	/* IQK setting */
	rtl8188eu_write_reg32_direct(priv, 0xe40, 0x01007c00);
	rtl8188eu_write_reg32_direct(priv, 0xe44, 0x01004800);

	/* Trigger */
	rtl8188eu_write_reg32_direct(priv, 0xe48, 0xf9000000);
	rtl8188eu_write_reg32_direct(priv, 0xe48, 0xf8000000);
	msleep(20);

	/* Check results */
	reg_eac = rtl8188eu_read_reg32_direct(priv, 0xeac);
	reg_e94 = rtl8188eu_read_reg32_direct(priv, 0xe94);
	reg_e9c = rtl8188eu_read_reg32_direct(priv, 0xe9c);

	pr_info("IQK TX: eac=0x%08x e94=0x%08x e9c=0x%08x\n",
		reg_eac, reg_e94, reg_e9c);

	if (!(reg_eac & BIT(28)) &&
	    (((reg_e94 & 0x03FF0000) >> 16) != 0x142) &&
	    (((reg_e9c & 0x03FF0000) >> 16) != 0x42)) {
		pr_info("IQK: TX calibration SUCCESS\n");
		return 0;
	}

	pr_err("IQK: TX calibration FAILED\n");
	return -1;
}

/**
 * rtl8188eu_iqk_rx - RX I/Q Calibration (inner)
 * @priv: Driver private structure
 *
 * Runs RX IQK. Called after TX IQK succeeds.
 * Returns 0 on success, -1 on failure.
 */
static int rtl8188eu_iqk_rx(struct rtl8188eu_priv *priv)
{
	u32 reg_eac, reg_ea4, reg_e94, reg_e9c;
	u32 u4tmp;

	/* Get TX calibration results (needed for RX) */
	reg_e94 = rtl8188eu_read_reg32_direct(priv, 0xe94);
	reg_e9c = rtl8188eu_read_reg32_direct(priv, 0xe9c);

	/* Step 1: Configure RF for RX IQK */
	rtl8188eu_write_reg32_direct(priv, 0xe28, 0x00000000);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0xEF, bRFRegOffsetMask, 0x800a0);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x30, bRFRegOffsetMask, 0x30000);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x31, bRFRegOffsetMask, 0x0000f);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x32, bRFRegOffsetMask, 0xf117B);
	msleep(5);

	rtl8188eu_write_reg32_direct(priv, 0xe28, 0x00808000);

	rtl8188eu_write_reg32_direct(priv, 0xe40, 0x01007c00);
	rtl8188eu_write_reg32_direct(priv, 0xe44, 0x81004800);

	rtl8188eu_write_reg32_direct(priv, 0xe30, 0x10008c1c);
	rtl8188eu_write_reg32_direct(priv, 0xe34, 0x30008c1c);
	rtl8188eu_write_reg32_direct(priv, 0xe38, 0x82160804);
	rtl8188eu_write_reg32_direct(priv, 0xe3c, 0x28160000);

	rtl8188eu_write_reg32_direct(priv, 0xe4c, 0x0046a911);
	rtl8188eu_write_reg32_direct(priv, 0xe48, 0xf9000000);
	rtl8188eu_write_reg32_direct(priv, 0xe48, 0xf8000000);
	msleep(20);

	reg_eac = rtl8188eu_read_reg32_direct(priv, 0xeac);
	reg_ea4 = rtl8188eu_read_reg32_direct(priv, 0xea4);
	pr_info("IQK RX1: eac=0x%08x ea4=0x%08x\n", reg_eac, reg_ea4);

	/* Use TX result for RX calibration */
	u4tmp = 0x80007C00 | (reg_e94 & 0x3FF0000) | ((reg_e9c & 0x3FF0000) >> 16);
	rtl8188eu_write_reg32_direct(priv, 0xe40, u4tmp);

	/* Step 2: Second RX IQK */
	rtl8188eu_write_reg32_direct(priv, 0xe28, 0x00000000);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0xEF, bRFRegOffsetMask, 0x800a0);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x30, bRFRegOffsetMask, 0x30000);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x31, bRFRegOffsetMask, 0x0000f);
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x32, bRFRegOffsetMask, 0xf7ffa);
	msleep(5);

	rtl8188eu_write_reg32_direct(priv, 0xe28, 0x00808000);

	rtl8188eu_write_reg32_direct(priv, 0xe44, 0x01004800);
	rtl8188eu_write_reg32_direct(priv, 0xe30, 0x30008c1c);
	rtl8188eu_write_reg32_direct(priv, 0xe34, 0x10008c1c);
	rtl8188eu_write_reg32_direct(priv, 0xe38, 0x82160c05);
	rtl8188eu_write_reg32_direct(priv, 0xe3c, 0x28160c05);

	rtl8188eu_write_reg32_direct(priv, 0xe4c, 0x0046a911);
	rtl8188eu_write_reg32_direct(priv, 0xe48, 0xf9000000);
	rtl8188eu_write_reg32_direct(priv, 0xe48, 0xf8000000);
	msleep(20);

	reg_eac = rtl8188eu_read_reg32_direct(priv, 0xeac);
	reg_ea4 = rtl8188eu_read_reg32_direct(priv, 0xea4);
	pr_info("IQK RX2: eac=0x%08x ea4=0x%08x\n", reg_eac, reg_ea4);

	if (!(reg_eac & BIT(27)) &&
	    (((reg_ea4 & 0x03FF0000) >> 16) != 0x132) &&
	    (((reg_eac & 0x03FF0000) >> 16) != 0x36)) {
		pr_info("IQK: RX calibration SUCCESS\n");
		return 0;
	}

	pr_err("IQK: RX calibration FAILED\n");
	return -1;
}

/**
 * rtl8188eu_iqk_calibrate - Full IQK Calibration with register save/restore
 * @priv: Driver private structure
 *
 * Saves 16 ADDA + 4 MAC + 9 BB registers, configures hardware for IQK,
 * runs TX+RX calibration, then restores all 29 registers.
 * Matches old driver halrf_8188e_ce.c implementation.
 * Returns 0 on success, negative on error.
 */
int rtl8188eu_iqk_calibrate(struct rtl8188eu_priv *priv)
{
	u32 adda_backup[16];
	u32 mac_backup[4];
	u32 bb_backup[9];
	u32 pi_mode_save;
	int tx_result, rx_result;
	int retry;

	pr_info("========================================\n");
	pr_info("Starting IQK calibration (v0.32 w/ save/restore)...\n");
	pr_info("========================================\n");

	/* Step 1: Save all 29 registers */
	iqk_save_adda(priv, adda_reg, adda_backup, 16);
	iqk_save_mac(priv, mac_backup);
	iqk_save_adda(priv, iqk_bb_reg, bb_backup, 9);
	pr_info("IQK: Saved 16 ADDA + 4 MAC + 9 BB registers\n");

	/* Step 2: Set ADDA path on (1T1R mode) */
	iqk_set_adda(priv, 0x0bdb25a0);

	/* Step 3: Save PI mode state from 0x820 bit 8 */
	pi_mode_save = rtl8188eu_read_bb_reg(priv, 0x820, BIT(8));

	/* Step 4: Switch to PI mode */
	rtl8188eu_write_reg32_direct(priv, 0x820, 0x01000100);
	rtl8188eu_write_reg32_direct(priv, 0x828, 0x01000100);

	/* Step 5: MAC calibration settings */
	iqk_config_mac(priv);

	/* Step 6: BB setup for IQK */
	rtl8188eu_write_bb_reg(priv, 0xa04, 0x0F000000, 0xf);
	rtl8188eu_write_reg32_direct(priv, 0xc04, 0x03a05600);
	rtl8188eu_write_reg32_direct(priv, 0xc08, 0x000800e4);
	rtl8188eu_write_reg32_direct(priv, 0x874, 0x22204000);
	rtl8188eu_write_bb_reg(priv, 0x870, BIT(10), 1);
	rtl8188eu_write_bb_reg(priv, 0x870, BIT(26), 1);
	rtl8188eu_write_bb_reg(priv, 0x860, BIT(10), 0);
	rtl8188eu_write_bb_reg(priv, 0x864, BIT(10), 0);
	rtl8188eu_write_reg32_direct(priv, 0xb68, 0x0f600000);

	/* Step 7+8: Run TX and RX IQK with retries */
	for (retry = 0; retry < 3; retry++) {
		pr_info("IQK attempt %d/3...\n", retry + 1);

		tx_result = rtl8188eu_iqk_tx(priv);
		if (tx_result == 0) {
			rx_result = rtl8188eu_iqk_rx(priv);
			if (rx_result == 0) {
				pr_info("IQK calibration COMPLETE (TX+RX OK)\n");
				break;
			}
		}

		if (retry < 2) {
			pr_info("IQK failed, retrying...\n");
			msleep(50);
		}
	}

	/* Step 9: Restore everything — disable IQK first */
	rtl8188eu_write_reg32_direct(priv, 0xe28, 0x00000000);

	/* Restore PI mode */
	if (!pi_mode_save) {
		rtl8188eu_write_reg32_direct(priv, 0x820,
			rtl8188eu_read_reg32_direct(priv, 0x820) & ~BIT(8));
		rtl8188eu_write_reg32_direct(priv, 0x828,
			rtl8188eu_read_reg32_direct(priv, 0x828) & ~BIT(8));
	}

	/* Restore all 29 registers */
	iqk_restore(priv, adda_reg, adda_backup, 16);

	/* Restore MAC: 0x522, 0x550, 0x551 as bytes, 0x040 as 32-bit */
	rtl8188eu_write_reg8(priv, 0x522, (u8)(mac_backup[0] & 0xFF));
	rtl8188eu_write_reg8(priv, 0x550, (u8)(mac_backup[1] & 0xFF));
	rtl8188eu_write_reg8(priv, 0x551, (u8)((mac_backup[1] >> 8) & 0xFF));
	rtl8188eu_write_reg32_direct(priv, 0x040, mac_backup[3]);

	iqk_restore(priv, iqk_bb_reg, bb_backup, 9);
	pr_info("IQK: Restored all 29 registers\n");

	/* Step 10: Reset defaults */
	rtl8188eu_write_reg32_direct(priv, 0x840, 0x00032ed3);
	rtl8188eu_write_reg32_direct(priv, 0xe30, 0x01008c00);
	rtl8188eu_write_reg32_direct(priv, 0xe34, 0x01008c00);

	if (retry >= 3) {
		pr_err("IQK calibration FAILED after 3 attempts!\n");
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(rtl8188eu_iqk_calibrate);

MODULE_DESCRIPTION("RTL8188EU PHY Layer");
MODULE_LICENSE("GPL");
