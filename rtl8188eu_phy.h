#ifndef RTL8188EU_PHY_H
#define RTL8188EU_PHY_H

#include <linux/types.h>
#include <net/cfg80211.h>

/* Forward declaration */
struct rtl8188eu_priv;

/* RF path enum */
enum rf_path {
	RF_PATH_A = 0,
	RF_PATH_B = 1,
};

/* BB register definition for RF access */
struct bb_register_definition {
	u32 rfintfs;
	u32 rfintfo;
	u32 rfintfe;
	u32 rf3wireOffset;
	u32 rfHSSIPara2;
	u32 rfLSSIReadBack;
	u32 rfLSSIReadBackPi;
};

/* Bitmasks */
#define bMaskDWord          0xffffffff
#define bRFRegOffsetMask    0xfffff

/* BB register addresses for RF access */
#define rFPGA0_XAB_RFInterfaceSW    0x870
#define rFPGA0_XA_RFInterfaceOE     0x860
#define rFPGA0_XA_LSSIParameter     0x840
#define rFPGA0_XA_HSSIParameter1    0x820
#define rFPGA0_XA_HSSIParameter2    0x824
#define rFPGA0_XA_LSSIReadBack      0x8a0
#define TransceiverA_HSPI_Readback  0x8b8

/* HT40 BB registers */
#define rFPGA1_RFMOD                0x900
#define rCCK0_System                0xA00
#define rOFDM1_LSTF                 0xD00

/* Bit definitions for RF readback */
#define bLSSIReadAddress    0x7f800000
#define bLSSIReadEdge       0x80000000
#define bLSSIReadBackData   0xfffff

/* RF register addresses */
#define RF_AC           0x00
#define RF_CHNLBW       0x18

/* Core functions */
u32 rtl8188eu_phy_calculate_bit_shift(u32 bitmask);
void rtl8188eu_init_phy_reg_def(struct rtl8188eu_priv *priv);

/* BB register access */
u32 rtl8188eu_read_bb_reg(struct rtl8188eu_priv *priv, u32 addr, u32 bitmask);
void rtl8188eu_write_bb_reg(struct rtl8188eu_priv *priv, u32 addr, u32 bitmask, u32 data);

/* RF register access */
u32 rtl8188eu_read_rf_reg(struct rtl8188eu_priv *priv, enum rf_path path,
			   u32 addr, u32 bitmask);
void rtl8188eu_write_rf_reg(struct rtl8188eu_priv *priv, enum rf_path path,
			     u32 addr, u32 bitmask, u32 data);

/* Table access functions from rtl8188eu_phy_tables.c */
const u32 *rtl8188eu_get_phy_reg_table(int *size);
const u32 *rtl8188eu_get_agc_table(int *size);
const u32 *rtl8188eu_get_radioa_table(int *size);
const u32 *rtl8188eu_get_mac_reg_table(int *size);

/* Table loader functions */
int rtl8188eu_load_phy_reg_table(struct rtl8188eu_priv *priv);
int rtl8188eu_load_agc_table(struct rtl8188eu_priv *priv);
int rtl8188eu_load_radioa_table(struct rtl8188eu_priv *priv);
int rtl8188eu_load_mac_reg_table(struct rtl8188eu_priv *priv);

/* Main PHY initialization functions */
int rtl8188eu_phy_bb_config(struct rtl8188eu_priv *priv);
int rtl8188eu_phy_rf_config(struct rtl8188eu_priv *priv);

/* Bandwidth setting (HT20/HT40) */
void rtl8188eu_set_bw(struct rtl8188eu_priv *priv,
		       const struct cfg80211_chan_def *chandef);

/* PHY test functions */
int rtl8188eu_phy_test_read_only(struct rtl8188eu_priv *priv);

/* RF Calibration functions */
void rtl8188eu_lc_calibrate(struct rtl8188eu_priv *priv);
int rtl8188eu_iqk_calibrate(struct rtl8188eu_priv *priv);

#endif /* RTL8188EU_PHY_H */
