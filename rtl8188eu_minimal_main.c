/*
 * RTL8188EUS USB WiFi Driver — mac80211 version
 *
 * Converted from custom netdev/wext to Linux mac80211 subsystem.
 * Supports station mode (managed) + monitor mode.
 * HT20/HT40, WPA2 via software crypto.
 *
 * Target: TP-Link TL-WN722N v2/v3 (USB ID: 2357:010c)
 * Kernel: 6.17+
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/firmware.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <net/mac80211.h>
#include "rtl8188eu.h"
#include "rtl8188eu_phy.h"

#define DRIVER_NAME "rtl8188eu_minimal"
#define DRIVER_VERSION "1.0.0"
#define FIRMWARE_NAME "rtlwifi/rtl8188eufw.bin"

/* Power Sequence Commands */
#define PWR_CMD_WRITE			0x01
#define PWR_CMD_POLLING			0x02
#define PWR_CMD_DELAY			0x03
#define PWR_CMD_END			0x04

struct pwr_cmd {
	u16 offset;
	u8  cmd;
	u8  mask;
	u8  value;
};

/* RTL8188E Register Addresses */
#define REG_SYS_CFG			0x00F0
#define REG_SYS_FUNC_EN			0x0002
#define FEN_ELDR			BIT(12)
#define REG_APS_FSMCO			0x0004
#define REG_SYS_CLKR			0x0008
#define ANA8M				BIT(1)
#define LOADER_CLK_EN			BIT(5)
#define REG_RSV_CTRL			0x001C
#define REG_RF_CTRL			0x001F
#define REG_AFE_XTAL_CTRL		0x0024
#define REG_AFE_PLL_CTRL		0x0028
#define REG_MCUFWDL			0x0080
#define MCUFWDL_EN			BIT(0)
#define MCUFWDL_RDY			BIT(1)
#define WINTINI_RDY			BIT(6)
#define RAM_DL_SEL			BIT(7)
#define REG_HIMR_88E			0x00B0
#define REG_HISR_88E			0x00B4
#define REG_CR				0x0100
#define REG_HMETFR			0x01CC
#define REG_PBP				0x0104
#define REG_TRXFF_BNDY			0x0114
#define REG_RQPN			0x0200
#define REG_TDECTRL			0x0208
#define REG_RQPN_NPQ			0x0214
#define REG_BCNQ_BDNY			0x0424
#define REG_MGQ_BDNY			0x0425
#define REG_WMAC_LBK_BF_HD		0x045D

/* RCR and related */
#define REG_RCR				0x0608
#define REG_MAR				0x0620
#define REG_WMAC_TRXPTCL_CTL		0x0668
#define REG_RXFLTMAP2			0x06A4

/* USB Aggregation Registers */
#define REG_TRXDMA_CTRL			0x010C
#define REG_RXPKT_NUM			0x0284
#define RW_RELEASE_EN			BIT(18)
#define RXDMA_IDLE			BIT(17)
#define REG_RXDMA_AGG_PG_TH		0x0280
#define REG_USB_SPECIAL_OPTION		0xFE55
#define REG_USB_AGG_TO			0xFE5C
#define REG_USB_AGG_TH			0xFE5D

/* TX Report Registers */
#define REG_TX_RPT_CTRL			0x04EC
#define REG_TX_RPT_TIME		0x04F0

#define REG_TXDMA_OFFSET_CHK		0x020C
#define REG_MAX_AGGR_NUM		0x04CA

/* TRXDMA_CTRL queue mapping macros */
#define _TXDMA_HIQ_MAP(x)		(((x) & 0x3) << 14)
#define _TXDMA_MGQ_MAP(x)		(((x) & 0x3) << 12)
#define _TXDMA_BKQ_MAP(x)		(((x) & 0x3) << 10)
#define _TXDMA_BEQ_MAP(x)		(((x) & 0x3) << 8)
#define _TXDMA_VIQ_MAP(x)		(((x) & 0x3) << 6)
#define _TXDMA_VOQ_MAP(x)		(((x) & 0x3) << 4)
#define QUEUE_LOW			1
#define QUEUE_NORMAL			2
#define QUEUE_HIGH			3

/* LLT */
#define REG_LLT_INIT			0x01E0
#define LAST_ENTRY_OF_TX_PKT_BUF	175

/* MAC Configuration Registers */
#define REG_FWHW_TXQ_CTRL		0x0420
#define REG_HWSEQ_CTRL			0x0423
#define REG_SPEC_SIFS			0x0428
#define REG_RL				0x042A
#define REG_DARFRC			0x0430
#define REG_RARFRC			0x0438
#define REG_RRSR			0x0440
#define REG_AMPDU_MAX_TIME		0x0456
#define REG_EDCA_VO_PARAM		0x0500
#define REG_EDCA_VI_PARAM		0x0504
#define REG_EDCA_BE_PARAM		0x0508
#define REG_EDCA_BK_PARAM		0x050C
#define REG_PIFS			0x0512
#define REG_SIFS_CTX			0x0514
#define REG_SIFS_TRX			0x0516
#define REG_TBTT_PROHIBIT		0x0540
#define REG_BCN_CTRL			0x0550
#define REG_RX_DRVINFO_SZ		0x060F
#define REG_RESP_SIFS_CCK		0x063C
#define REG_RESP_SIFS_OFDM		0x063E
#define REG_ACKTO			0x0640

/* Station mode registers */
#define REG_BSSID			0x0618
#define REG_BCN_INTERVAL		0x0554
#define REG_MSR				0x0102

/* REG_SYS_FUNC_EN bits */
#define FEN_BBRSTB			BIT(0)
#define FEN_BB_GLB_RSTn			BIT(1)
#define FEN_USBA			BIT(2)
#define FEN_USBD			BIT(4)
#define FEN_EN_25_1			BIT(13)

/* REG_RF_CTRL bits */
#define RF_EN				BIT(0)
#define RF_RSTB				BIT(1)
#define RF_SDMRSTB			BIT(2)

/* REG_CR bits */
#define CR_HCI_TXDMA_EN			BIT(0)
#define CR_HCI_RXDMA_EN			BIT(1)
#define CR_TXDMA_EN			BIT(2)
#define CR_RXDMA_EN			BIT(3)
#define CR_PROTOCOL_EN			BIT(4)
#define CR_SCHEDULE_EN			BIT(5)
#define CR_MACTXEN			BIT(6)
#define CR_MACRXEN			BIT(7)
#define CR_ENSEC			BIT(9)
#define CR_CALTMR_EN			BIT(10)

/* BB registers for PHY control */
#define rFPGA0_RFMOD			0x800
#define bCCKEn				0x1000000
#define bOFDMEn				0x2000000

/* REG_RCR bits */
#define RCR_AAP				BIT(0)
#define RCR_APM				BIT(1)
#define RCR_AM				BIT(2)
#define RCR_AB				BIT(3)
#define RCR_ADD3			BIT(4)
#define RCR_APWRMGT			BIT(5)
#define RCR_CBSSID_DATA			BIT(6)
#define RCR_CBSSID_BCN			BIT(7)
#define RCR_ACRC32			BIT(8)
#define RCR_AICV			BIT(9)
#define RCR_ADF				BIT(11)
#define RCR_ACF				BIT(12)
#define RCR_AMF				BIT(13)
#define RCR_HTC_LOC_CTRL		BIT(14)
#define RCR_APP_PHYST_RXFF		BIT(28)
#define RCR_APP_ICV			BIT(29)
#define RCR_APP_MIC			BIT(30)
#define RCR_APPFCS			BIT(31)

/* Firmware Download */
#define FW_START_ADDRESS		0x1000
#define MAX_FW_BLOCK_SIZE		196
#define MAX_FW_PAGE_SIZE		4096

/* TX/RX Queue Configuration */
#define TX_SELE_HQ			BIT(0)
#define TX_SELE_LQ			BIT(1)
#define TX_SELE_NQ			BIT(2)

#define NORMAL_PAGE_NUM_HPQ		0x0C
#define NORMAL_PAGE_NUM_LPQ		0x02
#define NORMAL_PAGE_NUM_NPQ		0x02

#define TOTAL_PAGE_NUMBER		0xAF
#define BCNQ_PAGE_NUM			0x08
#define TX_TOTAL_PAGE_NUMBER		(TOTAL_PAGE_NUMBER - BCNQ_PAGE_NUM)
#define TX_PAGE_BOUNDARY		(TX_TOTAL_PAGE_NUMBER + 1)

#define PBP_128				0x1

#define _HPQ(x)				((x) & 0xFF)
#define _LPQ(x)				(((x) & 0xFF) << 8)
#define _PUBQ(x)			(((x) & 0xFF) << 16)
#define _NPQ(x)				((x) & 0xFF)
#define LD_RQPN				BIT(31)

#define _PSRX(x)			(x)
#define _PSTX(x)			((x) << 4)

#define RX_BUFFER_SIZE			32768

/* Bandwidth mode register */
#define REG_BWOPMODE			0x0603
#define BW_OPMODE_20MHZ			BIT(2)

/* Crystal cap default */
#define EEPROM_DEFAULT_CRYSTAL_CAP	0x20

/* RTL8188EU RX Descriptor (24 bytes) */
struct rtl8188eu_rx_desc {
	__le32 rxdw0;
	__le32 rxdw1;
	__le32 rxdw2;
	__le32 rxdw3;
	__le32 rxdw4;
	__le32 rxdw5;
} __packed;

/* RX DW0 bit definitions */
#define RX_DW0_PKT_LEN_MASK		0x00003fff
#define RX_DW0_CRC32			BIT(14)
#define RX_DW0_ICV_ERR			BIT(15)
#define RX_DW0_DRVINFO_SZ_SHIFT	16
#define RX_DW0_DRVINFO_SZ_MASK		0xf
#define RX_DW0_SHIFT_SHIFT		24
#define RX_DW0_SHIFT_MASK		0x3
#define RX_DW0_PHYST			BIT(26)

/* RX DW2 bit definitions */
#define RX_DW2_PKT_CNT_SHIFT		16
#define RX_DW2_PKT_CNT_MASK		0xff

#define RXDESC_SIZE			sizeof(struct rtl8188eu_rx_desc)
#define RX_RND128(x)			(((x) + 127) & ~127)

/* RTL8188EU TX Descriptor (32 bytes) */
struct rtl8188eu_tx_desc {
	__le32 txdw0;
	__le32 txdw1;
	__le32 txdw2;
	__le32 txdw3;
	__le32 txdw4;
	__le32 txdw5;
	__le32 txdw6;
	__le32 txdw7;
} __packed;

#define TX_DESC_SIZE			sizeof(struct rtl8188eu_tx_desc)

/* TX DW0 bit definitions */
#define TX_DW0_PKT_SIZE_SHIFT		0
#define TX_DW0_OFFSET_SHIFT		16
#define TX_DW0_BMC			BIT(24)
#define TX_DW0_OWN			BIT(31)

/* TX DW1 bit definitions */
#define TX_DW1_MACID_SHIFT		0
#define TX_DW1_QUEUE_SEL_SHIFT		8
#define TX_DW1_QUEUE_SEL_VO		0x06
#define TX_DW1_QUEUE_SEL_VI		0x05
#define TX_DW1_QUEUE_SEL_BE		0x00
#define TX_DW1_QUEUE_SEL_BK		0x02
#define TX_DW1_QUEUE_SEL_MGNT		0x12

/* TX DW3 bit definitions */
#define TX_DW3_SEQ_SHIFT		16
#define TX_DW3_HW_SEQ_EN		BIT(15)

/* TX DW4 bit definitions */
#define TX_DW4_USE_RATE			BIT(8)
#define TX_DW4_DATA_BW			BIT(25)
#define TX_DW4_DATA_SHORT		BIT(24)
#define TX_DW4_DATA_RATE_SHIFT		0

/* TX DW5 bit definitions */
#define TX_DW5_RTS_RATE_SHIFT		0

/* Hardware rate values */
#define HW_RATE_CCK1			0x00
#define HW_RATE_CCK2			0x01
#define HW_RATE_CCK55			0x02
#define HW_RATE_CCK11			0x03
#define HW_RATE_OFDM6			0x04
#define HW_RATE_OFDM9			0x05
#define HW_RATE_OFDM12			0x06
#define HW_RATE_OFDM18			0x07
#define HW_RATE_OFDM24			0x08
#define HW_RATE_OFDM36			0x09
#define HW_RATE_OFDM48			0x0A
#define HW_RATE_OFDM54			0x0B
#define HW_RATE_MCS0			0x0C
#define HW_RATE_MCS7			0x13

static inline u16 rtl8188eu_chan_to_freq(u8 channel)
{
	if (channel == 14)
		return 2484;
	if (channel >= 1 && channel <= 13)
		return 2407 + 5 * channel;
	return 2412;
}

/* USB Device ID Table */
static const struct usb_device_id rtl8188eu_usb_ids[] = {
	{ USB_DEVICE(0x2357, 0x010c) },
	{ }
};
MODULE_DEVICE_TABLE(usb, rtl8188eu_usb_ids);

/* Prototypes for PHY layer wrapper functions */
u32 rtl8188eu_read_reg32_direct(void *priv_ptr, u16 addr);
int rtl8188eu_write_reg32_direct(void *priv_ptr, u16 addr, u32 val);

/*
 * ============================================================================
 * mac80211 Band / Rate / Channel Definitions
 * ============================================================================
 */

static struct ieee80211_channel rtl8188eu_channels_2ghz[] = {
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2412, .hw_value = 1,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2417, .hw_value = 2,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2422, .hw_value = 3,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2427, .hw_value = 4,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2432, .hw_value = 5,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2437, .hw_value = 6,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2442, .hw_value = 7,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2447, .hw_value = 8,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2452, .hw_value = 9,  .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2457, .hw_value = 10, .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2462, .hw_value = 11, .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2467, .hw_value = 12, .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2472, .hw_value = 13, .max_power = 20 },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2484, .hw_value = 14, .max_power = 20 },
};

static struct ieee80211_rate rtl8188eu_rates[] = {
	{ .bitrate = 10,  .hw_value = HW_RATE_CCK1,   .hw_value_short = HW_RATE_CCK1,
	  .flags = 0 },
	{ .bitrate = 20,  .hw_value = HW_RATE_CCK2,   .hw_value_short = HW_RATE_CCK2,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 55,  .hw_value = HW_RATE_CCK55,  .hw_value_short = HW_RATE_CCK55,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 110, .hw_value = HW_RATE_CCK11,  .hw_value_short = HW_RATE_CCK11,
	  .flags = IEEE80211_RATE_SHORT_PREAMBLE },
	{ .bitrate = 60,  .hw_value = HW_RATE_OFDM6,  .hw_value_short = HW_RATE_OFDM6 },
	{ .bitrate = 90,  .hw_value = HW_RATE_OFDM9,  .hw_value_short = HW_RATE_OFDM9 },
	{ .bitrate = 120, .hw_value = HW_RATE_OFDM12, .hw_value_short = HW_RATE_OFDM12 },
	{ .bitrate = 180, .hw_value = HW_RATE_OFDM18, .hw_value_short = HW_RATE_OFDM18 },
	{ .bitrate = 240, .hw_value = HW_RATE_OFDM24, .hw_value_short = HW_RATE_OFDM24 },
	{ .bitrate = 360, .hw_value = HW_RATE_OFDM36, .hw_value_short = HW_RATE_OFDM36 },
	{ .bitrate = 480, .hw_value = HW_RATE_OFDM48, .hw_value_short = HW_RATE_OFDM48 },
	{ .bitrate = 540, .hw_value = HW_RATE_OFDM54, .hw_value_short = HW_RATE_OFDM54 },
};

static struct ieee80211_supported_band rtl8188eu_band_2ghz = {
	.channels = rtl8188eu_channels_2ghz,
	.n_channels = ARRAY_SIZE(rtl8188eu_channels_2ghz),
	.bitrates = rtl8188eu_rates,
	.n_bitrates = ARRAY_SIZE(rtl8188eu_rates),
	.band = NL80211_BAND_2GHZ,
	.ht_cap = {
		.ht_supported = true,
		.cap = IEEE80211_HT_CAP_SUP_WIDTH_20_40 |
		       IEEE80211_HT_CAP_SGI_20 |
		       IEEE80211_HT_CAP_SGI_40 |
		       IEEE80211_HT_CAP_DSSSCCK40,
		.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K,
		.ampdu_density = IEEE80211_HT_MPDU_DENSITY_16,
		.mcs = {
			.rx_mask = { 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			.rx_highest = cpu_to_le16(150),
			.tx_params = IEEE80211_HT_MCS_TX_DEFINED,
		},
	},
};

/*
 * ============================================================================
 * Register Read/Write Functions
 * ============================================================================
 */

static int rtl8188eu_read_reg8(struct rtl8188eu_priv *priv, u16 addr, u8 *val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = usb_control_msg(priv->udev,
			      usb_rcvctrlpipe(priv->udev, 0),
			      0x05,
			      USB_TYPE_VENDOR | USB_DIR_IN,
			      addr, 0, buf, 1, 500);

	if (ret >= 0) {
		*val = buf[0];
		ret = 0;
	} else {
		pr_err("%s: Failed to read register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
	}

	kfree(buf);
	return ret;
}

static int rtl8188eu_read_reg16(struct rtl8188eu_priv *priv, u16 addr, u16 *val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = usb_control_msg(priv->udev,
			      usb_rcvctrlpipe(priv->udev, 0),
			      0x05,
			      USB_TYPE_VENDOR | USB_DIR_IN,
			      addr, 0, buf, 2, 500);

	if (ret >= 0) {
		*val = le16_to_cpu(*(u16 *)buf);
		ret = 0;
	} else {
		pr_err("%s: Failed to read register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
	}

	kfree(buf);
	return ret;
}

static int rtl8188eu_read_reg32(struct rtl8188eu_priv *priv, u16 addr, u32 *val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(4, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = usb_control_msg(priv->udev,
			      usb_rcvctrlpipe(priv->udev, 0),
			      0x05,
			      USB_TYPE_VENDOR | USB_DIR_IN,
			      addr, 0, buf, 4, 500);

	if (ret >= 0) {
		*val = le32_to_cpu(*(u32 *)buf);
		ret = 0;
	} else {
		pr_err("%s: Failed to read register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
	}

	kfree(buf);
	return ret;
}

int rtl8188eu_write_reg8(struct rtl8188eu_priv *priv, u16 addr, u8 val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = val;

	ret = usb_control_msg(priv->udev,
			      usb_sndctrlpipe(priv->udev, 0),
			      0x05,
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr, 0, buf, 1, 500);

	kfree(buf);

	if (ret < 0) {
		pr_err("%s: Failed to write register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(rtl8188eu_write_reg8);

static int rtl8188eu_write_reg16(struct rtl8188eu_priv *priv, u16 addr, u16 val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	*(u16 *)buf = cpu_to_le16(val);

	ret = usb_control_msg(priv->udev,
			      usb_sndctrlpipe(priv->udev, 0),
			      0x05,
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr, 0, buf, 2, 500);

	kfree(buf);

	if (ret < 0) {
		pr_err("%s: Failed to write register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
		return ret;
	}

	return 0;
}

static int rtl8188eu_write_reg32(struct rtl8188eu_priv *priv, u16 addr, u32 val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(4, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	*(u32 *)buf = cpu_to_le32(val);

	ret = usb_control_msg(priv->udev,
			      usb_sndctrlpipe(priv->udev, 0),
			      0x05,
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr, 0, buf, 4, 500);

	kfree(buf);

	if (ret < 0) {
		pr_err("%s: Failed to write register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
		return ret;
	}

	return 0;
}

/* PHY layer wrappers */
u32 rtl8188eu_read_reg32_direct(void *priv_ptr, u16 addr)
{
	struct rtl8188eu_priv *priv = (struct rtl8188eu_priv *)priv_ptr;
	u32 val = 0;

	if (rtl8188eu_read_reg32(priv, addr, &val) < 0)
		pr_warn("Failed to read register 0x%04x, returning 0\n", addr);

	return val;
}
EXPORT_SYMBOL(rtl8188eu_read_reg32_direct);

int rtl8188eu_write_reg32_direct(void *priv_ptr, u16 addr, u32 val)
{
	struct rtl8188eu_priv *priv = (struct rtl8188eu_priv *)priv_ptr;
	return rtl8188eu_write_reg32(priv, addr, val);
}
EXPORT_SYMBOL(rtl8188eu_write_reg32_direct);

static int rtl8188eu_write_block(struct rtl8188eu_priv *priv,
				  u16 addr, const u8 *data, u16 len)
{
	int ret;
	u8 *buf;

	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = usb_control_msg(priv->udev,
			      usb_sndctrlpipe(priv->udev, 0),
			      0x05,
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr, 0, buf, len, 500);

	kfree(buf);

	if (ret < 0) {
		pr_err("%s: Failed to write block at 0x%04x (%d bytes): %d\n",
		       DRIVER_NAME, addr, len, ret);
		return ret;
	}

	return 0;
}

/*
 * ============================================================================
 * Hardware Initialization Functions
 * ============================================================================
 */

static const struct pwr_cmd rtl8188e_power_on_seq[] = {
	{0x0006, PWR_CMD_POLLING, BIT(1), BIT(1)},
	{0x0002, PWR_CMD_WRITE, BIT(0) | BIT(1), 0},
	{0x0026, PWR_CMD_WRITE, BIT(7), BIT(7)},
	{0x0005, PWR_CMD_WRITE, BIT(7), 0},
	{0x0005, PWR_CMD_WRITE, BIT(4) | BIT(3), 0},
	{0x0005, PWR_CMD_WRITE, BIT(0), BIT(0)},
	{0x0005, PWR_CMD_POLLING, BIT(0), 0},
	{0x0023, PWR_CMD_WRITE, BIT(4), 0},
	{0xFFFF, PWR_CMD_END, 0, 0}
};

static int rtl8188eu_execute_power_sequence(struct rtl8188eu_priv *priv,
					     const struct pwr_cmd *seq)
{
	int i, retry, ret;
	u8 val8;

	pr_debug("%s: Executing power sequence (CARDEMU->ACT)...\n", DRIVER_NAME);

	for (i = 0; seq[i].cmd != PWR_CMD_END; i++) {
		u16 reg = seq[i].offset;

		switch (seq[i].cmd) {
		case PWR_CMD_WRITE:
			ret = rtl8188eu_read_reg8(priv, reg, &val8);
			if (ret < 0)
				return ret;

			val8 &= ~seq[i].mask;
			val8 |= (seq[i].value & seq[i].mask);

			ret = rtl8188eu_write_reg8(priv, reg, val8);
			if (ret < 0)
				return ret;
			break;

		case PWR_CMD_POLLING:
			for (retry = 0; retry < 5000; retry++) {
				ret = rtl8188eu_read_reg8(priv, reg, &val8);
				if (ret < 0)
					return ret;

				if ((val8 & seq[i].mask) == (seq[i].value & seq[i].mask))
					break;

				usleep_range(10, 20);
			}

			if (retry >= 5000) {
				pr_err("%s: Power sequence polling timeout at 0x%04x\n",
				       DRIVER_NAME, reg);
				return -ETIMEDOUT;
			}
			break;

		case PWR_CMD_DELAY:
			msleep(seq[i].value);
			break;
		}
	}

	pr_debug("%s: Power sequence complete\n", DRIVER_NAME);
	return 0;
}

static int rtl8188eu_power_on(struct rtl8188eu_priv *priv)
{
	u16 value16;
	int ret;

	if (priv->chip_powered_on)
		return 0;

	ret = rtl8188eu_execute_power_sequence(priv, rtl8188e_power_on_seq);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg16(priv, REG_CR, 0x00);
	if (ret < 0)
		return ret;

	msleep(10);

	ret = rtl8188eu_read_reg16(priv, REG_CR, &value16);
	if (ret < 0)
		return ret;

	value16 |= (CR_HCI_TXDMA_EN | CR_HCI_RXDMA_EN |
		    CR_TXDMA_EN | CR_RXDMA_EN |
		    CR_PROTOCOL_EN | CR_SCHEDULE_EN |
		    CR_ENSEC | CR_CALTMR_EN);

	ret = rtl8188eu_write_reg16(priv, REG_CR, value16);
	if (ret < 0)
		return ret;

	priv->chip_powered_on = true;
	pr_debug("%s: Chip powered on (REG_CR=0x%04x)\n", DRIVER_NAME, value16);
	return 0;
}

static int rtl8188eu_enable_mcu_clocks(struct rtl8188eu_priv *priv)
{
	u16 val16;
	int ret;

	ret = rtl8188eu_read_reg16(priv, REG_SYS_FUNC_EN, &val16);
	if (ret < 0)
		return ret;

	if (!(val16 & FEN_ELDR)) {
		val16 |= FEN_ELDR;
		ret = rtl8188eu_write_reg16(priv, REG_SYS_FUNC_EN, val16);
		if (ret < 0)
			return ret;
	}

	ret = rtl8188eu_read_reg16(priv, REG_SYS_CLKR, &val16);
	if (ret < 0)
		return ret;

	if (!(val16 & LOADER_CLK_EN) || !(val16 & ANA8M)) {
		val16 |= (LOADER_CLK_EN | ANA8M);
		ret = rtl8188eu_write_reg16(priv, REG_SYS_CLKR, val16);
		if (ret < 0)
			return ret;
	}

	ret = rtl8188eu_write_reg8(priv, REG_AFE_XTAL_CTRL + 1, 0x80);
	if (ret < 0)
		return ret;

	return 0;
}

static int rtl8188eu_init_interrupts(struct rtl8188eu_priv *priv)
{
	int ret;
	u32 himr;

	ret = rtl8188eu_write_reg32(priv, REG_HISR_88E, 0xFFFFFFFF);
	if (ret < 0)
		return ret;

	himr = BIT(0) | BIT(1) | BIT(8) | BIT(10);

	ret = rtl8188eu_write_reg32(priv, REG_HIMR_88E, himr);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * ============================================================================
 * TX/RX Queue Setup
 * ============================================================================
 */

static int rtl8188eu_init_transfer_page_size(struct rtl8188eu_priv *priv)
{
	return rtl8188eu_write_reg8(priv, REG_PBP,
				     _PSRX(PBP_128) | _PSTX(PBP_128));
}

static int rtl8188eu_init_page_boundary(struct rtl8188eu_priv *priv)
{
	int ret;

	ret = rtl8188eu_write_reg16(priv, REG_TRXFF_BNDY, TOTAL_PAGE_NUMBER);
	if (ret < 0)
		return ret;

	return rtl8188eu_write_reg16(priv, REG_TRXFF_BNDY + 2, 0x25FF);
}

static int rtl8188eu_init_tx_buffer_boundary(struct rtl8188eu_priv *priv)
{
	u8 bndy = TX_PAGE_BOUNDARY;
	int ret;

	ret = rtl8188eu_write_reg8(priv, REG_BCNQ_BDNY, bndy);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_MGQ_BDNY, bndy);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_WMAC_LBK_BF_HD, bndy);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_TRXFF_BNDY, bndy);
	if (ret < 0)
		return ret;
	return rtl8188eu_write_reg8(priv, REG_TDECTRL + 1, bndy);
}

static int rtl8188eu_init_queue_reserved_page(struct rtl8188eu_priv *priv)
{
	u32 numHQ = 0, numLQ = 0, numNQ = 0, numPubQ;
	u32 value32;
	int ret;

	if (priv->out_ep_queue_sel & TX_SELE_HQ)
		numHQ = NORMAL_PAGE_NUM_HPQ;
	if (priv->out_ep_queue_sel & TX_SELE_LQ)
		numLQ = NORMAL_PAGE_NUM_LPQ;
	if (priv->out_ep_queue_sel & TX_SELE_NQ)
		numNQ = NORMAL_PAGE_NUM_NPQ;

	ret = rtl8188eu_write_reg8(priv, REG_RQPN_NPQ, _NPQ(numNQ));
	if (ret < 0)
		return ret;

	numPubQ = TX_TOTAL_PAGE_NUMBER - numHQ - numLQ - numNQ;
	value32 = _HPQ(numHQ) | _LPQ(numLQ) | _PUBQ(numPubQ) | LD_RQPN;
	return rtl8188eu_write_reg32(priv, REG_RQPN, value32);
}

static int rtl8188eu_llt_write(struct rtl8188eu_priv *priv, u32 address, u32 data)
{
	u32 value32;
	int count, ret;

	value32 = (1 << 30) | ((address & 0xFF) << 8) | (data & 0xFF);
	ret = rtl8188eu_write_reg32(priv, REG_LLT_INIT, value32);
	if (ret < 0)
		return ret;

	for (count = 0; count < 20; count++) {
		ret = rtl8188eu_read_reg32(priv, REG_LLT_INIT, &value32);
		if (ret < 0)
			return ret;
		if (((value32 >> 30) & 0x3) == 0)
			return 0;
		usleep_range(10, 20);
	}

	return -ETIMEDOUT;
}

static int rtl8188eu_init_llt_table(struct rtl8188eu_priv *priv)
{
	int ret;
	u32 i;

	for (i = 0; i < TX_PAGE_BOUNDARY - 1; i++) {
		ret = rtl8188eu_llt_write(priv, i, i + 1);
		if (ret)
			return ret;
	}

	ret = rtl8188eu_llt_write(priv, TX_PAGE_BOUNDARY - 1, 0xFF);
	if (ret)
		return ret;

	for (i = TX_PAGE_BOUNDARY; i < LAST_ENTRY_OF_TX_PKT_BUF; i++) {
		ret = rtl8188eu_llt_write(priv, i, i + 1);
		if (ret)
			return ret;
	}

	return rtl8188eu_llt_write(priv, LAST_ENTRY_OF_TX_PKT_BUF, TX_PAGE_BOUNDARY);
}

static int rtl8188eu_init_tx_rx_queues(struct rtl8188eu_priv *priv)
{
	int ret;

	ret = rtl8188eu_init_transfer_page_size(priv);
	if (ret)
		return ret;
	ret = rtl8188eu_init_page_boundary(priv);
	if (ret)
		return ret;
	ret = rtl8188eu_init_tx_buffer_boundary(priv);
	if (ret)
		return ret;
	return rtl8188eu_init_queue_reserved_page(priv);
}

static int rtl8188eu_init_mac_regs(struct rtl8188eu_priv *priv)
{
	rtl8188eu_write_reg16(priv, REG_SPEC_SIFS, 0x100a);
	rtl8188eu_write_reg16(priv, 0x063A, 0x100a);
	rtl8188eu_write_reg16(priv, REG_SIFS_CTX, 0x100a);
	rtl8188eu_write_reg16(priv, REG_SIFS_TRX, 0x100a);
	rtl8188eu_write_reg8(priv, REG_RESP_SIFS_CCK, 0x08);
	rtl8188eu_write_reg8(priv, REG_RESP_SIFS_OFDM, 0x0E);
	rtl8188eu_write_reg16(priv, REG_RL, 0x3030);
	rtl8188eu_write_reg32(priv, REG_EDCA_VO_PARAM, 0x002FA226);
	rtl8188eu_write_reg32(priv, REG_EDCA_VI_PARAM, 0x005EA324);
	rtl8188eu_write_reg32(priv, REG_EDCA_BE_PARAM, 0x005EA42B);
	rtl8188eu_write_reg32(priv, REG_EDCA_BK_PARAM, 0x0000A44F);
	rtl8188eu_write_reg32(priv, REG_RRSR, 0x0000FFFF);
	rtl8188eu_write_reg8(priv, REG_ACKTO, 0x40);
	rtl8188eu_write_reg8(priv, REG_BCN_CTRL, 0x10);
	rtl8188eu_write_reg8(priv, REG_PIFS, 0x1C);
	rtl8188eu_write_reg8(priv, REG_HWSEQ_CTRL, 0xFF);
	rtl8188eu_write_reg8(priv, 0x066E, 0x05);
	rtl8188eu_write_reg8(priv, REG_AMPDU_MAX_TIME, 0x70);

	return 0;
}

/*
 * ============================================================================
 * Firmware Loading
 * ============================================================================
 */

static int rtl8188eu_download_firmware(struct rtl8188eu_priv *priv)
{
	const u8 *fw_data = priv->fw->data;
	u32 fw_size = priv->fw->size;
	u32 page_nums, remain_size;
	u32 page, offset;
	int ret;
	u8 val;
	u32 header_size = 0;

	if (fw_size >= 32 && fw_data[0] == 0xE1 && fw_data[1] == 0x88) {
		header_size = 32;
		fw_data += header_size;
		fw_size -= header_size;
	}

	page_nums = fw_size / MAX_FW_PAGE_SIZE;
	remain_size = fw_size % MAX_FW_PAGE_SIZE;

	for (page = 0; page < page_nums; page++) {
		u32 block_cnt, block_remain, block;

		offset = page * MAX_FW_PAGE_SIZE;

		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL + 2, &val);
		if (ret < 0)
			return ret;
		val = (val & 0xF8) | (page & 0x07);
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 2, val);
		if (ret < 0)
			return ret;

		block_cnt = MAX_FW_PAGE_SIZE / MAX_FW_BLOCK_SIZE;
		block_remain = MAX_FW_PAGE_SIZE % MAX_FW_BLOCK_SIZE;

		for (block = 0; block < block_cnt; block++) {
			ret = rtl8188eu_write_block(priv,
				FW_START_ADDRESS + (block * MAX_FW_BLOCK_SIZE),
				fw_data + offset + (block * MAX_FW_BLOCK_SIZE),
				MAX_FW_BLOCK_SIZE);
			if (ret < 0)
				return ret;
		}

		if (block_remain > 0) {
			ret = rtl8188eu_write_block(priv,
				FW_START_ADDRESS + (block_cnt * MAX_FW_BLOCK_SIZE),
				fw_data + offset + (block_cnt * MAX_FW_BLOCK_SIZE),
				block_remain);
			if (ret < 0)
				return ret;
		}
	}

	if (remain_size > 0) {
		u32 block_cnt, block_remain, block;

		offset = page_nums * MAX_FW_PAGE_SIZE;
		page = page_nums;

		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL + 2, &val);
		if (ret < 0)
			return ret;
		val = (val & 0xF8) | (page & 0x07);
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 2, val);
		if (ret < 0)
			return ret;

		block_cnt = remain_size / MAX_FW_BLOCK_SIZE;
		block_remain = remain_size % MAX_FW_BLOCK_SIZE;

		for (block = 0; block < block_cnt; block++) {
			ret = rtl8188eu_write_block(priv,
				FW_START_ADDRESS + (block * MAX_FW_BLOCK_SIZE),
				fw_data + offset + (block * MAX_FW_BLOCK_SIZE),
				MAX_FW_BLOCK_SIZE);
			if (ret < 0)
				return ret;
		}

		if (block_remain > 0) {
			ret = rtl8188eu_write_block(priv,
				FW_START_ADDRESS + (block_cnt * MAX_FW_BLOCK_SIZE),
				fw_data + offset + (block_cnt * MAX_FW_BLOCK_SIZE),
				block_remain);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

static int rtl8188eu_reset_8051(struct rtl8188eu_priv *priv)
{
	u8 val;
	int ret;

	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL, &val);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL, val & ~BIT(1));
	if (ret < 0)
		return ret;

	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL + 1, &val);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL + 1, val & ~BIT(3));
	if (ret < 0)
		return ret;

	ret = rtl8188eu_read_reg8(priv, REG_SYS_FUNC_EN + 1, &val);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_SYS_FUNC_EN + 1, val & ~BIT(2));
	if (ret < 0)
		return ret;

	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL, &val);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL, val & ~BIT(1));
	if (ret < 0)
		return ret;

	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL + 1, &val);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL + 1, val | BIT(3));
	if (ret < 0)
		return ret;

	ret = rtl8188eu_read_reg8(priv, REG_SYS_FUNC_EN + 1, &val);
	if (ret < 0)
		return ret;
	return rtl8188eu_write_reg8(priv, REG_SYS_FUNC_EN + 1, val | BIT(2));
}

static int rtl8188eu_fw_download_enable(struct rtl8188eu_priv *priv, bool enable)
{
	u8 val;
	int ret;

	if (enable) {
		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL, &val);
		if (ret < 0)
			return ret;

		if (val & RAM_DL_SEL) {
			ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL, 0x00);
			if (ret < 0)
				return ret;
			ret = rtl8188eu_reset_8051(priv);
			if (ret < 0)
				return ret;
		}

		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL, &val);
		if (ret < 0)
			return ret;
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL, val | BIT(2));
		if (ret < 0)
			return ret;

		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL, &val);
		if (ret < 0)
			return ret;
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL, val | MCUFWDL_EN);
		if (ret < 0)
			return ret;

		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL + 2, &val);
		if (ret < 0)
			return ret;
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 2, val & ~BIT(3));
		if (ret < 0)
			return ret;
	} else {
		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL, &val);
		if (ret < 0)
			return ret;
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL, val & ~MCUFWDL_EN);
		if (ret < 0)
			return ret;

		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 1, 0x00);
		if (ret < 0)
			return ret;

		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL + 2, &val);
		if (ret < 0)
			return ret;
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 2, val | BIT(3));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int rtl8188eu_verify_fw_checksum(struct rtl8188eu_priv *priv)
{
	u32 val32;
	int retry, ret;

	for (retry = 0; retry < 50; retry++) {
		ret = rtl8188eu_read_reg32(priv, REG_MCUFWDL, &val32);
		if (ret < 0)
			return ret;
		if (val32 & BIT(2))
			return 0;
		msleep(1);
	}

	pr_err("%s: Firmware checksum FAILED\n", DRIVER_NAME);
	return -EIO;
}

static int rtl8188eu_init_h2c(struct rtl8188eu_priv *priv)
{
	int ret;

	ret = rtl8188eu_write_reg8(priv, REG_HMETFR, 0x0F);
	if (ret < 0)
		return ret;

	priv->last_hmebox_num = 0;
	msleep(10);
	return 0;
}

static int rtl8188eu_fw_free_to_go(struct rtl8188eu_priv *priv)
{
	u32 val32;
	int retry, ret;

	ret = rtl8188eu_read_reg32(priv, REG_MCUFWDL, &val32);
	if (ret < 0)
		return ret;

	if (val32 & RAM_DL_SEL) {
		ret = rtl8188eu_write_reg32(priv, REG_MCUFWDL, 0x00);
		if (ret < 0)
			return ret;
	}

	ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 1, 0x00);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg32(priv, REG_MCUFWDL, MCUFWDL_RDY);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_reset_8051(priv);
	if (ret < 0)
		return ret;

	for (retry = 0; retry < 200; retry++) {
		ret = rtl8188eu_read_reg32(priv, REG_MCUFWDL, &val32);
		if (ret < 0)
			return ret;
		if (val32 & WINTINI_RDY) {
			pr_info("%s: Firmware ready (%d ms)\n", DRIVER_NAME, retry);
			return 0;
		}
		msleep(1);
	}

	pr_warn("%s: Firmware WINTINI_RDY timeout (continuing)\n", DRIVER_NAME);
	return 0;
}

static int rtl8188eu_load_firmware(struct rtl8188eu_priv *priv)
{
	int ret;

	ret = request_firmware(&priv->fw, FIRMWARE_NAME, &priv->udev->dev);
	if (ret)
		return ret;

	ret = rtl8188eu_fw_download_enable(priv, true);
	if (ret)
		goto err_release;

	ret = rtl8188eu_download_firmware(priv);
	if (ret) {
		rtl8188eu_fw_download_enable(priv, false);
		goto err_release;
	}

	ret = rtl8188eu_fw_download_enable(priv, false);
	if (ret)
		goto err_release;

	msleep(50);

	ret = rtl8188eu_verify_fw_checksum(priv);
	if (ret)
		pr_warn("%s: Firmware checksum failed (continuing)\n", DRIVER_NAME);

	ret = rtl8188eu_init_h2c(priv);
	if (ret)
		pr_warn("%s: H2C init failed (continuing)\n", DRIVER_NAME);

	ret = rtl8188eu_fw_free_to_go(priv);
	if (ret)
		goto err_release;

	pr_info("%s: Firmware loaded and activated\n", DRIVER_NAME);
	return 0;

err_release:
	release_firmware(priv->fw);
	priv->fw = NULL;
	return ret;
}

/*
 * ============================================================================
 * Channel Tuning
 * ============================================================================
 */

static int rtl8188eu_set_channel(struct rtl8188eu_priv *priv, u8 channel)
{
	u32 rf_val;

	if (channel < 1 || channel > 14)
		return -EINVAL;

	priv->rf_chnl_val = (priv->rf_chnl_val & 0xfffffc00) | channel;
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask,
			       priv->rf_chnl_val);

	rf_val = rtl8188eu_read_rf_reg(priv, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask);
	pr_debug("%s: RF_CHNLBW = 0x%05x after channel set\n", DRIVER_NAME, rf_val);

	priv->channel = channel;
	return 0;
}

/*
 * ============================================================================
 * RX Path — mac80211 conversion
 * ============================================================================
 */

/* Map HW rate code to rate_idx in our rates array */
static u8 rtl8188eu_hw_rate_to_rate_idx(u8 hw_rate)
{
	if (hw_rate <= HW_RATE_CCK11)
		return hw_rate;
	if (hw_rate >= HW_RATE_OFDM6 && hw_rate <= HW_RATE_OFDM54)
		return 4 + (hw_rate - HW_RATE_OFDM6);
	return 0;
}

static void rtl8188eu_rx_complete(struct urb *urb)
{
	struct rtl8188eu_rx_urb *rx_urb;
	struct rtl8188eu_priv *priv;
	struct ieee80211_hw *hw;
	struct rtl8188eu_rx_desc *rx_desc;
	struct sk_buff *skb;
	struct ieee80211_rx_status *rx_status;
	u32 rxdw0, rxdw2, rxdw3;
	u16 pkt_len, pkt_cnt;
	u16 drvinfo_sz, shift_sz;
	u32 pkt_offset;
	s32 transfer_len;
	u8 *pbuf;
	int ret;
	static unsigned int rx_callback_count;

	if (!urb)
		return;

	rx_urb = urb->context;
	if (!rx_urb || !rx_urb->priv || !rx_urb->priv->hw)
		return;

	priv = rx_urb->priv;
	hw = priv->hw;

	rx_callback_count++;

	if (rx_callback_count <= 3)
		pr_info("%s: RX callback #%u: status=%d, len=%d\n",
			DRIVER_NAME, rx_callback_count, urb->status, urb->actual_length);

	if (urb->status != 0) {
		switch (urb->status) {
		case -ENOENT:
		case -ECONNRESET:
		case -ESHUTDOWN:
		case -ENODEV:
			return;
		default:
			goto resubmit;
		}
	}

	if (urb->actual_length == 0 || urb->actual_length < RXDESC_SIZE)
		goto resubmit;

	pbuf = urb->transfer_buffer;
	transfer_len = (s32)urb->actual_length;

	if (priv->scanning)
		priv->scan_urb_count++;

	rx_desc = (struct rtl8188eu_rx_desc *)pbuf;
	rxdw2 = le32_to_cpu(rx_desc->rxdw2);
	pkt_cnt = (rxdw2 >> RX_DW2_PKT_CNT_SHIFT) & RX_DW2_PKT_CNT_MASK;
	if (pkt_cnt == 0)
		pkt_cnt = 1;

	do {
		if (transfer_len < (s32)RXDESC_SIZE)
			break;

		rx_desc = (struct rtl8188eu_rx_desc *)pbuf;
		rxdw0 = le32_to_cpu(rx_desc->rxdw0);

		pkt_len = rxdw0 & RX_DW0_PKT_LEN_MASK;
		drvinfo_sz = ((rxdw0 >> RX_DW0_DRVINFO_SZ_SHIFT) & RX_DW0_DRVINFO_SZ_MASK) * 8;
		shift_sz = (rxdw0 >> RX_DW0_SHIFT_SHIFT) & RX_DW0_SHIFT_MASK;

		pkt_offset = RXDESC_SIZE + drvinfo_sz + shift_sz + pkt_len;

		if (pkt_len == 0 || (s32)pkt_offset > transfer_len) {
			if (priv->scanning)
				priv->scan_rx_drop_len++;
			break;
		}

		if (rxdw0 & (RX_DW0_CRC32 | RX_DW0_ICV_ERR)) {
			if (priv->scanning)
				priv->scan_rx_drop_crc++;
			goto next_pkt;
		}

		/* Allocate skb for the 802.11 frame only (no radiotap — mac80211 adds it) */
		skb = dev_alloc_skb(pkt_len);
		if (!skb)
			goto next_pkt;

		skb_put_data(skb, pbuf + RXDESC_SIZE + drvinfo_sz + shift_sz, pkt_len);

		/* Fill ieee80211_rx_status via SKB control buffer */
		rx_status = IEEE80211_SKB_RXCB(skb);
		memset(rx_status, 0, sizeof(*rx_status));

		rx_status->freq = rtl8188eu_chan_to_freq(priv->channel);
		rx_status->band = NL80211_BAND_2GHZ;
		rx_status->antenna = 0;
		rx_status->signal = -50;

		/* Parse signal from PHY status */
		if ((rxdw0 & RX_DW0_PHYST) && drvinfo_sz >= 8) {
			u8 *phy_status = pbuf + RXDESC_SIZE;
			s8 sig = ((phy_status[0] & 0x3f) * 2) - 110;

			if (sig < -100)
				sig = -100;
			if (sig > 0)
				sig = 0;
			rx_status->signal = sig;
		}

		/* Parse rate from RX descriptor DW3 */
		rxdw3 = le32_to_cpu(rx_desc->rxdw3);
		{
			u8 hw_rate = rxdw3 & 0x3f;
			bool is_ht = (rxdw3 & BIT(6)) || (hw_rate >= HW_RATE_MCS0);

			if (is_ht) {
				rx_status->encoding = RX_ENC_HT;
				rx_status->rate_idx = hw_rate - HW_RATE_MCS0;
				rx_status->bw = RATE_INFO_BW_20;
			} else {
				rx_status->encoding = RX_ENC_LEGACY;
				rx_status->rate_idx = rtl8188eu_hw_rate_to_rate_idx(hw_rate);
				rx_status->bw = RATE_INFO_BW_20;
			}
		}

		if (priv->scanning)
			priv->scan_rx_count++;
		ieee80211_rx_irqsafe(hw, skb);

next_pkt:
		pkt_offset = RX_RND128(pkt_offset);
		pkt_cnt--;
		transfer_len -= pkt_offset;
		pbuf += pkt_offset;

		if (transfer_len > 0 && pkt_cnt == 0) {
			rx_desc = (struct rtl8188eu_rx_desc *)pbuf;
			rxdw2 = le32_to_cpu(rx_desc->rxdw2);
			pkt_cnt = (rxdw2 >> RX_DW2_PKT_CNT_SHIFT) & RX_DW2_PKT_CNT_MASK;
		}
	} while (transfer_len > 0 && pkt_cnt > 0);

	if ((rx_callback_count % 1000) == 0)
		pr_info("%s: RX stats: %u callbacks\n", DRIVER_NAME, rx_callback_count);

resubmit:
	if (priv->disconnecting || !priv->started)
		return;

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		pr_err("%s: RX URB resubmit failed: %d\n", DRIVER_NAME, ret);
}

static int rtl8188eu_start_rx(struct rtl8188eu_priv *priv)
{
	unsigned int pipe;
	int i, ret;

	pipe = usb_rcvbulkpipe(priv->udev, priv->rx_endpoint);

	for (i = 0; i < RX_URB_COUNT; i++) {
		struct rtl8188eu_rx_urb *rx_urb = &priv->rx_urbs[i];

		rx_urb->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!rx_urb->urb) {
			ret = -ENOMEM;
			goto err_free_urbs;
		}

		rx_urb->buffer = kmalloc(RX_BUFFER_SIZE, GFP_KERNEL);
		if (!rx_urb->buffer) {
			usb_free_urb(rx_urb->urb);
			rx_urb->urb = NULL;
			ret = -ENOMEM;
			goto err_free_urbs;
		}

		rx_urb->priv = priv;

		usb_fill_bulk_urb(rx_urb->urb, priv->udev, pipe,
				  rx_urb->buffer, RX_BUFFER_SIZE,
				  rtl8188eu_rx_complete, rx_urb);

		ret = usb_submit_urb(rx_urb->urb, GFP_KERNEL);
		if (ret < 0) {
			kfree(rx_urb->buffer);
			usb_free_urb(rx_urb->urb);
			rx_urb->urb = NULL;
			rx_urb->buffer = NULL;
			goto err_free_urbs;
		}
	}

	return 0;

err_free_urbs:
	for (i = 0; i < RX_URB_COUNT; i++) {
		struct rtl8188eu_rx_urb *rx_urb = &priv->rx_urbs[i];
		if (rx_urb->urb) {
			usb_kill_urb(rx_urb->urb);
			usb_free_urb(rx_urb->urb);
			rx_urb->urb = NULL;
		}
		kfree(rx_urb->buffer);
		rx_urb->buffer = NULL;
	}
	return ret;
}

static void rtl8188eu_stop_rx(struct rtl8188eu_priv *priv)
{
	int i;

	for (i = 0; i < RX_URB_COUNT; i++) {
		struct rtl8188eu_rx_urb *rx_urb = &priv->rx_urbs[i];

		if (rx_urb->urb) {
			usb_kill_urb(rx_urb->urb);
			usb_free_urb(rx_urb->urb);
			rx_urb->urb = NULL;
		}

		kfree(rx_urb->buffer);
		rx_urb->buffer = NULL;
	}
}

/*
 * ============================================================================
 * TX Path — Phase 2 (mac80211 TX)
 * ============================================================================
 */

struct rtl8188eu_tx_context {
	struct rtl8188eu_priv *priv;
	struct sk_buff *skb;
};

static void rtl8188eu_tx_complete(struct urb *urb)
{
	struct rtl8188eu_tx_context *ctx = urb->context;
	struct rtl8188eu_priv *priv = ctx->priv;
	struct ieee80211_hw *hw = priv->hw;
	struct sk_buff *skb = ctx->skb;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	ieee80211_tx_info_clear_status(info);

	if (urb->status == 0)
		info->flags |= IEEE80211_TX_STAT_ACK;

	ieee80211_tx_status_irqsafe(hw, skb);

	usb_free_urb(urb);
	kfree(ctx);
}

static void rtl8188eu_mac80211_tx(struct ieee80211_hw *hw,
				   struct ieee80211_tx_control *control,
				   struct sk_buff *skb)
{
	struct rtl8188eu_priv *priv = hw->priv;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_tx_rate *txrate = &info->control.rates[0];
	struct rtl8188eu_tx_context *ctx;
	struct rtl8188eu_tx_desc *tx_desc;
	struct urb *urb;
	unsigned int pipe;
	u8 *tx_buffer;
	u32 total_len;
	u8 hw_rate;
	u8 qsel;
	int ret;

	if (priv->disconnecting || !priv->started) {
		ieee80211_free_txskb(hw, skb);
		return;
	}

	total_len = TX_DESC_SIZE + skb->len;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		ieee80211_free_txskb(hw, skb);
		return;
	}

	tx_buffer = kmalloc(total_len, GFP_ATOMIC);
	if (!tx_buffer) {
		usb_free_urb(urb);
		ieee80211_free_txskb(hw, skb);
		return;
	}

	ctx = kmalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		kfree(tx_buffer);
		usb_free_urb(urb);
		ieee80211_free_txskb(hw, skb);
		return;
	}

	ctx->priv = priv;
	ctx->skb = skb;

	/* Build TX descriptor */
	memset(tx_buffer, 0, TX_DESC_SIZE);
	tx_desc = (struct rtl8188eu_tx_desc *)tx_buffer;

	/* DW0: packet size and descriptor offset */
	tx_desc->txdw0 = cpu_to_le32(
		(skb->len << TX_DW0_PKT_SIZE_SHIFT) |
		(TX_DESC_SIZE << TX_DW0_OFFSET_SHIFT) |
		TX_DW0_OWN
	);

	/* Multicast/broadcast detection */
	if (is_multicast_ether_addr(skb->data))
		tx_desc->txdw0 |= cpu_to_le32(TX_DW0_BMC);

	/* DW1: queue selection based on mac80211 queue mapping */
	switch (skb_get_queue_mapping(skb)) {
	case IEEE80211_AC_VO:
		qsel = TX_DW1_QUEUE_SEL_VO;
		break;
	case IEEE80211_AC_VI:
		qsel = TX_DW1_QUEUE_SEL_VI;
		break;
	case IEEE80211_AC_BK:
		qsel = TX_DW1_QUEUE_SEL_BK;
		break;
	default:
		qsel = TX_DW1_QUEUE_SEL_BE;
		break;
	}

	/* Use management queue for mgmt frames */
	if (ieee80211_is_mgmt(*((__le16 *)skb->data)))
		qsel = TX_DW1_QUEUE_SEL_MGNT;

	tx_desc->txdw1 = cpu_to_le32(qsel << TX_DW1_QUEUE_SEL_SHIFT);

	/* DW3: enable HW sequence numbering */
	tx_desc->txdw3 = cpu_to_le32(TX_DW3_HW_SEQ_EN);

	/* DW4: rate control — let mac80211/minstrel pick the rate */
	if (txrate->idx >= 0) {
		if (txrate->flags & IEEE80211_TX_RC_MCS) {
			hw_rate = HW_RATE_MCS0 + txrate->idx;
		} else {
			if ((size_t)txrate->idx < ARRAY_SIZE(rtl8188eu_rates))
				hw_rate = rtl8188eu_rates[txrate->idx].hw_value;
			else
				hw_rate = HW_RATE_OFDM6;
		}
	} else {
		hw_rate = HW_RATE_OFDM6;
	}

	tx_desc->txdw4 = cpu_to_le32(
		TX_DW4_USE_RATE |
		(hw_rate << TX_DW4_DATA_RATE_SHIFT)
	);

	if (txrate->flags & IEEE80211_TX_RC_40_MHZ_WIDTH)
		tx_desc->txdw4 |= cpu_to_le32(TX_DW4_DATA_BW);

	if (txrate->flags & IEEE80211_TX_RC_SHORT_GI)
		tx_desc->txdw4 |= cpu_to_le32(TX_DW4_DATA_SHORT);

	/* Copy 802.11 frame after descriptor */
	memcpy(tx_buffer + TX_DESC_SIZE, skb->data, skb->len);

	pipe = usb_sndbulkpipe(priv->udev, priv->tx_endpoint);

	usb_fill_bulk_urb(urb, priv->udev, pipe,
			  tx_buffer, total_len,
			  rtl8188eu_tx_complete, ctx);

	urb->transfer_flags |= URB_FREE_BUFFER;

	if (priv->scanning)
		priv->scan_tx_count++;

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret < 0) {
		kfree(ctx);
		usb_free_urb(urb);
		ieee80211_free_txskb(hw, skb);
	}
}

/*
 * ============================================================================
 * mac80211 Operations — Phases 1-5
 * ============================================================================
 */

static int rtl8188eu_mac80211_start(struct ieee80211_hw *hw)
{
	struct rtl8188eu_priv *priv = hw->priv;
	u32 val32;
	int ret;

	pr_info("%s: mac80211 start\n", DRIVER_NAME);

	/* Clear RW_RELEASE_EN */
	ret = rtl8188eu_read_reg32(priv, REG_RXPKT_NUM, &val32);
	if (ret == 0 && (val32 & RW_RELEASE_EN)) {
		val32 &= ~RW_RELEASE_EN;
		rtl8188eu_write_reg32(priv, REG_RXPKT_NUM, val32);
	}

	/* Start RX URBs */
	ret = rtl8188eu_start_rx(priv);
	if (ret)
		return ret;

	/* Set default channel 6 */
	rtl8188eu_set_channel(priv, 6);

	priv->started = true;
	return 0;
}

static void rtl8188eu_mac80211_stop(struct ieee80211_hw *hw, bool suspend)
{
	struct rtl8188eu_priv *priv = hw->priv;

	pr_info("%s: mac80211 stop\n", DRIVER_NAME);

	priv->started = false;
	rtl8188eu_stop_rx(priv);
}

static int rtl8188eu_mac80211_add_iface(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif)
{
	struct rtl8188eu_priv *priv = hw->priv;

	pr_info("%s: add_interface type=%d\n", DRIVER_NAME, vif->type);

	switch (vif->type) {
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_MONITOR:
		break;
	default:
		return -EOPNOTSUPP;
	}

	priv->vif = vif;
	return 0;
}

static void rtl8188eu_mac80211_rm_iface(struct ieee80211_hw *hw,
					  struct ieee80211_vif *vif)
{
	struct rtl8188eu_priv *priv = hw->priv;

	pr_info("%s: remove_interface\n", DRIVER_NAME);

	if (priv->vif == vif)
		priv->vif = NULL;
}

static int rtl8188eu_mac80211_config(struct ieee80211_hw *hw,
				      int radio_idx, u32 changed)
{
	struct rtl8188eu_priv *priv = hw->priv;
	struct ieee80211_conf *conf = &hw->conf;

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		struct ieee80211_channel *chan = conf->chandef.chan;
		u8 channel = chan->hw_value;

		pr_info("%s: config: channel -> %d (%d MHz)\n",
			DRIVER_NAME, channel, chan->center_freq);
		rtl8188eu_set_channel(priv, channel);

		/* Set bandwidth */
		rtl8188eu_set_bw(priv, &conf->chandef);
	}

	return 0;
}

static void rtl8188eu_mac80211_conf_filter(struct ieee80211_hw *hw,
					    unsigned int changed_flags,
					    unsigned int *total_flags,
					    u64 multicast)
{
	struct rtl8188eu_priv *priv = hw->priv;
	u32 rcr;

	/* Preserve only the flags we can handle */
	*total_flags &= FIF_ALLMULTI | FIF_FCSFAIL | FIF_CONTROL |
			FIF_OTHER_BSS | FIF_PSPOLL | FIF_PROBE_REQ;

	priv->rx_filter = *total_flags;

	/* Base RCR — always include FCS (matches IEEE80211_HW_RX_INCLUDES_FCS) */
	rcr = RCR_APM | RCR_AM | RCR_AB | RCR_AMF |
	      RCR_APP_PHYST_RXFF | RCR_APP_ICV | RCR_APP_MIC |
	      RCR_HTC_LOC_CTRL | RCR_ADF | RCR_APPFCS;

	/* Monitor mode or promiscuous — accept everything */
	if ((priv->vif && priv->vif->type == NL80211_IFTYPE_MONITOR) ||
	    (*total_flags & FIF_OTHER_BSS)) {
		rcr |= RCR_AAP | RCR_APWRMGT | RCR_ACF;
	} else {
		/* Station mode — filter by BSSID */
		rcr |= RCR_CBSSID_DATA | RCR_CBSSID_BCN;
	}

	if (*total_flags & FIF_ALLMULTI)
		rcr |= RCR_AM;
	if (*total_flags & FIF_FCSFAIL)
		rcr |= RCR_ACRC32;
	if (*total_flags & FIF_CONTROL)
		rcr |= RCR_ACF;

	rtl8188eu_write_reg32(priv, REG_RCR, rcr);

	/* Accept all multicast */
	rtl8188eu_write_reg32(priv, REG_MAR, 0xFFFFFFFF);
	rtl8188eu_write_reg32(priv, REG_MAR + 4, 0xFFFFFFFF);
}

/* Phase 3: Station mode — bss_info_changed */
static void rtl8188eu_mac80211_bss_changed(struct ieee80211_hw *hw,
					    struct ieee80211_vif *vif,
					    struct ieee80211_bss_conf *info,
					    u64 changed)
{
	struct rtl8188eu_priv *priv = hw->priv;

	if (changed & BSS_CHANGED_BSSID) {
		const u8 *bssid = info->bssid;

		pr_info("%s: BSSID changed to %pM\n", DRIVER_NAME, bssid);

		/* Write BSSID to hardware (6 bytes at REG_BSSID) */
		rtl8188eu_write_reg32(priv, REG_BSSID,
			bssid[0] | (bssid[1] << 8) |
			(bssid[2] << 16) | (bssid[3] << 24));
		rtl8188eu_write_reg16(priv, REG_BSSID + 4,
			bssid[4] | (bssid[5] << 8));
	}

	if (changed & BSS_CHANGED_ASSOC) {
		u8 msr;

		if (vif->cfg.assoc) {
			pr_info("%s: Associated\n", DRIVER_NAME);

			/* Set MSR to infrastructure (station) mode */
			rtl8188eu_read_reg8(priv, REG_MSR, &msr);
			msr = (msr & 0xFC) | MSR_INFRA;
			rtl8188eu_write_reg8(priv, REG_MSR, msr);

			/* Enable BSSID filtering */
			{
				u32 rcr;
				rtl8188eu_read_reg32(priv, REG_RCR, &rcr);
				rcr |= RCR_CBSSID_DATA | RCR_CBSSID_BCN;
				rcr &= ~RCR_AAP;
				rtl8188eu_write_reg32(priv, REG_RCR, rcr);
			}
		} else {
			pr_info("%s: Disassociated\n", DRIVER_NAME);

			/* Set MSR to no-link mode */
			rtl8188eu_read_reg8(priv, REG_MSR, &msr);
			msr = (msr & 0xFC) | MSR_NOLINK;
			rtl8188eu_write_reg8(priv, REG_MSR, msr);

			/* Disable BSSID filtering for scanning */
			{
				u32 rcr;
				rtl8188eu_read_reg32(priv, REG_RCR, &rcr);
				rcr &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);
				rtl8188eu_write_reg32(priv, REG_RCR, rcr);
			}
		}
	}

	if (changed & BSS_CHANGED_BEACON_INT) {
		rtl8188eu_write_reg16(priv, REG_BCN_INTERVAL, info->beacon_int);
		pr_debug("%s: Beacon interval set to %d\n", DRIVER_NAME,
			 info->beacon_int);
	}

	if (changed & BSS_CHANGED_BANDWIDTH) {
		struct cfg80211_chan_def *chandef = &vif->bss_conf.chanreq.oper;

		rtl8188eu_set_bw(priv, chandef);
	}
}

static void rtl8188eu_mac80211_scan_start(struct ieee80211_hw *hw,
					   struct ieee80211_vif *vif,
					   const u8 *mac_addr)
{
	struct rtl8188eu_priv *priv = hw->priv;
	u32 rcr;

	pr_info("%s: Scan start\n", DRIVER_NAME);

	/* Clear scan diagnostic counters */
	priv->scan_tx_count = 0;
	priv->scan_rx_count = 0;
	priv->scan_rx_drop_crc = 0;
	priv->scan_rx_drop_len = 0;
	priv->scan_urb_count = 0;
	priv->scanning = true;

	/* Disable BSSID filtering during scan */
	rtl8188eu_read_reg32(priv, REG_RCR, &rcr);
	rcr &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);
	rtl8188eu_write_reg32(priv, REG_RCR, rcr);
}

static void rtl8188eu_mac80211_scan_end(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif)
{
	struct rtl8188eu_priv *priv = hw->priv;

	priv->scanning = false;
	pr_info("%s: Scan stats: %u URBs, %u frames to mac80211, %u CRC drops, %u len drops, %u TX frames\n",
		DRIVER_NAME, priv->scan_urb_count, priv->scan_rx_count,
		priv->scan_rx_drop_crc, priv->scan_rx_drop_len, priv->scan_tx_count);

	/* Re-enable BSSID filtering if associated */
	if (priv->vif && priv->vif->cfg.assoc) {
		u32 rcr;

		rtl8188eu_read_reg32(priv, REG_RCR, &rcr);
		rcr |= RCR_CBSSID_DATA | RCR_CBSSID_BCN;
		rtl8188eu_write_reg32(priv, REG_RCR, rcr);
	}
}

static int rtl8188eu_mac80211_set_key(struct ieee80211_hw *hw,
				       enum set_key_cmd cmd,
				       struct ieee80211_vif *vif,
				       struct ieee80211_sta *sta,
				       struct ieee80211_key_conf *key)
{
	/* Return -EOPNOTSUPP to tell mac80211 to use software crypto */
	return -EOPNOTSUPP;
}

static const struct ieee80211_ops rtl8188eu_mac80211_ops = {
	.tx			= rtl8188eu_mac80211_tx,
	.start			= rtl8188eu_mac80211_start,
	.stop			= rtl8188eu_mac80211_stop,
	.add_interface		= rtl8188eu_mac80211_add_iface,
	.remove_interface	= rtl8188eu_mac80211_rm_iface,
	.config			= rtl8188eu_mac80211_config,
	.configure_filter	= rtl8188eu_mac80211_conf_filter,
	.bss_info_changed	= rtl8188eu_mac80211_bss_changed,
	.sw_scan_start		= rtl8188eu_mac80211_scan_start,
	.sw_scan_complete	= rtl8188eu_mac80211_scan_end,
	.set_key		= rtl8188eu_mac80211_set_key,
	.wake_tx_queue		= ieee80211_handle_wake_tx_queue,
	.add_chanctx		= ieee80211_emulate_add_chanctx,
	.remove_chanctx		= ieee80211_emulate_remove_chanctx,
	.change_chanctx		= ieee80211_emulate_change_chanctx,
};

/*
 * ============================================================================
 * USB Endpoint Detection
 * ============================================================================
 */

static int rtl8188eu_detect_endpoints(struct rtl8188eu_priv *priv)
{
	struct usb_interface *intf = priv->intf;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	int tx_ep_count = 0;

	iface_desc = intf->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_in(endpoint))
			priv->rx_endpoint = endpoint->bEndpointAddress;

		if (usb_endpoint_is_bulk_out(endpoint)) {
			if (tx_ep_count == 0)
				priv->tx_endpoint = endpoint->bEndpointAddress;
			tx_ep_count++;
		}
	}

	if (!priv->rx_endpoint || !priv->tx_endpoint)
		return -ENODEV;

	priv->out_ep_number = tx_ep_count;

	switch (tx_ep_count) {
	case 1:
		priv->out_ep_queue_sel = TX_SELE_LQ;
		break;
	case 2:
		priv->out_ep_queue_sel = TX_SELE_HQ | TX_SELE_NQ;
		break;
	default:
		priv->out_ep_queue_sel = TX_SELE_HQ | TX_SELE_LQ | TX_SELE_NQ;
		break;
	}

	return 0;
}

static int rtl8188eu_init_queue_priority(struct rtl8188eu_priv *priv)
{
	u16 value16;
	int ret;

	ret = rtl8188eu_read_reg16(priv, REG_TRXDMA_CTRL, &value16);
	if (ret < 0)
		return ret;

	value16 &= 0x0007;
	value16 |= _TXDMA_HIQ_MAP(QUEUE_HIGH) |
		   _TXDMA_MGQ_MAP(QUEUE_HIGH) |
		   _TXDMA_BKQ_MAP(QUEUE_NORMAL) |
		   _TXDMA_BEQ_MAP(QUEUE_NORMAL) |
		   _TXDMA_VIQ_MAP(QUEUE_HIGH) |
		   _TXDMA_VOQ_MAP(QUEUE_HIGH);

	return rtl8188eu_write_reg16(priv, REG_TRXDMA_CTRL, value16);
}

static int rtl8188eu_init_hw_drop_incorrect_bulkout(struct rtl8188eu_priv *priv)
{
	u32 value32;
	int ret;

	ret = rtl8188eu_read_reg32(priv, REG_TXDMA_OFFSET_CHK, &value32);
	if (ret < 0)
		return ret;

	value32 |= BIT(9);
	return rtl8188eu_write_reg32(priv, REG_TXDMA_OFFSET_CHK, value32);
}

static int rtl8188eu_init_tx_report(struct rtl8188eu_priv *priv)
{
	u8 val8;
	int ret;

	ret = rtl8188eu_read_reg8(priv, REG_TX_RPT_CTRL, &val8);
	if (ret < 0)
		return ret;

	val8 |= BIT(1) | BIT(0);
	ret = rtl8188eu_write_reg8(priv, REG_TX_RPT_CTRL, val8);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg8(priv, REG_TX_RPT_CTRL + 1, 0x02);
	if (ret < 0)
		return ret;

	return rtl8188eu_write_reg16(priv, REG_TX_RPT_TIME, 0xCDF0);
}

static int rtl8188eu_init_wmac_setting(struct rtl8188eu_priv *priv)
{
	u32 rcr;
	int ret;

	rcr = RCR_APM | RCR_AM | RCR_AB |
	      RCR_APP_ICV | RCR_AMF | RCR_HTC_LOC_CTRL |
	      RCR_APP_MIC | RCR_APP_PHYST_RXFF | RCR_ADF | RCR_APPFCS;

	ret = rtl8188eu_write_reg32(priv, REG_RCR, rcr);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg32(priv, REG_MAR, 0xFFFFFFFF);
	if (ret < 0)
		return ret;
	return rtl8188eu_write_reg32(priv, REG_MAR + 4, 0xFFFFFFFF);
}

/*
 * ============================================================================
 * Probe / Disconnect
 * ============================================================================
 */

static int rtl8188eu_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct rtl8188eu_priv *priv;
	struct ieee80211_hw *hw;
	int ret;
	u8 val8;

	pr_info("%s: RTL8188EUS detected (bus %d, dev %d)\n",
		DRIVER_NAME, udev->bus->busnum, udev->devnum);

	/* Allocate mac80211 hw + priv */
	hw = ieee80211_alloc_hw(sizeof(*priv), &rtl8188eu_mac80211_ops);
	if (!hw)
		return -ENOMEM;

	priv = hw->priv;
	priv->hw = hw;
	priv->udev = udev;
	priv->intf = intf;
	priv->chip_powered_on = false;
	priv->current_bw = NL80211_CHAN_WIDTH_20;
	mutex_init(&priv->rf_read_mutex);

	priv->hw_cfg.interface_type = 1;
	priv->hw_cfg.board_type = 0;
	priv->hw_cfg.cut_version = 0;
	priv->hw_cfg.package_type = 0;

	usb_set_intfdata(intf, hw);

	/* Set mac80211 hw properties */
	SET_IEEE80211_DEV(hw, &intf->dev);

	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, RX_INCLUDES_FCS);
	ieee80211_hw_set(hw, MFP_CAPABLE);
	ieee80211_hw_set(hw, REPORTS_TX_ACK_STATUS);

	hw->extra_tx_headroom = TX_DESC_SIZE;
	hw->queues = 4;

	hw->wiphy->interface_modes = BIT(NL80211_IFTYPE_STATION) |
				     BIT(NL80211_IFTYPE_MONITOR);

	hw->wiphy->max_scan_ssids = 4;
	hw->wiphy->max_scan_ie_len = IEEE80211_MAX_DATA_LEN;

	hw->wiphy->bands[NL80211_BAND_2GHZ] = &rtl8188eu_band_2ghz;

	/* Detect USB endpoints */
	ret = rtl8188eu_detect_endpoints(priv);
	if (ret)
		goto err_free_hw;

	/* Generate random MAC (later can read from EFUSE) */
	eth_random_addr(priv->mac_addr);
	SET_IEEE80211_PERM_ADDR(hw, priv->mac_addr);

	/* Program MAC address into hardware for unicast filtering (REG_MACID) */
	rtl8188eu_write_reg32(priv, 0x0610,
		priv->mac_addr[0] | (priv->mac_addr[1] << 8) |
		(priv->mac_addr[2] << 16) | (priv->mac_addr[3] << 24));
	rtl8188eu_write_reg16(priv, 0x0614,
		priv->mac_addr[4] | (priv->mac_addr[5] << 8));

	pr_info("%s: MAC Address: %pM\n", DRIVER_NAME, priv->mac_addr);

	/* Hardware initialization */
	ret = rtl8188eu_power_on(priv);
	if (ret)
		goto err_free_hw;

	ret = rtl8188eu_enable_mcu_clocks(priv);
	if (ret)
		goto err_free_hw;

	ret = rtl8188eu_init_tx_rx_queues(priv);
	if (ret)
		goto err_free_hw;

	rtl8188eu_init_queue_priority(priv);

	ret = rtl8188eu_init_llt_table(priv);
	if (ret)
		goto err_free_hw;

	ret = rtl8188eu_load_firmware(priv);
	if (ret)
		goto err_free_hw;

	rtl8188eu_load_mac_reg_table(priv);
	rtl8188eu_init_interrupts(priv);

	rtl8188eu_write_reg8(priv, REG_RX_DRVINFO_SZ, 0x04);

	{
		u32 cr32;
		ret = rtl8188eu_read_reg32(priv, REG_CR, &cr32);
		if (ret == 0) {
			cr32 = (cr32 & ~0x30000) | (0x2 << 16);
			rtl8188eu_write_reg32(priv, REG_CR, cr32);
		}
	}

	rtl8188eu_init_wmac_setting(priv);
	rtl8188eu_init_mac_regs(priv);

	/* AFE crystal + PLL */
	rtl8188eu_write_reg32(priv, REG_AFE_XTAL_CTRL, 0x000F81FB);
	msleep(5);

	{
		u32 afe_pll;
		ret = rtl8188eu_read_reg32(priv, REG_AFE_PLL_CTRL, &afe_pll);
		if (ret == 0) {
			afe_pll &= ~BIT(6);
			rtl8188eu_write_reg32(priv, REG_AFE_PLL_CTRL, afe_pll);
			msleep(5);
		}
	}

	/* Enable BB */
	{
		u16 sys_func;
		ret = rtl8188eu_read_reg16(priv, REG_SYS_FUNC_EN, &sys_func);
		if (ret == 0) {
			sys_func |= BIT(13) | FEN_BBRSTB | FEN_BB_GLB_RSTn;
			rtl8188eu_write_reg16(priv, REG_SYS_FUNC_EN, sys_func);
			msleep(2);
		}
	}

	/* Enable RF */
	{
		u8 rf_ctrl;
		ret = rtl8188eu_read_reg8(priv, REG_RF_CTRL, &rf_ctrl);
		if (ret == 0) {
			rf_ctrl |= 0x07;
			rtl8188eu_write_reg8(priv, REG_RF_CTRL, rf_ctrl);
			msleep(1);
		}
	}

	/* Read chip version */
	{
		u32 sys_cfg;
		ret = rtl8188eu_read_reg32(priv, REG_SYS_CFG, &sys_cfg);
		if (ret == 0) {
			u8 cut = (sys_cfg >> 12) & 0xf;

			priv->hw_cfg.cut_version = cut;
			pr_info("%s: Chip: RTL8188E, Cut %c, %s\n", DRIVER_NAME,
				'A' + cut,
				(sys_cfg & BIT(19)) ? "UMC" : "TSMC");
		}
	}

	/* Load PHY tables */
	ret = rtl8188eu_phy_bb_config(priv);
	if (ret)
		pr_err("%s: BB config failed: %d\n", DRIVER_NAME, ret);

	/* Crystal cap */
	{
		u32 xtal;
		ret = rtl8188eu_read_reg32(priv, REG_AFE_XTAL_CTRL, &xtal);
		if (ret == 0) {
			u32 cap_val = EEPROM_DEFAULT_CRYSTAL_CAP |
				      (EEPROM_DEFAULT_CRYSTAL_CAP << 6);
			xtal = (xtal & ~0x007FF800) | (cap_val << 11);
			rtl8188eu_write_reg32(priv, REG_AFE_XTAL_CTRL, xtal);
		}
	}

	msleep(10);

	ret = rtl8188eu_phy_rf_config(priv);
	if (ret)
		pr_err("%s: RF config failed: %d\n", DRIVER_NAME, ret);

	/* Cache RF_CHNLBW and set 20MHz BW */
	{
		u32 rf18;

		rf18 = rtl8188eu_read_rf_reg(priv, RF_PATH_A, 0x18, 0xFFFFF);
		priv->rf_chnl_val = rf18;
		priv->rf_chnl_val = (priv->rf_chnl_val & 0xfffff3ff) | BIT(10) | BIT(11);
		rtl8188eu_write_rf_reg(priv, RF_PATH_A, RF_CHNLBW,
				       bRFRegOffsetMask, priv->rf_chnl_val);
	}

	rtl8188eu_lc_calibrate(priv);
	rtl8188eu_iqk_calibrate(priv);

	/* Enable CCK + OFDM */
	{
		u32 rfmod;
		ret = rtl8188eu_read_reg32(priv, rFPGA0_RFMOD, &rfmod);
		if (ret == 0) {
			rfmod |= bCCKEn | bOFDMEn;
			rtl8188eu_write_reg32(priv, rFPGA0_RFMOD, rfmod);
		}
	}

	/* Set 20MHz BB mode */
	{
		u32 rfmod1;
		ret = rtl8188eu_read_reg32(priv, 0x900, &rfmod1);
		if (ret == 0) {
			rfmod1 &= ~BIT(0);
			rtl8188eu_write_reg32(priv, 0x900, rfmod1);
		}
	}

	/* BW operating mode 20MHz */
	{
		u8 bw_opmode;
		ret = rtl8188eu_read_reg8(priv, REG_BWOPMODE, &bw_opmode);
		if (ret == 0) {
			bw_opmode |= BW_OPMODE_20MHZ;
			rtl8188eu_write_reg8(priv, REG_BWOPMODE, bw_opmode);
		}
	}

	/* USB Aggregation */
	rtl8188eu_write_reg8(priv, 0x280, 0x30);
	rtl8188eu_write_reg8(priv, 0x281, 0x04);

	ret = rtl8188eu_read_reg8(priv, REG_TRXDMA_CTRL, &val8);
	if (ret == 0) {
		val8 |= BIT(2);
		rtl8188eu_write_reg8(priv, REG_TRXDMA_CTRL, val8);
	}

	ret = rtl8188eu_read_reg8(priv, REG_USB_SPECIAL_OPTION, &val8);
	if (ret == 0) {
		val8 &= ~BIT(3);
		rtl8188eu_write_reg8(priv, REG_USB_SPECIAL_OPTION, val8);
	}

	/* Enable MAC TX/RX */
	{
		u16 cr_val;
		ret = rtl8188eu_read_reg16(priv, REG_CR, &cr_val);
		if (ret == 0) {
			cr_val |= CR_MACTXEN | CR_MACRXEN |
				  CR_HCI_TXDMA_EN | CR_TXDMA_EN;
			rtl8188eu_write_reg16(priv, REG_CR, cr_val);
		}
	}

	rtl8188eu_init_hw_drop_incorrect_bulkout(priv);
	rtl8188eu_init_tx_report(priv);
	rtl8188eu_write_reg16(priv, REG_MAX_AGGR_NUM, 0x0707);

	/* RF front-end */
	{
		u32 ee8_val;
		ret = rtl8188eu_read_reg32(priv, 0xEE8, &ee8_val);
		if (ret == 0) {
			ee8_val |= 0x10000000;
			rtl8188eu_write_reg32(priv, 0xEE8, ee8_val);
		}
	}
	rtl8188eu_write_reg32(priv, 0x87C, 0x00000000);

	/* Default TX power */
	rtl8188eu_write_reg32(priv, 0xE00, 0x2D2D2D2D);
	rtl8188eu_write_reg32(priv, 0xE04, 0x2D2D2D2D);
	rtl8188eu_write_reg32(priv, 0xE08, 0x2626262A);
	rtl8188eu_write_reg32(priv, 0xE10, 0x26262626);

	/* Set MSR to no-link initially */
	rtl8188eu_write_reg8(priv, REG_MSR, MSR_NOLINK);

	msleep(50);

	pr_info("%s: Init complete\n", DRIVER_NAME);

	/* Register with mac80211 */
	ret = ieee80211_register_hw(hw);
	if (ret) {
		pr_err("%s: Failed to register mac80211 hw: %d\n", DRIVER_NAME, ret);
		goto err_free_hw;
	}

	pr_info("%s: Registered with mac80211 (MAC %pM)\n", DRIVER_NAME, priv->mac_addr);

	return 0;

err_free_hw:
	if (priv->fw)
		release_firmware(priv->fw);
	usb_set_intfdata(intf, NULL);
	ieee80211_free_hw(hw);
	return ret;
}

static void rtl8188eu_disconnect(struct usb_interface *intf)
{
	struct ieee80211_hw *hw = usb_get_intfdata(intf);
	struct rtl8188eu_priv *priv;

	if (!hw)
		return;

	priv = hw->priv;
	priv->disconnecting = true;

	rtl8188eu_stop_rx(priv);

	ieee80211_unregister_hw(hw);

	if (priv->fw)
		release_firmware(priv->fw);

	usb_set_intfdata(intf, NULL);
	ieee80211_free_hw(hw);

	pr_info("%s: Disconnected\n", DRIVER_NAME);
}

/* USB Driver structure */
static struct usb_driver rtl8188eu_driver = {
	.name       = DRIVER_NAME,
	.probe      = rtl8188eu_probe,
	.disconnect = rtl8188eu_disconnect,
	.id_table   = rtl8188eu_usb_ids,
};

static int __init rtl8188eu_init(void)
{
	int ret;

	pr_info("%s: v%s loaded\n", DRIVER_NAME, DRIVER_VERSION);

	ret = usb_register(&rtl8188eu_driver);
	if (ret)
		pr_err("%s: USB register failed: %d\n", DRIVER_NAME, ret);
	return 0;
}

static void __exit rtl8188eu_exit(void)
{
	pr_info("%s: Unloading driver\n", DRIVER_NAME);
	usb_deregister(&rtl8188eu_driver);
	pr_info("%s: Driver unloaded\n", DRIVER_NAME);
}

module_init(rtl8188eu_init);
module_exit(rtl8188eu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthew");
MODULE_DESCRIPTION("RTL8188EUS USB WiFi Driver — mac80211");
MODULE_VERSION(DRIVER_VERSION);
MODULE_FIRMWARE(FIRMWARE_NAME);
