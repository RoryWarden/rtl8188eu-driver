/*
 * Minimal RTL8188EUS USB WiFi Driver
 * Phase 1: Hardware Initialization - COMPLETE!
 *   Step 1: USB Device Detection ✓
 *   Step 2: Load Firmware ✓
 *   Step 3: Register Configuration ✓
 *   Step 4: TX/RX Queue Setup ✓
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
#include <net/iw_handler.h>
#include <net/ieee80211_radiotap.h>
#include "rtl8188eu.h"
#include "rtl8188eu_phy.h"

#define DRIVER_NAME "rtl8188eu_minimal"
#define DRIVER_VERSION "0.37"
#define FIRMWARE_NAME "rtlwifi/rtl8188eufw.bin"

/* Power Sequence Commands */
#define PWR_CMD_WRITE			0x01	/* Write register */
#define PWR_CMD_POLLING			0x02	/* Poll register */
#define PWR_CMD_DELAY			0x03	/* Delay */
#define PWR_CMD_END			0x04	/* End of sequence */

/* Power sequence structure */
struct pwr_cmd {
	u16 offset;		/* Register offset */
	u8  cmd;		/* Command type */
	u8  mask;		/* Bit mask */
	u8  value;		/* Value to write/poll */
};

/* RTL8188E Register Addresses - see README.md for documentation */
#define REG_SYS_FUNC_EN			0x0002	/* System Function Enable */
#define FEN_ELDR				BIT(12)	/* EEPROM Loader clock enable */
#define REG_APS_FSMCO			0x0004	/* APS FSM Control */
#define REG_SYS_CLKR			0x0008	/* System Clock Register */
#define ANA8M					BIT(1)	/* 8MHz clock from analog */
#define LOADER_CLK_EN			BIT(5)	/* Loader clock enable */
#define REG_RSV_CTRL			0x001C	/* Reserved Control (for 8051 reset) */
#define REG_RF_CTRL				0x001F	/* RF Control */
#define REG_AFE_XTAL_CTRL		0x0024	/* AFE Crystal Control */
#define REG_AFE_PLL_CTRL		0x0028	/* AFE PLL Control */
#define REG_MCUFWDL				0x0080	/* MCU Firmware Download Control */
#define MCUFWDL_EN				BIT(0)	/* Enable firmware download */
#define MCUFWDL_RDY				BIT(1)	/* Firmware download ready */
#define WINTINI_RDY				BIT(6)	/* Firmware init ready */
#define RAM_DL_SEL				BIT(7)	/* RAM download select */
#define REG_HIMR_88E			0x00B0	/* Hardware Interrupt Mask */
#define REG_HISR_88E			0x00B4	/* Hardware Interrupt Status */
#define REG_CR					0x0100	/* Command Register */
#define REG_HMETFR				0x01CC	/* H2C (Host-to-Card) Meta Frame register */
#define REG_PBP					0x0104	/* Protocol Buffer Page (TX/RX page size) */
#define REG_TRXFF_BNDY			0x0114	/* TX/RX FIFO Boundary */
#define REG_RQPN				0x0200	/* Request Queue Page Number */
#define REG_TDECTRL				0x0208	/* TX DMA Enable Control */
#define REG_RQPN_NPQ			0x0214	/* Normal Priority Queue Page Number */
#define REG_BCNQ_BDNY			0x0424	/* Beacon Queue Boundary */
#define REG_MGQ_BDNY			0x0425	/* Management Queue Boundary */
#define REG_WMAC_LBK_BF_HD		0x045D	/* WMAC Loopback Buffer Head */

/* Monitor Mode Registers (Phase 4) */
#define REG_RCR					0x0608	/* Receive Configuration Register */
#define REG_MAR					0x0620	/* Multicast Address Register */
#define REG_WMAC_TRXPTCL_CTL	0x0668	/* TX/RX Protocol Control */
#define REG_RXFLTMAP2			0x06A4	/* RX Data Frame Filter Map */

/* USB Aggregation Registers */
#define REG_TRXDMA_CTRL			0x010C	/* TX/RX DMA Control */
#define REG_RXPKT_NUM			0x0284	/* RX Packet Number */
#define RW_RELEASE_EN			BIT(18)	/* RX DMA release one packet then stop */
#define RXDMA_IDLE			BIT(17)	/* RX DMA idle */
#define REG_RXDMA_AGG_PG_TH		0x0280	/* RX DMA Aggregation Page Threshold */
#define REG_USB_SPECIAL_OPTION	0xFE55	/* USB Special Option */
#define REG_USB_AGG_TO			0xFE5C	/* USB Aggregation Timeout */
#define REG_USB_AGG_TH			0xFE5D	/* USB Aggregation Threshold */

/* TX Report Registers */
#define REG_TX_RPT_CTRL			0x04EC	/* TX Report Control */
#define REG_TX_RPT_TIME			0x04F0	/* TX Report Timer */

/* Hardware Drop Incorrect BulkOut */
#define REG_TXDMA_OFFSET_CHK		0x020C	/* TX DMA Offset Check */

/* Aggregation */
#define REG_MAX_AGGR_NUM		0x04CA	/* Max Aggregation Number */

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

/* LLT (Linked List Table) Register */
#define REG_LLT_INIT			0x01E0	/* LLT Init Register */
#define LAST_ENTRY_OF_TX_PKT_BUF	175	/* Last LLT entry (page 175) */

/* MAC Configuration Registers */
#define REG_FWHW_TXQ_CTRL		0x0420	/* FW HW TX Queue Control */
#define REG_HWSEQ_CTRL			0x0423	/* HW Sequence Control */
#define REG_SPEC_SIFS			0x0428	/* Special SIFS */
#define REG_RL				0x042A	/* Retry Limit */
#define REG_DARFRC			0x0430	/* Data Auto Rate Fallback Retry Count */
#define REG_RARFRC			0x0438	/* Response Auto Rate Fallback Retry Count */
#define REG_RRSR			0x0440	/* Response Rate Set Register */
#define REG_AMPDU_MAX_TIME		0x0456	/* AMPDU Max Time */
#define REG_EDCA_VO_PARAM		0x0500	/* EDCA Voice Parameter */
#define REG_EDCA_VI_PARAM		0x0504	/* EDCA Video Parameter */
#define REG_EDCA_BE_PARAM		0x0508	/* EDCA Best Effort Parameter */
#define REG_EDCA_BK_PARAM		0x050C	/* EDCA Background Parameter */
#define REG_PIFS			0x0512	/* PIFS */
#define REG_SIFS_CTX			0x0514	/* SIFS for CTX */
#define REG_SIFS_TRX			0x0516	/* SIFS for TRX */
#define REG_TBTT_PROHIBIT		0x0540	/* TBTT Prohibit */
#define REG_BCN_CTRL			0x0550	/* Beacon Control */
#define REG_RX_DRVINFO_SZ		0x060F	/* RX Driver Info Size */
#define REG_RESP_SIFS_CCK		0x063C	/* Response SIFS for CCK */
#define REG_RESP_SIFS_OFDM		0x063E	/* Response SIFS for OFDM */
#define REG_ACKTO			0x0640	/* ACK Timeout */

/* RF Channel Tuning Registers (Phase 4) */
/* Note: rFPGA0_XA_RFInterfaceOE, RF_CHNLBW, bRFRegOffsetMask defined in rtl8188eu_phy.h */

/* REG_SYS_FUNC_EN bits */
#define FEN_BBRSTB				BIT(0)
#define FEN_BB_GLB_RSTn			BIT(1)
#define FEN_USBA				BIT(2)
#define FEN_USBD				BIT(4)
#define FEN_EN_25_1				BIT(13)  /* 25.1MHz clock enable - CRITICAL for BB/RF! */

/* REG_RF_CTRL bits */
#define RF_EN					BIT(0)
#define RF_RSTB					BIT(1)
#define RF_SDMRSTB				BIT(2)

/* REG_CR bits */
#define CR_HCI_TXDMA_EN			BIT(0)
#define CR_HCI_RXDMA_EN			BIT(1)
#define CR_TXDMA_EN				BIT(2)
#define CR_RXDMA_EN				BIT(3)
#define CR_PROTOCOL_EN			BIT(4)
#define CR_SCHEDULE_EN			BIT(5)
#define CR_MACTXEN				BIT(6)  /* MAC TX enable */
#define CR_MACRXEN				BIT(7)  /* MAC RX enable */
#define CR_ENSEC				BIT(9)
#define CR_CALTMR_EN			BIT(10)

/* BB registers for PHY control */
#define rFPGA0_RFMOD			0x800	/* RF mode register */
#define bCCKEn				0x1000000  /* Bit 24: Enable CCK */
#define bOFDMEn				0x2000000  /* Bit 25: Enable OFDM */

/* REG_RCR bits (Receive Configuration Register) */
#define RCR_AAP					BIT(0)	/* Accept All unicast Packet */
#define RCR_APM					BIT(1)	/* Accept Physical Match packet */
#define RCR_AM					BIT(2)	/* Accept Multicast packet */
#define RCR_AB					BIT(3)	/* Accept Broadcast packet */
#define RCR_ADD3				BIT(4)	/* Accept address 3 match packet */
#define RCR_APWRMGT				BIT(5)	/* Accept power management packet */
#define RCR_CBSSID_DATA			BIT(6)	/* Accept BSSID match (Data) */
#define RCR_CBSSID_BCN			BIT(7)	/* Accept BSSID match (Beacon) */
#define RCR_ACRC32				BIT(8)	/* Accept CRC32 error packet */
#define RCR_AICV				BIT(9)	/* Accept ICV error packet */
#define RCR_ADF					BIT(11)	/* Accept Data type Frame */
#define RCR_ACF					BIT(12)	/* Accept Control Frame */
#define RCR_AMF					BIT(13)	/* Accept Management Frame */
#define RCR_HTC_LOC_CTRL		BIT(14)	/* HTC Location Control */
#define RCR_APP_PHYST_RXFF		BIT(28)	/* Append PHY status to RX */
#define RCR_APP_ICV				BIT(29)	/* Append ICV */
#define RCR_APP_MIC				BIT(30)	/* Append MIC */
#define RCR_APPFCS				BIT(31)	/* Append FCS to frame */

/* Firmware Download */
#define FW_START_ADDRESS		0x1000	/* Firmware RAM start address */
#define MAX_FW_BLOCK_SIZE		196		/* USB transfer block size */
#define MAX_FW_PAGE_SIZE		4096	/* Firmware page size */

/* TX/RX Queue Configuration (Step 4) */
#define TX_SELE_HQ				BIT(0)	/* High Priority Queue */
#define TX_SELE_LQ				BIT(1)	/* Low Priority Queue */
#define TX_SELE_NQ				BIT(2)	/* Normal Priority Queue */

/* Page counts for non-WMM mode (2-EP HQ|NQ config) */
#define NORMAL_PAGE_NUM_HPQ		0x0C
#define NORMAL_PAGE_NUM_LPQ		0x02
#define NORMAL_PAGE_NUM_NPQ		0x02

/* Total page numbers */
#define TOTAL_PAGE_NUMBER		0xAF	/* 0xB0 - 1 = 175 pages */
#define BCNQ_PAGE_NUM			0x08	/* 8 pages for beacon queue */
#define TX_TOTAL_PAGE_NUMBER	(TOTAL_PAGE_NUMBER - BCNQ_PAGE_NUM)	/* 167 pages */
#define TX_PAGE_BOUNDARY		(TX_TOTAL_PAGE_NUMBER + 1)			/* 168 */

/* Page buffer size */
#define PBP_128					0x1		/* 128 bytes per page */

/* Queue page number bit manipulation macros */
#define _HPQ(x)					((x) & 0xFF)
#define _LPQ(x)					(((x) & 0xFF) << 8)
#define _PUBQ(x)				(((x) & 0xFF) << 16)
#define _NPQ(x)					((x) & 0xFF)
#define LD_RQPN					BIT(31)	/* Load RQPN */

/* Transfer page size macros */
#define _PSRX(x)				(x)
#define _PSTX(x)				((x) << 4)

/* RX/TX URB Configuration (Phase 2, Step 5) */
/* Note: RX_URB_COUNT defined in rtl8188eu.h */
#define RX_BUFFER_SIZE			32768	/* 32KB per RX buffer */

/* RTL8188EU RX Descriptor (24 bytes) - prepended to each received packet */
struct rtl8188eu_rx_desc {
	__le32 rxdw0;	/* RX DW0 - length, CRC, ICV, etc */
	__le32 rxdw1;	/* RX DW1 - packet type, sequence */
	__le32 rxdw2;	/* RX DW2 - FRAG, more data, etc */
	__le32 rxdw3;	/* RX DW3 - reserved */
	__le32 rxdw4;	/* RX DW4 - timestamp low */
	__le32 rxdw5;	/* RX DW5 - timestamp high */
} __packed;

/* RX DW0 bit definitions */
#define RX_DW0_PKT_LEN_MASK		0x00003fff	/* Bits 0-13: packet length */
#define RX_DW0_CRC32			BIT(14)		/* CRC32 error */
#define RX_DW0_ICV_ERR			BIT(15)		/* ICV error */
#define RX_DW0_DRVINFO_SZ_SHIFT	16		/* Bits 16-19: driver info size (×8 bytes) */
#define RX_DW0_DRVINFO_SZ_MASK		0xf
#define RX_DW0_SHIFT_SHIFT		24		/* Bits 24-25: data shift (bytes) */
#define RX_DW0_SHIFT_MASK		0x3

/* RX DW2 bit definitions */
#define RX_DW2_PKT_CNT_SHIFT		16		/* Bits 16-23: aggregated packet count */
#define RX_DW2_PKT_CNT_MASK		0xff

/* RX descriptor size */
#define RXDESC_SIZE			sizeof(struct rtl8188eu_rx_desc)

/* Round up to 128-byte boundary for DMA aggregation */
#define RX_RND128(x)			(((x) + 127) & ~127)

/* Radiotap header for monitor mode RX packets (with MCS field for HT rates) */
struct rtl8188eu_radiotap_hdr {
	struct ieee80211_radiotap_header hdr;
	u8 flags;
	u8 rate;
	__le16 chan_freq;
	__le16 chan_flags;
	s8 signal;
	u8 antenna;
	u8 mcs_known;
	u8 mcs_flags;
	u8 mcs_index;
} __packed;

#define RTL8188EU_RADIOTAP_PRESENT ( \
	(1 << IEEE80211_RADIOTAP_FLAGS) | \
	(1 << IEEE80211_RADIOTAP_RATE) | \
	(1 << IEEE80211_RADIOTAP_CHANNEL) | \
	(1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL) | \
	(1 << IEEE80211_RADIOTAP_ANTENNA) | \
	(1 << IEEE80211_RADIOTAP_MCS))

/* Radiotap channel flags */
#define RTL_CHAN_2GHZ		0x0080
#define RTL_CHAN_OFDM		0x0040
#define RTL_CHAN_CCK		0x0020

/* RX DW0 bit definitions for PHY status */
#define RX_DW0_PHYST		BIT(26)		/* PHY status present */

/* Hardware rate code to radiotap rate (500kbps units) lookup table */
static const u8 rtl8188eu_hw_rate_to_radiotap[] = {
	2, 4, 11, 22,				/* CCK: 1/2/5.5/11 Mbps (0x00-0x03) */
	12, 18, 24, 36,				/* OFDM: 6/9/12/18 Mbps (0x04-0x07) */
	48, 72, 96, 108,			/* OFDM: 24/36/48/54 Mbps (0x08-0x0b) */
};

/* CCK LNA gain table for signal strength calculation */
static const s8 cck_lna_gain[] = { 17, -1, -13, -29, -32, -35, -38, -41 };

static inline u16 rtl8188eu_chan_to_freq(u8 channel)
{
	if (channel == 14)
		return 2484;
	if (channel >= 1 && channel <= 13)
		return 2407 + 5 * channel;
	return 2412;
}

/* RTL8188EU TX Descriptor (32 bytes) - prepended to each transmitted packet */
struct rtl8188eu_tx_desc {
	__le32 txdw0;	/* TX DW0 - packet size, offset */
	__le32 txdw1;	/* TX DW1 - queue, rate, etc */
	__le32 txdw2;	/* TX DW2 - aggregation, break, etc */
	__le32 txdw3;	/* TX DW3 - sequence, more data */
	__le32 txdw4;	/* TX DW4 - rate ID, data rate */
	__le32 txdw5;	/* TX DW5 - data/RTS rate, short/long GI */
	__le32 txdw6;	/* TX DW6 - checksum offload */
	__le32 txdw7;	/* TX DW7 - USB aggregation */
} __packed;

/* TX DW0 bit definitions */
#define TX_DW0_PKT_SIZE_SHIFT		0		/* Bits 0-15: packet size */
#define TX_DW0_OFFSET_SHIFT		16		/* Bits 16-23: descriptor offset (always 32) */
#define TX_DW0_OWN			BIT(31)		/* Owner bit */

/* TX DW1 bit definitions */
#define TX_DW1_MACID_SHIFT		0		/* Bits 0-4: MAC ID */
#define TX_DW1_QUEUE_SEL_SHIFT		8		/* Bits 8-12: Queue select */
#define TX_DW1_QUEUE_SEL_VO		0x06		/* Voice queue */
#define TX_DW1_QUEUE_SEL_VI		0x05		/* Video queue */
#define TX_DW1_QUEUE_SEL_BE		0x00		/* Best effort queue */
#define TX_DW1_QUEUE_SEL_BK		0x02		/* Background queue */
#define TX_DW1_QUEUE_SEL_MGNT		0x12		/* Management queue */

/* RX URB structure and driver private data structure are defined in rtl8188eu.h */

/* USB Device ID Table */
static const struct usb_device_id rtl8188eu_usb_ids[] = {
	{ USB_DEVICE(0x2357, 0x010c) },  /* TP-Link TL-WN722N v2/v3 */
	{ }
};
MODULE_DEVICE_TABLE(usb, rtl8188eu_usb_ids);

/* Prototypes for PHY layer wrapper functions (exported to rtl8188eu_phy.c) */
u32 rtl8188eu_read_reg32_direct(void *priv_ptr, u16 addr);
int rtl8188eu_write_reg32_direct(void *priv_ptr, u16 addr, u32 val);

/*
 * ============================================================================
 * Register Read/Write Functions
 * ============================================================================
 */

/*
 * Read 8-bit register via USB control transfer
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
			      0x05,		/* VENDOR_REQ */
			      USB_TYPE_VENDOR | USB_DIR_IN,
			      addr,		/* wValue: register address */
			      0,		/* wIndex */
			      buf, 1,	/* data */
			      500);		/* timeout */

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

/*
 * Read 16-bit register via USB control transfer
 */
static int rtl8188eu_read_reg16(struct rtl8188eu_priv *priv, u16 addr, u16 *val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(2, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = usb_control_msg(priv->udev,
			      usb_rcvctrlpipe(priv->udev, 0),
			      0x05,		/* VENDOR_REQ */
			      USB_TYPE_VENDOR | USB_DIR_IN,
			      addr,		/* wValue: register address */
			      0,		/* wIndex */
			      buf, 2,	/* data */
			      500);		/* timeout */

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

/*
 * Read 32-bit register via USB control transfer
 */
static int rtl8188eu_read_reg32(struct rtl8188eu_priv *priv, u16 addr, u32 *val)
{
	int ret;
	u8 *buf;

	buf = kmalloc(4, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = usb_control_msg(priv->udev,
			      usb_rcvctrlpipe(priv->udev, 0),
			      0x05,		/* VENDOR_REQ */
			      USB_TYPE_VENDOR | USB_DIR_IN,
			      addr,		/* wValue: register address */
			      0,		/* wIndex */
			      buf, 4,	/* data */
			      500);		/* timeout */

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

/*
 * Write 8-bit register via USB control transfer
 */
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
			      0x05,		/* VENDOR_REQ */
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr,		/* wValue: register address */
			      0,		/* wIndex */
			      buf, 1,	/* data */
			      500);		/* timeout */

	kfree(buf);

	if (ret < 0) {
		pr_err("%s: Failed to write register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(rtl8188eu_write_reg8);

/*
 * Write 16-bit register via USB control transfer
 */
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
			      0x05,		/* VENDOR_REQ */
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr,		/* wValue: register address */
			      0,		/* wIndex */
			      buf, 2,	/* data */
			      500);		/* timeout */

	kfree(buf);

	if (ret < 0) {
		pr_err("%s: Failed to write register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
		return ret;
	}

	return 0;
}

/*
 * Write 32-bit register via USB control transfer
 */
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
			      0x05,		/* VENDOR_REQ */
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr,		/* wValue: register address */
			      0,		/* wIndex */
			      buf, 4,	/* data */
			      500);		/* timeout */

	kfree(buf);

	if (ret < 0) {
		pr_err("%s: Failed to write register 0x%04x: %d\n",
		       DRIVER_NAME, addr, ret);
		return ret;
	}

	return 0;
}

/*
 * Wrapper functions for PHY layer (exported to rtl8188eu_phy.c)
 * These convert the error-code-returning static functions to direct value returns
 */
u32 rtl8188eu_read_reg32_direct(void *priv_ptr, u16 addr)
{
	struct rtl8188eu_priv *priv = (struct rtl8188eu_priv *)priv_ptr;
	u32 val = 0;
	int ret;

	ret = rtl8188eu_read_reg32(priv, addr, &val);
	if (ret < 0) {
		pr_warn("Failed to read register 0x%04x, returning 0\n", addr);
		return 0;
	}

	return val;
}
EXPORT_SYMBOL(rtl8188eu_read_reg32_direct);

int rtl8188eu_write_reg32_direct(void *priv_ptr, u16 addr, u32 val)
{
	struct rtl8188eu_priv *priv = (struct rtl8188eu_priv *)priv_ptr;
	return rtl8188eu_write_reg32(priv, addr, val);
}
EXPORT_SYMBOL(rtl8188eu_write_reg32_direct);

/*
 * Write block of data to chip via USB control transfer
 */
static int rtl8188eu_write_block(struct rtl8188eu_priv *priv,
				  u16 addr, const u8 *data, u16 len)
{
	int ret;
	u8 *buf;

	/* USB control messages need DMA-capable buffer */
	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = usb_control_msg(priv->udev,
			      usb_sndctrlpipe(priv->udev, 0),
			      0x05,		/* VENDOR_REQ */
			      USB_TYPE_VENDOR | USB_DIR_OUT,
			      addr,		/* wValue: register address */
			      0,		/* wIndex */
			      buf, len,	/* data */
			      500);		/* timeout */

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

/* RTL8188E Power-On Sequence: Transition from CARDEMU to ACT mode */
static const struct pwr_cmd rtl8188e_power_on_seq[] = {
	/* Wait for power ready (0x06[1] = 1) */
	{0x0006, PWR_CMD_POLLING, BIT(1), BIT(1)},

	/* Reset BB: 0x02[1:0] = 0 */
	{0x0002, PWR_CMD_WRITE, BIT(0) | BIT(1), 0},

	/* Schmitt trigger: 0x26[7] = 1 (0x26 is actually 0x24[23]) */
	{0x0026, PWR_CMD_WRITE, BIT(7), BIT(7)},

	/* Disable HWPDN: 0x05[7] = 0 (0x05 is 0x04[15]) */
	{0x0005, PWR_CMD_WRITE, BIT(7), 0},

	/* Disable WL suspend: 0x05[4:3] = 0 (0x05 is 0x04[12:11]) */
	{0x0005, PWR_CMD_WRITE, BIT(4) | BIT(3), 0},

	/* Set 0x05[0] = 1 (0x05 is 0x04[8]) */
	{0x0005, PWR_CMD_WRITE, BIT(0), BIT(0)},

	/* Poll 0x05[0] = 0 (wait till 0x04[8] = 0) */
	{0x0005, PWR_CMD_POLLING, BIT(0), 0},

	/* LDO normal mode: 0x23[4] = 0 */
	{0x0023, PWR_CMD_WRITE, BIT(4), 0},

	/* End of sequence */
	{0xFFFF, PWR_CMD_END, 0, 0}
};

/*
 * Execute power sequence to transition chip states
 */
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
			if (ret < 0) {
				pr_err("%s: Failed to read reg 0x%04x\n", DRIVER_NAME, reg);
				return ret;
			}

			val8 &= ~seq[i].mask;
			val8 |= (seq[i].value & seq[i].mask);

			ret = rtl8188eu_write_reg8(priv, reg, val8);
			if (ret < 0) {
				pr_err("%s: Failed to write reg 0x%04x\n", DRIVER_NAME, reg);
				return ret;
			}

			pr_debug("%s: PWR SEQ: Wrote 0x%02x to reg 0x%04x\n",
				DRIVER_NAME, val8, reg);
			break;

		case PWR_CMD_POLLING:
			pr_debug("%s: PWR SEQ: Polling reg 0x%04x for 0x%02x\n",
				DRIVER_NAME, reg, seq[i].value);

			for (retry = 0; retry < 5000; retry++) {
				ret = rtl8188eu_read_reg8(priv, reg, &val8);
				if (ret < 0) {
					pr_err("%s: Failed to poll reg 0x%04x\n", DRIVER_NAME, reg);
					return ret;
				}

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

/*
 * Power on the chip - CRITICAL: must be done before any other register access!
 */
static int rtl8188eu_power_on(struct rtl8188eu_priv *priv)
{
	u16 value16;
	int ret;

	pr_debug("%s: Powering on chip...\n", DRIVER_NAME);

	if (priv->chip_powered_on)
		return 0;

	/* Execute power-on sequence to transition from CARDEMU to ACT mode */
	ret = rtl8188eu_execute_power_sequence(priv, rtl8188e_power_on_seq);
	if (ret < 0) {
		pr_err("%s: Power sequence failed: %d\n", DRIVER_NAME, ret);
		return ret;
	}

	/* Reset REG_CR to 0 */
	ret = rtl8188eu_write_reg16(priv, REG_CR, 0x00);
	if (ret < 0)
		return ret;

	msleep(10);

	ret = rtl8188eu_read_reg16(priv, REG_CR, &value16);
	if (ret < 0)
		return ret;

	/* Enable MAC DMA/WMAC/SCHEDULE/SEC blocks including RX DMA at power-on */
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

/*
 * Enable MCU clocks for firmware operation
 */
static int rtl8188eu_enable_mcu_clocks(struct rtl8188eu_priv *priv)
{
	u16 val16;
	int ret;

	/* Enable FEN_ELDR (EEPROM Loader clock) if not already enabled */
	ret = rtl8188eu_read_reg16(priv, REG_SYS_FUNC_EN, &val16);
	if (ret < 0)
		return ret;

	if (!(val16 & FEN_ELDR)) {
		val16 |= FEN_ELDR;
		ret = rtl8188eu_write_reg16(priv, REG_SYS_FUNC_EN, val16);
		if (ret < 0)
			return ret;
	}

	/* Enable LOADER_CLK_EN and ANA8M clocks */
	ret = rtl8188eu_read_reg16(priv, REG_SYS_CLKR, &val16);
	if (ret < 0)
		return ret;

	if (!(val16 & LOADER_CLK_EN) || !(val16 & ANA8M)) {
		val16 |= (LOADER_CLK_EN | ANA8M);
		ret = rtl8188eu_write_reg16(priv, REG_SYS_CLKR, val16);
		if (ret < 0)
			return ret;
	}

	/* Set AFE_XTAL_CTRL+1 to 0x80 for crystal control */
	ret = rtl8188eu_write_reg8(priv, REG_AFE_XTAL_CTRL + 1, 0x80);
	if (ret < 0)
		return ret;

	pr_debug("%s: MCU clocks enabled\n", DRIVER_NAME);
	return 0;
}

/*
 * Clear and configure interrupts
 */
static int rtl8188eu_init_interrupts(struct rtl8188eu_priv *priv)
{
	int ret;
	u32 himr;

	/* Clear all pending interrupts */
	ret = rtl8188eu_write_reg32(priv, REG_HISR_88E, 0xFFFFFFFF);
	if (ret < 0)
		return ret;

	/* Enable RX interrupts: ROK, RDU, RXFOVW, RXERR */
	himr = BIT(0) | BIT(1) | BIT(8) | BIT(10);

	ret = rtl8188eu_write_reg32(priv, REG_HIMR_88E, himr);
	if (ret < 0)
		return ret;

	pr_debug("%s: RX interrupts enabled (HIMR=0x%08x)\n", DRIVER_NAME, himr);

	return 0;
}

/*
 * ============================================================================
 * TX/RX Queue Setup (Step 4)
 * ============================================================================
 */

/*
 * Set TX and RX transfer page size to 128 bytes
 */
static int rtl8188eu_init_transfer_page_size(struct rtl8188eu_priv *priv)
{
	u8 value;
	int ret;

	value = _PSRX(PBP_128) | _PSTX(PBP_128);
	ret = rtl8188eu_write_reg8(priv, REG_PBP, value);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * Set RX page boundary
 */
static int rtl8188eu_init_page_boundary(struct rtl8188eu_priv *priv)
{
	u16 rxff_bndy;
	int ret;

	rxff_bndy = TOTAL_PAGE_NUMBER;
	ret = rtl8188eu_write_reg16(priv, REG_TRXFF_BNDY, rxff_bndy);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg16(priv, REG_TRXFF_BNDY + 2, 0x25FF);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * Set TX buffer boundary registers
 */
static int rtl8188eu_init_tx_buffer_boundary(struct rtl8188eu_priv *priv)
{
	u8 txpktbuf_bndy;
	int ret;

	txpktbuf_bndy = TX_PAGE_BOUNDARY;

	ret = rtl8188eu_write_reg8(priv, REG_BCNQ_BDNY, txpktbuf_bndy);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_MGQ_BDNY, txpktbuf_bndy);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_WMAC_LBK_BF_HD, txpktbuf_bndy);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_TRXFF_BNDY, txpktbuf_bndy);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg8(priv, REG_TDECTRL + 1, txpktbuf_bndy);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * Configure queue reserved pages (how many pages each TX queue gets)
 */
static int rtl8188eu_init_queue_reserved_page(struct rtl8188eu_priv *priv)
{
	u32 numHQ, numLQ, numNQ, numPubQ;
	u32 value32;
	u8 value8;
	int ret;

	pr_debug("%s: EP config: %d EPs, queue_sel=0x%02x\n",
		DRIVER_NAME, priv->out_ep_number, priv->out_ep_queue_sel);

	/* Set page numbers for each queue (non-WMM mode) */
	if (priv->out_ep_queue_sel & TX_SELE_HQ)
		numHQ = NORMAL_PAGE_NUM_HPQ;
	else
		numHQ = 0;

	if (priv->out_ep_queue_sel & TX_SELE_LQ)
		numLQ = NORMAL_PAGE_NUM_LPQ;
	else
		numLQ = 0;

	if (priv->out_ep_queue_sel & TX_SELE_NQ)
		numNQ = NORMAL_PAGE_NUM_NPQ;
	else
		numNQ = 0;

	/* Write NPQ (Normal Priority Queue) page count */
	value8 = _NPQ(numNQ);
	ret = rtl8188eu_write_reg8(priv, REG_RQPN_NPQ, value8);
	if (ret < 0) {
		pr_err("%s: Failed to set REG_RQPN_NPQ\n", DRIVER_NAME);
		return ret;
	}

	/* Calculate public queue pages (remaining pages) */
	numPubQ = TX_TOTAL_PAGE_NUMBER - numHQ - numLQ - numNQ;

	/* Write RQPN register with all queue page counts */
	value32 = _HPQ(numHQ) | _LPQ(numLQ) | _PUBQ(numPubQ) | LD_RQPN;
	ret = rtl8188eu_write_reg32(priv, REG_RQPN, value32);
	if (ret < 0) {
		pr_err("%s: Failed to set REG_RQPN\n", DRIVER_NAME);
		return ret;
	}

	pr_debug("%s: Queue pages: HPQ=%d LPQ=%d NPQ=%d PUBQ=%d\n",
		DRIVER_NAME, numHQ, numLQ, numNQ, numPubQ);

	return 0;
}

/*
 * Write a single LLT (Linked List Table) entry
 * Format: bits[31:30]=op(1=write), bits[15:8]=address, bits[7:0]=data
 */
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

	pr_err("%s: LLT write timeout at address %d\n", DRIVER_NAME, address);
	return -ETIMEDOUT;
}

/*
 * Initialize the LLT table for TX/RX buffer management
 * Pages 0 to TX_PAGE_BOUNDARY-1: TX chain (each points to next, last = 0xFF)
 * Pages TX_PAGE_BOUNDARY to LAST_ENTRY: Beacon ring buffer (circular)
 */
static int rtl8188eu_init_llt_table(struct rtl8188eu_priv *priv)
{
	int ret;
	u32 i;

	pr_debug("%s: Initializing LLT table...\n", DRIVER_NAME);

	/* TX pages: each entry points to the next */
	for (i = 0; i < TX_PAGE_BOUNDARY - 1; i++) {
		ret = rtl8188eu_llt_write(priv, i, i + 1);
		if (ret)
			return ret;
	}

	/* End of TX chain */
	ret = rtl8188eu_llt_write(priv, TX_PAGE_BOUNDARY - 1, 0xFF);
	if (ret)
		return ret;

	/* Beacon ring buffer pages: each points to next */
	for (i = TX_PAGE_BOUNDARY; i < LAST_ENTRY_OF_TX_PKT_BUF; i++) {
		ret = rtl8188eu_llt_write(priv, i, i + 1);
		if (ret)
			return ret;
	}

	/* Last entry loops back to start of beacon ring */
	ret = rtl8188eu_llt_write(priv, LAST_ENTRY_OF_TX_PKT_BUF, TX_PAGE_BOUNDARY);
	if (ret)
		return ret;

	pr_debug("%s: LLT table initialized\n", DRIVER_NAME);

	return 0;
}

/*
 * Initialize all TX/RX queues (Step 4 main function)
 */
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
	ret = rtl8188eu_init_queue_reserved_page(priv);
	if (ret)
		return ret;

	pr_debug("%s: TX/RX queues configured\n", DRIVER_NAME);

	return 0;
}

/*
 * Initialize critical MAC registers (EDCA, SIFS, retry, rates, beacon)
 * These configure the MAC protocol layer for proper frame exchange
 */
static int rtl8188eu_init_mac_regs(struct rtl8188eu_priv *priv)
{
	pr_debug("%s: Initializing MAC registers...\n", DRIVER_NAME);

	/* SIFS timing (CCK=0x0A, OFDM=0x10) */
	rtl8188eu_write_reg16(priv, REG_SPEC_SIFS, 0x100a);
	rtl8188eu_write_reg16(priv, 0x063A, 0x100a);
	rtl8188eu_write_reg16(priv, REG_SIFS_CTX, 0x100a);
	rtl8188eu_write_reg16(priv, REG_SIFS_TRX, 0x100a);

	/* Response SIFS */
	rtl8188eu_write_reg8(priv, REG_RESP_SIFS_CCK, 0x08);
	rtl8188eu_write_reg8(priv, REG_RESP_SIFS_OFDM, 0x0E);

	/* Retry limits (short=0x30, long=0x30) */
	rtl8188eu_write_reg16(priv, REG_RL, 0x3030);

	/* EDCA parameters */
	rtl8188eu_write_reg32(priv, REG_EDCA_VO_PARAM, 0x002FA226);
	rtl8188eu_write_reg32(priv, REG_EDCA_VI_PARAM, 0x005EA324);
	rtl8188eu_write_reg32(priv, REG_EDCA_BE_PARAM, 0x005EA42B);
	rtl8188eu_write_reg32(priv, REG_EDCA_BK_PARAM, 0x0000A44F);

	/* RRSR - Response Rate Set (all CCK + OFDM basic rates) */
	rtl8188eu_write_reg32(priv, REG_RRSR, 0x0000FFFF);

	/* ACK timeout */
	rtl8188eu_write_reg8(priv, REG_ACKTO, 0x40);

	/* Beacon control */
	rtl8188eu_write_reg8(priv, REG_BCN_CTRL, 0x10);

	/* PIFS */
	rtl8188eu_write_reg8(priv, REG_PIFS, 0x1C);

	/* Hardware sequence control - enable for all TIDs */
	rtl8188eu_write_reg8(priv, REG_HWSEQ_CTRL, 0xFF);

	/* WMAC TX/RX Protocol Control */
	rtl8188eu_write_reg8(priv, 0x066E, 0x05);

	/* AMPDU max time */
	rtl8188eu_write_reg8(priv, REG_AMPDU_MAX_TIME, 0x70);

	pr_debug("%s: MAC registers initialized\n", DRIVER_NAME);

	return 0;
}

/*
 * ============================================================================
 * Firmware Loading (Step 2)
 * ============================================================================
 */

/*
 * Download firmware to chip memory
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

	/* Check if firmware has a header (starts with 0xE1 0x88) */
	if (fw_size >= 32 && fw_data[0] == 0xE1 && fw_data[1] == 0x88) {
		header_size = 32;
		pr_debug("%s: Firmware has 32-byte header, skipping it\n", DRIVER_NAME);
		fw_data += header_size;
		fw_size -= header_size;
	}

	pr_debug("%s: Downloading firmware (%u bytes)...\n", DRIVER_NAME, fw_size);

	page_nums = fw_size / MAX_FW_PAGE_SIZE;
	remain_size = fw_size % MAX_FW_PAGE_SIZE;

	/* Write each page */
	for (page = 0; page < page_nums; page++) {
		u32 block_cnt, block_remain;
		u32 block;

		offset = page * MAX_FW_PAGE_SIZE;

		/* Set page number in REG_MCUFWDL+2 register - preserve upper bits */
		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL + 2, &val);
		if (ret < 0) {
			pr_err("%s: Failed to read page register\n", DRIVER_NAME);
			return ret;
		}
		val = (val & 0xF8) | (page & 0x07);  /* Preserve bits 7:3, set bits 2:0 */
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 2, val);
		if (ret < 0) {
			pr_err("%s: Failed to set page number %u\n", DRIVER_NAME, page);
			return ret;
		}

		pr_debug("%s: Writing FW page %u/%u\n",
			DRIVER_NAME, page + 1, page_nums + (remain_size ? 1 : 0));

		/* Write page data in blocks */
		block_cnt = MAX_FW_PAGE_SIZE / MAX_FW_BLOCK_SIZE;
		block_remain = MAX_FW_PAGE_SIZE % MAX_FW_BLOCK_SIZE;

		for (block = 0; block < block_cnt; block++) {
			u32 block_offset = offset + (block * MAX_FW_BLOCK_SIZE);
			u16 fw_addr = FW_START_ADDRESS + (block * MAX_FW_BLOCK_SIZE);

			ret = rtl8188eu_write_block(priv, fw_addr,
						     fw_data + block_offset,
						     MAX_FW_BLOCK_SIZE);
			if (ret < 0) {
				pr_err("%s: Failed to write block %u of page %u\n",
				       DRIVER_NAME, block, page);
				return ret;
			}
		}

		if (block_remain > 0) {
			u32 block_offset = offset + (block_cnt * MAX_FW_BLOCK_SIZE);
			u16 fw_addr = FW_START_ADDRESS + (block_cnt * MAX_FW_BLOCK_SIZE);

			ret = rtl8188eu_write_block(priv, fw_addr,
						     fw_data + block_offset,
						     block_remain);
			if (ret < 0) {
				pr_err("%s: Failed to write remainder of page %u\n",
				       DRIVER_NAME, page);
				return ret;
			}
		}
	}

	/* Write remaining page if any */
	if (remain_size > 0) {
		u32 block_cnt, block_remain;
		u32 block;

		offset = page_nums * MAX_FW_PAGE_SIZE;
		page = page_nums;

		/* Set page number - preserve upper bits */
		ret = rtl8188eu_read_reg8(priv, REG_MCUFWDL + 2, &val);
		if (ret < 0) {
			pr_err("%s: Failed to read page register\n", DRIVER_NAME);
			return ret;
		}
		val = (val & 0xF8) | (page & 0x07);  /* Preserve bits 7:3, set bits 2:0 */
		ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 2, val);
		if (ret < 0) {
			pr_err("%s: Failed to set final page number %u\n", DRIVER_NAME, page);
			return ret;
		}

		pr_debug("%s: Writing final FW page %u (%u bytes)\n",
			DRIVER_NAME, page + 1, remain_size);

		block_cnt = remain_size / MAX_FW_BLOCK_SIZE;
		block_remain = remain_size % MAX_FW_BLOCK_SIZE;

		for (block = 0; block < block_cnt; block++) {
			u32 block_offset = offset + (block * MAX_FW_BLOCK_SIZE);
			u16 fw_addr = FW_START_ADDRESS + (block * MAX_FW_BLOCK_SIZE);

			ret = rtl8188eu_write_block(priv, fw_addr,
						     fw_data + block_offset,
						     MAX_FW_BLOCK_SIZE);
			if (ret < 0) {
				pr_err("%s: Failed to write block %u of final page\n",
				       DRIVER_NAME, block);
				return ret;
			}
		}

		if (block_remain > 0) {
			u32 block_offset = offset + (block_cnt * MAX_FW_BLOCK_SIZE);
			u16 fw_addr = FW_START_ADDRESS + (block_cnt * MAX_FW_BLOCK_SIZE);

			ret = rtl8188eu_write_block(priv, fw_addr,
						     fw_data + block_offset,
						     block_remain);
			if (ret < 0) {
				pr_err("%s: Failed to write final bytes\n", DRIVER_NAME);
				return ret;
			}
		}
	}

	pr_debug("%s: Firmware download complete\n", DRIVER_NAME);
	return 0;
}

/*
 * Reset 8051 MCU
 */
static int rtl8188eu_reset_8051(struct rtl8188eu_priv *priv)
{
	u8 val;
	int ret;

	pr_debug("%s: Resetting 8051 MCU...\n", DRIVER_NAME);

	/* Reset MCU IO Wrapper (as per old driver _MCUIO_Reset88E) */
	/* Clear REG_RSV_CTRL bit 1 */
	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL, &val);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL, val & ~BIT(1));
	if (ret < 0)
		return ret;

	/* Reset MCU IO Wrapper - clear REG_RSV_CTRL+1 bit 3 */
	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL + 1, &val);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL + 1, val & ~BIT(3));
	if (ret < 0)
		return ret;

	/* Disable CPU (REG_SYS_FUNC_EN+1 bit 2) */
	ret = rtl8188eu_read_reg8(priv, REG_SYS_FUNC_EN + 1, &val);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg8(priv, REG_SYS_FUNC_EN + 1, val & ~BIT(2));
	if (ret < 0)
		return ret;

	/* Enable MCU IO Wrapper (as per old driver _MCUIO_Reset88E) */
	/* Clear REG_RSV_CTRL bit 1 again */
	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL, &val);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL, val & ~BIT(1));
	if (ret < 0)
		return ret;

	/* Enable MCU IO Wrapper - set REG_RSV_CTRL+1 bit 3 */
	ret = rtl8188eu_read_reg8(priv, REG_RSV_CTRL + 1, &val);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg8(priv, REG_RSV_CTRL + 1, val | BIT(3));
	if (ret < 0)
		return ret;

	/* Enable CPU - re-read to get current value */
	ret = rtl8188eu_read_reg8(priv, REG_SYS_FUNC_EN + 1, &val);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg8(priv, REG_SYS_FUNC_EN + 1, val | BIT(2));
	if (ret < 0)
		return ret;

	pr_debug("%s: 8051 MCU reset complete\n", DRIVER_NAME);
	return 0;
}

/*
 * Enable/disable firmware download mode
 */
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

/*
 * Verify firmware checksum after download
 */
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

	pr_err("%s: Firmware checksum FAILED (REG_MCUFWDL=0x%08x)\n",
	       DRIVER_NAME, val32);
	return -EIO;
}

/*
 * Initialize H2C (Host-to-Card) command interface
 * This tells the firmware that the host is ready to communicate
 */
static int rtl8188eu_init_h2c(struct rtl8188eu_priv *priv)
{
	int ret;

	ret = rtl8188eu_write_reg8(priv, REG_HMETFR, 0x0F);
	if (ret < 0)
		return ret;

	priv->last_hmebox_num = 0;
	msleep(10);
	pr_debug("%s: H2C initialized\n", DRIVER_NAME);

	return 0;
}

/*
 * Activate firmware after download
 */
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
		val32 = 0;
	}

	ret = rtl8188eu_write_reg8(priv, REG_MCUFWDL + 1, 0x00);
	if (ret < 0)
		return ret;

	val32 = MCUFWDL_RDY;
	ret = rtl8188eu_write_reg32(priv, REG_MCUFWDL, val32);
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
	/* Some chips/firmware versions don't set WINTINI_RDY reliably - continue anyway */
	return 0;
}

/*
 * Load firmware from /lib/firmware/
 */
static int rtl8188eu_load_firmware(struct rtl8188eu_priv *priv)
{
	int ret;

	ret = request_firmware(&priv->fw, FIRMWARE_NAME, &priv->udev->dev);
	if (ret) {
		pr_err("%s: Failed to load firmware %s: %d\n",
		       DRIVER_NAME, FIRMWARE_NAME, ret);
		return ret;
	}

	/* Enable firmware download mode */
	ret = rtl8188eu_fw_download_enable(priv, true);
	if (ret) {
		pr_err("%s: Failed to enable FW download mode: %d\n", DRIVER_NAME, ret);
		release_firmware(priv->fw);
		priv->fw = NULL;
		return ret;
	}

	/* Download firmware to device RAM */
	ret = rtl8188eu_download_firmware(priv);
	if (ret) {
		pr_err("%s: Firmware download failed\n", DRIVER_NAME);
		rtl8188eu_fw_download_enable(priv, false);
		release_firmware(priv->fw);
		priv->fw = NULL;
		return ret;
	}

	/* Disable firmware download mode */
	ret = rtl8188eu_fw_download_enable(priv, false);
	if (ret) {
		pr_err("%s: Failed to disable FW download mode: %d\n", DRIVER_NAME, ret);
		release_firmware(priv->fw);
		priv->fw = NULL;
		return ret;
	}

	msleep(50);

	/* Verify firmware checksum */
	ret = rtl8188eu_verify_fw_checksum(priv);
	if (ret) {
		pr_warn("%s: Firmware checksum verification failed (continuing anyway): %d\n",
			DRIVER_NAME, ret);
		/* Continue anyway - some chips don't set the checksum bit reliably */
	}

	ret = rtl8188eu_init_h2c(priv);
	if (ret) {
		pr_err("%s: H2C initialization failed: %d\n", DRIVER_NAME, ret);
		pr_warn("%s: Continuing anyway - firmware may not work properly\n", DRIVER_NAME);
		/* Continue anyway - some variants might not need this */
	}

	/* Activate firmware and wait for it to initialize */
	ret = rtl8188eu_fw_free_to_go(priv);
	if (ret) {
		pr_err("%s: Firmware activation failed: %d\n", DRIVER_NAME, ret);
		release_firmware(priv->fw);
		priv->fw = NULL;
		return ret;
	}

	pr_info("%s: Firmware loaded and activated\n", DRIVER_NAME);

	return 0;
}

/*
 * ============================================================================
 * Post-Firmware Initialization Functions
 * ============================================================================
 */

/*
 * Initialize WMAC settings - configure RCR and multicast filters
 * This tells the hardware what packets to receive
 */
static int rtl8188eu_init_wmac_setting(struct rtl8188eu_priv *priv)
{
	u32 rcr;
	int ret;

	rcr = RCR_APM | RCR_AM | RCR_AB |
	      RCR_CBSSID_DATA | RCR_CBSSID_BCN |
	      RCR_APP_ICV | RCR_AMF | RCR_HTC_LOC_CTRL |
	      RCR_APP_MIC | RCR_APP_PHYST_RXFF;

	ret = rtl8188eu_write_reg32(priv, REG_RCR, rcr);
	if (ret < 0)
		return ret;

	ret = rtl8188eu_write_reg32(priv, REG_MAR, 0xFFFFFFFF);
	if (ret < 0)
		return ret;
	ret = rtl8188eu_write_reg32(priv, REG_MAR + 4, 0xFFFFFFFF);
	if (ret < 0)
		return ret;

	pr_debug("%s: WMAC configured (RCR=0x%08x)\n", DRIVER_NAME, rcr);
	return 0;
}

/*
 * ============================================================================
 * Monitor Mode Implementation (Phase 4)
 * ============================================================================
 */

/*
 * Enable or disable monitor mode by configuring RX filter
 */
static int rtl8188eu_set_monitor_mode(struct rtl8188eu_priv *priv, bool enable)
{
	u32 rcr;
	int ret;

	if (enable) {
		rcr = RCR_AAP | RCR_APM | RCR_AM | RCR_AB |
		      RCR_APWRMGT | RCR_ADF | RCR_ACF | RCR_AMF |
		      RCR_ACRC32 | RCR_APP_PHYST_RXFF |
		      RCR_APP_ICV | RCR_APP_MIC | RCR_APPFCS;

		ret = rtl8188eu_write_reg32(priv, REG_MAR, 0xFFFFFFFF);
		if (ret < 0)
			return ret;
		ret = rtl8188eu_write_reg32(priv, REG_MAR + 4, 0xFFFFFFFF);
		if (ret < 0)
			return ret;
		ret = rtl8188eu_write_reg16(priv, REG_RXFLTMAP2, 0xFFFF);
		if (ret < 0)
			return ret;
	} else {
		rcr = RCR_APM | RCR_AM | RCR_AB |
		      RCR_CBSSID_DATA | RCR_CBSSID_BCN |
		      RCR_APP_ICV | RCR_AMF | RCR_HTC_LOC_CTRL |
		      RCR_APP_MIC | RCR_APP_PHYST_RXFF;
	}

	/* Write RCR register */
	ret = rtl8188eu_write_reg32(priv, REG_RCR, rcr);
	if (ret < 0) {
		pr_err("%s: Failed to write REG_RCR\n", DRIVER_NAME);
		return ret;
	}

	return 0;
}

/*
 * Set WiFi channel (1-14 for 2.4GHz)
 * Tunes the radio to receive packets on the specified channel
 */
static int rtl8188eu_set_channel(struct rtl8188eu_priv *priv, u8 channel)
{
	u32 rf_val;

	if (channel < 1 || channel > 14) {
		pr_err("%s: Invalid channel %d (must be 1-14)\n", DRIVER_NAME, channel);
		return -EINVAL;
	}

	/* Read-modify-write RF_CHNLBW: preserve upper bits, set channel in [9:0] */
	priv->rf_chnl_val = (priv->rf_chnl_val & 0xfffffc00) | channel;
	rtl8188eu_write_rf_reg(priv, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask,
			       priv->rf_chnl_val);

	/* Verify */
	rf_val = rtl8188eu_read_rf_reg(priv, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask);
	pr_debug("%s: RF_CHNLBW = 0x%05x after channel set\n", DRIVER_NAME, rf_val);

	priv->channel = channel;
	pr_info("%s: Channel set to %d (%d MHz)\n",
		DRIVER_NAME, channel, rtl8188eu_chan_to_freq(channel));
	return 0;
}

/*
 * ============================================================================
 * Network Interface Setup (Phase 2)
 * ============================================================================
 */

/*
 * ============================================================================
 * RX Path Implementation (Phase 2, Step 5)
 * ============================================================================
 */

/*
 * RX URB completion callback (called when data received from USB)
 */
static void rtl8188eu_rx_complete(struct urb *urb)
{
	struct rtl8188eu_rx_urb *rx_urb;
	struct rtl8188eu_priv *priv;
	struct net_device *netdev;
	struct rtl8188eu_rx_desc *rx_desc;
	struct sk_buff *skb;
	u32 rxdw0, rxdw2;
	u16 pkt_len, pkt_cnt;
	u16 drvinfo_sz, shift_sz;
	u32 pkt_offset;
	s32 transfer_len;
	u8 *pbuf;
	int ret;
	static unsigned int rx_callback_count;
	static unsigned int rx_total_packets;

	/* Validate URB and context */
	if (!urb)
		return;

	rx_urb = urb->context;
	if (!rx_urb || !rx_urb->priv || !rx_urb->priv->netdev)
		return;

	priv = rx_urb->priv;
	netdev = priv->netdev;

	rx_callback_count++;

	if (rx_callback_count <= 3)
		pr_info("%s: RX callback #%u: status=%d, len=%d\n",
			DRIVER_NAME, rx_callback_count, urb->status, urb->actual_length);

	/* Handle URB errors */
	if (urb->status != 0) {
		switch (urb->status) {
		case -ENOENT:
		case -ECONNRESET:
		case -ESHUTDOWN:
		case -ENODEV:
			/* Device gone or URB killed - don't resubmit */
			return;
		case -EPROTO:
		case -EILSEQ:
		case -ETIME:
		case -ECOMM:
		case -EOVERFLOW:
			/* Recoverable errors - resubmit */
			if ((rx_callback_count % 100) == 0)
				pr_warn("%s: RX URB error %d (recoverable), resubmitting\n",
					DRIVER_NAME, urb->status);
			goto resubmit;
		default:
			pr_err("%s: RX URB failed: %d\n", DRIVER_NAME, urb->status);
			goto resubmit;
		}
	}

	/* Zero-length transfer - just resubmit */
	if (urb->actual_length == 0)
		goto resubmit;

	/* Need at least one RX descriptor */
	if (urb->actual_length < RXDESC_SIZE) {
		netdev->stats.rx_errors++;
		goto resubmit;
	}

	/*
	 * Parse aggregated packets from this URB.
	 * With DMA aggregation, one URB can contain multiple WiFi packets,
	 * each preceded by a 24-byte RX descriptor + optional driver info.
	 */
	pbuf = urb->transfer_buffer;
	transfer_len = (s32)urb->actual_length;

	/* Get initial packet count from first descriptor */
	rx_desc = (struct rtl8188eu_rx_desc *)pbuf;
	rxdw2 = le32_to_cpu(rx_desc->rxdw2);
	pkt_cnt = (rxdw2 >> RX_DW2_PKT_CNT_SHIFT) & RX_DW2_PKT_CNT_MASK;

	/* If pkt_cnt is 0, treat as single packet */
	if (pkt_cnt == 0)
		pkt_cnt = 1;

	do {
		if (transfer_len < (s32)RXDESC_SIZE)
			break;

		rx_desc = (struct rtl8188eu_rx_desc *)pbuf;
		rxdw0 = le32_to_cpu(rx_desc->rxdw0);

		/* Extract fields from descriptor */
		pkt_len = rxdw0 & RX_DW0_PKT_LEN_MASK;
		drvinfo_sz = ((rxdw0 >> RX_DW0_DRVINFO_SZ_SHIFT) & RX_DW0_DRVINFO_SZ_MASK) * 8;
		shift_sz = (rxdw0 >> RX_DW0_SHIFT_SHIFT) & RX_DW0_SHIFT_MASK;

		/* Calculate total packet envelope size */
		pkt_offset = RXDESC_SIZE + drvinfo_sz + shift_sz + pkt_len;

		/* Validate */
		if (pkt_len == 0 || (s32)pkt_offset > transfer_len) {
			netdev->stats.rx_errors++;
			break;
		}

		/* Skip CRC/ICV error packets */
		if (rxdw0 & (RX_DW0_CRC32 | RX_DW0_ICV_ERR)) {
			netdev->stats.rx_errors++;
			goto next_pkt;
		}

		/* Allocate skb with room for radiotap header */
		skb = dev_alloc_skb(sizeof(struct rtl8188eu_radiotap_hdr) + pkt_len);
		if (!skb) {
			netdev->stats.rx_dropped++;
			goto next_pkt;
		}

		/* Parse rate from RX descriptor DW3 and signal from PHY status */
		{
			struct rtl8188eu_radiotap_hdr *rthdr;
			u32 rxdw3 = le32_to_cpu(rx_desc->rxdw3);
			u8 hw_rate = rxdw3 & 0x3f;
			bool is_ht = (rxdw3 & BIT(6)) || (hw_rate >= 0x0c);
			bool is_cck = (hw_rate <= 0x03);
			s8 signal_dbm = -50;
			u16 chan_flags = RTL_CHAN_2GHZ;

			/* Parse PHY status for signal strength */
			if ((rxdw0 & RX_DW0_PHYST) && drvinfo_sz >= 8) {
				u8 *phy_status = pbuf + RXDESC_SIZE;

				if (is_cck) {
					u8 cck_agc = phy_status[5];
					u8 lna_idx = (cck_agc >> 5) & 0x7;
					u8 vga_idx = cck_agc & 0x1f;

					signal_dbm = cck_lna_gain[lna_idx] - (2 * vga_idx);
				} else {
					signal_dbm = ((phy_status[4] >> 1) & 0x7f) - 110;
				}

				if (signal_dbm < -100)
					signal_dbm = -100;
				if (signal_dbm > 0)
					signal_dbm = 0;

				if (rx_total_packets < 10)
					pr_info("%s: RX diag #%u: hw_rate=0x%02x is_cck=%d drvinfo=%u "
						"physt=%d phy[0-7]=%02x %02x %02x %02x %02x %02x %02x %02x "
						"signal=%d\n",
						DRIVER_NAME, rx_total_packets, hw_rate, is_cck,
						drvinfo_sz, !!(rxdw0 & RX_DW0_PHYST),
						phy_status[0], phy_status[1], phy_status[2], phy_status[3],
						phy_status[4], phy_status[5], phy_status[6], phy_status[7],
						signal_dbm);
			}

			chan_flags |= is_cck ? RTL_CHAN_CCK : RTL_CHAN_OFDM;

			rthdr = skb_put(skb, sizeof(*rthdr));
			memset(rthdr, 0, sizeof(*rthdr));
			rthdr->hdr.it_version = 0;
			rthdr->hdr.it_len = cpu_to_le16(sizeof(*rthdr));
			rthdr->flags = 0;
			rthdr->chan_freq = cpu_to_le16(
				rtl8188eu_chan_to_freq(priv->channel));
			rthdr->chan_flags = cpu_to_le16(chan_flags);
			rthdr->signal = signal_dbm;
			rthdr->antenna = 0;

			if (is_ht) {
				rthdr->hdr.it_present = cpu_to_le32(RTL8188EU_RADIOTAP_PRESENT);
				rthdr->rate = 0;
				rthdr->mcs_known = 0x07;
				rthdr->mcs_flags = 0;
				rthdr->mcs_index = hw_rate & 0x1f;
			} else {
				rthdr->hdr.it_present = cpu_to_le32(RTL8188EU_RADIOTAP_PRESENT
					& ~(1 << IEEE80211_RADIOTAP_MCS));
				rthdr->rate = (hw_rate < ARRAY_SIZE(rtl8188eu_hw_rate_to_radiotap))
					? rtl8188eu_hw_rate_to_radiotap[hw_rate] : 12;
				rthdr->mcs_known = 0;
				rthdr->mcs_flags = 0;
				rthdr->mcs_index = 0;
			}
		}

		/* Copy 802.11 frame data after radiotap header */
		skb_put_data(skb, pbuf + RXDESC_SIZE + drvinfo_sz + shift_sz, pkt_len);
		skb->dev = netdev;
		skb->pkt_type = PACKET_OTHERHOST;
		skb->protocol = htons(ETH_P_802_2);
		skb_reset_mac_header(skb);

		netdev->stats.rx_packets++;
		netdev->stats.rx_bytes += pkt_len;
		rx_total_packets++;

		netif_rx(skb);

next_pkt:
		/* Round offset to 128-byte boundary for DMA aggregation */
		pkt_offset = RX_RND128(pkt_offset);
		pkt_cnt--;
		transfer_len -= pkt_offset;
		pbuf += pkt_offset;

		/* Refresh pkt_cnt if we have more data but counter ran out */
		if (transfer_len > 0 && pkt_cnt == 0) {
			rx_desc = (struct rtl8188eu_rx_desc *)pbuf;
			rxdw2 = le32_to_cpu(rx_desc->rxdw2);
			pkt_cnt = (rxdw2 >> RX_DW2_PKT_CNT_SHIFT) & RX_DW2_PKT_CNT_MASK;
		}
	} while (transfer_len > 0 && pkt_cnt > 0);

	/* Periodic stats logging (every 1000 callbacks) */
	if ((rx_callback_count % 1000) == 0)
		pr_info("%s: RX stats: %u callbacks, %u packets delivered\n",
			DRIVER_NAME, rx_callback_count, rx_total_packets);

resubmit:
	/* Don't resubmit if interface is going down or device being removed */
	if (!netif_running(netdev) || priv->disconnecting) {
		pr_debug("%s: RX URB not resubmitted (running=%d, disconnecting=%d)\n",
			DRIVER_NAME, netif_running(netdev), priv->disconnecting);
		return;
	}

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret) {
		pr_err("%s: RX URB resubmit failed: %d (status was %d, len=%d)\n",
			DRIVER_NAME, ret, urb->status, urb->actual_length);
	} else if (rx_callback_count <= 3) {
		pr_debug("%s: RX URB resubmitted OK (callback #%u)\n",
			DRIVER_NAME, rx_callback_count);
	}
}

/*
 * Start RX operations (allocate and submit URBs)
 */
static int rtl8188eu_start_rx(struct rtl8188eu_priv *priv)
{
	unsigned int pipe;
	int i, ret;

	pr_debug("%s: Starting RX with %d URBs\n", DRIVER_NAME, RX_URB_COUNT);

	pipe = usb_rcvbulkpipe(priv->udev, priv->rx_endpoint);

	for (i = 0; i < RX_URB_COUNT; i++) {
		struct rtl8188eu_rx_urb *rx_urb = &priv->rx_urbs[i];

		/* Allocate URB */
		rx_urb->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!rx_urb->urb) {
			pr_err("%s: Failed to allocate RX URB %d\n", DRIVER_NAME, i);
			ret = -ENOMEM;
			goto err_free_urbs;
		}

		/* Allocate buffer */
		rx_urb->buffer = kmalloc(RX_BUFFER_SIZE, GFP_KERNEL);
		if (!rx_urb->buffer) {
			pr_err("%s: Failed to allocate RX buffer %d\n", DRIVER_NAME, i);
			usb_free_urb(rx_urb->urb);
			rx_urb->urb = NULL;
			ret = -ENOMEM;
			goto err_free_urbs;
		}

		rx_urb->priv = priv;

		/* Fill URB */
		usb_fill_bulk_urb(rx_urb->urb, priv->udev, pipe,
				  rx_urb->buffer, RX_BUFFER_SIZE,
				  rtl8188eu_rx_complete, rx_urb);

		/* Submit URB */
		ret = usb_submit_urb(rx_urb->urb, GFP_KERNEL);
		if (ret < 0) {
			pr_err("%s: Failed to submit RX URB %d: %d\n",
			       DRIVER_NAME, i, ret);
			kfree(rx_urb->buffer);
			usb_free_urb(rx_urb->urb);
			rx_urb->urb = NULL;
			rx_urb->buffer = NULL;
			goto err_free_urbs;
		}
	}

	pr_debug("%s: RX started (%d URBs)\n", DRIVER_NAME, RX_URB_COUNT);
	return 0;

err_free_urbs:
	/* Clean up any URBs that were allocated */
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

/*
 * Stop RX operations (kill and free URBs)
 */
static void rtl8188eu_stop_rx(struct rtl8188eu_priv *priv)
{
	int i;

	pr_debug("%s: Stopping RX...\n", DRIVER_NAME);

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

	pr_debug("%s: RX stopped\n", DRIVER_NAME);
}

/*
 * ============================================================================
 * Network Device Operations (Phase 2, Step 3)
 * ============================================================================
 */

/*
 * Open network interface (called when bringing interface up)
 */
static int rtl8188eu_ndo_open(struct net_device *netdev)
{
	struct rtl8188eu_priv *priv = netdev->ml_priv;
	u32 val32;
	int ret;

	pr_info("%s: Interface %s opened\n", DRIVER_NAME, netdev->name);

	/* Always start in monitor mode with radiotap headers */
	netdev->type = ARPHRD_IEEE80211_RADIOTAP;
	netdev->hard_header_len = 0;
	netdev->header_ops = NULL;

	/* Clear RW_RELEASE_EN - when set, RXDMA delivers one packet then stops */
	ret = rtl8188eu_read_reg32(priv, REG_RXPKT_NUM, &val32);
	if (ret == 0 && (val32 & RW_RELEASE_EN)) {
		val32 &= ~RW_RELEASE_EN;
		rtl8188eu_write_reg32(priv, REG_RXPKT_NUM, val32);
	}

	/* Start RX operations - submit URBs */
	ret = rtl8188eu_start_rx(priv);
	if (ret) {
		pr_err("%s: Failed to start RX: %d\n", DRIVER_NAME, ret);
		return ret;
	}

	/* Enable monitor mode (Phase 4) */
	ret = rtl8188eu_set_monitor_mode(priv, true);
	if (ret) {
		pr_err("%s: Failed to enable monitor mode: %d\n", DRIVER_NAME, ret);
		rtl8188eu_stop_rx(priv);
		return ret;
	}

	/* Set default channel 6 (2437 MHz) for packet reception */
	ret = rtl8188eu_set_channel(priv, 6);
	if (ret) {
		pr_err("%s: Failed to set channel: %d\n", DRIVER_NAME, ret);
		rtl8188eu_stop_rx(priv);
		return ret;
	}

	/* Enable TX queue */
	netif_start_queue(netdev);

	/* Set carrier on (link is up) */
	netif_carrier_on(netdev);

	return 0;
}

/*
 * Close network interface (called when bringing interface down)
 */
static int rtl8188eu_ndo_stop(struct net_device *netdev)
{
	struct rtl8188eu_priv *priv = netdev->ml_priv;

	pr_info("%s: Interface %s closed\n", DRIVER_NAME, netdev->name);

	/* Set carrier off (link is down) */
	netif_carrier_off(netdev);

	/* Disable TX queue */
	netif_stop_queue(netdev);

	/* Stop RX operations */
	rtl8188eu_stop_rx(priv);

	return 0;
}

/*
 * ============================================================================
 * TX Path Implementation (Phase 2, Step 6)
 * ============================================================================
 */

/*
 * TX URB context structure to track skb and stats
 */
struct rtl8188eu_tx_context {
	struct rtl8188eu_priv *priv;
	struct sk_buff *skb;
};

/*
 * TX URB completion callback (called when packet transmitted via USB)
 */
static void rtl8188eu_tx_complete(struct urb *urb)
{
	struct rtl8188eu_tx_context *ctx = urb->context;
	struct rtl8188eu_priv *priv = ctx->priv;
	struct net_device *netdev = priv->netdev;
	struct sk_buff *skb = ctx->skb;

	if (urb->status == 0) {
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += skb->len;
	} else {
		/* Error - update error statistics */
		netdev->stats.tx_errors++;
		if (urb->status != -ENOENT && urb->status != -ECONNRESET &&
		    urb->status != -ESHUTDOWN) {
			pr_err("%s: TX URB failed: %d\n", DRIVER_NAME, urb->status);
		}
	}

	/* Free the socket buffer */
	dev_kfree_skb_any(skb);

	/* Free the URB (buffer freed automatically via URB_FREE_BUFFER) */
	usb_free_urb(urb);

	/* Free context */
	kfree(ctx);

	/* Wake TX queue if it was stopped */
	if (netif_queue_stopped(netdev))
		netif_wake_queue(netdev);
}

/*
 * Transmit packet
 */
static netdev_tx_t rtl8188eu_ndo_start_xmit(struct sk_buff *skb,
					     struct net_device *netdev)
{
	struct rtl8188eu_priv *priv = netdev->ml_priv;
	struct rtl8188eu_tx_context *ctx;
	struct rtl8188eu_tx_desc *tx_desc;
	struct urb *urb;
	unsigned int pipe;
	u8 *tx_buffer;
	u32 total_len;
	int ret;

	/* Don't transmit if device is being removed */
	if (priv->disconnecting) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	/* Total length = TX descriptor (32 bytes) + packet data */
	total_len = sizeof(struct rtl8188eu_tx_desc) + skb->len;

	/* Allocate URB */
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		pr_err("%s: Failed to allocate TX URB\n", DRIVER_NAME);
		netdev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	/* Allocate TX buffer (descriptor + packet data) */
	tx_buffer = kmalloc(total_len, GFP_ATOMIC);
	if (!tx_buffer) {
		pr_err("%s: Failed to allocate TX buffer\n", DRIVER_NAME);
		netdev->stats.tx_dropped++;
		usb_free_urb(urb);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	/* Allocate context to track skb and priv */
	ctx = kmalloc(sizeof(*ctx), GFP_ATOMIC);
	if (!ctx) {
		pr_err("%s: Failed to allocate TX context\n", DRIVER_NAME);
		netdev->stats.tx_dropped++;
		kfree(tx_buffer);
		usb_free_urb(urb);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	ctx->priv = priv;
	ctx->skb = skb;

	/* Clear TX descriptor */
	memset(tx_buffer, 0, sizeof(struct rtl8188eu_tx_desc));
	tx_desc = (struct rtl8188eu_tx_desc *)tx_buffer;

	/* Fill TX descriptor */
	/* DW0: packet size and descriptor offset */
	tx_desc->txdw0 = cpu_to_le32(
		(skb->len << TX_DW0_PKT_SIZE_SHIFT) |
		(sizeof(struct rtl8188eu_tx_desc) << TX_DW0_OFFSET_SHIFT) |
		TX_DW0_OWN
	);

	/* DW1: queue selection (use best effort queue) */
	tx_desc->txdw1 = cpu_to_le32(
		(TX_DW1_QUEUE_SEL_BE << TX_DW1_QUEUE_SEL_SHIFT)
	);

	/* Copy packet data after the descriptor */
	memcpy(tx_buffer + sizeof(struct rtl8188eu_tx_desc), skb->data, skb->len);

	/* Create USB bulk pipe for TX endpoint */
	pipe = usb_sndbulkpipe(priv->udev, priv->tx_endpoint);

	/* Fill URB with descriptor + packet data */
	usb_fill_bulk_urb(urb, priv->udev, pipe,
			  tx_buffer, total_len,
			  rtl8188eu_tx_complete, ctx);

	/* Set flag to auto-free buffer after transmission */
	urb->transfer_flags |= URB_FREE_BUFFER;

	/* Submit URB */
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret < 0) {
		pr_err("%s: Failed to submit TX URB: %d\n", DRIVER_NAME, ret);
		netdev->stats.tx_errors++;
		kfree(ctx);
		/* URB_FREE_BUFFER is set, so usb_free_urb frees tx_buffer */
		usb_free_urb(urb);
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	return NETDEV_TX_OK;
}

/*
 * Wireless extension handlers for airodump-ng / iwconfig compatibility
 */
static int rtl8188eu_wx_get_name(struct net_device *dev,
	struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	strcpy(wrqu->name, "IEEE 802.11bgn");
	return 0;
}

static int rtl8188eu_wx_set_mode(struct net_device *dev,
	struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	if (wrqu->mode != IW_MODE_MONITOR)
		return -EOPNOTSUPP;
	dev->type = ARPHRD_IEEE80211_RADIOTAP;
	return 0;
}

static int rtl8188eu_wx_get_mode(struct net_device *dev,
	struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	wrqu->mode = IW_MODE_MONITOR;
	return 0;
}

static int rtl8188eu_wx_set_freq(struct net_device *dev,
	struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	struct rtl8188eu_priv *priv = dev->ml_priv;
	u8 channel;

	if (wrqu->freq.e == 0) {
		channel = wrqu->freq.m;
	} else {
		int freq_mhz = wrqu->freq.m;
		int i;

		for (i = wrqu->freq.e; i > 6; i--)
			freq_mhz *= 10;
		for (i = wrqu->freq.e; i < 6; i++)
			freq_mhz /= 10;

		if (freq_mhz == 2484)
			channel = 14;
		else if (freq_mhz >= 2412 && freq_mhz <= 2472)
			channel = (freq_mhz - 2407) / 5;
		else
			return -EINVAL;
	}

	if (channel < 1 || channel > 14)
		return -EINVAL;

	return rtl8188eu_set_channel(priv, channel);
}

static int rtl8188eu_wx_get_freq(struct net_device *dev,
	struct iw_request_info *info, union iwreq_data *wrqu, char *extra)
{
	struct rtl8188eu_priv *priv = dev->ml_priv;

	wrqu->freq.m = priv->channel;
	wrqu->freq.e = 0;
	wrqu->freq.flags = IW_FREQ_FIXED;
	return 0;
}

static const iw_handler rtl8188eu_wx_handlers[] = {
	[IW_IOCTL_IDX(SIOCGIWNAME)]	= rtl8188eu_wx_get_name,
	[IW_IOCTL_IDX(SIOCSIWFREQ)]	= rtl8188eu_wx_set_freq,
	[IW_IOCTL_IDX(SIOCGIWFREQ)]	= rtl8188eu_wx_get_freq,
	[IW_IOCTL_IDX(SIOCSIWMODE)]	= rtl8188eu_wx_set_mode,
	[IW_IOCTL_IDX(SIOCGIWMODE)]	= rtl8188eu_wx_get_mode,
};

static const struct iw_handler_def rtl8188eu_wx_def = {
	.standard	= rtl8188eu_wx_handlers,
	.num_standard	= ARRAY_SIZE(rtl8188eu_wx_handlers),
};

/*
 * Network device operations structure
 */
static const struct net_device_ops rtl8188eu_netdev_ops = {
	.ndo_open		= rtl8188eu_ndo_open,
	.ndo_stop		= rtl8188eu_ndo_stop,
	.ndo_start_xmit		= rtl8188eu_ndo_start_xmit,
};

/*
 * Allocate and initialize network device
 */
static int rtl8188eu_netdev_init(struct rtl8188eu_priv *priv)
{
	struct net_device *netdev;

	pr_debug("%s: Initializing network device\n", DRIVER_NAME);

	/* Allocate ethernet device (no extra private data) */
	netdev = alloc_etherdev(0);
	if (!netdev) {
		pr_err("%s: Failed to allocate network device\n", DRIVER_NAME);
		return -ENOMEM;
	}

	/* Set device parent for sysfs hierarchy */
	SET_NETDEV_DEV(netdev, &priv->intf->dev);

	/* Generate random MAC address (later will read from EFUSE) */
	eth_random_addr(priv->mac_addr);

	/* Set MAC address to network device */
	eth_hw_addr_set(netdev, priv->mac_addr);

	pr_info("%s: MAC Address: %pM\n", DRIVER_NAME, priv->mac_addr);

	/* Set network device operations */
	netdev->netdev_ops = &rtl8188eu_netdev_ops;
	netdev->wireless_handlers = &rtl8188eu_wx_def;

	/* Store references */
	priv->netdev = netdev;
	netdev->priv_flags |= IFF_LIVE_ADDR_CHANGE;  /* Allow MAC change while up */

	/* Link back to our private data */
	netdev->ml_priv = priv;  /* Use ml_priv to store our priv pointer */

	pr_debug("%s: Network device initialized\n", DRIVER_NAME);
	return 0;
}

/*
 * Register network device with kernel (Phase 2, Step 4)
 */
static int rtl8188eu_register_netdev(struct rtl8188eu_priv *priv)
{
	int ret;

	ret = register_netdev(priv->netdev);
	if (ret < 0) {
		pr_err("%s: Failed to register network device: %d\n",
		       DRIVER_NAME, ret);
		return ret;
	}

	pr_info("%s: Registered as %s (MAC %pM)\n",
		DRIVER_NAME, priv->netdev->name, priv->mac_addr);

	return 0;
}

/*
 * Detect USB bulk endpoints for RX/TX
 * Counts TX endpoints and sets queue selection accordingly
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

	if (!priv->rx_endpoint || !priv->tx_endpoint) {
		pr_err("%s: Failed to find required endpoints (RX=0x%02x, TX=0x%02x)\n",
		       DRIVER_NAME, priv->rx_endpoint, priv->tx_endpoint);
		return -ENODEV;
	}

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

	pr_debug("%s: USB endpoints: RX=0x%02x TX=0x%02x (%d TX EPs)\n",
		DRIVER_NAME, priv->rx_endpoint, priv->tx_endpoint, tx_ep_count);
	return 0;
}

/*
 * Initialize TRXDMA queue priority mapping
 * Maps TX DMA queues to USB endpoints based on endpoint configuration
 * For 2-EP (HQ|NQ): VO/VI/MGT/HI -> HIGH, BE/BK -> NORMAL
 */
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

	ret = rtl8188eu_write_reg16(priv, REG_TRXDMA_CTRL, value16);
	if (ret < 0)
		return ret;

	pr_debug("%s: TRXDMA_CTRL=0x%04x\n", DRIVER_NAME, value16);
	return 0;
}

/*
 * Enable hardware drop of incorrect bulk-out packets
 * Sets DROP_DATA_EN in REG_TXDMA_OFFSET_CHK
 */
static int rtl8188eu_init_hw_drop_incorrect_bulkout(struct rtl8188eu_priv *priv)
{
	u32 value32;
	int ret;

	ret = rtl8188eu_read_reg32(priv, REG_TXDMA_OFFSET_CHK, &value32);
	if (ret < 0)
		return ret;

	value32 |= BIT(9);
	ret = rtl8188eu_write_reg32(priv, REG_TXDMA_OFFSET_CHK, value32);
	if (ret < 0)
		return ret;

	pr_debug("%s: HW drop incorrect bulk-out enabled\n", DRIVER_NAME);
	return 0;
}

/*
 * Initialize TX report configuration
 * Enables TX status reporting for rate adaptation
 */
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

	ret = rtl8188eu_write_reg16(priv, REG_TX_RPT_TIME, 0xCDF0);
	if (ret < 0)
		return ret;

	pr_debug("%s: TX report configured\n", DRIVER_NAME);
	return 0;
}

/*
 * ============================================================================
 * USB Driver (Phase 1)
 * ============================================================================
 */

/*
 * Probe function - called when device is plugged in
 */
static int rtl8188eu_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct rtl8188eu_priv *priv;
	int ret;
	u8 val8;

	pr_info("%s: RTL8188EUS detected (bus %d, dev %d)\n",
		DRIVER_NAME, udev->bus->busnum, udev->devnum);

	/* Allocate private data structure */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		pr_err("%s: Failed to allocate memory\n", DRIVER_NAME);
		return -ENOMEM;
	}

	priv->udev = udev;
	priv->intf = intf;
	priv->chip_powered_on = false;
	mutex_init(&priv->rf_read_mutex);

	priv->hw_cfg.interface_type = 1;  /* USB interface */
	priv->hw_cfg.board_type = 0;
	priv->hw_cfg.cut_version = 0;
	priv->hw_cfg.package_type = 0;

	usb_set_intfdata(intf, priv);

	/* Detect USB endpoints for Phase 2 */
	ret = rtl8188eu_detect_endpoints(priv);
	if (ret) {
		pr_err("%s: Failed to detect USB endpoints\n", DRIVER_NAME);
		goto err_free_priv;
	}

	/* Initialize network device (Phase 2, Step 2) */
	ret = rtl8188eu_netdev_init(priv);
	if (ret) {
		pr_err("%s: Failed to initialize network device\n", DRIVER_NAME);
		goto err_free_priv;
	}

	ret = rtl8188eu_power_on(priv);
	if (ret)
		goto err_free_priv;

	ret = rtl8188eu_enable_mcu_clocks(priv);
	if (ret)
		goto err_free_priv;

	/* TX/RX queues (BEFORE firmware) */
	ret = rtl8188eu_init_tx_rx_queues(priv);
	if (ret) {
		pr_err("%s: TX/RX queue setup failed: %d\n", DRIVER_NAME, ret);
		goto err_free_priv;
	}

	ret = rtl8188eu_init_queue_priority(priv);
	if (ret)
		pr_warn("%s: Queue priority init failed: %d\n", DRIVER_NAME, ret);

	ret = rtl8188eu_init_llt_table(priv);
	if (ret)
		goto err_free_priv;

	ret = rtl8188eu_load_firmware(priv);
	if (ret)
		goto err_free_priv;

	ret = rtl8188eu_load_mac_reg_table(priv);
	if (ret)
		pr_warn("%s: MAC register table failed: %d\n", DRIVER_NAME, ret);

	ret = rtl8188eu_init_interrupts(priv);
	if (ret)
		pr_warn("%s: Interrupt init failed: %d\n", DRIVER_NAME, ret);

	rtl8188eu_write_reg8(priv, REG_RX_DRVINFO_SZ, 0x04);

	{
		u32 cr32;
		ret = rtl8188eu_read_reg32(priv, REG_CR, &cr32);
		if (ret == 0) {
			cr32 = (cr32 & ~0x30000) | (0x2 << 16);
			rtl8188eu_write_reg32(priv, REG_CR, cr32);
		}
	}

	ret = rtl8188eu_init_wmac_setting(priv);
	if (ret)
		pr_warn("%s: WMAC init failed: %d\n", DRIVER_NAME, ret);

	ret = rtl8188eu_init_mac_regs(priv);
	if (ret)
		pr_warn("%s: MAC register init failed: %d\n", DRIVER_NAME, ret);

	/* AFE (Analog Front End) - CRITICAL for BB/RF clock */
	ret = rtl8188eu_write_reg32(priv, REG_AFE_XTAL_CTRL, 0x000F81FB);
	if (ret == 0)
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

	/* Enable BB (25.1MHz clock + reset bits) */
	{
		u16 sys_func;
		ret = rtl8188eu_read_reg16(priv, REG_SYS_FUNC_EN, &sys_func);
		if (ret == 0) {
			sys_func |= BIT(13) | FEN_BBRSTB | FEN_BB_GLB_RSTn;
			rtl8188eu_write_reg16(priv, REG_SYS_FUNC_EN, sys_func);
			msleep(2);
		}
	}

	/* Enable RF interface */
	{
		u8 rf_ctrl;
		ret = rtl8188eu_read_reg8(priv, REG_RF_CTRL, &rf_ctrl);
		if (ret == 0) {
			rf_ctrl |= 0x07;
			rtl8188eu_write_reg8(priv, REG_RF_CTRL, rf_ctrl);
			msleep(1);
		}
	}

	/* Load PHY tables (BB + AGC + RF) */
	ret = rtl8188eu_phy_bb_config(priv);
	if (ret)
		pr_err("%s: BB config failed: %d\n", DRIVER_NAME, ret);

	msleep(10);

	ret = rtl8188eu_phy_rf_config(priv);
	if (ret)
		pr_err("%s: RF config failed: %d\n", DRIVER_NAME, ret);

	/* RF register diagnostics - verify RF chip is responding */
	{
		u32 rf00, rf18;

		rf00 = rtl8188eu_read_rf_reg(priv, RF_PATH_A, 0x00, 0xFFFFF);
		rf18 = rtl8188eu_read_rf_reg(priv, RF_PATH_A, 0x18, 0xFFFFF);
		pr_debug("%s: Post-RF: RF 0x00=0x%05x RF 0x18=0x%05x\n",
			DRIVER_NAME, rf00, rf18);

		/* Cache RF_CHNLBW for channel switching */
		priv->rf_chnl_val = rf18;
	}

	rtl8188eu_lc_calibrate(priv);
	rtl8188eu_iqk_calibrate(priv);

	/* Enable CCK + OFDM in BB */
	{
		u32 rfmod;
		ret = rtl8188eu_read_reg32(priv, rFPGA0_RFMOD, &rfmod);
		if (ret == 0) {
			rfmod |= bCCKEn | bOFDMEn;
			rtl8188eu_write_reg32(priv, rFPGA0_RFMOD, rfmod);
		}
	}

	/* USB Aggregation (CRITICAL for RX!) */
	rtl8188eu_write_reg8(priv, 0x280, 0x30);
	rtl8188eu_write_reg8(priv, 0x281, 0x04);

	/* Enable RXDMA_AGG_EN - OR into TRXDMA_CTRL low byte */
	/* Queue mapping was already set in init_queue_priority */
	ret = rtl8188eu_read_reg8(priv, REG_TRXDMA_CTRL, &val8);
	if (ret == 0) {
		val8 |= BIT(2);
		rtl8188eu_write_reg8(priv, REG_TRXDMA_CTRL, val8);
	}

	/* USB special option */
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

	/* RF front-end configuration */
	{
		u32 ee8_val;
		ret = rtl8188eu_read_reg32(priv, 0xEE8, &ee8_val);
		if (ret == 0) {
			ee8_val |= 0x10000000;
			rtl8188eu_write_reg32(priv, 0xEE8, ee8_val);
		}
	}
	rtl8188eu_write_reg32(priv, 0x87C, 0x00000000);

	/* Default TX power levels */
	rtl8188eu_write_reg32(priv, 0xE00, 0x2D2D2D2D);
	rtl8188eu_write_reg32(priv, 0xE04, 0x2D2D2D2D);
	rtl8188eu_write_reg32(priv, 0xE08, 0x2626262A);
	rtl8188eu_write_reg32(priv, 0xE10, 0x26262626);

	msleep(50);

	pr_info("%s: Init complete\n", DRIVER_NAME);

	/* Register network device */
	ret = rtl8188eu_register_netdev(priv);
	if (ret) {
		pr_err("%s: Failed to register network device\n", DRIVER_NAME);
		goto err_free_priv;
	}

	return 0;

err_free_priv:
	if (priv->fw)
		release_firmware(priv->fw);
	if (priv->netdev)
		free_netdev(priv->netdev);
	usb_set_intfdata(intf, NULL);
	kfree(priv);
	return ret;
}

/*
 * Disconnect function - called when device is unplugged
 */
static void rtl8188eu_disconnect(struct usb_interface *intf)
{
	struct rtl8188eu_priv *priv = usb_get_intfdata(intf);

	if (!priv)
		return;

	priv->disconnecting = true;

	if (priv->netdev) {
		netif_device_detach(priv->netdev);
		rtl8188eu_stop_rx(priv);
		unregister_netdev(priv->netdev);
		free_netdev(priv->netdev);
	}

	if (priv->fw)
		release_firmware(priv->fw);

	usb_set_intfdata(intf, NULL);
	kfree(priv);
	pr_info("%s: Disconnected\n", DRIVER_NAME);
}

/* USB Driver structure */
static struct usb_driver rtl8188eu_driver = {
	.name       = DRIVER_NAME,
	.probe      = rtl8188eu_probe,
	.disconnect = rtl8188eu_disconnect,
	.id_table   = rtl8188eu_usb_ids,
};

/*
 * Module initialization
 */
static int __init rtl8188eu_init(void)
{
	int ret;

	pr_info("%s: v%s loaded\n", DRIVER_NAME, DRIVER_VERSION);

	ret = usb_register(&rtl8188eu_driver);
	if (ret)
		pr_err("%s: USB register failed: %d\n", DRIVER_NAME, ret);
	return 0;
}

/*
 * Module cleanup
 */
static void __exit rtl8188eu_exit(void)
{
	pr_info("%s: Unloading driver\n", DRIVER_NAME);
	usb_deregister(&rtl8188eu_driver);
	pr_info("%s: Driver unloaded\n", DRIVER_NAME);
}

module_init(rtl8188eu_init);
module_exit(rtl8188eu_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Minimal RTL8188EUS USB WiFi Driver - Hardware Initialization");
MODULE_VERSION(DRIVER_VERSION);
MODULE_FIRMWARE(FIRMWARE_NAME);
