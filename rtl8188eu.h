#ifndef RTL8188EU_H
#define RTL8188EU_H

#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/mutex.h>
#include <net/mac80211.h>
#include "rtl8188eu_phy.h"

/* RX URB management */
#define RX_URB_COUNT 4
#define RX_URB_SIZE 32768

struct rtl8188eu_rx_urb {
	struct urb *urb;
	u8 *buffer;
	struct rtl8188eu_priv *priv;
};

/* Hardware configuration for conditional table parsing */
struct hw_config {
	u8 interface_type;	/* 0: PCI, 1: USB, 2: SDIO */
	u8 board_type;		/* Board type identifier */
	u8 cut_version;		/* Chip cut version */
	u8 package_type;	/* Package type */
};

/* MSR (Media Status Register) network type values */
#define MSR_NOLINK		0x00
#define MSR_ADHOC		0x01
#define MSR_INFRA		0x02
#define MSR_AP			0x03

/* Main driver private structure */
struct rtl8188eu_priv {
	struct usb_device *udev;
	struct usb_interface *intf;
	const struct firmware *fw;
	bool chip_powered_on;
	u8 out_ep_queue_sel;		/* USB endpoint queue selection */
	u8 out_ep_number;		/* Number of OUT endpoints */

	/* mac80211 interface */
	struct ieee80211_hw *hw;	/* mac80211 hardware struct */
	struct ieee80211_vif *vif;	/* Current virtual interface */

	/* USB endpoints */
	u8 rx_endpoint;			/* USB IN endpoint for RX */
	u8 tx_endpoint;			/* USB OUT endpoint for TX */
	u8 mac_addr[ETH_ALEN];		/* MAC address */

	/* RX URBs */
	struct rtl8188eu_rx_urb rx_urbs[RX_URB_COUNT];

	/* PHY layer */
	struct bb_register_definition phy_reg_def[2];
	struct mutex rf_read_mutex;

	/* Hardware configuration for PHY table parsing */
	struct hw_config hw_cfg;

	/* H2C command interface tracking */
	u8 last_hmebox_num;

	/* Radio state */
	u8 channel;			/* Current WiFi channel (1-14) */
	u32 rf_chnl_val;		/* Cached RF_CHNLBW register value */

	/* Bandwidth state */
	enum nl80211_chan_width current_bw;
	u8 sec_ch_offset;		/* Secondary channel offset for HT40 */

	/* Device state */
	bool started;			/* HW started via mac80211 start() */
	bool disconnecting;		/* Set during rmmod/disconnect */
	unsigned int rx_filter;		/* Current FIF_* filter flags */

	/* Scan diagnostics */
	bool scanning;
	unsigned int scan_tx_count;
	unsigned int scan_rx_count;
	unsigned int scan_rx_drop_crc;
	unsigned int scan_rx_drop_len;
	unsigned int scan_urb_count;
};

#endif /* RTL8188EU_H */
