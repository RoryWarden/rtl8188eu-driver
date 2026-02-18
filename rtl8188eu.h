#ifndef RTL8188EU_H
#define RTL8188EU_H

#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/mutex.h>
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

/* Main driver private structure */
struct rtl8188eu_priv {
	struct usb_device *udev;
	struct usb_interface *intf;
	const struct firmware *fw;
	bool chip_powered_on;
	u8 out_ep_queue_sel;		/* USB endpoint queue selection */
	u8 out_ep_number;			/* Number of OUT endpoints */

	/* Phase 2: Network interface */
	struct net_device *netdev;	/* Network device */
	u8 rx_endpoint;				/* USB IN endpoint for RX */
	u8 tx_endpoint;				/* USB OUT endpoint for TX */
	u8 mac_addr[ETH_ALEN];		/* MAC address */

	/* Phase 2, Step 5: RX URBs */
	struct rtl8188eu_rx_urb rx_urbs[RX_URB_COUNT];

	/* Phase 1: PHY layer */
	struct bb_register_definition phy_reg_def[2];  /* Path A and B (only A used) */
	struct mutex rf_read_mutex;  /* Protect RF read operations */

	/* Hardware configuration for PHY table parsing */
	struct hw_config hw_cfg;

	/* H2C command interface tracking */
	u8 last_hmebox_num;  /* Last H2C mailbox number used (0-3) */

	/* Radio state */
	u8 channel;  /* Current WiFi channel (1-14) */

	/* Device state */
	bool disconnecting;  /* Set during rmmod/disconnect to stop TX/RX */
};

#endif /* RTL8188EU_H */
