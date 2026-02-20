# RTL8188EU Minimal USB WiFi Driver

> **This driver is under active development and is not ready for general use.**
> It may crash your kernel, disconnect your USB device, or behave unpredictably.
> Do not use this on a production system.

## What Works

- USB device detection and firmware download
- Network interface creation (monitor mode only)
- TX path (packet transmission)
- Continuous RX (packet reception — 10+ packets/session)
- PHY/RF initialization with IQK calibration
- Clean module load/unload (no kernel panics)

## What Doesn't Work Yet

- Station mode (no connecting to WiFi networks)
- Channel hopping (fixed to channel 6)
- Packet capture with monitor mode tools
- Hardware encryption
- Power management

## Overview

A from-scratch Linux kernel driver for the Realtek RTL8188EU USB WiFi adapter (TP-Link TL-WN722N v2/v3). Implements monitor mode with continuous packet reception.

**Version:** v0.35 (Feb 18, 2026)
**Target:** USB ID 2357:010c, Kernel 6.17+
**Status:** Under development — continuous RX working, monitor mode packet capture not yet verified

## Driver Architecture

### Source Files

| File | Lines | Purpose |
|------|-------|---------|
| `rtl8188eu_minimal_main.c` | ~3100 | Main driver: USB, netdev, init, TX/RX |
| `rtl8188eu_phy.c` | ~1200 | PHY register access, calibration (IQK/LC) |
| `rtl8188eu_phy.h` | | PHY function declarations |
| `rtl8188eu_phy_tables.c` | ~350 | BB/AGC/RF config tables |
| `rtl8188eu.h` | | Common definitions and structures |
| `Makefile` | | Kernel module build system |
| `rtl8188eu_8188e_fw.bin` | 15KB | Firmware binary |

### Core Components

**USB Interface** — Probe/disconnect, bulk endpoints (IN 0x81, OUT 0x02), URB management, control transfers for register access.

**Network Interface** — Standard netdev with TX/RX paths, statistics, interface lifecycle. Monitor mode auto-enabled on open.

**PHY/RF Layer** — BB register read/write (direct USB), RF register read/write (3-wire LSSI serial with 100us readback), IQK and LC calibration.

**Configuration Tables** — PHY_REG (192 pairs), AGC_TAB (276 entries filtered from 552), RadioA (95 pairs). Conditional parsing skips PCI/SDIO-only entries.

---

## Hardware Register Reference

### Power Control
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_SYS_FUNC_EN | 0x0002 | 16-bit | System function enable (BB, RF) |
| REG_APS_FSMCO | 0x0004 | 32-bit | Power state machine (CARDEMU/ACT) |
| REG_AFE_XTAL_CTRL | 0x0024 | 32-bit | AFE crystal clock control |
| REG_RSV_CTRL | 0x001C | 8-bit | Reserved control (8051 MCU enable) |
| REG_RF_CTRL | 0x001F | 8-bit | RF_EN, RF_RSTB, RF_SDMRSTB |
| REG_SYS_CFG | 0x00F0 | 32-bit | System configuration |

### Command and Control
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_CR | 0x0100 | 16-bit | Command register — HCI_TXDMA, HCI_RXDMA, TXDMA, RXDMA, PROTOCOL, SCHEDULE, SEC, CALTMR |
| MSR | 0x0102 | 8-bit | Media Status (No Link / STA / Ad-Hoc / AP) |

### Firmware Control
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_MCUFWDL | 0x0080 | 32-bit | MCU firmware download control; +2 = page select |
| FW_START_ADDRESS | 0x1000 | — | Firmware RAM start (15,262 bytes in 4KB pages) |
| REG_HMETFR | 0x01CC | 8-bit | H2C initialization (write 0x0F before FW activation) |
| REG_WINTINI_RDY | 0x01EC | 8-bit | Firmware ready signal (~2ms after activation) |

### Interrupts
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_HIMR_88E | 0x00B0 | 32-bit | Interrupt mask (IMR_PSTIMEOUT, IMR_TBDER, IMR_CPWM) |
| REG_HISR_88E | 0x00B4 | 32-bit | Interrupt status (write 0xFFFFFFFF to clear) |

### TX/RX Queue Configuration
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_PBP | 0x0104 | 8-bit | Page size: bits[3:0]=RX, bits[7:4]=TX (0x11 = 128B each) |
| REG_TRXDMA_CTRL | 0x010C | 16-bit | TX/RX DMA control (RXDMA_AGG_EN = BIT2) |
| REG_TRXFF_BNDY | 0x0114 | 16-bit | TX/RX FIFO boundary (page 175 = 0x00AF) |
| REG_RQPN | 0x0200 | 32-bit | Queue pages: [7:0]=HPQ, [15:8]=LPQ, [23:16]=PUBQ, BIT31=LD_RQPN |
| REG_TDECTRL+1 | 0x0209 | 8-bit | TX buffer boundary (page 168) |
| REG_RQPN_NPQ | 0x0214 | 8-bit | Normal Priority Queue pages |
| REG_RXDMA_AGG_PG_TH | 0x0280 | 32-bit | RX DMA aggregation threshold (MUST be 0x30) |
| REG_BCNQ_BDNY | 0x0424 | 8-bit | Beacon queue boundary |
| REG_MGQ_BDNY | 0x0425 | 8-bit | Management queue boundary |
| REG_WMAC_LBK_BF_HD | 0x045D | 8-bit | WMAC loopback buffer head |

**Queue page allocation (non-WMM):** HPQ=0, LPQ=9, NPQ=0, PUBQ=158, Total=167 pages. TX boundary=page 168. Total chip pages=175 (0xB0-1), beacon=8 pages.

### USB Specific
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_USB_SPECIAL_OPTION | 0xFE55 | 8-bit | USB bulk transfer config |
| REG_USB_DMA_AGG_TO | 0xFE5B | 8-bit | USB DMA aggregation timeout (0x04) |

### Receive Configuration (Monitor Mode)
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_RCR | 0x0608 | 32-bit | Receive Configuration Register |
| REG_MAR | 0x0620 | 64-bit | Multicast filter (0xFFFFFFFF x2 for monitor) |
| REG_RXFLTMAP2 | 0x06A4 | 16-bit | RX filter map (0xFFFF = all frame types) |
| REG_WMAC_TRXPTCL_CTL | 0x0668 | 32-bit | TX/RX protocol control (ACK disable) |

**RCR bit definitions:**
| Bit | Name | Purpose |
|-----|------|---------|
| 0 | RCR_AAP | Accept All unicast Packets (THE monitor mode bit) |
| 1 | RCR_APM | Accept Physical Match |
| 2 | RCR_AM | Accept Multicast |
| 3 | RCR_AB | Accept Broadcast |
| 4 | RCR_ADD3 | Accept address 3 match |
| 5 | RCR_APWRMGT | Accept power management |
| 6 | RCR_CBSSID_DATA | Check BSSID match (Data) — remove for monitor |
| 7 | RCR_CBSSID_BCN | Check BSSID match (Beacon) — remove for monitor |
| 8 | RCR_ACRC32 | Accept CRC32 errors |
| 9 | RCR_AICV | Accept ICV errors |
| 12 | RCR_ACF | Accept Control Frames |
| 13 | RCR_AMF | Accept Management Frames |
| 14 | RCR_HTC_LOC_CTRL | HTC Location Control |
| 28 | RCR_APP_PHYST_RXFF | Append PHY status to RX |
| 29 | RCR_APP_ICV | Retain ICV |
| 30 | RCR_APP_MIC | Retain MIC |

**Monitor mode RCR:** 0xF000392F (AAP+APM+AM+AB+ACF+AMF+ACRC32+APP_PHYST+APP_ICV+APP_MIC)

### Bluetooth Coexistence
| Register | Address | Size | Purpose |
|----------|---------|------|---------|
| REG_GPIO_MUXCFG | 0x0040 | 8-bit | GPIO config (0xA0 for BT coex) |
| REG_BT_COEX_TABLE | 0x06C0 | 4x32-bit | BT coex params (+4=0xAAAA9AAA, +8=0xFFBD0040, +C=0x40000010) |

### Baseband Registers (0x800-0xFFF)
| Register | Address | Purpose |
|----------|---------|---------|
| rFPGA0_RFMOD | 0x0800 | RF mode control |
| rFPGA0_XA_HSSIParameter1 | 0x0820 | HSSI param 1 (PI mode flag at BIT8) |
| rFPGA0_XA_HSSIParameter2 | 0x0824 | RF serial interface control (read trigger) |
| rFPGA0_XA_LSSIParameter | 0x0840 | RF write data (3-wire serial) |
| rFPGA0_XA_RFInterfaceOE | 0x0860 | RF output enable / RF environment |
| rFPGA0_XAB_RFInterfaceSW | 0x0870 | RF software control / environment |
| rOFDM0_AGCCore1 | 0x0C78 | AGC table write target |
| rOFDM0_TRxPathEnable | 0x0C04 | OFDM TX/RX path enable (1T1R) |
| rOFDM1_TRxPathEnable | 0x0D04 | OFDM path enable (secondary) |
| rFPGA0_XA_LSSIReadBack | 0x08A0 | RF read data (SI mode) |
| TransceiverA_HSPI_Readback | 0x08B8 | RF read data (PI mode) |

### IQK Calibration & Compensation Registers
| Register | Address | Purpose |
|----------|---------|---------|
| TX_POWER_BEFORE_IQK_A | 0x0E94 | TX IQK result X (bits [25:16] = 10-bit signed) |
| TX_POWER_AFTER_IQK_A | 0x0E9C | TX IQK result Y (bits [25:16] = 10-bit signed) |
| RX_POWER_BEFORE_IQK_A_2 | 0x0EA4 | RX IQK result X (bits [25:16] = 10-bit signed) |
| RX_POWER_AFTER_IQK_A_2 | 0x0EAC | RX IQK result Y (bits [25:16] = 10-bit signed) |
| OFDM_XA_TX_IQ_IMBALANCE | 0x0C80 | TX I/Q compensation (bits [9:0] = TX_A, [21:16] = TX_C low) |
| OFDM_XC_TX_AFE | 0x0C94 | TX AFE correction (bits [31:28] = TX_C high) |
| OFDM_ECCA_THRESHOLD | 0x0C4C | Extra compensation bits (BIT31 = TX_A overflow, BIT29 = TX_C overflow) |
| OFDM_XA_RX_IQ_IMBALANCE | 0x0C14 | RX I/Q compensation (bits [9:0] = RX_X, [15:10] = RX_Y low) — CRITICAL for OFDM RX |
| OFDM_RX_IQ_EXT_ANTA | 0x0CA0 | RX IQ extended (bits [31:28] = RX_Y high) |
| rFPGA1_RFMOD | 0x0900 | RF mode 2 (bit 0 = 0 for 20MHz BW) |

**IQK result application (from `_phy_path_a_fill_iqk_matrix`):** TX results are scaled by the current TX IQ base value (0xC80 bits [31:22]) before writing. RX results are written directly. Without applying IQK results to compensation registers, OFDM demodulation fails due to I/Q image interference — CCK uses DSSS which is robust to I/Q mismatch.

### RF Registers (0x00-0xFF via 3-wire serial)
| Register | Address | Purpose |
|----------|---------|---------|
| RF_AC | 0x00 | RF mode/enable (expect 0x33E60 when working) |
| RF_IPA | 0x0C | Internal PA control |
| RF_TXBIAS | 0x0D | TX bias |
| RF_CHNLBW | 0x18 | Channel and bandwidth control |
| RF_RCK | 0x1D | RC calibration |
| RF_LCK | 0x1E | LC calibration |
| RF_TXPA_G1/G2 | 0x1E-0x1F | TX power amp gain |
| RF_WE_LUT | 0xEF | LUT write enable |

**RF_CHNLBW (0x18) bit fields:**
- Bits [9:0]: Channel number (1-14)
- Bits [11:10]: Bandwidth (0xC00 = 20MHz, 0x400 = 40MHz)
- Bits [19:12]: Calibration/factory data (MUST preserve on channel switch)
- Channel mask: `(val & 0xFFFFFC00) | new_channel`

---

## RF 3-Wire Serial Interface

RF registers are NOT directly accessible. They use a baseband-mediated 3-wire serial protocol:

**Write:** Encode `((rf_addr << 20) | (data & 0xFFFFF)) & 0x0FFFFFFF`, write to rFPGA0_XA_LSSIParameter (0x0840).

**Read:** Set address in rFPGA0_XA_HSSIParameter2 (0x0824), trigger read edge, wait **100us** (critical — 10us is insufficient), read from 0x08A0 (SI mode) or 0x08B8 (PI mode).

---

## Initialization Sequence

### 1. Power-On (CARDEMU -> ACT)
1. Enable AFE clock (REG_AFE_XTAL_CTRL)
2. Transition from CARDEMU to ACT state
3. Enable system functions (FEN_USBA, FEN_USBD, FEN_BB_GLB_RSTn)
4. Initialize 8051 MCU (RSV_CTRL manipulation)

### 2. Firmware Download
1. Enable firmware download mode (REG_MCUFWDL)
2. Download 15,262 bytes in 4KB pages to 0x1000
3. Clear RAM_DL_SEL, initialize H2C (REG_HMETFR = 0x0F)
4. Activate firmware, wait for WINTINI_RDY (~2ms)

### 3. PHY Initialization
1. Enable BB/RF clocks (REG_SYS_FUNC_EN, REG_RF_CTRL)
2. Load PHY_REG table (BB registers, conditional parsing for USB)
3. Load AGC_TAB table (258 entries after filtering)
4. Configure RF environment (BB 0x860, 0x870)
5. Load RadioA table (RF 0x00-0xFF via 3-wire serial)
6. Enable CCK/OFDM blocks
7. Run IQK calibration (TX+RX, save/restore 29 registers)
8. Run LC calibration (save/restore RF_AC)

### 4. MAC Configuration
1. Set RCR for monitor mode (0xF000392F)
2. Enable MAC TX/RX (REG_CR)
3. Configure TX queues and boundaries
4. Enable interrupts (HIMR = 0x503)
5. Set RXFLTMAP2 = 0xFFFF

### 5. USB Aggregation (Critical for RX!)
1. RXDMA_AGG_PG_TH = 0x30 (48 pages) — wrong value breaks RX completely
2. RXDMA_AGG timeout = 0x04
3. Enable RXDMA_AGG_EN in TRXDMA_CTRL
4. Configure USB_SPECIAL_OPTION

---

## Packet Descriptors

### TX Descriptor (32 bytes, prepended to every TX packet)
```c
struct rtl8188eu_tx_desc {
    __le32 txdw0;  // Bits[15:0]=pkt size, Bits[23:16]=offset(32), BIT31=OWN
    __le32 txdw1;  // Bits[4:0]=MAC ID, Bits[12:8]=queue (0=BE, 0x12=MGNT)
    __le32 txdw2;  // Aggregation, break
    __le32 txdw3;  // Sequence number
    __le32 txdw4;  // RTS rate, data rate
    __le32 txdw5;  // Data rate fallback, GI
    __le32 txdw6;  // Checksum offload
    __le32 txdw7;  // USB aggregation
};
```

### RX Descriptor (24 bytes, prepended by hardware to every RX packet)
```c
struct rtl8188eu_rx_desc {
    __le32 rxdw0;  // Bits[13:0]=pkt length, BIT14=CRC32 err, BIT15=ICV err
    __le32 rxdw1;  // PHY status, rate info
    __le32 rxdw2;  // Sequence number
    __le32 rxdw3;  // Fragment info
    __le32 rxdw4;  // QoS info
    __le32 rxdw5;  // Timestamp
};
```

---

## Building and Loading

**Build:**
```bash
cd /home/matthew/rtl8188eu_new
make clean && make
```

**Load (sudo):**
```bash
sudo rmmod rtl8xxxu
sudo rmmod rtl8188eu_minimal
sudo insmod ./rtl8188eu_minimal.ko
```

**Check:**
```bash
dmesg | tail -80
ip link show | grep enx
ip -s link show <interface>
```

---

## Critical Timing Requirements
1. **Firmware Ready:** Wait for WINTINI_RDY after activation (~2ms)
2. **RF Readback:** 100us delay after LSSI read edge trigger (10us is too short!)
3. **PHY Stabilization:** 50ms delay after PHY init
4. **Channel Switch:** 3 attempts with 10ms delays
5. **URB Submission:** Must use GFP_ATOMIC in callback context

## Key Discoveries
1. **Conditional Tables:** PHY tables contain IF/ELSE blocks — loading PCI/SDIO entries crashes USB device
2. **Power Sequence:** CARDEMU->ACT transition required before any register access (0xEAEAEAEA = unpowered)
3. **H2C Timing:** Must initialize H2C BEFORE firmware activation
4. **RXDMA_AGG_PG_TH:** Must be 0x30 (48 pages), not 0x05 — wrong value = no RX
5. **RF Readback Delay:** 100us required, 10us returns stale data
6. **URB_FREE_BUFFER:** Cannot manually kfree() a buffer that has this flag — double-free panic
7. **rmmod Race:** Must set disconnecting flag + netif_device_detach() before cleanup

## Known Limitations
1. **Monitor Mode Only** — No station/AP mode (no mac80211/cfg80211)
2. **Fixed Channel 6** — No channel hopping
3. **No Encryption** — Hardware crypto not configured
4. **No Radiotap Headers** — monitor mode tools need these
5. **No Rate Control** — Fixed data rate
6. **No Power Save** — Always active
7. **LC Cal Readback** — RF reads 0x00000 inside calibration (doesn't affect RX)

## Development History

| Version | Date | Change |
|---------|------|--------|
| v0.40.1 | Feb 19 | Fix IQK result application to compensation registers (OFDM RX) |
| v0.40 | Feb 19 | DC offset cancellation attempt (removed — not supported on 8188E) |
| v0.39.1 | Feb 19 | Fix crystal cap (CX_IN/CX_OUT match) |
| v0.39 | Feb 19 | RF bandwidth (20MHz) + BWOPMODE init |
| v0.38 | Feb 19 | Fix signal strength (byte 0 AGC) + chip version detection |
| v0.37.1 | Feb 18 | Fix radiotap MCS flag + signal diagnostics |
| v0.37 | Feb 18 | Real radiotap data + channel hopping + debug cleanup |
| v0.36 | Feb 18 | Fix RF read address mask (bLSSIReadAddress) |
| v0.35 | Feb 18 | Fix PF_PACKET delivery (skb_reset_mac_header) |
| v0.34 | Feb 18 | Wireless extensions + radiotap headers for monitor mode |
| v0.33 | Feb 17 | Fix RF readback timing 10us->100us — **CONTINUOUS RX!** |
| v0.32 | Feb 16 | Fix calibration RF state (LC cal + IQK save/restore) |
| v0.31 | Feb 16 | Fix RF environment setup + RF diagnostics |
| v0.30 | Feb 16 | Fix reg 0x800/0xEE8 overwrite + RX diagnostic dump |
| v0.29 | Feb 16 | RW_RELEASE_EN + RX DMA at power-on + RCR data frames |
| v0.28 | Feb 16 | MAC table + queue priority + init reorder |
| v0.25 | Feb 16 | Fix rmmod crash (TX double-free + disconnect race) |
| v0.24 | Feb 14 | **First RX packet!** RXDMA_AGG_PG_TH=0x30 |
| v0.22 | Feb 14 | Fix kernel panic (NULL checks + RX DMA timing) |
| v0.20 | Feb 13 | Firmware starts — WINTINI_RDY! |
| v0.15 | Feb 12 | Power sequence — 0xEAEA solved! |
| v0.14 | Feb 11 | Conditional table parsing — device stable! |
| v0.11 | Feb 10 | PHY layer implementation complete |
| v0.1-0.8 | Feb 9 | USB, firmware, netdev, monitor mode RCR |

## References
- Old driver source: `/home/matthew/rtl8188eus/`
- Realtek vendor driver (GPL release)
- Linux kernel USB subsystem documentation
- IEEE 802.11 specifications
