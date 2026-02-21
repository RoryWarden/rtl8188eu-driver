# RTL8188EU Driver Status - Feb 20, 2026

## Version: 1.0.0 - mac80211 conversion (managed + monitor mode)

---

## What Works

### Hardware Initialization (COMPLETE)
- USB device detection (2357:010c)
- Power sequence (CARDEMU->ACT transition)
- Firmware download (15,262 bytes, checksum verified, WINTINI_RDY confirmed)
- MCU clocks, H2C init, 8051 reset
- Register configuration (power on, interrupts)
- TX/RX queue setup (167 pages)

### PHY Layer (COMPLETE)
- BB register read/write (direct 32-bit USB access)
- RF register read/write (3-wire LSSI serial, 100us readback delay)
- PHY_REG table loaded (conditional parsing for USB interface)
- AGC table loaded (258 entries after filtering PCI/SDIO-only entries)
- RadioA table loaded (RF registers 0x00-0xFF)
- Post-PHY BB block enable (CCK/OFDM)
- RF environment setup (BB 0x860, 0x870)

### Calibration (COMPLETE)
- IQK calibration (TX+RX paths, save/restore 29 registers)
- IQK results applied to compensation registers (0xC80/0xC94/0xC4C TX, 0xC14/0xCA0 RX)
- LC calibration (packet TX mode — pauses queues, doesn't touch RF_AC)
- Post-cal RF 0x00 reads correctly (0x33e60) — address mask bug fixed in v0.36

### Network Interface (COMPLETE)
- USB endpoints detected (RX: 0x81, TX: 0x02)
- Network device allocated and registered
- Interface comes up (enx* naming, random MAC each boot)
- Monitor mode RCR configured (0xf000392f)
- RXFLTMAP2 = 0xFFFF (accept all frame types)

### TX Path (WORKING)
- 32-byte TX descriptor prepending
- 17+ packets per session
- URB_FREE_BUFFER for cleanup (no double-free)
- Disconnect-safe (drops packets when disconnecting)

### RX Path (WORKING - CONTINUOUS!)
- 24-byte RX descriptor parsing
- 4 URBs active for RX
- USB aggregation configured (RXDMA_AGG_PG_TH=0x30, timeout=0x04)
- RXDMA_AGG_EN enabled in TRXDMA_CTRL
- RX DMA enabled in ndo_open, disabled in ndo_stop
- URB resubmission working correctly
- **Monitor mode capture with real radiotap data — actual rates and signal strength (v0.37)**
- **Fixed signal strength: use path AGC gain (byte 0) instead of stuck CCK AGC (byte 5) (v0.38)**

### Driver Stability (SOLID)
- No kernel panics
- Clean rmmod (disconnecting flag + netif_device_detach)
- NULL pointer guards in RX callback
- URB state validation before resubmit

---

### mac80211 Integration (COMPLETE - v1.0.0)
- ieee80211_alloc_hw / ieee80211_register_hw
- ieee80211_ops: tx, start, stop, add/remove_interface, config, configure_filter
- bss_info_changed: BSSID, association, beacon interval, bandwidth
- sw_scan_start/complete for software scanning
- set_key returns -EOPNOTSUPP (software crypto via mac80211)
- Channel context emulation (ieee80211_emulate_*_chanctx)
- minstrel_ht rate control algorithm
- HT20/HT40 bandwidth switching (MAC + BB + RF)

## What Doesn't Work Yet

### Not Yet Tested
- AP association via wpa_supplicant
- WPA2 authentication (software crypto should work)
- OFDM/HT rate reception (still only CCK 1.0 Mb/s)

---

## Current Interface State

```
Interface: wlx* (changes each boot - random MAC)
Modes: managed + monitor (via iw)
mac80211: phy2, minstrel_ht rate control
Monitor: tcpdump capture with radiotap headers
Managed: iw scan finds APs with full BSS details
HT caps: HT20/HT40, SGI, MCS 0-7
```

---

## Version History

| Version | Date | Change |
|---------|------|--------|
| v1.0.0 | Feb 20 | **mac80211 conversion — managed + monitor mode + scanning** |
| v0.39.1| Feb 19 | Fix crystal cap (CX_IN/CX_OUT mismatch → OFDM broken) |
| v0.39 | Feb 19 | RF bandwidth (20MHz) + BWOPMODE init |
| v0.38 | Feb 19 | Fix signal strength (byte 0 AGC) + chip version detection |
| v0.37 | Feb 18 | Real radiotap data + channel hopping fix + debug cleanup |
| v0.36 | Feb 18 | Fix RF read address mask (bLSSIReadAddress) — **MONITOR MODE WORKING!** |
| v0.35 | Feb 17 | Fix PF_PACKET delivery (skb_reset_mac_header + radiotap header_ops) |
| v0.34 | Feb 17 | Wireless extensions + radiotap headers for monitor mode |
| v0.33 | Feb 17 | Fix RF readback timing 10us->100us — **CONTINUOUS RX!** |
| v0.32 | Feb 16 | Fix calibration RF state (LC cal + IQK save/restore) |
| v0.31 | Feb 16 | Fix RF environment setup + RF diagnostics |
| v0.30 | Feb 16 | Fix reg 0x800/0xEE8 overwrite + RX diagnostic dump |
| v0.29 | Feb 16 | RW_RELEASE_EN + RX DMA at power-on + RCR data frames |
| v0.28 | Feb 16 | MAC table + queue priority + init reorder |
| v0.25 | Feb 16 | Fix rmmod crash (TX double-free + disconnect race) |
| v0.24 | Feb 14 | **RX WORKING!** RXDMA_AGG_PG_TH=0x30, first packet! |
| v0.23 | Feb 14 | USB config fix (TRXDMA_CTRL + RXDMA_AGG_EN) |
| v0.22 | Feb 14 | Fix kernel panic (NULL checks + RX DMA timing) |
| v0.21 | Feb 13 | Post-firmware init (RCR, USB AGG, RX DMA) |
| v0.20 | Feb 13 | MCUFWDL cleared — firmware starts, WINTINI_RDY! |
| v0.19 | Feb 13 | H2C timing fix (before firmware activation) |
| v0.18 | Feb 13 | H2C initialization (REG_HMETFR) |
| v0.17 | Feb 12 | MCU clocks + IO Wrapper |
| v0.16 | Feb 12 | Firmware fixes (checksum, interface UP) |
| v0.15 | Feb 12 | Power sequence (CARDEMU->ACT) — 0xEAEA solved! |
| v0.14 | Feb 11 | Conditional table parsing — device stable! |
| v0.13 | Feb 11 | Root cause: conditional parsing missing |
| v0.12 | Feb 11 | Progressive PHY testing |
| v0.11 | Feb 10 | PHY layer code complete |
| v0.10 | Feb 10 | Channel tuning, monitor mode |

---

## Next Steps

1. Test AP association via wpa_supplicant (WPA2-PSK)
2. Debug OFDM/HT rate capture (still only CCK 1.0 Mb/s)
3. Test packet injection in monitor mode

---

## Key Files

- `rtl8188eu_minimal_main.c` - Main driver (~2400 lines, mac80211)
- `rtl8188eu.h` - Common structures and definitions
- `rtl8188eu_phy.h` - PHY function declarations
- `rtl8188eu_phy.c` - PHY register access + calibration (~1200 lines)
- `rtl8188eu_phy_tables.c` - Config tables (~350 lines)
- `Makefile` - Build system (multi-file)

---

**Last Updated**: Feb 20, 2026
**Version**: 1.0.0
**Status**: mac80211 conversion — managed + monitor mode working
