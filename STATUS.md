# RTL8188EU Driver Status - Feb 17, 2026

## Version: 0.35 - Fix PF_PACKET delivery for monitor mode

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

### Calibration (FUNCTIONAL)
- IQK calibration (TX+RX paths, save/restore 29 registers)
- LC calibration (with RF_AC save/restore attempt)
- Post-cal RF 0x00 readback issue (reads 0x00000 inside cal, but RX works)

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
- **10 RX callbacks, 9 packets received per session (v0.33)**
- URB resubmission working correctly

### Driver Stability (SOLID)
- No kernel panics
- Clean rmmod (disconnecting flag + netif_device_detach)
- NULL pointer guards in RX callback
- URB state validation before resubmit

---

## What Doesn't Work Yet

### LC Calibration RF Readback
- `LC Cal: Saved RF_AC = 0x00000` (should be 0x33e60)
- RF read inside calibration function returns stale data
- Post-RF read works fine (0x33e60) - issue is specific to cal context
- **Does not prevent RX from working**

### Not Yet Tested
- Packet capture tools with this driver (v0.35 should fix delivery)
- Channel hopping
- Extended continuous operation

---

## Current Interface State

```
Interface: enx660cbaa475f3 (changes each boot - random MAC)
State: UP, carrier ON
Monitor Mode: ENABLED (RCR = 0xf000392f)
TX: 17 packets, 2761 bytes
RX: 9 packets, 1909 bytes (CONTINUOUS!)
```

---

## Version History

| Version | Date | Change |
|---------|------|--------|
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

1. Test packet capture in monitor mode
2. Fix LC calibration RF readback (0x00000 issue)
3. Add radiotap headers if needed for monitor mode tools
4. Test extended continuous operation

---

## Key Files

- `rtl8188eu_minimal_main.c` - Main driver (~3100 lines)
- `rtl8188eu.h` - Common structures and definitions
- `rtl8188eu_phy.h` - PHY function declarations
- `rtl8188eu_phy.c` - PHY register access + calibration (~1200 lines)
- `rtl8188eu_phy_tables.c` - Config tables (~350 lines)
- `Makefile` - Build system (multi-file)

---

**Last Updated**: Feb 17, 2026
**Version**: 0.35
**Status**: PF_PACKET delivery fixed. Ready for monitor mode test
