# RTL8188EU Driver - Quick Start & Progress

**Last Updated:** Feb 17, 2026
**Current Version:** v0.35 (Fix packet delivery to PF_PACKET sockets for monitor mode)
**Previous Version:** v0.34 (Add wireless extensions + radiotap headers for monitor mode)

---

## Quick Resume Point

**What works:**
- ✅ USB detection, firmware download
- ✅ Network interface with TX/RX paths
- ✅ Monitor mode RCR configured
- ✅ TX: 17+ packets working
- ✅ **Continuous RX: 10 callbacks, 9 packets received!**
- ✅ PHY layer + calibration complete

**Current Status (Feb 17, 2026 - v0.35 READY TO TEST):**
- ✅ **v0.34:** Add wireless extensions + radiotap headers for monitor mode
- ✅ **v0.35:** Fix packet delivery to PF_PACKET sockets for monitor mode
  - Added `skb_reset_mac_header()` before `netif_rx()` — fixes uninitialized mac_header
  - Cleared `hard_header_len` and `header_ops` for radiotap device (not ethernet)
  - Without these, PF_PACKET's `packet_rcv()` silently drops all frames
- ⏳ **NEXT:** Test packet capture in monitor mode

**Key Finding:** Conditional table parsing was the solution! AGC table had 266 PCI/SDIO-only entries!

---

## Progress Tracker

### Phase 0: Documentation ✅ COMPLETE
- [x] PLAN.md created - Complete implementation plan
- [x] START.md created - This file
- [x] CLAUDE.md updated - Guidelines for Claude Code
- [x] Old driver research completed via plan mode

### Phase 1: Core Helper Functions ✅ COMPLETE (Code ready, testing deferred)
- [x] Create rtl8188eu_phy.h (header file)
- [x] Create rtl8188eu_phy.c (implementation)
- [x] Implement BB register read/write
- [x] Implement RF register read/write (3-wire serial)
- [x] Implement bit shift calculator
- [x] Implement register definition init
- [x] Add PHY structures to rtl8188eu_priv
- [x] Add Phase 1 test code to probe
- [x] Update Makefile for multi-file build
- [x] Created rtl8188eu.h common header
- [x] Build successful with multi-file structure
- [x] Found: 25.1MHz clock (BIT13) needed for BB/RF
- [ ] **DEFERRED:** Test BB/RF access (needs Phase 2/3 tables first!)

### Phase 2: Configuration Tables ✅ COMPLETE
- [x] Create rtl8188eu_phy_tables.c
- [x] Extract PHY_REG table (384 values, 192 pairs)
- [x] Extract AGC_TAB table (552 values, 276 pairs)
- [x] Extract RadioA table (190 values, 95 pairs)

### Phase 3: Table Loaders ✅ COMPLETE
- [x] Implement PHY_REG loader
- [x] Implement AGC loader
- [x] Implement RadioA loader
- [x] Handle delay codes (0xFFE, 0xFFD, etc.)

### Phase 4: PHY Init Functions ✅ COMPLETE
- [x] Implement rtl8188eu_phy_bb_config()
- [x] Implement rtl8188eu_phy_rf_config()
- [ ] Implement rtl8188eu_phy_set_tx_power() (deferred - not needed for RX)
- [x] Build successful with all functions

### Phase 5: Integration ✅ COMPLETE
- [x] Add PHY init to probe sequence
- [x] Remove old test code and replace with proper init
- [x] Makefile already updated (Phase 2)
- [x] Build successful - no compilation errors!

### Phase 6: Testing ⚠️ BLOCKED
- [x] Load driver with PHY init - **CAUSES USB DISCONNECT**
- [x] Verify BB/RF register counts in dmesg - **Partial: BB loads, RF crashes**
- [x] Check interface comes up - **Yes, but without PHY**
- [ ] **Verify RX counter increments!** - **BLOCKED: PHY can't initialize**
- [ ] Test channel switching - **BLOCKED**
- [ ] Test packet capture in monitor mode - **BLOCKED**

### Phase 7: PHY Initialization Debug ✅ ROOT CAUSE FOUND
- [x] Stage 1: Read-only BB/RF register tests - Device stable!
- [x] Stage 2: BB/RF enable with AFE init - Still shows 0xeaeaeaea
- [x] Stage 3: Loaded 10 PHY_REG entries - Device stable, BB still 0xeaeaeaea
- [x] Stage 4: Loaded 50 PHY_REG entries - Device stable, BB still 0xeaeaeaea
- [x] Stage 5: Loaded ALL tables (559 registers) - USB disconnect after completion
- [x] Stage 6: **ROOT CAUSE IDENTIFIED** - Conditional table parsing missing!
- [x] Stage 7: Missing post-PHY init steps found (BB enable, MAC TX/RX, USB aggregation)

### Phase 8: Fix Implementation ✅ COMPLETE!
- [x] Implement conditional table parsing (IF/ELSE blocks in PHY tables)
- [x] Add hardware config structure for USB interface detection
- [x] Add all delay commands (0xF9-0xFE) support
- [x] Add post-PHY BB block enable (CCK/OFDM)
- [x] Add MAC TX/RX enable after PHY
- [x] Add USB aggregation initialization
- [x] Test fixed implementation - **DEVICE STAYS CONNECTED!**

### Phase 9: RX Testing ✅ PARTIAL SUCCESS
- [x] Re-enable network registration
- [x] Build and load driver with network support
- [x] Check interface comes up - **enxb2021e9f85f3 active!**
- [x] TX works - **35+ packets sent**
- [ ] RX not working - **0 packets received**
- [ ] Test packet capture - **Blocked by missing RX**

### Phase 10: Fix RX Reception ✅ PARTIAL
- [x] Fix AGC table loading - **258 loaded successfully!**
- [x] Add post-PHY RF configuration (BB 0x40, 0xEE8, 0x87C)
- [x] Add RX debugging to trace packet flow
- [ ] **No URB callbacks - hardware not sending data**
- [ ] Implement IQK/LCK calibration - **CRITICAL FOR RX**

### Phase 19: Wireless Extensions + Radiotap (v0.34) ✅ COMPLETE
- [x] Add wireless extension handlers (SIOCGIWNAME, SIOCSIWMODE, SIOCGIWMODE, SIOCSIWFREQ, SIOCGIWFREQ)
- [x] Add radiotap header struct and prepend to RX packets
- [x] Set device type ARPHRD_IEEE80211_RADIOTAP in ndo_open
- [x] Wire wireless_handlers to netdev
- [x] Track channel in priv struct, set_channel saves it
- [x] Update set_channel to use actual channel parameter (was hardcoded ch6)
- [x] Build successful

### Phase 20: Fix PF_PACKET delivery (v0.35) ⏳ READY TO TEST
- [x] Add `skb_reset_mac_header()` in RX path before `netif_rx()`
- [x] Clear `hard_header_len` and `header_ops` in ndo_open for radiotap device
- [x] Build successful
- [ ] Test packet capture in monitor mode

### Phase 11: RF Calibration ✅ FUNCTIONAL (RX working!)
- [x] Research IQK (I/Q Calibration) implementation
- [x] Research LCK (LC Tank Calibration) implementation
- [x] Implement IQK calibration (TX + RX paths)
- [x] Implement LC calibration
- [x] Test with both calibrations - **Still 0 RX packets**
- [x] **v0.31 FINDING:** RF chip works (0x33e60) but LC cal kills it (→ 0x00000)
- [x] **v0.32:** Fix LC cal (save/restore RF_AC) + rewrite IQK (save/restore 29 regs)
- [x] **v0.33:** Fix RF readback timing (10µs → 100µs) — **RX now continuous!**
- [ ] LC cal save still reads 0x00000 (RF read inside cal function still broken)

### Phase 12: BB/RF Hardware Access Issue ✅ SOLVED!
- [x] Debug why all BB/RF registers read 0xeaeaeaea - Missing power sequence
- [x] Check if missing AFE power/clock initialization - Added AFE init
- [x] Verify BB/RF blocks are properly powered - Power sequence fixed it
- [x] Fix hardware access before calibration - CARDEMU→ACT transition
- [x] Re-test with working BB/RF access - Registers work! (0x063f not 0xeaea)

### Phase 13: Firmware Initialization Issue ✅ ROOT CAUSE FOUND!
- [x] Implement power sequence (CARDEMU→ACT transition) - **WORKS!**
- [x] Add firmware download enable/disable - **WORKS!**
- [x] Add 8051 MCU reset function - **WORKS!**
- [x] Add firmware activation (FW_FREE_TO_GO) - **IMPLEMENTED**
- [x] Add firmware checksum verification - **ADDED**
- [x] Decompress firmware file (.zst → .bin) - **DONE**
- [x] Made checksum/WINTINI failures non-fatal - **Interface comes up!**
- [x] Research checksum failure - **FOUND: Checking wrong bit!**
- [x] **Critical finding: Checksum ACTUALLY PASSES (bit 2 is set)**
- [x] Fix: Check BIT(2) not BIT(5) for checksum - **IMPLEMENTED**
- [x] Fix: Preserve upper bits in page register writes - **IMPLEMENTED**
- [x] Fix: Add REG_RSV_CTRL manipulation in 8051 reset - **IMPLEMENTED**
- [x] Add MCU IO Wrapper reset sequence - **IMPLEMENTED**
- [x] Enable MCU clocks (FEN_ELDR, LOADER_CLK_EN, ANA8M) - **IMPLEMENTED**
- [x] Add AFE crystal control - **IMPLEMENTED**
- [x] **CRITICAL FINDING: Missing H2C initialization (REG_HMETFR)**

### Phase 14: H2C Initialization Fix ✅ TIMING FIXED!
- [x] Add REG_HMETFR (0x01CC) register definition - **DONE**
- [x] Add firmware header detection and skip (32 bytes) - **DONE**
- [x] Implement H2C initialization (write 0x0f to REG_HMETFR) - **DONE**
- [x] **CRITICAL FIX: Move H2C init BEFORE firmware activation!** - **FIXED**
- [x] Add last_hmebox_num tracking field - **ADDED**
- [x] **MAJOR FIX: Clear REG_MCUFWDL+1 before activation (was 0x0b)!** - **FIXED**
- [x] **Check RAM_DL_SEL and clear if needed** - **IMPLEMENTED**
- [x] Test if WINTINI_RDY finally sets - **SUCCESS! Sets in 2ms!** 🎉

### Phase 15: Fix RX Reception (Post-Firmware) ✅ IMPLEMENTED
- [x] Add rtl8188eu_init_wmac_setting() - **Sets RCR register**
- [x] Add rtl8188eu_init_usb_aggregation() - **Configures USB RX**
- [x] Add HCI_RXDMA_EN to REG_CR - **Enable RX DMA**
- [x] Call post-firmware init after firmware ready - **DONE**
- [ ] Test if RX packets finally work! - **READY TO TEST**

### Phase 16: Fix Kernel Panic (v0.22) ✅ COMPLETE
- [x] Add NULL pointer guards in RX callback - **CRITICAL FIX**
- [x] Move HCI_RXDMA_EN from probe to ndo_open - **Timing fix**
- [x] Add URB state validation before resubmit - **Safety check**
- [x] Disable RX DMA in ndo_stop - **Proper cleanup**
- [x] Test driver stability - **STABLE! No more panics**

### Phase 17: Debug RX URB Issue (v0.23-24) ✅ COMPLETE - RX WORKING!
- [x] Added RX callback debugging - **Callbacks now occurring!**
- [x] Enabled RX interrupts (HIMR = 0x503) - **Working**
- [x] Verified URBs submitted successfully - **4 URBs active**
- [x] **FOUND BUG:** Duplicate USB aggregation code in probe function
- [x] **FIXED:** Moved TRXDMA_CTRL config to Post-PHY init (executes)
- [x] **ADDED:** RXDMA_AGG_EN bit in TRXDMA_CTRL (0x010C)
- [x] **ADDED:** USB_SPECIAL_OPTION (0xFE55) configuration attempt
- [x] **CRITICAL FIX (v0.24):** RXDMA_AGG_PG_TH = 0x30 (not 0x05)
- [x] **CRITICAL FIX (v0.24):** RXDMA_AGG timeout = 0x04
- [x] **SUCCESS:** RX packets finally received! Counter shows 1 packet!

### Phase 18: Fix rmmod Kernel Crash (v0.25) ✅ COMPLETE
- [x] Analyzed kdump crash dump from Feb 16 freeze
- [x] **ROOT CAUSE:** Double-free in TX error path (kfree + URB_FREE_BUFFER)
- [x] **CRASH SCENARIO:** rmmod while avahi-daemon sending packets
- [x] **FIX 1:** Removed duplicate kfree(tx_buffer) - usb_free_urb handles it
- [x] **FIX 2:** Added `disconnecting` flag to priv struct
- [x] **FIX 3:** TX path drops packets when disconnecting
- [x] **FIX 4:** RX callback skips resubmit when disconnecting
- [x] **FIX 5:** disconnect() calls netif_device_detach() to stop network stack
- [x] Build successful

---

## Build Commands

```bash
cd /home/matthew/rtl8188eu_new
make clean && make
sudo rmmod rtl8xxxu
sudo rmmod rtl8188eu_minimal
sudo insmod ./rtl8188eu_minimal.ko
dmesg | tail -80
ip link show | grep enx
ip -s link show <interface_name>
```

---

## File Status

### Current Files
- `rtl8188eu_minimal_main.c` - Main driver (~3100 lines) ✅
- `rtl8188eu.h` - Common structures and definitions ✅
- `rtl8188eu_phy.h` - PHY function declarations ✅
- `rtl8188eu_phy.c` - PHY register access functions (~250 lines) ✅
- `rtl8188eu_phy_tables.c` - Config tables (~350 lines) ✅
- `Makefile` - Build system (multi-file) ✅
- `STATUS.md` - Detailed status log ✅
- `README.md` - Driver docs + full register reference ✅ **UPDATED**
- `PLAN.md` - PHY implementation plan ✅
- `START.md` - This file ✅
- `CLAUDE.md` - Developer guidelines ✅
- `rtl8188eu_8188e_fw.bin` - Firmware binary (15KB) ✅

### All Phases Complete!
All planned modifications have been completed:
- `rtl8188eu_phy.c` - Added table loaders and init functions ✅
- `rtl8188eu_minimal_main.c` - Integrated PHY init sequence ✅
- Total code: ~2500 lines across 5 source files

---

## Current Interface

**Latest test (v0.10):**
```
Interface: enx924f60db9420 (MAC changes each boot)
State: UP, carrier ON
Monitor Mode: ENABLED (RCR = 0x7000310f)
TX: 20 packets, 3260 bytes ✅
RX: 0 packets ❌ (blocked on PHY init)
```

**After PHY implementation:**
```
Expected: RX counter starts incrementing
Goal: Can capture packets in monitor mode
```

---

## USB Disconnect Issue - Troubleshooting Log

### What happens:
1. Driver loads firmware successfully
2. Network interface creation works
3. BB initialization starts loading tables
4. Around register 0x840 (RF interface) or 0xC50 (AGC), USB disconnects
5. Device disappears from USB bus and won't re-enumerate

### Attempts to fix:
1. **Increased delays:** Added 10μs-50μs delays between writes - **Still disconnects**
2. **Added msleep:** Added 1ms delays every 16/32 writes - **Still disconnects**
3. **Disabled RF init:** Skipped RF, only BB init - **Still disconnects**
4. **Safe mode:** Disabled ALL PHY init - **Device stays connected but no RX**

### ROOT CAUSE IDENTIFIED (Feb 11, 2026):
**Critical Issue #1: Conditional Table Parsing Missing**
- PHY tables contain conditional entries (IF/ELSE blocks) based on hardware config
- We're loading ALL entries including PCI/SDIO-specific values incompatible with USB
- Table format: 0x80000000 = IF, 0x40000000 = condition check, normal = register write
- **Result:** Wrong register values programmed into USB device causing crash

**Critical Issue #2: Missing Post-PHY Initialization**
- BB blocks (CCK/OFDM) not enabled after PHY tables
- MAC TX/RX not enabled (REG_CR MACTXEN|MACRXEN)
- USB aggregation parameters not configured
- No IQK/LCK calibration performed
- **Result:** Hardware expects these steps, times out when missing

### Current workaround:
Running in "safe mode" with no PHY init - device stable but RX non-functional

---

## Key Milestones

- ✅ v0.1-0.4: USB detection, firmware, basic init
- ✅ v0.5-0.7: Network interface, TX/RX paths, descriptors
- ✅ v0.8: Monitor mode RCR configuration
- ✅ v0.9-0.10: Channel tuning attempts
- ✅ v0.11: PHY layer implementation complete (all code written)
- ✅ v0.12: Progressive PHY testing (10, 50, then ALL tables)
- ✅ v0.13: **Root cause found: Conditional parsing missing, wrong values loaded**
- ✅ v0.14: **Conditional parsing implemented, device stable!**
- ✅ v0.15: **MAJOR FIX: Power sequence (CARDEMU→ACT) - 0xEAEA SOLVED!**
- ✅ v0.16: **Firmware fixes implemented - checksum passes, interface UP!**
- ✅ v0.17: **MCU clocks + IO Wrapper fixed - still no WINTINI_RDY**
- ✅ v0.18: **H2C initialization implemented - REG_HMETFR configured!**
- ✅ v0.19: **H2C TIMING FIXED - moved BEFORE firmware activation!**
- ✅ v0.20: **MCUFWDL+1 CLEARED - firmware starts! WINTINI_RDY sets!**
- ✅ v0.21: **POST-FIRMWARE INIT - RCR, USB AGG, RX DMA added!**
- ✅ v0.22: **KERNEL PANIC FIXED - NULL checks + RX DMA timing!**
- ✅ v0.23: **USB CONFIG FIXED - TRXDMA_CTRL with RXDMA_AGG_EN!**
- ✅ v0.24: **RX WORKING! First packet received with correct DMA values!**
- ✅ v0.25: **RMMOD CRASH FIXED - TX double-free + disconnect race condition!**
- ✅ v0.28: **MAC table + queue priority + init reorder for continuous RX**
- ✅ v0.29: **RW_RELEASE_EN fix + RX DMA at power-on + RCR data frames**
- ✅ v0.30: **Fix reg 0x800/0xEE8 overwrite + RX diagnostic dump**
- ✅ v0.31: **Fix RF environment setup + RF diagnostics — RF chip works! (0x33e60)**
- ✅ v0.32: **Fix calibration RF state corruption (LC cal + IQK save/restore)**
- ✅ v0.33: **Fix RF readback timing — 10µs → 100µs — CONTINUOUS RX WORKING!**
- ✅ v0.34: **Wireless extensions + radiotap headers for monitor mode**
- ⏳ v0.35: **Fix PF_PACKET delivery (skb_reset_mac_header + radiotap header_ops)**
- 🎯 v1.0: **Continuous RX + monitor mode capture working (goal: capture WiFi networks!)**

---

## Troubleshooting RX Issues (v0.24)

### Current Problem: Only 1 Packet Received
Despite being in an apartment complex with heavy WiFi traffic, we only receive 1 packet per driver load.

### Symptoms
- First packet received successfully (60 bytes)
- URB callback occurs once
- No further callbacks despite WiFi activity
- URB resubmission code executes but no effect
- `ip -s link show` shows RX: 1 packet, then stuck

### Potential Root Causes
1. **URB State Issue**
   - URB may be in unexpected state after first callback
   - Context pointer might be corrupted

2. **USB Endpoint Stall**
   - Endpoint might need clearing after first packet
   - Missing USB_CLEAR_HALT command

3. **Hardware Expects Different Flow**
   - Firmware might expect acknowledgment after each packet
   - Missing "packet consumed" signal to hardware

4. **DMA Buffer Issue**
   - Buffer might not be properly cleared/reset
   - Alignment requirements not met

5. **Missing Continuous RX Enable**
   - Hardware might need re-enabling after each packet
   - Missing register write to continue RX

### Debug Approach
1. Added URB resubmission logging
2. Monitor dmesg for "Resubmitting RX URB" messages
3. Check USB endpoint status
4. Compare with old driver's RX callback flow

---

## Important Notes

### Critical Discovery (Feb 16, 2026):
- **RMMOD CRASH FIXED (v0.25):**
  - **ROOT CAUSE:** Double-free in TX error path - `kfree(tx_buffer)` followed by `usb_free_urb()` with `URB_FREE_BUFFER` set (which also frees the buffer)
  - **TRIGGER:** avahi-daemon sent packet during `rmmod`, `usb_submit_urb()` returned -ENODEV, error path hit double-free → `kernel BUG at mm/slub.c:563`
  - **FIX:** Removed duplicate `kfree(tx_buffer)`, added `disconnecting` flag, `netif_device_detach()` in disconnect
  - **CRASH DUMP:** kdump captured 1.3GB vmcore at `/var/crash/202602161500/`

### Critical Discovery (Feb 14, 2026):
- **BREAKTHROUGH - RX WORKING (v0.24):**
  - **ROOT CAUSE:** Wrong RXDMA_AGG_PG_TH value (0x05 vs 0x30)
  - **FIX:** Set RXDMA_AGG_PG_TH = 0x30 (48 pages) to match old driver
  - **FIX:** Set RXDMA_AGG timeout = 0x04 for proper timing
  - **RESULT:** URB callbacks now occurring, first RX packet received!
- **Previous fixes that were necessary (v0.23):**
  - Fixed duplicate USB aggregation code
  - Moved TRXDMA_CTRL config to Post-PHY init
  - Added RXDMA_AGG_EN bit (BIT2) to enable RX DMA
  - Added USB_SPECIAL_OPTION configuration

### Before Testing:
1. Unload competing driver: `sudo rmmod rtl8xxxu`
2. Interface name changes each boot (random MAC)
3. Check dmesg for: "TRXDMA_CTRL set to" and "RXDMA_AGG_EN enabled"

### What to Look For:
1. **TRXDMA_CTRL messages** showing RXDMA_AGG_EN is set
2. **RX callback** messages finally appearing
3. **RX packet counter** incrementing from 0

### Device Recovery (if not showing in lsusb):
1. Unplug device completely
2. **Option A:** Wait 30+ seconds and replug
3. **Option B:** Full system shutdown, wait, restart
4. **Option C:** Try device on Windows/another Linux machine
5. If LED doesn't light: Device may be damaged/locked

### For Claude Code Sessions:
1. **CRITICAL:** Do NOT attempt PHY init until USB issue resolved
2. Use safe mode driver for now
3. Only user can run sudo commands
4. Device may need recovery between tests

---

## Quick Reference

**PHY Init Sequence:**
1. Initialize BB register definitions
2. Enable BB and RF blocks (REG_SYS_FUNC_EN, REG_RF_CTRL)
3. Load PHY_REG table (500 entries → BB registers 0x800-0xDFF)
4. Load AGC table (200 entries → BB register 0xC78)
5. Configure RF environment (BB registers 0x860, 0x870)
6. Load RadioA table (275 entries → RF registers 0x00-0xFF)
7. Set TX power (registers 0xE00-0xE1C)
8. Tune to channel (RF register 0x18)

**Critical Registers:**
- BB: 0x800 (RF mode), 0x840 (RF write), 0x860 (RF enable), 0xC78 (AGC)
- RF: 0x18 (channel/BW), 0x00 (RF enable), 0x08 (RF gain)
- TX Power: 0xE00-0xE1C

---

## Success Criteria

**Phase completion:**
- [x] Phase 1-5: PHY layer implementation ✅
- [x] Phase 6-12: BB/RF hardware access fixed ✅
- [x] Phase 13-14: Firmware initialization fixed ✅
- [x] Phase 15: Post-firmware init added ✅
- [x] Phase 16: Kernel panic fixed (v0.22) ✅
- [x] Phase 17: **RX URB issue FIXED (v0.24)** ✅
- [x] Phase 18: **rmmod crash FIXED (v0.25)** ✅

**Current Status:**
- [x] Device stable, no crashes ✅
- [x] Network interface comes up ✅
- [x] TX packets working (20+ sent) ✅
- [x] Monitor mode enabled ✅
- [x] **RX packets received** ✅ **Continuous! 10 callbacks, 9 packets (v0.33)**

**Final goal:**
- [x] RX packets counter > 0 ✅ **ACHIEVED!**
- [x] Continuous RX (more than 1 packet) ✅ **ACHIEVED v0.33! 10 callbacks!**
- [ ] Monitor mode packet capture shows WiFi networks
- [ ] Can capture packets in monitor mode

---

**This file tracks overall progress. See PLAN.md for detailed implementation steps.**
