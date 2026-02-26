# RTL8188EU Driver - Quick Start & Progress

**Last Updated:** Feb 26, 2026
**Current Version:** v1.0.0 (mac80211 conversion — managed + monitor mode)
**Previous Version:** v0.40.1 (IQK result application to compensation registers)

---

## Quick Resume Point

**What works:**
- ✅ USB detection, firmware download
- ✅ **mac80211/cfg80211 integration** — replaces custom netdev/wext
- ✅ **Monitor mode** — tcpdump capture with radiotap headers via mac80211
- ✅ **Managed mode** — scanning finds APs with full BSS details (HT caps, RSN, etc.)
- ✅ **rtl8xxxu blacklisted** — `/etc/modprobe.d/blacklist-rtl8xxxu.conf` prevents auto-loading
- ✅ **minstrel_ht rate control** — selected automatically by mac80211
- ✅ TX path with mac80211 queue selection + rate control feedback
- ✅ RX path with ieee80211_rx_status (signal, rate, encoding)
- ✅ Continuous RX: 4000+ callbacks in managed mode
- ✅ PHY layer + calibration complete
- ✅ Signal strength: -4 to -92 dBm range (realistic)
- ✅ Channel hopping (sw_scan)
- ✅ HT20/HT40 bandwidth support (BB + RF configuration)
- ✅ Software crypto (set_key returns -EOPNOTSUPP)

**What doesn't work yet:**
- ❌ Only seeing CCK 1.0 Mb/s packets (no OFDM/HT captured yet)
- ❌ No AP association tested yet
- ❌ No WPA supplicant tested
- ❌ No packet injection

**Fixed (Feb 26, 2026) — Scan now returns APs:**
- **Root cause 1:** `configure_filter` re-enabled BSSID filtering during scan, overriding `scan_start`'s disable. Fix: skip BSSID filtering when `priv->scanning` is true.
- **Root cause 2:** `rx_status->boottime_ns` was never set (stayed 0 from memset). Fix: set `ktime_get_boottime_ns()` on all RX frames.
- Result: `iw scan` now finds 16+ APs across all channels

**Current Status (Feb 20, 2026 - v1.0.0):**
- ✅ **v1.0.0:** Full mac80211 conversion
  - Replaced custom netdev + wireless extensions with ieee80211_ops
  - Station mode: BSSID programming, MSR, association handling, BSSID filtering
  - Monitor mode: RCR configuration via configure_filter
  - TX: proper descriptor with queue sel (VO/VI/BE/BK/MGMT), rate from mac80211
  - RX: ieee80211_rx_status with signal/rate/encoding, ieee80211_rx_irqsafe
  - HT40: rtl8188eu_set_bw() configures MAC/BB/RF for 20/40 MHz
  - Scanning: sw_scan_start/complete with RCR filter toggling
  - Chanctx: uses ieee80211_emulate_{add,remove,change}_chanctx
  - Key: returns -EOPNOTSUPP for software crypto via mac80211

---

## Completion Estimates

### Monitor mode sniffer: ~70%
What we have: mac80211-based packet capture with radiotap headers, signal, scanning.
What's missing for a usable sniffer:
- [x] Correct signal strength (byte 0 path AGC gain — v0.38)
- [x] mac80211 integration (v1.0.0)
- [x] Channel scanning/hopping (sw_scan via mac80211)
- [x] Filter controls (configure_filter with RCR)
- [x] Multiple channel width support (HT20/HT40 — v1.0.0)
- [ ] OFDM and HT rate capture (currently only 1.0 Mb/s CCK)
- [ ] Packet injection (TX in monitor mode)
- [ ] Noise floor estimation

### Full WiFi driver: ~40-45%
What we have: mac80211 integration, scanning, monitor mode, HT20/HT40.
What's done:
- [x] mac80211 subsystem integration (v1.0.0)
- [x] cfg80211 interface (nl80211 via mac80211)
- [x] Scanning (sw_scan — probe requests, BSS list)
- [x] Rate control algorithm (minstrel_ht via mac80211)
- [x] Proper TX queues (QoS queue selection VO/VI/BE/BK/MGMT)
- [x] Station mode basics (BSSID, MSR, association callbacks)
- [x] Software crypto (mac80211 handles WPA2)
What's missing:
- [ ] Association testing (connect to AP via wpa_supplicant)
- [ ] Power management (sleep/wake, dynamic PS)
- [ ] A-MPDU / A-MSDU aggregation
- [ ] RTS/CTS, ACK handling
- [ ] AP mode / SoftAP
- [ ] P2P / Wi-Fi Direct
- [ ] Regulatory domain handling
- [ ] LED control
- [ ] Suspend/resume
- [ ] Proper error recovery

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

### Phase 20: Fix PF_PACKET delivery (v0.35) ✅ COMPLETE
- [x] Add `skb_reset_mac_header()` in RX path before `netif_rx()`
- [x] Clear `hard_header_len` and `header_ops` in ndo_open for radiotap device
- [x] Build successful
- [x] Test packet capture in monitor mode — **WORKS!**

### Phase 21: Fix RF read address mask (v0.36) ✅ COMPLETE
- [x] Fix `bLSSIReadAddress` mask from `0x07f80000` to `0x7f800000` in `rtl8188eu_phy.h`
- [x] Remove safety net force-restore of RF_AC (no longer needed)
- [x] LC cal RF readback now correct (0x33e60)
- [x] tcpdump captures 20 packets with full radiotap headers

### Phase 22: Real radiotap data + channel hopping + debug cleanup (v0.37) ✅ COMPLETE
- [x] Parse actual rate from RX descriptor DW3 (CCK/OFDM legacy + HT MCS)
- [x] Parse signal strength from PHY status (OFDM path + CCK LNA/VGA lookup)
- [x] Add MCS field to radiotap header for HT rates
- [x] Fix channel hopping: proper RF read-modify-write of RF_CHNLBW register
- [x] Cache RF_CHNLBW value after RF table load
- [x] Debug cleanup: ~100 pr_info → pr_debug conversions
- [x] Remove per-packet TX logging, reduce first-N RX logs from 10 to 3
- [x] Remove unused variables from debug-cleaned functions
- [x] Version bump to 0.37
- [x] Build successful

### Phase 23: Fix radiotap MCS flag + signal diagnostics (v0.37.1) ✅ COMPLETE
- [x] Clear MCS bit from it_present for legacy (non-HT) packets
- [x] tcpdump now shows "11b" not "11n" for CCK beacons
- [x] Added diagnostic pr_info for first 10 packets (raw PHY status bytes)
- [x] Diagnostic findings:
  - All packets CCK (hw_rate=0x00), drvinfo=32 (correct), physt=1
  - phy[5]=0xff → LNA_idx=7, VGA_idx=31 → -100 dBm (clamped, likely invalid)
  - phy[5]=0x20 → LNA_idx=1, VGA_idx=0 → -1 dBm (wrong table or AGC mode)
  - Old driver has TSMC table {29,20,12,3,-6,-15,-24,-33} vs our SMIC table
  - Old driver checks cut_version to pick table — we don't check cut_version
- [x] **DONE: chip version detection + signal fix using byte 0 AGC**

### Phase 24: mac80211 Conversion (v1.0.0) ✅ COMPLETE
- [x] Replace custom netdev + wireless extensions with ieee80211_ops
- [x] ieee80211_alloc_hw / ieee80211_register_hw integration
- [x] TX path: mac80211 queue selection, rate control, tx_status feedback
- [x] RX path: ieee80211_rx_status (signal, rate, encoding), ieee80211_rx_irqsafe
- [x] Band/rate/channel definitions (14 channels, CCK+OFDM+HT MCS0-7)
- [x] Monitor mode via configure_filter (RCR configuration)
- [x] Station mode: add/remove_interface, bss_info_changed (BSSID, MSR, assoc)
- [x] SW scan: sw_scan_start/complete with RCR filter toggling
- [x] HT20/HT40 bandwidth: rtl8188eu_set_bw() for MAC/BB/RF
- [x] Software crypto (set_key returns -EOPNOTSUPP)
- [x] Channel context emulation (ieee80211_emulate_*_chanctx)
- [x] Fix: chanctx ops required by kernel 6.17 mac80211
- [x] **Monitor mode tested: tcpdump captures beacons/data/ACKs with signal**
- [x] **Managed mode tested: iw scan finds APs with full BSS details**

### Phase 25: Fix Zero Scan Results ✅ COMPLETE
- [x] Add scan diagnostic counters (URB, frame, CRC drop, len drop, TX counts)
- [x] Instrument RX path and TX path with scan counters
- [x] Print scan stats at scan_end
- [x] Test result: 13 URBs, 13 frames to mac80211, 0 drops, 11 TX frames
- [x] **FINDING:** Frames reach ieee80211_rx_irqsafe() but mac80211 discards them
- [x] Add per-frame logging (frame type FC + freq + channel) during scan
- [x] Set `rx_status->boottime_ns = ktime_get_boottime_ns()` on all RX frames
- [x] **FIX:** `configure_filter` was re-enabling BSSID filtering during scan (0 URBs)
- [x] Added `priv->scanning` check to skip BSSID filtering in `configure_filter`
- [x] **Blacklisted rtl8xxxu** — `/etc/modprobe.d/blacklist-rtl8xxxu.conf`
- [x] **RESULT: `iw scan` finds 16+ APs with full BSS details!**

### Phase 11: RF Calibration ✅ FUNCTIONAL (RX working!)
- [x] Research IQK (I/Q Calibration) implementation
- [x] Research LCK (LC Tank Calibration) implementation
- [x] Implement IQK calibration (TX + RX paths)
- [x] Implement LC calibration
- [x] Test with both calibrations - **Still 0 RX packets**
- [x] **v0.31 FINDING:** RF chip works (0x33e60) but LC cal kills it (→ 0x00000)
- [x] **v0.32:** Fix LC cal (save/restore RF_AC) + rewrite IQK (save/restore 29 regs)
- [x] **v0.33:** Fix RF readback timing (10µs → 100µs) — **RX now continuous!**
- [x] LC cal RF read fixed! (was bLSSIReadAddress mask bug, not cal-specific issue)

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
sudo rmmod rtl8188eu_minimal
sudo insmod ./rtl8188eu_minimal.ko
dmesg | tail -80
ip link show | grep wlx
ip -s link show <interface_name>
```

**Note:** `rtl8xxxu` is blacklisted via `/etc/modprobe.d/blacklist-rtl8xxxu.conf`.
Interface name uses `wlx` prefix (wireless, not `enx` ethernet) since mac80211 conversion.

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
- ✅ v0.35: **Fix PF_PACKET delivery (skb_reset_mac_header + radiotap header_ops)**
- ✅ v0.36: **Fix RF read address mask (bLSSIReadAddress) — monitor mode capture working!**
- ✅ v0.37: **Real radiotap data + channel hopping fix + debug cleanup**
- ✅ v0.37.1: **Fix radiotap MCS flag (11b not 11n) + signal strength diagnostics**
- ✅ v0.38: **Fix signal strength (byte 0 AGC) + chip version detection**
- ✅ v0.39: **RF bandwidth (20MHz) + BWOPMODE init**
- ✅ v0.39.1: **Fix crystal cap for OFDM reception (CX_IN/CX_OUT mismatch)**
- ✅ v0.40: **DC cancellation removed (not supported on 8188E)**
- ✅ v0.40.1: **IQK result application — identity values, still CCK only**
- ✅ v1.0.0: **mac80211 conversion — monitor mode + managed mode + scanning WORKING!**
- ✅ **Scan fixed** — configure_filter BSSID race + boottime_ns — 16+ APs found
- 🎯 Next: **Remove diagnostic logging → AP association via wpa_supplicant**

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
- [x] **RX packets received** ✅ **Continuous! 12,000+ callbacks, 3,700+ packets**
- [x] Radiotap headers with correct rate field ✅
- [x] Channel hopping works ✅
- [x] Signal strength working (-62 to -64 dBm) ✅ **FIXED v0.38!**
- [ ] Only CCK 1.0 Mb/s packets captured (no OFDM/HT)

**Original goals (monitor mode sniffer):**
- [x] RX packets counter > 0 ✅ **ACHIEVED!**
- [x] Continuous RX (more than 1 packet) ✅ **ACHIEVED v0.33!**
- [x] Monitor mode packet capture shows WiFi networks ✅ **ACHIEVED v0.36!**
- [x] Can capture packets in monitor mode ✅ **ACHIEVED v0.36!**
- [x] Correct signal strength reporting ✅ **ACHIEVED v0.38!**
- [ ] Multi-rate capture (OFDM + HT, not just CCK)
- [ ] Usable with Wireshark/tcpdump for real analysis

---

**This file tracks overall progress. See PLAN.md for detailed implementation steps.**
