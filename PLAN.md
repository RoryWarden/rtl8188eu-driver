# RTL8188EU PHY Layer Implementation Plan

**Goal:** Implement complete PHY initialization to enable RX packet reception

**Status:** Phases 0–6 COMPLETE — Continuous RX working as of v0.33 (Feb 17, 2026)

---

## Problem Statement

**Current State (v0.33):**
- ✅ USB detection, firmware download, basic registers
- ✅ Network interface, TX/RX paths with descriptors
- ✅ Monitor mode RCR configured
- ✅ TX working (17+ packets)
- ✅ **Continuous RX working! (10 callbacks, 9 packets)**

**Root Cause:** RF and baseband chips require extensive initialization before they can receive packets. Simple channel tuning alone is insufficient.

---

## Solution: Full PHY Layer Implementation

The PHY layer has 3 components:
1. **Baseband (BB)** - Digital signal processing (~500 register writes to 0x800-0xDFF)
2. **RF** - Radio frequency chip (~275 RF register writes to 0x00-0xFF via 3-wire serial)
3. **AGC** - Automatic Gain Control (~200 entries to register 0xC78)

**Scope:** ~1500 lines of code + ~6KB configuration tables

---

## Implementation Phases

### Phase 0: Documentation ✅
- [x] Create PLAN.md (this file)
- [x] Create START.md (progress tracker)
- [x] Update CLAUDE.md (guidelines)

### Phase 1: Core Helper Functions ✅
**File:** Create `rtl8188eu_phy.c` (~500 lines)

**Functions to implement:**
1. `rtl8188eu_read_bb_reg()` - Read baseband register with bitmask
2. `rtl8188eu_write_bb_reg()` - Write baseband register with bitmask
3. `rtl8188eu_read_rf_reg()` - Read RF register via 3-wire serial (0x840)
4. `rtl8188eu_write_rf_reg()` - Write RF register via 3-wire serial
5. `phy_calculate_bit_shift()` - Calculate bit position from mask
6. `rtl8188eu_init_phy_reg_def()` - Initialize BB/RF register definitions

**Key Technical Details:**
- BB registers: Direct 32-bit USB access
- RF registers: 20-bit values via baseband register 0x840 (LSSI)
- RF write format: `((addr << 20) | (data & 0xfffff)) & 0xfffffff`
- RF read: Via BB register 0x8A0 or 0x8B8

**Testing:**
```c
// Test BB access
rtl8188eu_write_bb_reg(priv, 0x800, 0xFFFFFFFF, 0x80040000);
u32 val = rtl8188eu_read_bb_reg(priv, 0x800, 0xFFFFFFFF);
// Should read back 0x80040000

// Test RF access
rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x00, 0xFFFFF, 0x30000);
val = rtl8188eu_read_rf_reg(priv, RF_PATH_A, 0x00, 0xFFFFF);
// Should read back 0x30000
```

### Phase 2: Configuration Tables ✅
**File:** Create `rtl8188eu_phy_tables.c` (~1000 lines of data)

**Tables to extract from old driver:**

**1. PHY_REG table** (~500 register/value pairs, ~2KB)
- **Source:** `/home/matthew/rtl8188eus/hal/phydm/rtl8188e/halhwimg8188e_bb.c` line 1115
- **Array:** `array_mp_8188e_phy_reg[]`
- **Purpose:** Initialize baseband registers (0x800-0xDFF)
- **Format:** `{0xaddr, 0xvalue, 0xaddr, 0xvalue, ...}`

**2. AGC_TAB table** (~200 entries, ~1.6KB)
- **Source:** Same file, line 117
- **Array:** `array_mp_8188e_agc_tab[]`
- **Purpose:** Set RX gain curves
- **Target register:** All writes go to 0xC78 (AGC table data)

**3. RadioA table** (~275 register/value pairs, ~2.2KB)
- **Source:** `/home/matthew/rtl8188eus/hal/phydm/rtl8188e/halhwimg8188e_rf.c` line 117
- **Array:** `array_mp_8188e_radioa[]`
- **Purpose:** Initialize RF chip
- **Registers:** RF 0x00-0xFF accessed via 3-wire serial

**Special delay codes in tables:**
- `0xFFE` = 50ms delay
- `0xFFD` = 5ms delay
- `0xFFC` = 1ms delay
- `0xFFB` = 50μs delay
- `0xFFA` = 5μs delay
- `0xFF9` = 1μs delay

### Phase 3: Table Loaders ✅
**Add to `rtl8188eu_phy.c`:**

**Functions:**
1. `rtl8188eu_load_phy_reg_table()` - Load PHY_REG table
2. `rtl8188eu_load_agc_table()` - Load AGC table
3. `rtl8188eu_load_rf_table()` - Load RadioA table

**Implementation notes:**
- Process register/value pairs sequentially
- Handle delay codes with msleep()/udelay()
- Skip conditional entries (0x88000001, etc.) initially
- Add 1μs delay between RF register writes

### Phase 4: PHY Init Functions ✅
**Add to `rtl8188eu_phy.c`:**

**1. rtl8188eu_phy_bb_config()** - Baseband initialization
```c
int rtl8188eu_phy_bb_config(struct rtl8188eu_priv *priv)
{
    // 1. Initialize register definitions
    rtl8188eu_init_phy_reg_def(priv);

    // 2. Enable BB and RF blocks
    //    REG_SYS_FUNC_EN (0x02): Enable BB + RF
    //    REG_RF_CTRL (0x1F): RF_EN | RF_RSTB | RF_SDMRSTB

    // 3. Load PHY_REG table (~500 entries)
    rtl8188eu_load_phy_reg_table(priv);

    // 4. Load AGC table (~200 entries)
    rtl8188eu_load_agc_table(priv);

    // 5. Set crystal cap (default: 0x20)

    return 0;
}
```

**2. rtl8188eu_phy_rf_config()** - RF initialization
```c
int rtl8188eu_phy_rf_config(struct rtl8188eu_priv *priv)
{
    // 1. Determine RF type (1T1R for RTL8188EU)
    priv->num_rf_paths = 1;

    // 2. For RF_PATH_A:
    //    a. Save RFENV control state (BB 0x870)
    //    b. Enable RF environment (BB 0x860, 0x870)
    //    c. Configure address/data length (BB 0x824)
    //    d. Load RadioA table (~275 entries)
    //    e. Restore RFENV control state

    return 0;
}
```

**3. rtl8188eu_phy_set_tx_power()** - TX power calibration
```c
void rtl8188eu_phy_set_tx_power(struct rtl8188eu_priv *priv, u8 channel)
{
    // Set TX power per rate:
    // - CCK rates: 0xE08, 0xE10
    // - OFDM rates: 0xE00-0xE04
    // - MCS rates: 0xE10-0xE1C

    // Use default value 0x30 (mid-power) initially
    // Later: Read from EFUSE for calibrated values
}
```

### Phase 5: Integration ✅
**Modify `rtl8188eu_minimal.c`:**

**Add to probe sequence (after TX/RX queue setup, before netdev registration):**
```c
pr_info("========================================\n");
pr_info("Phase 4: PHY Initialization\n");
pr_info("========================================\n");

ret = rtl8188eu_phy_bb_config(priv);
if (ret) {
    pr_err("Failed to initialize baseband: %d\n", ret);
    goto error_cleanup;
}
pr_info("Baseband configured (%d register writes)\n", phy_reg_count);

ret = rtl8188eu_phy_rf_config(priv);
if (ret) {
    pr_err("Failed to initialize RF: %d\n", ret);
    goto error_cleanup;
}
pr_info("RF configured (%d register writes)\n", rf_reg_count);

rtl8188eu_phy_set_tx_power(priv, 6);  // Default to channel 6
pr_info("TX power configured for channel 6\n");

pr_info("PHY initialization complete!\n");
pr_info("========================================\n");
```

**Update channel tuning function:**
Replace current `rtl8188eu_set_channel()` with proper read-modify-write:
```c
// Read current RF_CHNLBW value (preserves calibration bits [19:12])
u32 old_val = rtl8188eu_read_rf_reg(priv, RF_PATH_A, 0x18, 0xFFFFF);

// Mask out channel bits [9:0], keep [19:10]
old_val = (old_val & 0xfffffc00) | (0xC00 | channel);

// Write back with new channel
rtl8188eu_write_rf_reg(priv, RF_PATH_A, 0x18, 0xFFFFF, old_val);
```

**Update Makefile:**
```makefile
obj-m += rtl8188eu_minimal.o
rtl8188eu_minimal-objs := rtl8188eu_minimal.o rtl8188eu_phy.o rtl8188eu_phy_tables.o
```

### Phase 6: Testing & Verification ✅
**After each phase, verify:**

**Phase 1 verification:**
```bash
# Build and load
make clean && make
sudo rmmod rtl8xxxu rtl8188eu_minimal 2>/dev/null
sudo insmod ./rtl8188eu_minimal.ko
dmesg | tail -50

# Check for BB/RF register test success
```

**Phase 4 verification:**
```bash
# After PHY init, check counts
dmesg | grep "configured"
# Should show:
# - Baseband configured (500 register writes)
# - RF configured (275 register writes)
# - TX power configured
```

**Phase 6 final test:**
```bash
# Find interface
ip link show | grep enx

# Watch for RX packets (should finally increment!)
watch -n 1 'ip -s link show <interface_name>'

# RX counter should start showing packets! 🎉
```

---

## Critical Registers Reference

### Baseband (BB) Registers (0x800-0xDFF)
- **0x800** rFPGA0_RFMOD - RF mode control
- **0x824** rFPGA0_XA_HSSIParameter2 - RF serial interface control
- **0x840** rFPGA0_XA_LSSIParameter - RF write data (3-wire)
- **0x860** rFPGA0_XA_RFInterfaceOE - RF output enable
- **0x870** rFPGA0_XAB_RFInterfaceSW - RF software control
- **0x8A0** rFPGA0_XA_LSSIReadBack - RF read data (SI mode)
- **0x8B8** TransceiverA_HSPI_Readback - RF read data (PI mode)
- **0xC78** rOFDM0_AGCCore1 - AGC table write

### RF Registers (0x00-0xFF via 3-wire serial)
- **0x00** RF_AC - RF mode/enable
- **0x08** RF_IPA - RF gain
- **0x18** RF_CHNLBW - Channel and bandwidth (THE KEY REGISTER!)
- **0x19** - Channel configuration
- **0x1B** - RF power control
- **0x1E-0x1F** RF_TXPA_G1/G2 - TX power amp gain

### TX Power Registers (0xE00-0xE1C)
- **0xE08** rTxAGC_A_CCK1_Mcs32 - CCK 1M
- **0xE10** rTxAGC_B_CCK11_A_CCK2_11 - CCK 2/5.5/11M
- **0xE00-0xE1C** rTxAGC_A_Rate* - OFDM/MCS power

### System Control
- **REG_SYS_FUNC_EN (0x02)** - Enable BB/RF blocks
- **REG_RF_CTRL (0x1F)** - RF control signals

---

## File Structure After Implementation

```
/home/matthew/rtl8188eu_new/
├── rtl8188eu_minimal.c      (main driver, add PHY init calls)
├── rtl8188eu_phy.c          (NEW - PHY functions ~500 lines)
├── rtl8188eu_phy.h          (NEW - PHY declarations)
├── rtl8188eu_phy_tables.c   (NEW - config tables ~1000 lines)
├── Makefile                 (update to build new files)
├── PLAN.md                  (this file)
├── START.md                 (progress tracker)
├── STATUS.md                (current status)
├── README.md                (register documentation)
└── CLAUDE.md                (Claude Code guidelines)
```

---

## Expected Outcome

**After full PHY implementation (ACHIEVED v0.33!):**
- ✅ RX packet counter increments — **10 callbacks, 9 packets**
- ✅ Radio properly tuned to WiFi channel 6
- ⏳ Can add radiotap headers for airodump-ng
- ⏳ Monitor mode fully functional (needs airodump-ng test)

**Estimated timeline:**
- Phase 1-2: 2-3 hours (functions + tables)
- Phase 3-4: 1-2 hours (loaders + init)
- Phase 5-6: 1 hour (integration + testing)
- **Total: One focused day**

---

## Key Lessons Learned (v0.10)

1. **RF/BB chips are complex** - Can't just write channel registers
2. **Factory calibration matters** - RF registers contain chip-specific data
3. **PHY init is extensive** - Hundreds of register writes required
4. **TX power affects RX** - Even RX-only needs TX path configured
5. **"Minimal" has limits** - Some hardware needs full initialization

---

## References

**Old driver files:**
- `/home/matthew/rtl8188eus/hal/rtl8188e/rtl8188e_phycfg.c` - PHY config functions
- `/home/matthew/rtl8188eus/hal/rtl8188e/rtl8188e_rf6052.c` - RF init
- `/home/matthew/rtl8188eus/hal/rtl8188e/usb/usb_halinit.c` - Init sequence (lines 1400-1551)
- `/home/matthew/rtl8188eus/hal/phydm/rtl8188e/halhwimg8188e_bb.c` - BB tables
- `/home/matthew/rtl8188eus/hal/phydm/rtl8188e/halhwimg8188e_rf.c` - RF tables

---

**Last Updated:** Feb 17, 2026
**Current Phase:** All phases complete — continuous RX working (v0.33)
**Next Step:** Test with airodump-ng, fix LC cal RF readback (reads 0x00000 inside cal)
