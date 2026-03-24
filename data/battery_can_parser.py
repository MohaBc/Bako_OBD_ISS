"""
╔══════════════════════════════════════════════════════════════════╗
║  SAE J1939 Battery CAN Frame Parser                              ║
║  Based on: bat72230Ah BMS Communication Protocol                 ║
╠══════════════════════════════════════════════════════════════════╣
║  CAN IDs decoded (250 kbps, J1939 29-bit):                       ║
║                                                                  ║
║  0x18FF28F4  →  BMS Basic Info Frame 1                           ║
║    Byte 1   : Status flags (charging connected, charge state…)   ║
║    Byte 2   : SOC  [0–100 %,  1 %/bit]                           ║
║    Bytes 3-4: Pack current  (offset -500, × 0.1 A/bit)           ║ 
║    Bytes 5-6: Pack voltage  (offset 0,     × 0.1 V/bit)          ║
║    Byte 7   : Fault level                                        ║
║    Byte 8   : Fault code                                         ║
║                                                                  ║
║  0x18FE28F4  →  BMS Basic Info Frame 2                           ║
║    Bytes 1-2: Max cell voltage  [mV, 1 mV/bit]                   ║
║    Bytes 3-4: Min cell voltage  [mV, 1 mV/bit]                   ║
║    Byte 5   : Max cell temp     [°C, offset -40, 1 °C/bit]       ║
║    Byte 6   : Min cell temp     [°C, offset -40, 1 °C/bit]       ║
║    Bytes 7-8: Max allowed discharge current [A, × 0.1 A/bit]     ║
║                                                                  ║
║  0x18B428F4  →  BMS Temperature Detail Frame                     ║
║    Each byte: one probe [°C, offset -40, 1 °C/bit]               ║
║                                                                  ║
║  0x18FFE5F4  →  BMS Charging Request Frame                       ║
║    Bytes 1-2: Max allowed charge voltage [× 0.1 V/bit]           ║
║    Bytes 3-4: Max allowed charge current [× 0.1 A/bit]           ║
║    Byte 5   : Charge control flag                                ║
║                                                                  ║
║  0x18C828F4..0x18CD28F4  →  Cell Voltage Detail Frames           ║
║    4 cells × 2 bytes each [mV, 1 mV/bit]                         ║
╚══════════════════════════════════════════════════════════════════╝
"""

import re

FILE_PATH = "C:/Users/legio/Desktop/Bako/batterie.txt"

def le16(b_lo, b_hi):
    return (b_hi << 8) | b_lo

def decode_FF28(data):
    flags      = data[0]
    soc        = data[1]
    raw_i      = le16(data[2], data[3])
    raw_v      = le16(data[4], data[5])
    fault_lvl  = data[6]
    fault_code = data[7]

    # Protocol: offset -5000, factor 0.1 → range -500 to +500 A; charging = negative
    current = raw_i * 0.1 - 500.0
    voltage = raw_v * 0.1

    charge_wire   = bool(flags & 0x01)
    is_charging   = bool(flags & 0x02)
    low_battery   = bool(flags & 0x04)
    ready         = bool(flags & 0x08)
    dis_contactor = bool(flags & 0x10)
    chg_contactor = bool(flags & 0x20)

    if is_charging:
        charge_state = "🔋 CHARGING"
    elif dis_contactor and not is_charging:
        charge_state = "⚡ DISCHARGING"
    else:
        charge_state = "⏸  IDLE / STANDBY"

    fault_str = "✅ No Fault"
    if fault_lvl == 0x01:
        fault_str = f"🚨 Level-1 Fault (STOP NOW) — Code: 0x{fault_code:02X}"
    elif fault_lvl != 0x00:
        fault_str = f"⚠️  Fault Level {fault_lvl} — Code: 0x{fault_code:02X}"

    return {
        "voltage_V"   : round(voltage, 1),
        "current_A"   : round(current, 1),
        "soc_pct"     : soc,
        "charge_state": charge_state,
        "charge_wire" : "Connected" if charge_wire else "Not connected",
        "ready"       : "Yes" if ready else "No",
        "fault"       : fault_str,
    }

def decode_FE28(data):
    max_cell_v = le16(data[0], data[1])
    min_cell_v = le16(data[2], data[3])
    max_temp   = data[4] - 40
    min_temp   = data[5] - 40
    max_dis_i  = le16(data[6], data[7]) * 0.1
    return {
        "max_cell_V": round(max_cell_v / 1000, 3),
        "min_cell_V": round(min_cell_v / 1000, 3),
        "max_temp_C": max_temp,
        "min_temp_C": min_temp,
        "max_dis_A" : round(max_dis_i, 1),
    }

def decode_B428(data):
    temps = [b - 40 for b in data if b != 0x00]
    return {"probes_C": temps}

def decode_FFE5(data):
    max_chg_v  = le16(data[0], data[1]) * 0.1
    max_chg_i  = le16(data[2], data[3]) * 0.1
    ctrl_flag  = data[4]
    chg_allowed = not bool(ctrl_flag & 0x02)
    return {
        "max_chg_V"  : round(max_chg_v, 1),
        "max_chg_A"  : round(max_chg_i, 1),
        "chg_allowed": "✅ Charging ALLOWED" if chg_allowed else "🚫 Charging BLOCKED",
    }

def decode_cell_voltages(cid, data):
    pf   = int(cid[2:4], 16)
    base = (pf - 0xC8) * 4 + 1
    cells = {}
    for i in range(4):
        raw = (data[i*2] << 8) | data[i*2+1]
        if raw > 0:
            cells[base + i] = raw
    return cells

LINE_RE = re.compile(
    r"\[(\d+)ms\]\s+ID:\s+0x([0-9A-Fa-f]+)\s+DLC:\s+\d+\s+Data:\s+([\dA-Fa-f\s]+)"
)

def parse_line(line):
    m = LINE_RE.search(line)
    if not m:
        return None
    return int(m.group(1)), m.group(2).upper(), [int(b, 16) for b in m.group(3).split()]

def main():
    with open(FILE_PATH, "r") as f:
        raw_text = f.read()

    lines = raw_text.splitlines()

    last = {}
    cell_voltages = {}
    temp_probes   = []

    for line in lines:
        parsed = parse_line(line)
        if not parsed:
            continue
        ts, cid, data = parsed

        if cid.endswith("FF28F4"):
            last["ff28"] = decode_FF28(data)
        elif cid.endswith("FE28F4"):
            last["fe28"] = decode_FE28(data)
        elif cid.endswith("B428F4"):
            r = decode_B428(data)
            last["b428"] = r
            temp_probes = r["probes_C"]
        elif cid.endswith("FFE5F4"):
            last["ffe5"] = decode_FFE5(data)
        elif cid[2:4] in ["C8","C9","CA","CB","CC","CD"]:
            cell_voltages.update(decode_cell_voltages(cid, data))

    SEP = "═" * 65
    print(f"\n{SEP}")
    print("  SAE J1939 BATTERY ANALYSIS  —  Based on Manufacturer Protocol")
    print(SEP)

    ff28 = last.get("ff28", {})
    fe28 = last.get("fe28", {})
    ffe5 = last.get("ffe5", {})

    print("\n📊  PACK STATUS (last known values)\n")
    print(f"  🔌 Pack Voltage          : {ff28.get('voltage_V','N/A')} V")
    print(f"  ⚡ Pack Current          : {ff28.get('current_A','N/A')} A  (neg = charging)")
    print(f"  📈 State of Charge (SOC) : {ff28.get('soc_pct','N/A')} %")
    print(f"  ⚙️  Charge Status         : {ff28.get('charge_state','N/A')}")
    print(f"  🔌 Charge Cable          : {ff28.get('charge_wire','N/A')}")
    print(f"  🟢 Pack Ready            : {ff28.get('ready','N/A')}")
    print(f"  🚨 Fault                 : {ff28.get('fault','N/A')}")

    print(f"\n🌡  TEMPERATURE\n")
    print(f"  Max cell temperature     : {fe28.get('max_temp_C','N/A')} °C")
    print(f"  Min cell temperature     : {fe28.get('min_temp_C','N/A')} °C")
    if temp_probes:
        for i, t in enumerate(temp_probes):
            print(f"  Probe {i+1}                  : {t} °C")

    print(f"\n⚡  CELL VOLTAGES\n")
    if cell_voltages:
        print(f"  Max cell voltage         : {fe28.get('max_cell_V','N/A')} V")
        print(f"  Min cell voltage         : {fe28.get('min_cell_V','N/A')} V")
        for idx in sorted(cell_voltages):
            mv  = cell_voltages[idx]
            pct = max(0, min(20, int((mv - 3200) / 400 * 20)))
            bar = "█" * pct + "░" * (20 - pct)
            print(f"  Cell {idx:>2}  {mv:>4} mV  |{bar}|")
    else:
        print("  No cell voltage detail frames decoded.")

    print(f"\n🔋  CHARGE REQUEST (BMS → Charger)\n")
    if ffe5:
        print(f"  Max charge voltage       : {ffe5.get('max_chg_V','N/A')} V")
        print(f"  Max charge current       : {ffe5.get('max_chg_A','N/A')} A")
        print(f"  Charge permission        : {ffe5.get('chg_allowed','N/A')}")
    else:
        print("  No charge request frame found.")

    print(f"\n{SEP}\n")

if __name__ == "__main__":
    main()
