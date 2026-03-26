
from __future__ import annotations

import argparse
import json
import random
import signal
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from typing import Optional, Tuple

try:
    import serial  # pyserial
except ImportError:
    serial = None


@dataclass
class TelemetryFrame:
    ts: str
    seq: int
    battery_voltage_v: float
    battery_current_a: float
    battery_soc_pct: float
    vehicle_speed_kmh: float
    motor_temp_c: float
    battery_temp_c: float
    solar_panel_voltage_v: float
    solar_panel_current_a: float
    aux_12v_voltage_v: float
    handbrake: int


class BakoVehicleEmulator:
    def __init__(self, seed: Optional[int] = None) -> None:
        self.random = random.Random(seed)
        self.seq = 0
        self.battery_soc_pct = self.random.uniform(45.0, 80.0)
        self.vehicle_speed_kmh = 0.0
        self.motor_temp_c = self.random.uniform(28.0, 34.0)
        self.battery_temp_c = self.random.uniform(26.0, 32.0)
        self.solar_irradiance = self.random.uniform(0.2, 0.9)
        self.handbrake = 1

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _jitter(self, scale: float) -> float:
        return self.random.uniform(-scale, scale)

    def _update_speed(self) -> None:
        phase_roll = self.random.random()

        if self.handbrake == 1 and phase_roll < 0.75:
            target = 0.0
        else:
            if phase_roll < 0.18:
                target = 0.0
            elif phase_roll < 0.45:
                target = self.random.uniform(8.0, 22.0)
            elif phase_roll < 0.78:
                target = self.random.uniform(20.0, 38.0)
            else:
                target = self.random.uniform(38.0, 50.0)

        delta = target - self.vehicle_speed_kmh
        step = self._clamp(delta, -4.5, 4.5)
        self.vehicle_speed_kmh = self._clamp(
            self.vehicle_speed_kmh + step + self._jitter(0.6), 0.0, 50.0
        )

        if self.vehicle_speed_kmh < 0.8 and self.random.random() < 0.35:
            self.handbrake = 1
        elif self.vehicle_speed_kmh > 1.5:
            self.handbrake = 0

    def _update_solar(self) -> Tuple[float, float]:
        self.solar_irradiance = self._clamp(
            self.solar_irradiance + self._jitter(0.08), 0.0, 1.0
        )

        solar_panel_voltage_v = self._clamp(
            42.0 * (0.82 + 0.22 * self.solar_irradiance) + self._jitter(1.0),
            0.0,
            50.0,
        )
        solar_panel_current_a = self._clamp(
            10.24 * self.solar_irradiance + self._jitter(0.35),
            0.0,
            10.5,
        )
        return solar_panel_voltage_v, solar_panel_current_a

    def _update_battery_and_temps(
        self, solar_panel_current_a: float
    ) -> Tuple[float, float, float, float]:
        speed = self.vehicle_speed_kmh

        traction_draw_a = 3.0 + 0.62 * speed + max(0.0, self._jitter(2.0))
        solar_charge_a = solar_panel_current_a * 0.72
        battery_current_a = traction_draw_a - solar_charge_a

        if speed < 1.0:
            battery_current_a = self._clamp(
                1.2 - solar_charge_a + self._jitter(1.2), -6.0, 8.0
            )

        battery_current_a = self._clamp(battery_current_a, -10.0, 45.0)

        soc_delta = -(max(battery_current_a, -5.0) / 3600.0) * 0.9
        self.battery_soc_pct = self._clamp(
            self.battery_soc_pct + soc_delta + self._jitter(0.03), 5.0, 100.0
        )

        ocv = 60.0 + (self.battery_soc_pct / 100.0) * 9.0
        sag = max(battery_current_a, 0.0) * 0.045
        rise = abs(min(battery_current_a, 0.0)) * 0.03
        battery_voltage_v = self._clamp(
            ocv - sag + rise + self._jitter(0.18), 60.0, 69.4
        )

        self.motor_temp_c = self._clamp(
            self.motor_temp_c + 0.05 * speed / 10.0 - 0.05 + self._jitter(0.25),
            22.0,
            85.0,
        )
        self.battery_temp_c = self._clamp(
            self.battery_temp_c + 0.02 * max(battery_current_a, 0.0) - 0.03 + self._jitter(0.12),
            20.0,
            60.0,
        )

        return (
            battery_voltage_v,
            battery_current_a,
            self.motor_temp_c,
            self.battery_temp_c,
        )

    def _update_aux_voltage(self) -> float:
        return self._clamp(12.6 + self._jitter(0.35), 11.8, 13.2)

    def next_frame(self) -> TelemetryFrame:
        self.seq += 1
        self._update_speed()
        solar_panel_voltage_v, solar_panel_current_a = self._update_solar()
        (
            battery_voltage_v,
            battery_current_a,
            motor_temp_c,
            battery_temp_c,
        ) = self._update_battery_and_temps(solar_panel_current_a)
        aux_12v_voltage_v = self._update_aux_voltage()

        return TelemetryFrame(
            ts=datetime.now(timezone.utc).isoformat(),
            seq=self.seq,
            battery_voltage_v=round(battery_voltage_v, 2),
            battery_current_a=round(battery_current_a, 2),
            battery_soc_pct=round(self.battery_soc_pct, 1),
            vehicle_speed_kmh=round(self.vehicle_speed_kmh, 1),
            motor_temp_c=round(motor_temp_c, 1),
            battery_temp_c=round(battery_temp_c, 1),
            solar_panel_voltage_v=round(solar_panel_voltage_v, 2),
            solar_panel_current_a=round(solar_panel_current_a, 2),
            aux_12v_voltage_v=round(aux_12v_voltage_v, 2),
            handbrake=self.handbrake,
        )


def open_serial_port(port: str, baud: int):
    if serial is None:
        raise RuntimeError(
            "Missing dependency: pyserial. Install it with: pip install pyserial"
        )
    return serial.Serial(port=port, baudrate=baud, timeout=1)


def format_terminal_row(frame: TelemetryFrame) -> str:
    return (
        f"[{frame.seq:06d}] "
        f"speed={frame.vehicle_speed_kmh:5.1f} km/h | "
        f"batt={frame.battery_voltage_v:5.2f} V | "
        f"curr={frame.battery_current_a:6.2f} A | "
        f"soc={frame.battery_soc_pct:5.1f}% | "
        f"motor_t={frame.motor_temp_c:4.1f} C | "
        f"batt_t={frame.battery_temp_c:4.1f} C | "
        f"solar_v={frame.solar_panel_voltage_v:5.2f} V | "
        f"solar_i={frame.solar_panel_current_a:4.2f} A | "
        f"aux12={frame.aux_12v_voltage_v:4.2f} V | "
        f"handbrake={frame.handbrake}"
    )


def run_emulator(
    port: Optional[str],
    baud: int,
    period: float,
    dry_run: bool,
    seed: Optional[int],
) -> int:
    emulator = BakoVehicleEmulator(seed=seed)
    ser = None
    stop = False

    def handle_stop(signum, _frame):
        nonlocal stop
        stop = True
        print("\nStopping emulator on signal %s..." % signum)

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    if not dry_run:
        if not port:
            raise ValueError("A serial port is required unless --dry-run is used.")
        ser = open_serial_port(port, baud)
        print("Connected to serial port %s @ %s baud" % (port, baud))
    else:
        print("Running in dry-run mode: no serial port will be opened")

    print("BAKO vehicle emulator started")
    print("Press Ctrl+C to stop\n")

    try:
        while not stop:
            frame = emulator.next_frame()
            payload = json.dumps(asdict(frame), separators=(",", ":"))
            print(format_terminal_row(frame))

            if ser is not None:
                ser.write((payload + "\n").encode("utf-8"))
                ser.flush()

            time.sleep(period)
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("Serial port closed")

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="BAKO vehicle sensor emulator")
    parser.add_argument(
        "--port",
        help="Serial port or virtual serial port, e.g. COM8 or /dev/pts/3",
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Serial baud rate (default: 115200)"
    )
    parser.add_argument(
        "--period",
        type=float,
        default=1.0,
        help="Seconds between telemetry frames (default: 1.0)",
    )
    parser.add_argument(
        "--seed", type=int, default=None, help="Optional random seed for reproducible runs"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Run without opening a serial port"
    )
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return run_emulator(
        port=args.port,
        baud=args.baud,
        period=args.period,
        dry_run=args.dry_run,
        seed=args.seed,
    )


if __name__ == "__main__":
    sys.exit(main())
