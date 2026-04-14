#!/usr/bin/env python3
"""Simple USB-CDC test: drive 2D gimbal to draw a wave-like trace.

Protocol:
  EN <0|1>
  VEL <PAN_RPM> <TILT_RPM>
  STOP
"""

import argparse
import math
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def clamp(value: float, limit: int) -> int:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return int(value)


def send_line(port: serial.Serial, text: str, read_reply: bool, timeout_s: float) -> None:
    port.write((text + "\n").encode("ascii"))
    if not read_reply:
        return

    end_time = time.time() + timeout_s
    while time.time() < end_time:
        line = port.readline().decode("ascii", errors="ignore").strip()
        if line:
            print(f"<- {line}")
            return


def main() -> int:
    parser = argparse.ArgumentParser(description="2D gimbal wave-line USB CDC test")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200, help="Virtual COM baudrate")
    parser.add_argument("--duration", type=float, default=10.0, help="Test duration (seconds)")
    parser.add_argument("--dt", type=float, default=0.05, help="Command period (seconds)")
    parser.add_argument("--pan-rpm", type=int, default=300, help="Pan axis base RPM")
    parser.add_argument("--pan-amp-rpm", type=int, default=0, help="Pan axis sine amplitude RPM (0 disables oscillation)")
    parser.add_argument("--pan-freq", type=float, default=None, help="Pan axis sine frequency (Hz), default uses --freq")
    parser.add_argument("--pan-phase-deg", type=float, default=0.0, help="Pan axis sine phase (degree)")
    parser.add_argument("--tilt-amp-rpm", type=int, default=500, help="Tilt axis sine amplitude RPM")
    parser.add_argument("--freq", type=float, default=0.25, help="Sine frequency (Hz)")
    parser.add_argument("--max-rpm", type=int, default=5000, help="Safety clamp RPM")
    parser.add_argument("--read-reply", action="store_true", help="Read one reply line per command")
    parser.add_argument("--send-enable", action="store_true", help="Send EN 1 on start and EN 0 on exit")
    parser.add_argument("--split-axis-cmd", action="store_true", help="Send VEL1 + VEL2 instead of combined VEL")
    parser.add_argument("--axis-cmd-gap", type=float, default=0.01, help="Gap between VEL1 and VEL2 in split mode")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baudrate, timeout=0.05, write_timeout=0.2) as ser:
        time.sleep(1.0)  # let USB CDC stabilize after open

        if args.send_enable:
            print("-> EN 1")
            send_line(ser, "EN 1", args.read_reply, 0.3)

        start = time.time()
        try:
            while True:
                t = time.time() - start
                if t >= args.duration:
                    break

                pan_freq = args.freq if args.pan_freq is None else args.pan_freq
                pan_phase = math.radians(args.pan_phase_deg)
                pan_rpm = clamp(
                    args.pan_rpm + args.pan_amp_rpm * math.sin(2.0 * math.pi * pan_freq * t + pan_phase),
                    args.max_rpm,
                )
                tilt_rpm = clamp(args.tilt_amp_rpm * math.sin(2.0 * math.pi * args.freq * t), args.max_rpm)
                if args.split_axis_cmd:
                    cmd1 = f"VEL1 {pan_rpm}"
                    cmd2 = f"VEL2 {tilt_rpm}"
                    print(f"-> {cmd1}")
                    send_line(ser, cmd1, args.read_reply, 0.1)
                    if args.axis_cmd_gap > 0:
                        time.sleep(args.axis_cmd_gap)
                    print(f"-> {cmd2}")
                    send_line(ser, cmd2, args.read_reply, 0.1)
                else:
                    cmd = f"VEL {pan_rpm} {tilt_rpm}"
                    print(f"-> {cmd}")
                    send_line(ser, cmd, args.read_reply, 0.1)
                time.sleep(args.dt)
        finally:
            print("-> STOP")
            send_line(ser, "STOP", args.read_reply, 0.3)
            if args.send_enable:
                print("-> EN 0")
                send_line(ser, "EN 0", args.read_reply, 0.3)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
