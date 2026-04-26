import argparse
import serial
import time

BAUD = 115200


def read_available(ser, wait=0.2):
    time.sleep(wait)
    n = ser.in_waiting
    if n > 0:
        return ser.read(n)
    return b""


def read_exact(ser, n, timeout=2.0):
    end = time.time() + timeout
    out = bytearray()
    while len(out) < n and time.time() < end:
        chunk = ser.read(n - len(out))
        if chunk:
            out.extend(chunk)
    return bytes(out)


def print_help():
    print()
    print("Commands:")
    print("  c  -> clear capture")
    print("  1  -> start SPI capture")
    print("  2  -> start I2C capture")
    print("  3  -> start UART probe capture (auto-baud)")
    print("  4  -> start CAN capture (auto-baud)")
    print("  ?  -> request 4-byte status packet")
    print("  r  -> read + decode captured bytes")
    print("  x  -> send a raw single-character command")
    print("  h  -> show this help")
    print("  q  -> quit")
    print()


def decode_uart_capture(data, clk_hz=12_000_000, max_baud=250_000, oversample=16):
    if len(data) < 2:
        print("  [UART] Too short to contain header.")
        return

    sample_hz = max_baud * oversample
    bit_ticks = (data[0] << 8) | data[1]

    if bit_ticks == 0:
        print("  [UART] Invalid bit_ticks = 0 (baud never locked).")
        return

    baud_est = sample_hz / bit_ticks
    print(f"  [UART] bit_ticks = {bit_ticks}  ->  estimated baud ~= {baud_est:.0f}")

    frames = bytes(data[2:])
    if not frames:
        print("  [UART] No decoded bytes after header.")
        return

    hex_str = " ".join(f"{b:02X}" for b in frames)
    ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in frames)
    print(f"  [UART] Decoded {len(frames)} byte(s): {hex_str}")
    


def decode_can_capture(data):
    if not data:
        print("  [CAN] No data.")
        return

    i = 0
    frame_num = 0
    while i < len(data):
        if i >= len(data):
            break

        desc = data[i]
        ext = bool(desc & 0x80)
        rtr = bool(desc & 0x40)
        dlc = desc & 0x0F
        if dlc > 8:
            dlc = 8  # clamp per CAN spec

        frame_type = "EXT" if ext else "STD"
        id_bytes   = 4 if ext else 2
        total_len  = 1 + id_bytes + dlc + 2

        if i + total_len > len(data):
            print(f"  [CAN] Frame {frame_num}: truncated (need {total_len} bytes, "
                  f"only {len(data) - i} remain)")
            break

        if ext:
            can_id = (
                ((data[i+1] & 0x1F) << 24) |
                (data[i+2] << 16) |
                (data[i+3] <<  8) |
                 data[i+4]
            )
            payload_start = i + 5
        else:
            can_id = ((data[i+1] & 0x07) << 8) | data[i+2]
            payload_start = i + 3

        payload = data[payload_start : payload_start + dlc]
        crc_high = data[payload_start + dlc]
        crc_low  = data[payload_start + dlc + 1]
        rx_crc   = ((crc_high & 0x7F) << 8) | crc_low

        payload_hex = payload.hex(" ") if payload else "(none)"
        rtr_str     = " RTR" if rtr else ""
        print(f"  [CAN] Frame {frame_num}: {frame_type}{rtr_str}  "
              f"ID=0x{can_id:08X}  DLC={dlc}  "
              f"Payload=[{payload_hex}]  CRC=0x{rx_crc:04X}")

        i += total_len
        frame_num += 1

    if frame_num == 0:
        print("  [CAN] No complete frames decoded.")
    else:
        print(f"  [CAN] {frame_num} frame(s) decoded.")


def display_capture(data, proto_hint=None):
    """Pretty-print captured bytes and attempt protocol decode."""
    print(f"Captured bytes ({len(data)}): {data.hex(' ')}")
    if not data:
        return

    if proto_hint == 3:
        decode_uart_capture(data)
    elif proto_hint == 4:
        decode_can_capture(data)
    else:
        try:
            ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
            
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Interactive host utility for protocol snooper (SPI/I2C/UART/CAN) on Cmod S7.")
    parser.add_argument("--port",         default="COM4",  help="Serial port")
    parser.add_argument("--baud",         type=int,  default=BAUD)
    parser.add_argument("--startup-wait", type=float, default=2.0)
    parser.add_argument("--clk-hz",       type=int,  default=12_000_000,
                        help="FPGA clock frequency (for UART baud estimation)")
    parser.add_argument("--uart-max-baud",type=int,  default=250_000,
                        help="MAX_BAUD parameter passed to uart_rx_buffered_autobaud")
    parser.add_argument("--uart-oversample", type=int, default=16)
    args = parser.parse_args()

    last_count  = 0
    last_proto  = None

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(f"Opened {args.port} at {args.baud} baud")
        print(f"Waiting {args.startup_wait:.1f} s for board/USB-UART startup...")
        time.sleep(args.startup_wait)

        stale = read_available(ser, wait=0.1)
        if stale:
            print("Discarded stale bytes:", stale.hex(" "))

        print_help()

        while True:
            try:
                cmd = input("Enter command: ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting.")
                break

            if not cmd:
                continue

            cmd_lower = cmd.lower()

            if cmd_lower == "q":
                print("Exiting.")
                break

            if cmd_lower == "h":
                print_help()
                continue

            if cmd_lower == "x":
                raw = input("Enter exactly one character to send: ")
                if len(raw) != 1:
                    print("Please enter exactly one character.")
                    continue
                ser.write(raw.encode("latin1"))
                print(f"Sent raw command: {raw!r}")
                resp = read_available(ser, wait=0.2)
                if resp:
                    print("Immediate response:", resp.hex(" "))
                else:
                    print("No immediate response.")
                continue

            if cmd not in ["c", "1", "2", "3", "4", "?", "r"]:
                print("Unknown command. Type 'h' for help.")
                continue

            ser.write(cmd.encode("ascii"))
            print(f"Sent: {cmd!r}")

            if cmd == "c":
                resp = read_available(ser, wait=0.1)
                if resp:
                    print("Unexpected response:", resp.hex(" "))
                else:
                    print("Clear sent.")

            elif cmd == "1":
                last_proto = 1
                resp = read_available(ser, wait=0.1)
                if resp:
                    print("Unexpected immediate response:", resp.hex(" "))
                else:
                    print("SPI capture started.")

            elif cmd == "2":
                last_proto = 2
                resp = read_available(ser, wait=0.1)
                if resp:
                    print("Unexpected immediate response:", resp.hex(" "))
                else:
                    print("I2C capture started.")

            elif cmd == "3":
                last_proto = 3
                resp = read_available(ser, wait=0.1)
                if resp:
                    print("Unexpected immediate response:", resp.hex(" "))
                else:
                    print("UART probe capture started (auto-baud detection running).")

            elif cmd == "4":
                last_proto = 4
                resp = read_available(ser, wait=0.1)
                if resp:
                    print("Unexpected immediate response:", resp.hex(" "))
                else:
                    print("CAN capture started (auto-baud detection running).")

            elif cmd == "?":
                status = read_exact(ser, 4, timeout=1.0)
                if len(status) == 4:
                    print("Status raw:", status.hex(" "))
                    if status[0] == 0xA5:
                        busy  = status[1]
                        done  = status[2]
                        count = status[3]
                        last_count = count
                        proto_str = {1:"SPI", 2:"I2C", 3:"UART probe", 4:"CAN"}.get(last_proto, "?")
                        print(f"proto={proto_str}  busy={busy}  done={done}  count={count}")
                    else:
                        print("Warning: first status byte was not 0xA5")
                else:
                    print("Bad or no status response.")
                    if len(status) > 0:
                        print("Partial bytes:", status.hex(" "))

            elif cmd == "r":
                if last_count <= 0:
                    print("Last status count is 0. Use '?' first, or there is no captured data.")
                    continue
                data = read_exact(ser, last_count, timeout=3.0)
                if len(data) == last_count:
                    display_capture(data, proto_hint=last_proto)
                else:
                    print(f"Only received {len(data)} of {last_count} bytes.")
                    if data:
                        display_capture(bytes(data), proto_hint=last_proto)


if __name__ == "__main__":
    main()