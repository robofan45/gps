#!/usr/bin/env python3
import sys, time, glob, webbrowser
import serial

# ---------------- Config ---------------- #
SIMULATION_MODE = False           # set True only if you feed test strings
PORT_CANDIDATES = [
    "/dev/serial0",               # stable alias on Pi
    "/dev/ttyAMA0",
    "/dev/ttyS0",
] + sorted(glob.glob("/dev/ttyUSB*"))  # USB dongles, if any
BAUDRATE = 9600
OPEN_MAP_ON_FIRST_FIX = True
PRINT_RAW = False                 # set True to debug raw NMEA spam

# -------------- Pretty colors -------------- #
class C:
    OK = "\033[92m"
    WARN = "\033[93m"
    BAD = "\033[91m"
    DIM = "\033[90m"
    RST = "\033[0m"

# -------------- Helpers -------------- #
def nmea_dm_to_dd(dm_str: str, hemi: str):
    """Convert NMEA ddmm.mmmm (lat) / dddmm.mmmm (lon) to decimal degrees."""
    if not dm_str:
        return None
    try:
        dm = float(dm_str)
        deg = int(dm // 100)
        minutes = dm - (deg * 100)
        dd = deg + (minutes / 60.0)
        if hemi in ("S", "W"):
            dd = -dd
        return dd
    except ValueError:
        return None

def build_maps_url(lat, lon):
    """
    Use normal Google Maps (no API key needed) so it's safe:
    - Opens a live map centered on the coordinate.
    """
    return f"https://www.google.com/maps/search/?api=1&query={lat:.6f},{lon:.6f}"

def parse_gpgga(fields):
    """
    GPGGA format (indices after splitting by ','):
    0: hhmmss.ss (UTC)  1: lat  2: N/S  3: lon  4: E/W
    5: fix quality (0=no fix)   6: num sats   7: HDOP   8: altitude ...
    Returns dict or None if not valid.
    """
    if len(fields) < 9:
        return None
    fix_q = fields[5]
    if not fix_q or fix_q == "0":
        return {"fix": 0}

    lat = nmea_dm_to_dd(fields[1], fields[2])
    lon = nmea_dm_to_dd(fields[3], fields[4])
    if lat is None or lon is None:
        return None

    try:
        sats = int(fields[6]) if fields[6] else None
    except ValueError:
        sats = None
    try:
        hdop = float(fields[7]) if fields[7] else None
    except ValueError:
        hdop = None

    return {
        "fix": int(fix_q),
        "lat": lat,
        "lon": lon,
        "sats": sats,
        "hdop": hdop,
        "utc": fields[0] if fields[0] else None,
    }

def open_first_available_port():
    errors = []
    for port in PORT_CANDIDATES:
        try:
            ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=1)
            print(f"{C.OK}Opened {port} @ {BAUDRATE}{C.RST}")
            return ser, port
        except Exception as e:
            errors.append((port, str(e)))
    print(f"{C.BAD}FATAL: Could not open any serial port.{C.RST}")
    for p, e in errors:
        print(f"  {p}: {e}")
    print("\nTroubleshooting:")
    print(" • Check wiring/VCC/GND/RX↔TX")
    print(" • Enable UART: sudo raspi-config  → Interface Options → Serial (login shell OFF, serial port ON)")
    print(" • Add user to dialout: sudo usermod -aG dialout $USER (then reboot)")
    sys.exit(1)

# -------------- Main -------------- #
def run():
    if SIMULATION_MODE:
        print(f"{C.WARN}Simulation mode is not implemented in this snippet.{C.RST}")
        sys.exit(0)

    print(f"{C.DIM}Tip: If no data appears, make sure gpsd is stopped if you use raw serial:")
    print(f"     sudo systemctl stop gpsd gpsd.socket{C.RST}\n")

    ser, port_name = open_first_available_port()
    print(f"Listening for GPS on {port_name}… Press Ctrl+C to exit.\n")
    print("Waiting for a valid GPS fix (this can take a few minutes, go outdoors with sky view)…\n")

    last_map_url = None
    map_opened_once = False
    last_status_print = 0

    try:
        while True:
            line = ser.readline()
            if not line:
                # Rate-limited keepalive
                now = time.time()
                if now - last_status_print > 5:
                    print(f"{C.DIM}…listening (no NMEA yet){C.RST}")
                    last_status_print = now
                continue

            try:
                s = line.decode("utf-8", errors="ignore").strip()
            except UnicodeDecodeError:
                continue

            if PRINT_RAW:
                print(f"RAW: {s}")

            if not s.startswith("$GPGGA"):
                continue

            # Split by comma, drop leading "$GPGGA,"
            fields = s.split(",")[1:]
            parsed = parse_gpgga(fields)
            if not parsed:
                continue

            if parsed["fix"] == 0:
                # No fix yet
                now = time.time()
                if now - last_status_print > 3:
                    sats = parsed.get("sats")
                    print(f"{C.WARN}GPS: acquiring…"
                          f"{(' sats=' + str(sats)) if sats is not None else ''}{C.RST}")
                    last_status_print = now
                continue

            # Valid fix
            hdop = parsed.get("hdop")
            sats = parsed.get("sats")
            lat = parsed["lat"]
            lon = parsed["lon"]
            utc = parsed.get("utc")

            if hdop is not None and hdop > 5:
                qual = f"{C.WARN}HDOP={hdop:.1f} (poor){C.RST}"
            elif hdop is not None:
                qual = f"{C.OK}HDOP={hdop:.1f} (good){C.RST}"
            else:
                qual = f"{C.DIM}HDOP=NA{C.RST}"

            print(f"{C.OK}GPS FIX{C.RST}  "
                  f"lat={lat:.6f}  lon={lon:.6f}  "
                  f"{('sats='+str(sats)) if sats is not None else ''}  {qual}  "
                  f"{('UTC='+utc) if utc else ''}")

            last_map_url = build_maps_url(lat, lon)

            if OPEN_MAP_ON_FIRST_FIX and not map_opened_once:
                print("Opening map for the first valid fix…")
                webbrowser.open(last_map_url)
                map_opened_once = True

            # keep looping to update last_map_url; you can sleep lightly
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting…")
        if last_map_url:
            print("Opening map for the last known location…")
            webbrowser.open(last_map_url)
        else:
            print("No valid fix received; nothing to open.")
    finally:
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    run()
