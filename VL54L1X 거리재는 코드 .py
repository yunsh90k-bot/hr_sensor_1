import re
import time
import serial

PORT = "COM4"      # 장치관리자에서 Cmod S7 포트로 바꿔
BAUD = 115200

pattern = re.compile(r"D:(\d+)cm\s+VL:([01])\s+I:([01])")

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)

print("reading... Ctrl+C to stop")

try:
    while True:
        raw = ser.readline()
        if not raw:
            continue

        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            continue

        print("raw:", line)

        m = pattern.search(line)
        if not m:
            continue

        ultrasonic_cm = int(m.group(1))
        vl_connected = m.group(2) == "1"
        tof_int = m.group(3) == "1"

        print(
            f"ultrasonic={ultrasonic_cm} cm | "
            f"VL53L1X_ACK={'OK' if vl_connected else 'NO'} | "
            f"INT={int(tof_int)}"
        )

except KeyboardInterrupt:
    print("stopped")

finally:
    ser.close()
