import argparse
from datetime import datetime
import serial
import time


def build_time_payload(value: str | None) -> str:
    if value:
        datetime.strptime(value, "%m%d%H%M%Y")
        return value
    return datetime.now().strftime("%m%d%H%M%Y")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--value")
    args = parser.parse_args()

    payload = build_time_payload(args.value)
    cmd = f"date {payload}\r\n"

    try:
        ser = serial.Serial(args.port, args.baud, timeout=3)
        time.sleep(2)
        ser.write(cmd.encode("utf-8"))
        print(f"Sent: {cmd.strip()}")
        time.sleep(2)

        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
            print("Response:")
            print(response)
        else:
            print("No response received")

        ser.close()
        print("Done.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except ValueError:
        print("Error: --value must use MMDDHHMMYYYY")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
