import serial
import RPi.GPIO as GPIO
import sys
import time

SERIAL_PORT = "/dev/ttyS0" # GPIO14 (TX) and GPIO15 (RX)
PIN_20 = 12 # GPIO 12

SUPPORTED_COMMANDS = {
    "up": bytearray(b'\x9b\x06\x02\x01\x00\xfc\xa0\x9d'),
    "down": bytearray(b'\x9b\x06\x02\x02\x00\x0c\xa0\x9d'),
    "m": bytearray(b'\x9b\x06\x02\x20\x00\xac\xb8\x9d'),
    "wake_up": bytearray(b'\x9b\x06\x02\x00\x00\x6c\xa1\x9d'),
    "preset_1": bytearray(b'\x9b\x06\x02\x04\x00\xac\xa3\x9d'),
    "preset_2": bytearray(b'\x9b\x06\x02\x08\x00\xac\xa6\x9d'),
    "preset_3": bytearray(b'\x9b\x06\x02\x10\x00\xac\xac\x9d'),
    "preset_4": bytearray(b'\x9b\x06\x02\x00\x01\xac\x60\x9d'),
}

class LoctekMotion():

    def __init__(self, serial, pin_20):
        """Initialize LoctekMotion"""
        self.serial = serial

        # Or GPIO.BOARD - GPIO Numbering vs Pin numbering
        GPIO.setmode(GPIO.BCM)

        # Turn desk in operating mode by setting controller pin20 to HIGH
        # This will allow us to send commands and to receive the current height
        GPIO.setup(pin_20, GPIO.OUT)
        GPIO.output(pin_20, GPIO.HIGH)

    def execute_command(self, command_name: str):
        """Execute command"""
        command = SUPPORTED_COMMANDS.get(command_name)

        if not command:
            raise Exception("Command not found")

        self.serial.write(command)

    def decode_seven_segment(self, byte):
        binaryByte = bin(byte).replace("0b","").zfill(8)
        decimal = False
        if binaryByte[0] == "1":
            decimal = True
        if binaryByte[1:] == "0111111":
            return 0, decimal
        if binaryByte[1:] == "0000110":
            return 1, decimal
        if binaryByte[1:] == "1011011":
            return 2, decimal
        if binaryByte[1:] == "1001111":
            return 3, decimal
        if binaryByte[1:] == "1100110":
            return 4, decimal
        if binaryByte[1:] == "1101101":
            return 5, decimal
        if binaryByte[1:] == "1111101":
            return 6, decimal
        if binaryByte[1:] == "0000111":
            return 7, decimal
        if binaryByte[1:] == "1111111":
            return 8, decimal
        if binaryByte[1:] == "1101111":
            return 9, decimal
        if binaryByte[1:] == "1000000":
            return 10, decimal
        return -1, decimal



def current_height(self):
    start_time = time.time()  # Record the start time
    timeout = 10  # Set a timeout duration in seconds
    history = [None] * 5
    while True:
        # Check if the current time exceeds the start time by the timeout duration
        if time.time() - start_time > timeout:
            print("Timeout reached, exiting loop")
            break

        try:
            data = self.serial.read(1)
            if not data:
                print("No data received, exiting loop")
                break  # Exit the loop if no data is received

            # Update the history buffer
            history.pop(0)  # Remove the oldest item
            history.append(data[0])  # Add the newest item

            # Example condition to break the loop, adjust according to your data processing logic
            if len(set(history)) == 1 and history[0] == 0x9b:  # Just an example condition
                print("Specific condition met, exiting loop")
                break

            # Place your data processing logic here

        except Exception as e:
            print(f"Error reading serial data: {e}")
            break


def main():
    try:
        command = sys.argv[1]
        ser = serial.Serial(SERIAL_PORT, 9600, timeout=500)
        locktek = LoctekMotion(ser, PIN_20)
        locktek.execute_command(command)
        locktek.current_height()
    # Error handling for serial port
    except serial.SerialException as e:
        print(e)
        return
    # Error handling for command line arguments
    except IndexError:
        program = sys.argv[0]
        print("Usage: python3",program,"[COMMAND]")
        print("Supported Commands:")
        for command in SUPPORTED_COMMANDS:
            print("\t", command)
        sys.exit(1)
        return
    except KeyboardInterrupt:
        sys.exit(1)
        return
    finally:
        GPIO.cleanup()
        return()

if __name__ == "__main__":
    main()
