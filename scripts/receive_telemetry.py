import serial

def receive_telemetry(serial_port):
    try:
        # Open the serial port
        ser = serial.Serial(serial_port, 57600, timeout=1)
        print(f"Serial port {serial_port} opened successfully.")

        # Receive telemetry data
        telemetry_data = ser.readline().decode('utf-8').strip()
        print(f"Telemetry received: {telemetry_data}")

        # Close the serial port
        ser.close()
        print("Serial port closed.")

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Replace '/dev/ttyUSB0' with the appropriate serial port on your Raspberry Pi
    serial_port = '/dev/ttyACM0'

    receive_telemetry(serial_port)
