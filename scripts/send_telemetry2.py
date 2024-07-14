#!/usr/bin/env python

import serial
import datetime
import threading
import subprocess

def send_serial_data(data):
    # Ganti '/dev/ttyACM0' dengan port serial yang sesuai
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=57600,
        # bytesize=serial.EIGHTBITS,  # Atur bit data menjadi 8 bit
        # parity=serial.PARITY_NONE,  # Atur bit parity menjadi None (tanpa parity)
        # stopbits=serial.STOPBITS_ONE,  # Atur bit stop menjadi 1
        timeout=1
    )

    ser.write(data.encode('utf-8'))  # Mengirim data setelah diencode menjadi byte object
    ser.close()

def receive_serial_data():
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=57600,
        timeout=1
    )

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f'\n \n {timestamp} Respon from Raspi: {line} \n')

def sendImage():
    try:
        # Panggil program python lain
        result = subprocess.run(['python3', '/home/rafidahmadd/catkin_ws/src/drone_pkg/scripts/recv_encoded_img.py'], capture_output=True, text=True)
        
        # # Tampilkan output dari program yang dipanggil
        # print("Gambar diterima \n")
        
        # # Tampilkan error (jika ada)
        # if result.stderr:
        #     print("Errorr")
            
    except Exception as e:
        print(f"Terjadi kesalahan saat memanggil other_program.py: {e}")        

def menu():      
    print("Press")
    print("1: to Check connection")
    print("2: to send Image from Drone")
    print("3: to set mode to ARM the drone")
    print("4: to set mode to DISARM the drone")
    print("5: to set mode to TAKEOFF")
    print("6: to set mode to LAND")
    print("7: to set current position as home position")
    print("8: upload WAYPOINT TRAJECTORY")
    print("9: start WAYPOINT TRAJECTORY")
    print("q: to quit")
    

if __name__ == '__main__':
    
    receive_serial_data_thread = threading.Thread(target=receive_serial_data)
    receive_serial_data_thread.start()

    while True:
        menu()
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        data_to_send = input("Enter data to send (or 'q' to quit): ")  # Meminta pengguna memasukkan data
        print (f"\n{timestamp} Send to raspi: {data_to_send}\n")
        if data_to_send.lower() == 'q':
            print("Exiting...")
            break
        elif (data_to_send == '2'):
            sendImage()
        send_serial_data(data_to_send)