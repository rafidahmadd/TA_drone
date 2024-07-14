# import base64
# import serial

# # Mengatur koneksi serial
# ser = serial.Serial(
#     port='/dev/ttyUSB0',
#     baudrate=57600,
#     bytesize=serial.EIGHTBITS,
#     parity=serial.PARITY_NONE,
#     stopbits=serial.STOPBITS_ONE,
#     timeout=1
# )

# # Fungsi untuk menerima data sampai menemukan end_marker
# while True:
#     if ser.in_waiting > 0:
#         line = ser.readlines()
#         print(line)

# # import base64
# # import serial

# # # Mengatur koneksi serial
# # ser = serial.Serial(
# #     port='/dev/ttyUSB0',
# #     baudrate=57600,
# #     bytesize=serial.EIGHTBITS,
# #     parity=serial.PARITY_NONE,
# #     stopbits=serial.STOPBITS_ONE,
# #     timeout=1
# # )

# # # Fungsi untuk menerima data dari serial
# # def receive_data(ser):
# #     data = ""
# #     while True:
# #         chunk_raw = ser.read(10000)
# #         chunk = chunk_raw.decode('utf-8')  # Baca chunk sebesar 100 byte
# #         if "<END>" in chunk:
# #             data += chunk.split("<END>")[0]
# #             break
# #         data += chunk
# #         ser.write('ACK'.encode('utf-8'))  # Kirim konfirmasi (ACK) untuk setiap chunk yang diterima
# #     return data

# # # Menerima data dari serial
# # received_data = receive_data(ser)

# # # Mengekstrak data base64 dari data yang diterima
# # print("Received base64 data:", received_data)

# # # Decode base64 ke gambar
# # image_data = base64.b64decode(received_data)
# # with open("/home/rafidahmadd/Pictures/captured/hasil_decode.png", "wb") as image_file:
# #     image_file.write(image_data)


# import paramiko

# def send_file_sftp(host, port, username, password, local_file_path, remote_file_path):
#     try:
#         # Membuat transport (koneksi SSH)
#         print(f"Connecting to {host} on port {port}")
#         transport = paramiko.Transport((host, port))
#         transport.connect(username=username, password=password)
#         print("Connected to server")

#         # Membuat SFTP client dari transport
#         sftp = paramiko.SFTPClient.from_transport(transport)
#         print("SFTP client created")

#         # Mengirim file
#         print(f"Sending file from {local_file_path} to {remote_file_path}")
#         sftp.put(local_file_path, remote_file_path)
#         print(f"Successfully sent {local_file_path} to {remote_file_path}")

#         # Menutup koneksi
#         sftp.close()
#         transport.close()
#         print("Connection closed")

#     except paramiko.AuthenticationException as e:
#         print(f"Authentication failed: {e}")
#     except paramiko.SSHException as e:
#         print(f"Unable to establish SSH connection: {e}")
#     except paramiko.BadHostKeyException as e:
#         print(f"Unable to verify server's host key: {e}")
#     except FileNotFoundError as e:
#         print(f"File not found: {e}")
#     except Exception as e:
#         print(f"Operation error: {e}")

# # Ganti dengan informasi server Anda
# host = "192.168.104.29"
# port = 22  # default port for SFTP
# username = "mfproject2"
# password = "12345678"
# local_file_path = "/home/rafidahmadd/aruco"
# remote_file_path = "/home/mfproject2/"

# send_file_sftp(host, port, username, password, local_file_path, remote_file_path)

import paramiko
import os

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(hostname='192.168.104.29', username='mfproject2', password='12345678', port=22)

sftp_client=ssh.open_sftp()
# sftp_client.get('/home/mfproject2/Picture/captured/captured_image1.png', '/home/rafidahmadd/Pictures/captured/fromDrone/reference.png')
sftp_client.get('/home/mfproject2/Picture/captured/captured_image2.png', '/home/rafidahmadd/Pictures/captured/fromDrone/move3.png')
sftp_client.get('/home/mfproject2/Picture/captured/captured_image3.png', '/home/rafidahmadd/Pictures/captured/fromDrone/move4.png')
# sftp_client.get('/home/mfproject2/Picture/captured/LogoPolman.png', '/home/rafidahmadd/Pictures/captured/fromDrone/LogoPolman_transfered.png')
# sftp_client.get('/home/mfproject2/Picture/captured/fotopolman.jpg', '/home/rafidahmadd/Pictures/captured/fromDrone/fotopolman_transfered.jpg')
# sftp_client.get('/home/mfproject2/Picture/captured/move.jpg', '/home/rafidahmadd/Pictures/captured/fromDrone/move_transfered.jpg')
# sftp_client.get('/home/mfproject2/Picture/captured/palestineflag.png', '/home/rafidahmadd/Pictures/captured/fromDrone/palestineflag_transfered.png')
# sftp_client.get('/home/mfproject2/Picture/captured/watermelon.png', '/home/rafidahmadd/Pictures/captured/fromDrone/watermelon_transfered.png')
# sftp_client.put('/home/rafidahmadd')

print("Image transfered")
sftp_client.close()
ssh.close()