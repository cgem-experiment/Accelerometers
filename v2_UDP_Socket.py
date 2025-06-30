#v2_UDP_Socket
import socket
import csv  
from datetime import datetime  
import time
import os

NUM_PACKETS_PER_FILE = 26400 # Number of packets to write to each file
NUM_FILES = -1  # Set to -1 for infinite, or specify the number of files 
BASE_PATH = r"C:\Users\natal\Accelerometer_data"  # Change this variable to set the base directory

# Configuration
UDP_IP = "10.20.3.3"
UDP_PORT = 8
LISTEN_IP = "10.20.1.3"
LISTEN_PORT = 12345 #55151 #CHANGE IF ON SITE
PACKET_SIZE = 601*2 + 42  # 600 bytes of data + 42 bytes UDP header

# Take timestamp and define function for filenames
timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")
def generate_filename(file_idx):
    return f"{timestamp}_FILE{file_idx:04d}_cgem_accel.csv"

# Create the folder
folder_name = f"{timestamp}_cgem_accel"
full_path = os.path.join(BASE_PATH, folder_name)
os.makedirs(full_path, exist_ok=True)  # Create folder if it doesn't exist

# Initial filename, file and packet index
packet_idx = 0
file_idx = 1
filename = os.path.join(full_path, generate_filename(file_idx))

# Configure ethernet socket
protocol = socket.SOCK_DGRAM  # SOCK_DGRAM is for UDP
ip_family = socket.AF_INET  # AF_INET is for ipv4
sock = socket.socket(ip_family, protocol)

# Bind to the specific IP and port
try:
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(" Bind successful. Listening for data...") # Can try using sock.bind(("0.0.0.0", LISTEN_PORT)) instead if having problems
except OSError as e:
    print(f" Bind failed: {e}")

# Print confirmation
print(f"Listening for UDP packets from {UDP_IP}:{UDP_PORT} on port {LISTEN_PORT}...")
print("Folder created at ", full_path)

def process_payload(payload):

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    current_time_ns = time.time_ns() % 1_000_000_000
    
    payload_hex = payload.hex()
    
    # Split payload_hex by the separator
    samples_hex = payload_hex.split("89abcdef")
    
    value0_array = []
    value1_array = []
    value2_array = []
    status0_array = []
    status1_array = []
    status2_array = []
    time_array = []
    
    for i, sample_hex in enumerate(samples_hex):
        if len(sample_hex) < 32:
            if i != 60:
                print(f"Warning: Skipping invalid sample at index {i}: {sample_hex} with surrounding {samples_hex[i-5:i+5]} and payload {payload_hex[i*16+i*4-50:i+i*16*4+50]}")
            continue
    
    # Convert each hex segment to an integer 
    #can make this way more efficient using numpy arrays and indexing directly, instead of 7 individual lists
        try:
            value0 = int(sample_hex[6:8] + sample_hex[2:4] + sample_hex[0:2], 16)
            status0 = int(sample_hex[4:6], 16)
            
            value1 = int(sample_hex[14:16] + sample_hex[10:12] + sample_hex[8:10], 16)
            status1 = int(sample_hex[12:14])
      
            value2 = int(sample_hex[22:24] + sample_hex[18:20] + sample_hex[16:18], 16)
            status2 = int(sample_hex[20:22])
            
            time_32bit = int(sample_hex[40:32] + sample_hex[28:30] + sample_hex[26:28] + sample_hex[24:26], 16) #24-32
            
            value0_array.append(value0)
            value1_array.append(value1)
            value2_array.append(value2)

            status0_array.append(status0)
            status1_array.append(status1)
            status2_array.append(status2)

            time_array.append(time_32bit)
            
            print(f"Value 0: {value0}, Value 1: {value1}, Value 2: {value2}, Time Int: {time_32bit}")
        
        except ValueError as e:
            print(f"Error processing sample: {sample_hex} - {e}")
        
    write = value0_array + value1_array + value2_array + status0_array + status1_array + status2_array + time_array
    
    #write.append(int( (samples_hex[-1][2:4] + samples_hex[-1][0:2]) ,16))
    write.append(current_time)
    write.append(current_time_ns)
    
    # Write the list of integers to the CSV file
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(write)
    
    print(len(write), "samples written to file. First value:", write[0])
 
# MAIN LOOP
try:
    while True:
        sock.settimeout(2.0)  # seconds
        try:
            data, addr = sock.recvfrom(PACKET_SIZE)
        except socket.timeout:
            print("No data received within 2 seconds.")
        #data, addr = sock.recvfrom(PACKET_SIZE)  # Receive packet
         #os.delay(1000)
        print(data, addr)
        if addr[0] == UDP_IP and addr[1] == UDP_PORT:
            data_payload = data[0:]  # UDP header is removed
            process_payload(data_payload)  # Process the payload (function above)
            packet_idx += 1
            if packet_idx == NUM_PACKETS_PER_FILE:
                print("All packets processed for current file. Next file initialized.")
                packet_idx = 0
                file_idx += 1
                if NUM_FILES != -1 and file_idx >= NUM_FILES:
                    print("All specified files processed. Exiting...")
                    break
                else:
                    filename = os.path.join(full_path, generate_filename(file_idx))
                    print(f"Writing to new file: {filename}")
        else:
            print(f"Ignored packet from {addr}")  # Ignore packets from other addresses/ports
except KeyboardInterrupt:
    print("Server stopped.")
finally:
    sock.close()