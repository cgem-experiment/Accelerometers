#v2_UDP_Socket
import socket
import csv  
from datetime import datetime  
import time
import os
import logging
from collections import deque

# -----------------------------
# Logging configuration
# Options: "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"
LOG_LEVEL = "INFO"
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL),
    format="%(asctime)s | %(levelname)s | %(message)s",
)
log = logging.getLogger("udp_logger")

_now = time.perf_counter  

# ---- Sample-rate display tuning ----
# How often to print the rate
RATE_REPORT_PERIOD_SEC = 1.0
# Moving-average window (seconds) for wall-clock rate
RATE_MOVING_WINDOW_SEC = 5.0
# MCU timestamp tick period (TIM23 = 1 MHz = 1 us/tick)

MCU_TICK_SEC = 1e-6

NUM_PACKETS_PER_FILE = 26400 # Number of packets to write to each file
NUM_FILES = -1  # Set to -1 for infinite, or specify the number of files 
BASE_PATH = r"C:\Users\Public\Accelerometer_data"  # Change this variable to set the base directory

# Configuration
#UDP_IP = "10.20.3.3"
UDP_IP = "192.168.1.30" #Remote (MCU) IP
UDP_PORT = 8
#LISTEN_IP = "10.20.1.3"
LISTEN_IP = "192.168.1.10" #Host (This PC) IPasdf
LISTEN_PORT = 12345 #55151 #CHANGE IF ON SITE
PACKET_SIZE = 601*2 + 42  # 600 bytes of data + 42 bytes UDP header

metadata_filename = os.path.join(BASE_PATH, "metadata_log.txt")

# Prompt the user for a note at the start
session_note = input("Notes: ")

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

# Initialize the metadata log
with open(metadata_filename, "a") as meta_file:
    meta_file.write(f"folder path: {full_path}\n")
    meta_file.write(f"initial file path: {filename}\n")
    meta_file.write(f"Session note: {session_note}\n\n")

# Configure ethernet socket
protocol = socket.SOCK_DGRAM  # SOCK_DGRAM is for UDP
ip_family = socket.AF_INET  # AF_INET is for ipv4
sock = socket.socket(ip_family, protocol)

# Bind to the specific IP and port
try:
    sock.bind((LISTEN_IP, LISTEN_PORT))
    log.info(" Bind successful. Listening for data...") # Can try using sock.bind(("0.0.0.0", LISTEN_PORT)) instead if having problems
except OSError as e:
    log.error(f" Bind failed: {e}")

# Print confirmation
log.info(f"Listening for UDP packets from {UDP_IP}:{UDP_PORT} on port {LISTEN_PORT}...")
log.info(f"Folder created at {full_path}")

def process_payload(payload):

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    current_time_ns = time.time_ns() % 1_000_000_000
    
    # Work in bytes (more robust than hex slicing)
    SEPARATOR = b"\x89\xab\xcd\xef"
    samples_bytes = payload.split(SEPARATOR)
    
    value0_array = []
    value1_array = []
    value2_array = []
    status0_array = []
    status1_array = []
    status2_array = []
    time_array = []
    
    for i, chunk in enumerate(samples_bytes):
        if len(chunk) < 16:
             if i != 60:
                log.warning(f"Warning: Skipping invalid sample at index {i}: {sample_hex} with surrounding {samples_hex[i-5:i+5]} and payload {payload_hex[i*16+i*4-50:i+i*16*4+50]}")
                log.debug(f"Skipping short sample at index {i} (len={len(chunk)})")
             continue
    
        try:
            # Layout per ISR:
            # bytes 0..1: ch0 low16 (LE),  2: ch0_id,  3: ch0 high8
            # bytes 4..5: ch1 low16 (LE),  6: ch1_id,  7: ch1 high8
            # bytes 8..9: ch2 low16 (LE), 10: ch2_id, 11: ch2 high8
            # bytes 12..15: TIM23 timestamp (low16 then high16) → 32-bit LE
            value0  = chunk[0] | (chunk[1] << 8) | (chunk[3] << 16)
            status0 = chunk[2]
            value1  = chunk[4] | (chunk[5] << 8) | (chunk[7] << 16)
            status1 = chunk[6]
            value2  = chunk[8] | (chunk[9] << 8) | (chunk[11] << 16)
            status2 = chunk[10]
            time_32bit = int.from_bytes(chunk[12:16], "little")

            value0_array.append(value0); value1_array.append(value1); value2_array.append(value2)
            status0_array.append(status0); status1_array.append(status1); status2_array.append(status2)
            time_array.append(time_32bit)

            log.debug(f"Value 0: {value0}, Value 1: {value1}, Value 2: {value2}, Time Int: {time_32bit}")
        except Exception as e:
            log.warning(f"Error processing sample chunk {i} (len={len(chunk)}): {e}")
        
    write = value0_array + value1_array + value2_array + status0_array + status1_array + status2_array + time_array
    
    #write.append(int( (samples_hex[-1][2:4] + samples_hex[-1][0:2]) ,16))
    write.append(current_time)
    write.append(current_time_ns)
    
    # Write the list of integers to the CSV file
    with open(filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(write)
    
    log.debug(f"{len(write)} fields written to file. First value: {write[0]}")

    # ---- sample-rate tracking (wall clock, moving average) ----
    n = min(60, len(value0_array))  # number of samples in this packet
    now = _now()
    process_payload.window.append((now, n))
    process_payload.window_samples += n
    # Trim old entries outside the moving window
    cutoff = now - RATE_MOVING_WINDOW_SEC
    while process_payload.window and process_payload.window[0][0] < cutoff:
        _, old_n = process_payload.window.popleft()
        process_payload.window_samples -= old_n

    # ---- sample-rate tracking (from MCU timestamps) ----
    # Accumulate tick deltas across this packet (handle 32-bit wrap)
    last = process_payload.last_mcu_ts
    for t in time_array:
        if last is not None:
            dt = (t - last) & 0xFFFFFFFF
            if dt > 0:
                process_payload.mcu_tick_accum += dt
                process_payload.mcu_sample_accum += 1
        last = t
    process_payload.last_mcu_ts = last

    # Wall-clock duration is the span covered by the window, clamped to the window size
    if process_payload.window:
        duration_wc = max(min(now - process_payload.window[0][0], RATE_MOVING_WINDOW_SEC), 1e-6)
    else:
        duration_wc = 1e-6

    # ---- packet-based estimator (packets/sec × 60) ----
    process_payload.pkt_times.append(now)
    # trim pkt deque to window
    while process_payload.pkt_times and process_payload.pkt_times[0] < cutoff:
        process_payload.pkt_times.popleft()
    if len(process_payload.pkt_times) >= 2:
        duration_pkts = max(process_payload.pkt_times[-1] - process_payload.pkt_times[0], 1e-6)
        sps_pkts = (len(process_payload.pkt_times) - 1) / duration_pkts * 60.0
    else:
        sps_pkts = 0.0    

    # Periodic reporting
    if (now - process_payload.last_report) >= RATE_REPORT_PERIOD_SEC:
        # Wall-clock moving average
        sps_wc = process_payload.window_samples / duration_wc
        
        # MCU timestamp-based rate
        if process_payload.mcu_tick_accum > 0:
            sps_mcu = process_payload.mcu_sample_accum / (process_payload.mcu_tick_accum * MCU_TICK_SEC)
        else:
            sps_mcu = 0.0

        log.info(
            f"Incoming sample rate: {sps_wc:8.1f} SPS (avg {RATE_MOVING_WINDOW_SEC:.1f}s)"
            f" | pkts×60: {sps_pkts:8.1f} SPS"
            f" | MCU: {sps_mcu:8.1f} SPS"
        )
        process_payload.last_report = now
        process_payload.mcu_tick_accum = 0
        process_payload.mcu_sample_accum = 0

# static vars
process_payload.window = deque()
process_payload.window_samples = 0
process_payload.last_report = _now()
process_payload.last_mcu_ts = None
process_payload.mcu_tick_accum = 0
process_payload.mcu_sample_accum = 0
process_payload.pkt_times = deque()
 
# MAIN LOOP
try:
    while True:
        sock.settimeout(2.0)  # seconds
        try:
            data, addr = sock.recvfrom(PACKET_SIZE)
        except socket.timeout:
            log.warning("No data received within 2 seconds.")
        #data, addr = sock.recvfrom(PACKET_SIZE)  # Receive packet
         #os.delay(1000)
        log.debug(f"rx {len(data)} B from {addr}")
        if addr[0] == UDP_IP and addr[1] == UDP_PORT:
        #if addr[0] == UDP_IP:   # only check IP, not port
            data_payload = data[0:]  # UDP header is removed
            process_payload(data_payload)  # Process the payload (function above)
            packet_idx += 1
            if packet_idx == NUM_PACKETS_PER_FILE:
                log.info("All packets processed for current file. Next file initialized.")
                packet_idx = 0
                file_idx += 1
                if NUM_FILES != -1 and file_idx >= NUM_FILES:
                    log.info("All specified files processed. Exiting...")
                    break
                else:
                    filename = os.path.join(full_path, generate_filename(file_idx))
                    log.info(f"Writing to new file: {filename}")
        else:
            log.warning(f"Ignored packet from {addr}")  # Ignore packets from other addresses/ports
except KeyboardInterrupt:
    log.info("Server stopped.")
finally:
    sock.close()