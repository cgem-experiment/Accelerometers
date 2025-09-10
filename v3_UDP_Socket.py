#v2_UDP_Socket
import socket
import csv  
from datetime import datetime  
import time
import os
import logging
from pathlib import Path
import atexit
import sys


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
# Exponential moving average smoothing
EMA_ALPHA = 0.1

MCU_TICK_SEC = 1e-6

NUM_PACKETS_PER_FILE = 26400 # Number of packets to write to each file
NUM_FILES = -1  # Set to -1 for infinite, or specify the number of files 
BASE_PATH = str(Path("~/accel_hk_data").expanduser())  # Change this variable to set the base directory

# Configuration
#UDP_IP = "10.20.3.3"
UDP_IP = "192.168.1.30" #Remote (MCU) IP
UDP_PORT = 8
#LISTEN_IP = "10.20.1.3"
LISTEN_IP = "192.168.1.10" #Host (This PC) IP
LISTEN_PORT = 12345 #55151 #CHANGE IF ON SITE
#PACKET_SIZE = 601*2 + 42  # 600 bytes of data + 42 bytes UDP header
PACKET_SIZE = 2048  # Maximum number of bytes to take from UDP packet

SEPARATOR = b"\x89\xab\xcd\xef"
EXPECTED_SAMPLES_PER_PACKET = 60

# --- Housekeeping (HK) packet config ---
HK_SYNC = b"HKPK"
HK_VERSION = 1
HK_NUM_CH = 11
HK_PKT_SIZE = 8 + 4 + 4 + (2 * HK_NUM_CH)  # 38 bytes: header+seq+tick+channels

# --- Accel packet header (matches MCU "ACCL" + v1) ---
ACC_SYNC = b"ACCL"
ACC_VERSION = 1
ACC_HDR_LEN = 8  # 4 magic + 1 version + 3 pad

# Long-lived CSV writer state
FLUSH_PERIOD_SEC = 2.0           # periodic buffered flush cadence
BUFFER_BYTES = 2 * 1024 * 1024   # 2 MB user-space buffer for the file
FSYNC_PERIOD_SEC = 10.0   # force to disk this often so Explorer shows growth
metadata_filename = os.path.join(BASE_PATH, "metadata_log.txt")

if sys.stdin.isatty():
    # Interactive terminal: ask the user
    session_note = input("Notes: ")
else:
    # Non-interactive (e.g. running under systemd)
    session_note = ""

# Take timestamp and define function for filenames
timestamp = datetime.now().strftime("%Y%m%dT%H%M%S")


def generate_filename(file_idx):
    return f"{timestamp}_FILE{file_idx:04d}_cgem_accel.csv"

def hk_generate_filename(file_idx):
    return f"{timestamp}_FILE{file_idx:04d}_cgem_housekeeping.csv"

# Create the folder (accel + housekeping)
folder_name = f"{timestamp}_cgem_accel"
full_path = os.path.join(BASE_PATH, folder_name)
os.makedirs(full_path, exist_ok=True)  # Create folder if it doesn't exist

hk_folder_name = f"{timestamp}_cgem_housekeeping"
hk_full_path = os.path.join(BASE_PATH, hk_folder_name)
os.makedirs(hk_full_path, exist_ok=True)

# Accel writer state
packet_idx = 0
file_idx = 1
current_file = None
csv_writer = None
last_flush = _now()
last_fsync = _now()

# Housekeeping writer state
hk_packet_idx = 0
hk_file_idx = 1
hk_current_file = None
hk_csv_writer = None
hk_last_flush = _now()
hk_last_fsync = _now()

def open_new_file():
    """Close current file (if any) with fsync, then open the next CSV with a large buffer."""
    global current_file, csv_writer, filename, last_flush, file_idx
    if current_file:
        try:
            current_file.flush()
            os.fsync(current_file.fileno())
        except Exception:
            pass
        current_file.close()

    filename = os.path.join(full_path, generate_filename(file_idx))
    # newline="" is correct for Python csv on Windows; large 'buffering' reduces syscalls
    current_file = open(filename, mode="a", buffering=BUFFER_BYTES, newline="")
    csv_writer = csv.writer(current_file)
    last_flush = _now()
    log.info(f"Writing to new file: {filename}")

def open_new_hk_file():
    """Rotate housekeeping CSV with big user-space buffer."""
    global hk_current_file, hk_csv_writer, hk_filename, hk_last_flush, hk_file_idx
    if hk_current_file:
        try:
            hk_current_file.flush()
            os.fsync(hk_current_file.fileno())
        except Exception:
            pass
        hk_current_file.close()
    hk_filename = os.path.join(hk_full_path, hk_generate_filename(hk_file_idx))
    hk_current_file = open(hk_filename, mode="a", buffering=BUFFER_BYTES, newline="")
    hk_csv_writer = csv.writer(hk_current_file)
    hk_last_flush = _now()
    log.info(f"Writing housekeeping to: {hk_filename}")    

@atexit.register
def _close_file_at_exit():
    """Make sure the last file hits disk on normal exit or Ctrl+C."""
    if current_file:
        try:
            current_file.flush()
            os.fsync(current_file.fileno())
        except Exception:
            pass
        current_file.close()
    # Close socket
    if 'hk_current_file' in globals() and hk_current_file:
        try:
            hk_current_file.flush()
            os.fsync(hk_current_file.fileno())
        except Exception:
            pass
        hk_current_file.close()    
    try:
        if 'sock' in globals() and sock:
            sock.close()
            log.info("Closed UDP socket at exit")
    except Exception:
        pass

# Open the first CSV now so we can record its exact path in metadata
open_new_file()
open_new_hk_file()

def _ema(prev, x):
    return x if prev is None else (1.0 - EMA_ALPHA) * prev + EMA_ALPHA * x
# Initialize the metadata log
with open(metadata_filename, "a") as meta_file:
    meta_file.write(f"folder path: {full_path}\n")
    meta_file.write(f"initial file path: {filename}\n")
    meta_file.write(f"housekeeping folder path: {hk_full_path}\n")
    meta_file.write(f"initial housekeeping file path: {hk_filename}\n")    
    meta_file.write(f"Session note: {session_note}\n\n")

# Configure ethernet socket
protocol = socket.SOCK_DGRAM  # SOCK_DGRAM is for UDP
ip_family = socket.AF_INET  # AF_INET is for ipv4
sock = socket.socket(ip_family, protocol)
# Increase UDP receive buffer (helps absorb brief pauses)
try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFFER_BYTES * 2)
except Exception as e:
    log.debug(f"SO_RCVBUF set failed: {e}")
try:
    actual_rcvbuf = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
    log.info(f"UDP recv buffer: {actual_rcvbuf} bytes")
except Exception:
    pass

# Bind to the specific IP and port
try:
    sock.bind((LISTEN_IP, LISTEN_PORT))
    log.info(" Bind successful. Listening for data...") # Can try using sock.bind(("0.0.0.0", LISTEN_PORT)) instead if having problems
except OSError as e:
    log.error(f" Bind failed: {e}")

# Print confirmation
log.info(f"Listening for UDP packets from {UDP_IP} on port {LISTEN_PORT}...")
log.info(f"Folder created at {full_path}")

def process_payload(payload, addr):

    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    current_time_ns = time.time_ns() % 1_000_000_000
    
    parts = payload.split(SEPARATOR)
    # trailer after the last separator holds the 16-bit sampleNum (LE)
    tail = parts[-1] if parts else b""
    sample_num = int.from_bytes(tail[:2], "little") if len(tail) >= 2 else None
    samples_bytes = parts[:-1]  # the 60 real 16-byte records
    # Detect dropped/reordered UDP packets via sampleNum (not saved)
    if sample_num is not None:
        prev = getattr(process_payload, "prev_sample_num", None)
        if prev is not None:
            step = (sample_num - prev) & 0xFFFF
            if step == 0:
                process_payload.packet_drop_count += 1
                log.warning(
                    f"packet loss/reorder: sampleNum {prev} -> {sample_num} (Δ=0) "
                    f"from {addr}"
                )
                return None
            if step != 1:
                process_payload.packet_drop_count += 1
                log.warning(f"packet loss/reorder: sampleNum {prev} -> {sample_num} (Δ={step})")
        process_payload.prev_sample_num = sample_num
    if len(samples_bytes) != EXPECTED_SAMPLES_PER_PACKET:
        log.warning(f"packet had {len(samples_bytes)} samples; trailing tail={len(tail)}B")
    
    value0_array = []
    value1_array = []
    value2_array = []
    status0_array = []
    status1_array = []
    status2_array = []
    time_array = []

    for i, chunk in enumerate(samples_bytes):
        if len(chunk) != 16:
            # malformed record—skip; should be rare
            log.warning(f"Skipping malformed sample at index {i} (len={len(chunk)})")
            continue
    
        try:
            # Layout per ISR:
            # bytes 0..1: ch0 low16 (LE),  2: ch0_id,  3: ch0 high8
            # bytes 4..5: ch1 low16 (LE),  6: ch1_id,  7: ch1 high8
            # bytes 8..9: ch2 low16 (LE), 10: ch2_id, 11: ch2 high8
            # bytes 12..15: TIM23 timestamp (low16 then high16) is 32-bit LE
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
    
    log.debug(f"{len(write)} fields written to file. First value: {write[0]}")

    # ---- sample-rate tracking (simple per-period counters) ----
    n = len(value0_array)  # samples in this packet
    now = _now()
    process_payload.period_samples += n
    process_payload.packets_period += 1

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


    # Periodic reporting
    if (now - process_payload.last_report) >= RATE_REPORT_PERIOD_SEC:
        # Unbiased per-period rates
        dt = max(now - process_payload.last_report, 1e-6)
        sps_wc   = process_payload.period_samples / dt
        sps_pkts = (process_payload.packets_period * 60.0) / dt
        
        # MCU timestamp-based rate
        if process_payload.mcu_tick_accum > 0:
            sps_mcu = process_payload.mcu_sample_accum / (process_payload.mcu_tick_accum * MCU_TICK_SEC)
        else:
            sps_mcu = 0.0

        # Estimate the *actual* MCU tick from packets×60 (trusted on-wire truth)
        # tick_freq_est [Hz] = (ticks/sample) * (samples/sec) = (mcu_tick_accum/mcu_sample_accum) * sps_pkts
        if process_payload.mcu_sample_accum > 0 and sps_pkts > 0:
            tick_freq_est = (process_payload.mcu_tick_accum / process_payload.mcu_sample_accum) * sps_pkts
            tick_sec_est  = 1.0 / max(tick_freq_est, 1e-9)
            ppm = (tick_freq_est / 1_000_000.0 - 1.0) * 1_000_000.0
            tick_str = f"{tick_freq_est/1e6:.6f} MHz ({ppm:+.0f} ppm)"
        else:
            tick_str = "n/a"

        # EMA smoothing
        process_payload.sps_pkts_ema = _ema(process_payload.sps_pkts_ema, sps_pkts)
        process_payload.sps_wc_ema   = _ema(process_payload.sps_wc_ema,   sps_wc)

        log.info(
            f"Incoming(avg): {process_payload.sps_pkts_ema:8.1f} SPS"
            f" | wall(avg): {process_payload.sps_wc_ema:8.1f} SPS (period ~{dt:.2f}s)"
            f" | MCU: {sps_mcu:8.1f} SPS @ tick≈{tick_str}"
            f" | drops: {process_payload.packet_drop_count}"            
        )
        process_payload.last_report = now
        process_payload.mcu_tick_accum = 0
        process_payload.mcu_sample_accum = 0
        process_payload.period_samples = 0
        process_payload.packets_period = 0
        process_payload.packet_drop_count = 0
    return write

# static vars
process_payload.period_samples = 0
process_payload.packets_period = 0
process_payload.last_report = _now()
process_payload.last_mcu_ts = None
process_payload.mcu_tick_accum = 0
process_payload.mcu_sample_accum = 0
process_payload.sps_pkts_ema = None
process_payload.sps_wc_ema   = None
process_payload.prev_sample_num = None
process_payload.packet_drop_count = 0

def process_hk_payload(payload):
    # Expect: [sync(4)][ver(1)][pad(3)][seq u32][tick u32][11*uint16]
    if len(payload) < HK_PKT_SIZE or payload[:4] != HK_SYNC or payload[4] != HK_VERSION:
        return None
    # Unpack without struct for portability/style
    seq   = int.from_bytes(payload[8:12],  "little")
    tick  = int.from_bytes(payload[12:16], "little")
    chraw = [int.from_bytes(payload[16+2*i:18+2*i], "little") for i in range(HK_NUM_CH)]
    now_txt = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    now_ns  = time.time_ns() % 1_000_000_000
    # Row: seq, tick, 11 raw codes, host_time_str, host_time_ns
    return [seq, tick, *chraw, now_txt, now_ns]

# MAIN LOOP
sock.settimeout(2.0)  # seconds
try:
    while True:
        try:
            data, addr = sock.recvfrom(PACKET_SIZE)
            log.debug(f"rx {len(data)} B from {addr}")
        except socket.timeout:
            log.info("No data received within 2 seconds.")
            continue
        #data, addr = sock.recvfrom(PACKET_SIZE)  # Receive packet
         #os.delay(1000)
        if addr[0] == UDP_IP:  # accept MCU regardless of source port; demux by content
            data_payload = data  # already payload for user; we don't prepend headers in MCU
            # 1) Housekeeping packets start with "HKPK"
            if len(data_payload) >= 8 and data_payload[:4] == HK_SYNC:
                hk_row = process_hk_payload(data_payload)
                if hk_row is not None:
                    hk_csv_writer.writerow(hk_row)
                    hk_packet_idx += 1
                    # buffered flush/fsync (same cadence as accel)
                    nowt = _now()
                    if (nowt - hk_last_flush) >= FLUSH_PERIOD_SEC:
                        try:
                            hk_current_file.flush()
                            if (nowt - hk_last_fsync) >= FSYNC_PERIOD_SEC:
                                os.fsync(hk_current_file.fileno())
                                hk_last_fsync = nowt
                        except Exception as e:
                            log.warning(f"HK flush/fsync failed: {e}")
                        hk_last_flush = nowt
                    if hk_packet_idx == NUM_PACKETS_PER_FILE:
                        log.info("HK file limit reached; rotating.")
                        hk_packet_idx = 0
                        hk_file_idx += 1
                        open_new_hk_file()

            # 2) Else treat as accelerometer packet (existing path/format)
            elif len(data_payload) >= ACC_HDR_LEN and data_payload[:4] == ACC_SYNC and data_payload[4] == ACC_VERSION:
                # remove accel header and send rest to parser
                row = process_payload(data_payload[ACC_HDR_LEN:], addr)
                if row is not None:
                    csv_writer.writerow(row)
                packet_idx += 1
                nowt = _now()
                if (nowt - last_flush) >= FLUSH_PERIOD_SEC:
                    try:
                        current_file.flush()
                        if (nowt - last_fsync) >= FSYNC_PERIOD_SEC:
                            os.fsync(current_file.fileno())
                            last_fsync = nowt
                    except Exception as e:
                        log.warning(f"flush/fsync failed: {e}")
                    last_flush = nowt
                if packet_idx == NUM_PACKETS_PER_FILE:
                    log.info("All packets processed for current file. Next file initialized.")
                    packet_idx = 0
                    file_idx += 1
                    if NUM_FILES != -1 and file_idx >= NUM_FILES:
                        log.info("All specified files processed. Exiting...")
                        break
                    open_new_file()
        else:
            log.debug(f"Ignored packet from {addr}")
except KeyboardInterrupt:
    log.info("Server stopped.")