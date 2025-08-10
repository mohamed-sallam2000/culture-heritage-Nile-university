#!/usr/bin/env python3

import serial
import serial.tools.list_ports
import time
import csv
import datetime
import glob
import logging
import re
import os # Needed for checking file existence
import cv2 
# --- Configuration ---
INPUT_CSV_PATH = "/home/clt/Downloads/Environmental-test-chamber-main/Sample CSV input.csv"
OUTPUT_CSV_PATH = "/home/clt/Downloads/Environmental-test-chamber-main/sensor_log.csv "
SERIAL_BAUD_RATE = 115200
SERIAL_TIMEOUT = 1  # seconds for read timeout
RECONNECT_DELAY = 5 # seconds
LOOP_DELAY = 0.1 # seconds between loop iterations
LOG_LEVEL = logging.INFO
SECONDS_PER_DAY = 24 * 60 * 60
OUTPUT_CSV_HEADER = ["Timestamp", "Temperature_C", "Humidity_RH", "Lux", "UVA", "UVB", "UVC"]
save_dir = "/home/clt/Downloads/Environmental-test-chamber-main/captured_images"
# --- Setup Logging ---
logging.basicConfig(level=LOG_LEVEL,
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    handlers=[logging.StreamHandler()]) # Log to console

# --- Global Variables ---
ser = None # Serial connection object
start_time = None # Will be set when script starts running main loop
schedule = [] # List to hold parsed schedule events
last_known_state = {
    "temp": None,
    "LEDS": None,
    "Rain": None,
    "Camera":False
}
output_csv_writer = None
output_csv_file = None
pico_port = None # Store the detected port globally
camera_index=0
camera_on=False
# --- Functions ---
#########canmera script  functions  ##############
def find_camera():
    """Tries to find a connected USB camera."""
    # Check indices 0 to 9 for cameras
    for index in range(10):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            print(f"Camera found at index {index}")
            cap.release() # Release immediately after finding
            return index
    return -1 # Return -1 if no camera is found


def find_pico_port():
    """Find the serial port corresponding to the Raspberry Pi Pico."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == 0x2E8A and port.pid == 0x0005:
            logging.info(f"Found Raspberry Pi Pico at {port.device}")
            return port.device

    pico_ports = glob.glob("/dev/ttyACM*")
    if pico_ports:
        logging.warning(f"Pico VID/PID not found. Found potential Pico port(s) using glob: {pico_ports}. Using first one: {pico_ports[0]}")
        return pico_ports[0]

    logging.error("Could not find Raspberry Pi Pico serial port.")
    return None

def connect_serial(port):
    """Establish serial connection to the specified port."""
    global ser
    if ser and ser.is_open:
        logging.debug("Serial connection already open.")
        return True
    try:
        ser = serial.Serial(port, SERIAL_BAUD_RATE, timeout=SERIAL_TIMEOUT)
        time.sleep(2) # Wait for Pico to initialize
        ser.flushInput()
        logging.info(f"Successfully connected to55555 Pico on {port}")
        return True
    except serial.SerialException as e:
        logging.error(f"Failed to connect to {port}: {e}")
        ser = None
        return False
    except Exception as e:
        logging.error(f"An unexpected error occurred during serial connection: {e}")
        ser = None
        return False

def parse_schedule(csv_path):
    """Parses the control schedule from the input CSV file."""
    global schedule
    schedule = [] # Clear previous schedule if any
    current_time_offset_sec = 0
    header_found = False

    try:
        with open(csv_path, mode='r', encoding='utf-8') as infile:
            reader = csv.reader(infile)
            for row in reader:
                if not row or not row[0]: # Skip empty rows
                    continue

                # Find the header row
                if row[0].strip() == "Module" and row[1].strip() == "Value" and row[2].strip() == "Days":
                    header_found = True
                    continue

                if not header_found:
                    continue # Skip rows before the header

                # --- Parse data rows ---
                module_raw = row[0].strip()
                value_raw = row[1].strip()
                days_raw = row[2].strip()

                # Calculate duration in seconds
                try:
                    if '/' in days_raw: # Handle fractional days like "1/24"
                        num, den = map(float, days_raw.split('/'))
                        duration_days = num / den
                    else:
                        duration_days = float(days_raw)
                    duration_sec = duration_days 
                    #* SECONDS_PER_DAY
                except ValueError:
                    logging.warning(f"Could not parse days value \'{days_raw}\' for module \'{module_raw}\'. Skipping row.")
                    continue

                start_sec = current_time_offset_sec
                end_sec = start_sec + duration_sec

                # --- Process based on module type ---
                if module_raw.startswith("Temp"):
                    try:
                        # Extract temperature value (e.g., "15 °C" -> 15.0)
                        temp_match = re.match(r"([\d\.]+)", value_raw)
                        if temp_match:
                            temp_value = float(temp_match.group(1))
                            schedule.append({
                                "type": "temp",
                                "value": temp_value,
                                "start_sec": start_sec,
                                "end_sec": end_sec
                            })
                            # Temperature steps are sequential
                            current_time_offset_sec = end_sec
                           #logging.info(f"Successffinish temp{len(schedule)} events from {csv_path}")
        
                        else:
                             logging.warning(f"Could not parse temperature value \'{value_raw}\'. Skipping row.")
                    except ValueError:
                        logging.warning(f"Could not parse temperature value \'{value_raw}\'. Skipping row.")

                elif module_raw.startswith("Light"):
					
					
                    light_state = "ON" if value_raw == "100" else "OFF"
                    schedule.append({
                        "type": "Light",
                        "state": light_state,
                         "start_sec": start_sec,
                        "end_sec": start_sec + duration_sec # Ends after its duration
                    })
					 # Temperature steps are sequential
                    current_time_offset_sec = end_sec

                elif  module_raw.startswith("Rain"):
					
                    rain_state = "ON" if value_raw == "100" else "OFF"
                    schedule.append({
                        "type": "Rain",
                        "state": rain_state,
                        "start_sec": start_sec,
                        "end_sec": start_sec + duration_sec # Ends after its duration
                    })
                     # Temperature steps are sequential
                    current_time_offset_sec = end_sec
                elif  module_raw.startswith("Camera"):
					
					
                    Camera_state = True if value_raw == True else False
                    schedule.append({
                        "type": "Camera",
                        "state": Camera_state,
                        "start_sec": start_sec,
                        "end_sec": start_sec + duration_sec # Ends after its duration
                    })
                     # Temperature steps are sequential
                    current_time_offset_sec = end_sec
                else:
					
                    logging.warning(f"Unknown module type: \'{module_raw}\'. Skipping row.")

        if not header_found:
            logging.error(f"Could not find header row \'Module,Value,Days\' in {csv_path}")
            return False
        if not schedule:
            logging.error(f"No valid schedule entries found in {csv_path}")
            return False

        logging.info(f"Successfully parsed schedule with {len(schedule)} events from {csv_path}")
        return True

    except FileNotFoundError:
        logging.error(f"Input CSV file not found: {csv_path}")
        return False
    except Exception as e:
        logging.error(f"Error parsing schedule CSV: {e}")
        return False
        
        
def cam_cap():
    """Trigger the camera capture process."""
    logging.info(" Camera capture triggered.")
    find_camera()
    if camera_index == -1:
        logging.info("Error: No USB camera detected. Please ensure a camera is connected.")
        return

    logging.info(f"Connecting to camera at index {camera_index}...")
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        logging.info(f"Error: Could not open camera at index {camera_index}.")
        return

    logging.info("Camera connected successfully.")

    # Get interval from user
    interval_hours = 0.5
    interval_seconds = interval_hours * 3600
    logging.info(f"Capturing images every {interval_hours} hours ({interval_seconds} seconds).")
    logging.info("Press 'q' in the image window to quit.")

    window_name = "USB Camera Capture"
    cv2.namedWindow(window_name)

    try:
    
        # Capture frame-by-frame
        ret, frame = cap.read()

        # if frame is read correctly ret is True
        if not ret:
            logging.info("Error: Can't receive frame (stream end?). Exiting ...")
            

        # Get timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(save_dir, f"capture_{timestamp}.jpg")

        # Save the resulting frame
        cv2.imwrite(filename, frame)
        logging.info(f"Image saved: {filename}")
        raise KeyboardInterrupt
        # Display the resulting frame
        # cv2.imshow(window_name, frame)

        # Wait for the specified interval OR check for quit key ('q')
        # cv2.waitKey waits for a key event for a specific number of milliseconds.
        # We need to wait for interval_seconds, but also check for 'q' periodically.
        # wait_start_time = time.time()
        # while time.time() - wait_start_time < interval_seconds:
        #     # Check for 'q' key every 100ms
        #     if cv2.waitKey(100) & 0xFF == ord('q'):
        #         logging.info("Quit key pressed. Exiting...")
        #         raise KeyboardInterrupt # Use an exception to break out of the outer loop
        #     # Allow GUI events to process
        #     time.sleep(0.01) # Small sleep to prevent busy-waiting

    except KeyboardInterrupt:
        logging.info("Stopping capture loop.")
    finally:
        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()
        logging.info("Camera released and windows closed.")
        
        ############      

def send_command(command):
    """Sends a command string to the connected Pico."""
    global ser
    if ser and ser.is_open:
        try:
            full_command = command + '\n'
            ser.write(full_command.encode('utf-8'))
            logging.info(f"Sent command: {command}")
            return True
        except serial.SerialException as e:
            logging.error(f"Serial error while sending command \'{command}\': {e}")
            ser = None # Assume connection is lost
            return False
        except Exception as e:
            logging.error(f"Unexpected error sending command \'{command}\': {e}")
            return False
    else:
        logging.warning(f"Cannot send command \'{command}\'. Serial port not connected.")
        return False

def read_serial_data():
    """Reads a line of text from the serial port, if available."""
    global ser
    if ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                line = ser.readline()
                decoded_line = line.decode('utf-8', errors='ignore').strip()
                if decoded_line:
                    logging.debug(f"Received serial data: {decoded_line}")
                    return decoded_line
                else:
                    return None
            else:
                return None
        except serial.SerialException as e:
            logging.error(f"Serial error while reading data: {e}")
            ser = None
            return None
        except Exception as e:
            logging.error(f"Unexpected error reading serial data: {e}")
            return None
    else:
        return None

def setup_csv_logging(filepath):
    """Initializes the output CSV file and writer."""
    global output_csv_writer, output_csv_file
    file_exists = os.path.isfile(filepath)
    try:
        # Use newline=\'\' to prevent extra blank rows in CSV
        output_csv_file = open(filepath, mode='a', newline= '\n' , encoding= "utf-8")
        output_csv_writer = csv.writer(output_csv_file)
        if not file_exists or os.path.getsize(filepath) == 0:
            output_csv_writer.writerow(OUTPUT_CSV_HEADER)
            output_csv_file.flush() # Ensure header is written immediately
            logging.info(f"Created/initialized output CSV: {filepath}")
        else:
            logging.info(f"Appending to existing output CSV: {filepath}")
        return True
    except IOError as e:
        logging.error(f"Failed to open or write header to output CSV {filepath}: {e}")
        output_csv_writer = None
        output_csv_file = None
        return False

def parse_and_log_sensor_data(data_line):
    """Parses a sensor data string and logs it to the CSV file."""
    global output_csv_writer, output_csv_file
    if not output_csv_writer or not output_csv_file:
        logging.warning("CSV logger not initialized. Cannot log data.")
        return

    # Example line: "Temp: 24.50°C | Humidity: 55.20% | Lux: 150.00 | UVA:0.15, UVB:0.25, UVC:0.02"
    temp_match = re.search(r"Temp: ([\d\.]+)", data_line)
    humidity_match = re.search(r"Humidity: ([\d\.]+)", data_line)
    lux_match = re.search(r"Lux: ([\d\.]+)", data_line)
    uva_match = re.search(r"UVA:([\d\.]+)", data_line)
    uvb_match = re.search(r"UVB:([\d\.]+)", data_line)
    uvc_match = re.search(r"UVC:([\d\.]+)", data_line)

    if temp_match and humidity_match and lux_match and uva_match and uvb_match and uvc_match:
        try:
            timestamp = datetime.datetime.now().isoformat()
            temp = float(temp_match.group(1))
            humidity = float(humidity_match.group(1))
            lux = float(lux_match.group(1))
            uva = float(uva_match.group(1))
            uvb = float(uvb_match.group(1))
            uvc = float(uvc_match.group(1))

            log_row = [timestamp, temp, humidity, lux, uva, uvb, uvc]
            output_csv_writer.writerow(log_row)
            output_csv_file.flush() # Ensure data is written to disk periodically
            logging.debug(f"Logged sensor data: {log_row}")
        except ValueError as e:
            logging.warning(f"Could not parse values from sensor data line: \'{data_line}\'. Error: {e}")
        except IOError as e:
            logging.error(f"IOError writing to CSV: {e}")
        except Exception as e:
            logging.error(f"Unexpected error logging data: {e}")
    else:
        # Log lines that don\'t match the expected sensor format, but aren\'t errors from Pico
        if not ("Error" in data_line or "failed" in data_line or "ACTIVATED" in data_line or "STABLE" in data_line or "ON" in data_line or "OFF" in data_line or "Target Temp" in data_line or "Invalid command" in data_line or "System Ready" in data_line):
             logging.warning(f"Received non-sensor data line: \'{data_line}\'")

def get_scheduled_state(elapsed_seconds):
    """Determines the target state based on the schedule and elapsed time."""
    target_temp = None
    target_leds = "OFF" # Default state
    target_rain = "OFF" # Default state
    target_cam  = None
    global camera_on
    for event in schedule:
        if event["start_sec"] <= elapsed_seconds < event["end_sec"]:
            if event["type"] == "temp":
                target_temp = event["value"]
            elif event["type"] == "Light":
                target_leds = event["state"]
            elif event["type"] == "Rain":
                target_rain = event["state"]
            elif event["type"] == "Camera":
                target_cam = event["state"]
                camera_on=True

    # If no temp schedule matches current time, find the last one that should have run
    # (This handles the case after the last temp entry finishes)
    if target_temp is None:
        last_temp_event = None
        for event in schedule:
            if event["type"] == "temp":
                if event["start_sec"] <= elapsed_seconds:
                    last_temp_event = event
                else:
                    break # Temp events are sequential
        if last_temp_event:
            target_temp = last_temp_event["value"]
        else: # Should not happen if schedule has temp entries
            logging.warning("Could not determine target temperature for current time.")

################


    # If no temp schedule matches current time, find the last one that should have run
    # (This handles the case after the last temp entry finishes)
    if target_leds is None:
        last_LEDs_event = None
        for event in schedule:
            if event["type"] == "LEDs":
                if event["start_sec"] <= elapsed_seconds:
                    last_LEDs_event = event
                else:
                    break # Temp events are sequential
        if last_LEDs_event:
            target_leds = last_LEDs_event["state"]
        else: # Should not happen if schedule has temp entries
            logging.warning("Could not determine target temperature for current time.")





############

    return {"temp": target_temp, "LEDS": target_leds, "Rain": target_rain,"Camera":target_cam}

def main_loop():
    """Main control loop for the Pico controller."""
    global camera_on,camera_index,ser, start_time, last_known_state, pico_port,save_dir

    start_time = time.time()
    logging.info("Starting main control loop...")
    os.makedirs(save_dir, exist_ok=True) # Create directory if it doesn't exist
    logging.info(f"Images will be saved in the '{save_dir}' directory.")


    # camera_index = find_camera()
    
    while True:
        try:
            # --- Check Connection ---
            if not ser or not ser.is_open:
                logging.warning("Serial connection lost. Attempting to reconnect...")
                if pico_port is None:
                    #pico_port = "/dev/serial0"       or "/dev/ttyAMA0" if you prefer

                    pico_port = find_pico_port()   
                if pico_port and connect_serial(pico_port):
                    logging.info("Reconnected successfully.")
                    # Resend last known state upon reconnection?
                    # For simplicity, we wait for the next schedule check.
                else:
                    logging.error(f"Reconnect failed. Retrying in {RECONNECT_DELAY} seconds.")
                    time.sleep(RECONNECT_DELAY)
                    continue # Skip rest of loop and try reconnecting again

            # --- Read and Log Sensor Data ---
            serial_line = read_serial_data()
            if serial_line:
                parse_and_log_sensor_data(serial_line)

            # --- Check Schedule and Update State ---
            elapsed_time = time.time() - start_time
            current_target_state = get_scheduled_state(elapsed_time)

            # Update Temperature
            if current_target_state["temp"] is not None and current_target_state["temp"] != last_known_state["temp"]:
                command = f"SET TEMP {current_target_state['temp']:.1f}"
                if send_command(command):
                    last_known_state["temp"] = current_target_state["temp"]

            # Update LEDs
            if current_target_state["LEDS"] != last_known_state["LEDS"]:
                command = f"{current_target_state['LEDS']} LEDS"
                if send_command(command):
                    last_known_state["LEDS"] = current_target_state["LEDS"]

            # Update Rain
            if current_target_state["Rain"] != last_known_state["Rain"]:
                command = f"{current_target_state['Rain']} Rain"
                if send_command(command):
                    last_known_state["Rain"] = current_target_state["Rain"]
                    
            if current_target_state["Camera"] != last_known_state["Camera"]:
               
                if camera_on==True:
                   
                    cam_cap()
                    current_target_state["Camera"] =False
                    last_known_state["Camera"]=False
                    camera_on=False
                #command = f"{current_target_state['Camera']} Camera"
               
            # --- Loop Delay ---
            time.sleep(LOOP_DELAY)

        except KeyboardInterrupt:
            logging.info("KeyboardInterrupt received. Shutting down...")
            break
        except Exception as e:
            logging.error(f"An unexpected error occurred in the main loop: {e}")
            # Attempt to continue running if possible, maybe add a longer delay
            time.sleep(5)

# --- Main Execution ---
if __name__ == "__main__":
    logging.info("Starting Pico Controller Script")

    if not parse_schedule(INPUT_CSV_PATH):
        logging.error("Failed to load schedule. Exiting.")
        exit(1)

    if not setup_csv_logging(OUTPUT_CSV_PATH):
        logging.error("Failed to setup CSV logging. Exiting.")
        exit(1)

    pico_port = find_pico_port()
    #pico_port = "/dev/serial0"       or "/dev/ttyAMA0" if you prefer

    if not pico_port:
        logging.error("No Pico port found. Exiting.")
        if output_csv_file:
            output_csv_file.close()
        exit(1)

    # Start the main loop
    main_loop()

    # --- Cleanup ---
    if ser and ser.is_open:
		
        # Optional: Turn things off before closing?
        send_command("OFF LEDS")
        send_command("OFF RAIN")
        ser.close()
        logging.info("Closed serial connection.")

    if output_csv_file:
        output_csv_file.close()
        logging.info("Closed output CSV file.")

    logging.info("Pico Controller Script finished.")

