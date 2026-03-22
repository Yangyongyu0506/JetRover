from .readline import ReadLine
import serial
import json
import threading
import queue


class BaseController:
    def __init__(self, uart_dev_set, baud_set, logger):
        self.logger = logger  # Logger for logging messages
        self.ser = serial.Serial(
            uart_dev_set, baud_set, timeout=1
        )  # Open serial connection
        self.rl = ReadLine(self.ser)  # Initialize ReadLine helper
        self.command_queue = queue.Queue()  # Command queue for sending data
        self.stop_event = threading.Event()
        self.command_thread = threading.Thread(
            target=self.process_commands, daemon=True
        )  # Start a separate thread for processing commands
        self.command_thread.start()
        self.data_buffer = None  # Buffer for holding received data
        # Base data structure to hold sensor values
        self.base_data = {
            "T": 1001,
            "L": 0,
            "R": 0,
            "ax": 0,
            "ay": 0,
            "az": 0,
            "gx": 0,
            "gy": 0,
            "gz": 0,
            "mx": 0,
            "my": 0,
            "mz": 0,
            "odl": 0,
            "odr": 0,
            "v": 0,
            "pan": 0,
            "tilt": 0,
        }

    # Function to read and return feedback data from the serial input
    def feedback_data(self):
        line = ""
        try:
            line_bytes = self.rl.readline()  # Read line from UART
            if not line_bytes:
                return None
            line = line_bytes.decode("utf-8", errors="replace").strip()
            if not line:
                return None
            self.data_buffer = json.loads(line)  # Parse JSON data
            if not isinstance(self.data_buffer, dict):
                self.logger.error(
                    f"Unexpected base data type: {type(self.data_buffer)}"
                )
                return None
            self.base_data = self.data_buffer  # Store received data
            return self.base_data  # Return base data
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")  # Log error
            self.rl.clear_buffer()  # Clear buffer on error
        except (serial.SerialException, OSError) as e:
            self.logger.error(f"Serial read error: {e}")
            self.rl.clear_buffer()
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()

    # Receive and decode data from the serial connection
    def on_data_received(self):
        line = ""
        try:
            self.ser.reset_input_buffer()
            line = self.rl.readline().decode("utf-8", errors="replace").strip()
            if not line:
                return None
            data_read = json.loads(line)  # Read and parse JSON data
            return data_read
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")
            self.rl.clear_buffer()
            return None
        except (serial.SerialException, OSError) as e:
            self.logger.error(f"Serial read error: {e}")
            self.rl.clear_buffer()
            return None

    # Add a command to the queue to be sent via UART
    def send_command(self, data):
        if self.stop_event.is_set():
            return
        self.command_queue.put(data)

    # Thread function to process and send commands from the queue
    def process_commands(self):
        while not self.stop_event.is_set():
            try:
                data = self.command_queue.get(timeout=0.1)  # Get command from the queue
            except queue.Empty:
                continue
            try:
                self.ser.write(
                    (json.dumps(data) + "\n").encode("utf-8")
                )  # Send command as JSON over UART
            except (serial.SerialException, OSError) as e:
                self.logger.error(f"Serial write error: {e}")
                self.stop_event.set()

    # Send control data as JSON via UART
    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

    def destroy_base(self):
        self.stop_event.set()
        if self.command_thread.is_alive():
            self.command_thread.join(timeout=1.0)
        try:
            self.ser.close()  # Close the serial connection
        except (serial.SerialException, OSError) as e:
            self.logger.error(f"Serial close error: {e}")
