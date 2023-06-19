import numpy as np
import sys
import struct
import time
import threading
from collections import namedtuple
import serial
from crc import Calculator, Configuration

class ForceSensor:

    BOTA_PRODUCT_CODE = 123456
    BAUDERATE = 460800
    SINC_LENGTH = 512
    CHOP_ENABLE = 0
    FAST_ENABLE = 0
    FIR_DISABLE = 1
    TEMP_COMPENSATION = 0  # 0: Disabled (recommended), 1: Enabled
    USE_CALIBRATION = 1  # 1: calibration matrix active, 0: raw measurements
    DATA_FORMAT = 0  # 0: binary, 1: CSV
    BAUDERATE_CONFIG = 4  # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    FRAME_HEADER = b"\xAA"
    # Note that the time step is set according to the sinc filter size!
    time_step = 0.01

    def __init__(self, port, force_offset):
        self._port = port
        self._ser = serial.Serial()
        self._pd_thread_stop_event = threading.Event()
        DeviceSet = namedtuple("DeviceSet", "name product_code config_func")
        self._expected_device_layout = {
            0: DeviceSet(
                "BFT-SENS-SER-M8", self.BOTA_PRODUCT_CODE, self.force_sensor_setup
            )
        }

        self.prev_force = np.zeros(3)
        self.force_offset = force_offset

    #    self.writer = SummaryWriter(f"runs/force_readings")

    def force_sensor_setup(self):

        self._ser.baudrate = self.BAUDERATE
        self._ser.port = self._port
        self._ser.timeout = 10

        try:
            self._ser.open()
        except:
            print("Could not open port")

        if not self._ser.is_open:
            print("Could not open port")

        # print("bota_sensor_setup")
        # Wait for streaming of data
        self._ser.read_until(bytes("App Init", "ascii"))
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Go to CONFIG mode
        cmd = bytes("C", "ascii")
        self._ser.write(cmd)
        self._ser.read_until(bytes("r,0,C,0", "ascii"))

        # Communication setup
        comm_setup = f"c,{self.TEMP_COMPENSATION},{self.USE_CALIBRATION},{self.DATA_FORMAT},{self.BAUDERATE_CONFIG}"
        # print(comm_setup)
        cmd = bytes(comm_setup, "ascii")
        self._ser.write(cmd)
        self._ser.read_until(bytes("r,0,c,0", "ascii"))
        time_step = 0.00001953125 * self.SINC_LENGTH
        # print("Timestep: {}".format(time_step))

        # Filter setup
        filter_setup = f"f,{self.SINC_LENGTH},{self.CHOP_ENABLE},{self.FAST_ENABLE},{self.FIR_DISABLE}"
        # print(filter_setup)
        cmd = bytes(filter_setup, "ascii")
        self._ser.write(cmd)
        self._ser.read_until(bytes("r,0,f,0", "ascii"))

        # Go to RUN mode
        cmd = bytes("R", "ascii")
        self._ser.write(cmd)
        self._ser.read_until(bytes("r,0,R,0", "ascii"))


    def get_force_obs(self):
        # while True:
        self._ser.flushInput()
        self._ser.flushOutput()
        frame_synced = False
        crc16X25Configuration = Configuration(16, 0x1021, 0xFFFF, 0xFFFF, True, True)
        crc_calculator = Calculator(crc16X25Configuration)
        while not frame_synced and not self._pd_thread_stop_event.is_set():
            possible_header = self._ser.read(1)
            if self.FRAME_HEADER == possible_header:
                data_frame = self._ser.read(34)
                crc16_ccitt_frame = self._ser.read(2)
                crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                checksum = crc_calculator.checksum(data_frame)
                if checksum == crc16_ccitt:
                    # print("Frame synced")
                    frame_synced = True
                else:
                    self._ser.read(1)
           
        start_time = time.perf_counter()
        frame_header = self._ser.read(1)

        if frame_header != self.FRAME_HEADER:
            print("Lost sync")
            data_frame = self._ser.read(34)
            crc16_ccitt_frame = self._ser.read(2)
            return self.prev_force

        data_frame = self._ser.read(34)
        crc16_ccitt_frame = self._ser.read(2)

        crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
        checksum = crc_calculator.checksum(data_frame)
        if checksum != crc16_ccitt:
            print("CRC mismatch received")
            return self.prev_force

        status = struct.unpack_from('H', data_frame, 0)[0]
        Fx = struct.unpack_from('f', data_frame, 2)[0]
        Fy = struct.unpack_from('f', data_frame, 6)[0]
        Fz = struct.unpack_from('f', data_frame, 10)[0]
        Mx = struct.unpack_from('f', data_frame, 14)[0]
        My = struct.unpack_from('f', data_frame, 18)[0]
        Mz = struct.unpack_from('f', data_frame, 22)[0]
        timestamp = struct.unpack_from('I', data_frame, 26)[0]
        temperature = struct.unpack_from('f', data_frame, 30)[0]
        self.prev_force = np.array([Fx,Fy,Fz]) - self.force_offset
        return self.prev_force

    @staticmethod
    def _sleep(duration, get_now=time.perf_counter):
        now = get_now()
        end = now + duration
        while now < end:
            now = get_now()
