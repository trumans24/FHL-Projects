import threading
import pandas as pd
import time
import os

class DataSource():
    def __init__(self, name) -> None:
        self.name = name
        self.recording = False
        self.data = []
        self.time_origin = time.perf_counter()
        self.time_end = None

    def connect(self) -> bool:
        ...
        
    def disconnect(self):
        ...

    def trigger(self, trigger_event):
        self.time_origin = trigger_event
        self.recording = True
    
    def clear_data(self):
        self.data = []

    def get_data(self):
        ...

    def disconnect(self):
        self.board.close()
        
    def save(self, file_path):
        df = self.get_data()
        dir = f'{file_path}_{self.name}'
        if not os.path.isdir(dir):
            os.mkdir(dir)
        path = dir + f'/{self.name}.csv'
        counter = 1
        while os.path.exists(path):
            print(f'{path} already exists. Check input parameters.')
            path = dir + f'/{self.name} ({counter}).csv'
            counter += 1
        df.to_csv(path, index=False, mode='x')


class PosSampler(DataSource):
    def __init__(self, name, arm) -> None:
        super().__init__(name)
        self.arm = arm
        self.proc_thread = None
        self._pd_thread_stop_event = threading.Event()

    def _processdata_thread(self):
        frame_synced = False
        while not self._pd_thread_stop_event.is_set():
            while not frame_synced and not self._pd_thread_stop_event.is_set():
                frame_synced = True

            while frame_synced and not self._pd_thread_stop_event.is_set():
                # start_time = time.perf_counter()
                t = time.perf_counter() - self.time_origin
                state, (x, y, z, theta_x, theta_y, theta_z) = self.arm.get_position()
                if self.recording:
                    self.data.append((t, x, y, z, theta_x, theta_y, theta_z))
                time.sleep(2e-3)
                # time_diff = time.perf_counter() - start_time
                
    def connect(self):
        proc_thread = threading.Thread(target=self._processdata_thread)
        proc_thread.start()
        self.proc_thread = proc_thread
        return True

    def disconnect(self):
        self._pd_thread_stop_event.set()
        self.proc_thread.join()
        
    def get_data(self):
        df = pd.DataFrame(self.data, columns=['t', 'x', 'y', 'z', 'theta_x', 'theta_y', 'theta_z'])
        return df



import sys
import struct
import serial

from collections import namedtuple
from crc import Calculator, Configuration

class FTSampler(DataSource):

    BOTA_PRODUCT_CODE = 123456
    BAUDERATE = 460800
    SINC_LENGTH = 57 # 512  #
    CHOP_ENABLE = 0
    FAST_ENABLE = 0
    FIR_DISABLE = 1
    TEMP_COMPENSATION = 0 # 0: Disabled (recommended), 1: Enabled
    USE_CALIBRATION = 1 # 1: calibration matrix active, 0: raw measurements
    DATA_FORMAT = 0 # 0: binary, 1: CSV
    BAUDERATE_CONFIG = 4 # 0: 9600, 1: 57600, 2: 115200, 3: 230400, 4: 460800
    FRAME_HEADER = b'\xAA'
    # Note that the time step is set according to the sinc filter size!
    time_step = 0.01

    def __init__(self, name, port) -> None:
        super().__init__(name)
        self._port = port
        self._ser = serial.Serial()
        self._pd_thread_stop_event = threading.Event()
        DeviceSet = namedtuple('DeviceSet', 'name product_code config_func')
        self._expected_device_layout = {0: DeviceSet('BFT-SENS-SER-M8', self.BOTA_PRODUCT_CODE, self.bota_sensor_setup)}
        self._status = None
        self._fx = 0.0
        self._fy = 0.0
        self._fz = 0.0
        self._mx = 0.0
        self._my = 0.0
        self._mz = 0.0
        self._timestamp = 0.0
        self._temperature = 0.0
        self._fx_bias = 0.0
        self._fy_bias = 0.0
        self._fz_bias = 0.0
        self._mx_bias = 0.0
        self._my_bias = 0.0
        self._mz_bias = 0.0
        self._timestamp_bias = 0.0
        self.proc_thread = None

    def connect(self) -> bool:
        self.proc_thread = self.open()
        time.sleep(0.1)
        self.zero()
        
    def disconnect(self):
        self.close()
    
    def get_data(self, subtract_bias=True):
        df = pd.DataFrame(self.data, columns=['t', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
        df['Fx'] = df['Fx'] - (self._fx_bias if subtract_bias else 0)
        df['Fy'] = df['Fy'] - (self._fy_bias if subtract_bias else 0)
        df['Fz'] = df['Fz'] - (self._fz_bias if subtract_bias else 0)
        df['Mx'] = df['Mx'] - (self._mx_bias if subtract_bias else 0)
        df['My'] = df['My'] - (self._my_bias if subtract_bias else 0)
        df['Mz'] = df['Mz'] - (self._mz_bias if subtract_bias else 0)
        df['t'] -= df['t'].min()
        df['t'] /= df['t'].max()
        df['t'] *= self.time_end
        return df

    # def save(self, file_path):
    #     df = self.get_data()
    #     dir = f'{file_path}_{self.name}'
    #     if not os.path.isdir(dir):
    #         os.mkdir(dir)
    #     path = dir + '/position.csv'
    #     counter = 1
    #     while os.path.exists(path):
    #         print(f'{path} already exists. Check input parameters.')
    #         path = f'{file_path}_{self.name}/{self.name} ({counter}).csv'
    #         counter += 1
    #     df.to_csv(path, index=False, mode='x')


    def get_ft(self, subtract_bias):
        if subtract_bias:
            return [
                time.time_ns(), #self._timestamp - self._timestamp_bias,
                self._fx - self._fx_bias, 
                self._fy - self._fy_bias, 
                self._fz - self._fz_bias, 
                self._mx - self._mx_bias, 
                self._my - self._my_bias, 
                self._mz - self._mz_bias,
                ]
        return [time.time_ns(), self._fx, self._fy, self._fz, self._mx, self._my, self._mz]
    
    def bota_sensor_setup(self):
        print("Trying to setup the sensor.")
        # Wait for streaming of data
        out = self._ser.read_until(bytes('App Init', 'ascii'))
        if not self.contains_bytes(bytes('App Init', 'ascii'), out):
            print("Sensor not streaming, check if correct port selected!")
            return False
        time.sleep(0.5)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()

        # Go to CONFIG mode
        cmd = bytes('C', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,C,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,C,0', 'ascii'), out):
            print("Failed to go to CONFIG mode.")
            return False

        # Communication setup
        comm_setup = f"c,{self.TEMP_COMPENSATION},{self.USE_CALIBRATION},{self.DATA_FORMAT},{self.BAUDERATE_CONFIG}"
        #print(comm_setup)
        cmd = bytes(comm_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,c,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,c,0', 'ascii'), out):
            print("Failed to set communication setup.")
            return False
        self.time_step = 0.00001953125*self.SINC_LENGTH
        print("Timestep: {}".format(self.time_step))

        # Filter setup
        filter_setup = f"f,{self.SINC_LENGTH},{self.CHOP_ENABLE},{self.FAST_ENABLE},{self.FIR_DISABLE}"
        #print(filter_setup)
        cmd = bytes(filter_setup, 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,f,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,f,0', 'ascii'), out):
            print("Failed to set filter setup.")
            return False

        # Go to RUN mode
        cmd = bytes('R', 'ascii')
        self._ser.write(cmd)
        out = self._ser.read_until(bytes('r,0,R,0', 'ascii'))
        if not self.contains_bytes(bytes('r,0,R,0', 'ascii'), out):
            print("Failed to go to RUN mode.")
            return False

        return True

    def contains_bytes(self, subsequence, sequence):
        return subsequence in sequence

    def _processdata_thread(self):
        while not self._pd_thread_stop_event.is_set():
            frame_synced = False
            crc16X25Configuration = Configuration(16, 0x1021, 0xFFFF, 0xFFFF, True, True)
            crc_calculator = Calculator(crc16X25Configuration)

            while not frame_synced and not self._pd_thread_stop_event.is_set():
                possible_header = self._ser.read(1)
                if self.FRAME_HEADER == possible_header:
                    #print(possible_header)
                    data_frame = self._ser.read(34)
                    crc16_ccitt_frame = self._ser.read(2)

                    crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                    checksum = crc_calculator.checksum(data_frame)
                    if checksum == crc16_ccitt:
                        print("Frame synced")
                        frame_synced = True
                    else:
                        self._ser.read(1)

            while frame_synced and not self._pd_thread_stop_event.is_set():            
                start_time = time.perf_counter()
                frame_header = self._ser.read(1)

                if frame_header != self.FRAME_HEADER:
                    print("Lost sync")
                    frame_synced = False
                    break

                data_frame = self._ser.read(34)
                crc16_ccitt_frame = self._ser.read(2)
                
                t = time.perf_counter() - self.time_origin

                crc16_ccitt = struct.unpack_from('H', crc16_ccitt_frame, 0)[0]
                checksum = crc_calculator.checksum(data_frame)
                if checksum != crc16_ccitt:
                    print("CRC mismatch received")
                    break

                self._status = struct.unpack_from('H', data_frame, 0)[0]

                self._fx = struct.unpack_from('f', data_frame, 2)[0]
                self._fy = struct.unpack_from('f', data_frame, 6)[0]
                self._fz = struct.unpack_from('f', data_frame, 10)[0]
                self._mx = struct.unpack_from('f', data_frame, 14)[0]
                self._my = struct.unpack_from('f', data_frame, 18)[0]
                self._mz = struct.unpack_from('f', data_frame, 22)[0]

                self._timestamp = struct.unpack_from('I', data_frame, 26)[0]

                self._temperature = struct.unpack_from('f', data_frame, 30)[0]
                
                if self.recording:
                    self.data.append((self._timestamp*1e-6, self._fx, self._fy, self._fz, self._mx, self._my, self._mz))
                
                time_diff = time.perf_counter() - start_time

    def open(self):

        self._ser.baudrate = self.BAUDERATE
        self._ser.port = self._port
        self._ser.timeout = 10

        try:
            self._ser.open()
            print("Opened serial port {}".format(self._port))
        except:
            raise BotaSerialSensorError('Could not open port')

        if not self._ser.is_open:
            raise BotaSerialSensorError('Could not open port')

        if not self.bota_sensor_setup():
            print('Could not setup sensor!')
            return

        #check_thread = threading.Thread(target=self._check_thread)
        #check_thread.start()
        proc_thread = threading.Thread(target=self._processdata_thread)
        proc_thread.start()

        return proc_thread

    def close(self, device_running=True):
        self._pd_thread_stop_event.set()
        self.proc_thread.join()
        #check_thread.join()

        self._ser.close()

        if not device_running:
            raise BotaSerialSensorError('Device is not running')
    
    def zero(self, duration=10, subtract_bias=False, *args, **kwargs):
        df = self.collect_sample(duration=duration, subtract_bias=subtract_bias, *args, **kwargs)
        self._fx_bias = df['Fx'].mean()
        self._fy_bias = df['Fy'].mean()
        self._fz_bias = df['Fz'].mean()
        self._mx_bias = df['Mx'].mean()
        self._my_bias = df['My'].mean()
        self._mz_bias = df['Mz'].mean()
    
    def collect_sample(self, duration=1, plot=None, signal_length=100, subtract_bias=True):
        self.data = []
        df = pd.DataFrame(columns=['t', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz'])
        print(f'Collecting...')
        t0 = t1 = t2 = time.time()
        t_event = time.perf_counter()
        self.trigger(t_event)
        self.recording = True
        while t1 - t0 <= duration:
            df.loc[len(df)] = self.get_ft(subtract_bias)
            if plot is not None and t1 - t2 > 0.1:
                y_range = max(10, df[['Fx', 'Fy', 'Fz']].abs().max().max() + 5)
                y2_range = max(1, df[['Mx', 'My', 'Mz']].abs().max().max() + 0.5)
                plot.update_layout(yaxis_range=[-y_range, y_range], yaxis2_range=[-y2_range, y2_range])
                s = -signal_length if len(df) > signal_length else None
                plot.data[0].y = df.iloc[s:,1]
                plot.data[1].y = df.iloc[s:,2]
                plot.data[2].y = df.iloc[s:,3]
                plot.data[3].y = df.iloc[s:,4]
                plot.data[4].y = df.iloc[s:,5]
                plot.data[5].y = df.iloc[s:,6]
                t2 = time.time()
            t1 = time.time()
        self.recording = False
        self.time_end = time.perf_counter() - t_event
        print(f'Collected for {t1-t0} seconds.')
        return self.get_data(subtract_bias=subtract_bias)
    
    @staticmethod
    def _sleep(duration, get_now=time.perf_counter):
        now = get_now()
        end = now + duration
        while now < end:
            now = get_now()

class BotaSerialSensorError(Exception):
    def __init__(self, message):
        super(BotaSerialSensorError, self).__init__(message)
        self.message = message


class Experiment():
    def __init__(self, sensors: list[DataSource]) -> None:
        self.sensors = sensors
        self.current_parameters = {}
    
    def trigger(self):
        trigger_event = time.perf_counter()
        for sensor in self.sensors:
            sensor.trigger(trigger_event)

    def connect_sensors(self):
        sensors_ready = []
        for sensor in self.sensors:
            sensors_ready.append(sensor.connect())
        return False not in sensors_ready

    def new_trial(self, parameters, duration=10, save=True):
        self.current_parameters = parameters
        for sensor in self.sensors:
            sensor.clear_data()

        self.trigger()
        
        time.sleep(duration)

        for sensor in self.sensors:
            sensor.recording = False
            sensor.time_end = time.perf_counter() - sensor.time_origin

        if save:
            self.save_results()

    def save_results(self):
        dir = self.get_dir_from_parameters()
        for sensor in self.sensors:
            sensor.save(dir)

    
    def disconnect_sensors(self):
        for sensor in self.sensors:
            sensor.disconnect()
    
    def get_dir_from_parameters(self):
        return 'data/' + '_'.join([str(x) for x in self.current_parameters.values()])