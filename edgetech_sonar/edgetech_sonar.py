#! /usr/bin/env python3

import socket
import struct

import rclpy
from rclpy.node import Node

from marine_acoustic_msgs.msg import RawSonarImage
from marine_acoustic_msgs.msg import SonarImageData


class SonarDataMessage:
    def __init__(self, data):
        self.timestamp = struct.unpack('<i', data[0:4])[0]
        self.starting_sample = struct.unpack('<I', data[4:8])[0]
        self.ping_number = struct.unpack('<I', data[8:12])[0]
        self.reserved = [struct.unpack('<2h', data[12:16])[0]]
        self.most_significant_bits = struct.unpack('<H', data[16:18])[0]
        self.least_significant_bits = struct.unpack('<H', data[18:20])[0]
        self.least_significant_bits2 = struct.unpack('<H', data[20:22])[0]
        self.reserved.append(struct.unpack('<3h', data[22:28])[0])
        self.id_code = struct.unpack('<h', data[28:30])[0]

        self.validity_flag = struct.unpack('<H', data[30:32])[0]
        self.validity = {}
        self.validity['position'] = (self.validity_flag & 0x01) != 0
        self.validity['course'] = (self.validity_flag & 0x02) != 0
        self.validity['speed'] = (self.validity_flag & 0x04) != 0
        self.validity['heading'] = (self.validity_flag & 0x08) != 0
        self.validity['pressure'] = (self.validity_flag & 0x10) != 0
        self.validity['attitude'] = (self.validity_flag & 0x20) != 0
        self.validity['altitude'] = (self.validity_flag & 0x40) != 0
        self.validity['heave'] = (self.validity_flag & 0x80) != 0
        self.validity['water_temperature'] = (self.validity_flag & 0x100) != 0
        self.validity['depth'] = (self.validity_flag & 0x200) != 0
        self.validity['annotation'] = (self.validity_flag & 0x400) != 0
        self.validity['counter'] = (self.validity_flag & 0x800) != 0
        self.validity['kp'] = (self.validity_flag & 0x1000) != 0
        self.validity['position_interpolated'] = (self.validity_flag & 0x2000) != 0
        self.validity['water_sound_speed'] = (self.validity_flag & 0x4000) != 0

        self.reserved.append(struct.unpack('<H', data[32:34])[0])
        self.data_format = struct.unpack('<h', data[34:36])[0]
        self.data_format_propietary = self.data_format > 255
        self.data_format_complex = None
        if self.data_format in (0, 2):
            self.data_format_complex = False
        if self.data_format in (1, 9):
            self.data_format_complex = True
        self.data_format_before_match_filter = None
        if self.data_format in (0, 1):
            self.data_format_before_match_filter = False
        if self.data_format in (2, 9):
            self.data_format_before_match_filter = True
        self.data_format_bytes_per_sample = None
        if self.data_format in (0, 2):
            self.data_format_bytes_per_sample = 2
        if self.data_format in (1, 9):
            self.data_format_bytes_per_sample = 4
        
        self.antenna_to_tow_point_offset_aft_cm = struct.unpack('<h', data[36:38])[0]
        self.antenna_to_tow_point_offset_starboard_cm = struct.unpack('<h', data[38:40])[0]
        self.reserved.append(struct.unpack('<2h', data[40:44])[0])
        self.kilometers_of_pipe = struct.unpack('<f', data[44:48])[0]
        self.heave_meters_down = struct.unpack('<f', data[48:52])[0]
        self.reserved.append(struct.unpack('<12h', data[52:76])[0])
        self.gap_filler_offset_starboard_meters = struct.unpack('<f', data[76:80])[0]
        self.longitude_or_x = struct.unpack('<i', data[80:84])[0]
        self.latitude_or_y = struct.unpack('<i', data[84:88])[0]
        self.coordinate_units = struct.unpack('<h', data[88:90])[0]
        self.annotation_string = data[92:114].decode('ascii').rstrip('\x00')
        self.samples = struct.unpack('<H', data[114:116])[0]
        self.sampling_interval_ns = struct.unpack('<i', data[116:120])[0]
        self.gain_factor = struct.unpack('<H', data[120:122])[0]
        self.user_transmit_level_percent = struct.unpack('<h', data[122:124])[0]
        self.reserved.append(struct.unpack('<h', data[124:126])[0])
        self.transmit_pulse_starting_frequency_dahz = struct.unpack('<H', data[126:128])[0]
        self.transmit_pulse_ending_frequency_dahz = struct.unpack('<H', data[128:130])[0]
        self.sweep_length_ms = struct.unpack('<H', data[130:132])[0]
        self.pressure_milli_psi = struct.unpack('<i', data[132:136])[0]
        self.depth_mm = struct.unpack('<i', data[136:140])[0]
        self.sample_frequency_hz = struct.unpack('<H', data[140:142])[0]
        self.outgoing_pulse_id = struct.unpack('<H', data[142:144])[0]
        self.altitude_mm = struct.unpack('<i', data[144:148])[0]
        self.sound_speed = struct.unpack('<f', data[148:152])[0]
        self.mixer_frequency_hz = struct.unpack('<f', data[152:156])[0]
        self.year = struct.unpack('<h', data[156:158])[0]
        self.day = struct.unpack('<h', data[158:160])[0]
        self.hour = struct.unpack('<h', data[160:162])[0]
        self.minute = struct.unpack('<h', data[162:164])[0]
        self.second = struct.unpack('<h', data[164:166])[0]
        self.time_basis = struct.unpack('<h', data[166:168])[0]
        self.weighting_factor = struct.unpack('<h', data[168:170])[0]
        self.number_of_pulses_in_water = struct.unpack('<h', data[170:172])[0]

        self.compass_heading = struct.unpack('<H', data[172:174])[0] / 100.0
        self.pitch = struct.unpack('<h', data[174:176])[0] *180.0/32768.0
        self.roll = struct.unpack('<h', data[176:178])[0] *180.0/32768.0
        self.reserved.append(struct.unpack('<h', data[178:180])[0])

        self.reserved.append(struct.unpack('<h', data[180:182])[0])
        self.trigger_source = struct.unpack('<h', data[182:184])[0]
        self.mark_number = struct.unpack('<H', data[184:186])[0]

        self.position_fix_hour = struct.unpack('<h', data[186:188])[0]
        self.position_fix_minutes = struct.unpack('<h', data[188:190])[0]
        self.position_fix_seconds = struct.unpack('<h', data[190:192])[0]
        self.course = struct.unpack('<H', data[192:194])[0]
        self.speed_knots = struct.unpack('<H', data[194:196])[0]/10.0
        self.position_fix_day = struct.unpack('<h', data[196:198])[0]
        self.position_fix_year = struct.unpack('<h', data[198:200])[0]
        self.milliseconds_since_midnight = struct.unpack('<I', data[200:204])[0]
        self.adc_maximum_absolute_value = struct.unpack('<H', data[204:206])[0]
        self.reserved.append(struct.unpack('<H', data[206:208])[0])
        self.reserved.append(struct.unpack('<H', data[208:210])[0])
        self.sonar_software_version = struct.unpack('<6s', data[210:216])[0].decode('ascii').rstrip('\x00')
        self.initial_spherical_correction_factor = struct.unpack('<i', data[216:220])[0]
        self.packet_number = struct.unpack('<H', data[220:222])[0]
        self.adc_decimation = struct.unpack('<h', data[222:224])[0]
        self.reserved.append(struct.unpack('<h', data[224:226])[0])
        self.water_temperature_celsius = struct.unpack('<h', data[226:228])[0]/10.0
        self.layback = struct.unpack('<f', data[228:232])[0]
        self.reserved.append(struct.unpack('<i', data[232:236])[0])
        self.cable_out_dm = struct.unpack('<h', data[236:238])[0]
        self.reserved.append(struct.unpack('<H', data[238:240])[0])
        self.sonar_data = data[240:]


    def __str__(self):
        return f'SonarDataMessage: timestamp: {self.timestamp}, starting_sample: {self.starting_sample}, ping_number: {self.ping_number}, id_code: {self.id_code}, validity_flag: {self.validity_flag}, data format: {self.data_format}, annotation: {self.annotation_string}, samples: {self.samples}, data size: {len(self.sonar_data)}'
    def to_ros_message(self):
        msg = RawSonarImage()

        msg.header.stamp.sec = self.timestamp
        milliseconds = self.milliseconds_since_midnight%1000
        msg.header.stamp.nanosec = milliseconds * 1000

        msg.tx_angles = [0.0]
        msg.rx_angles = [0.0]
        msg.image.beam_count = 1
        msg.image.data = self.sonar_data
        msg.image.dtype = SonarImageData.DTYPE_INT16

        return msg

class JSFMessage:
    header_size = 16

    message_types = {
        80: SonarDataMessage
    }

    def __init__(self):
        self.payload_length = -1
        self.data = None


    def need_more_data(self):
        if self.data is None:
            return True
        if self.payload_length < 0:
            return True
        return len(self.data) < self.header_size + self.payload_length


    def add_data(self, data):
        ''' Add more data to the message. Return number of bytes consumed from data. '''

        bytes_consumed = 0
        if self.data is None:
            # we don't have part of a message yet, so find one
            # first, find the start of the message
            offset = 0
            while offset < len(data)+2:
                marker = struct.unpack('<H', data[offset:offset + 2])[0]
                if marker == 0x1601:
                    break
                offset += 2
            if offset >= len(data):
                return len(data)
            self.data = data[offset:]
            bytes_consumed = offset
        else:
            self.data += data

        if self.payload_length < 0:
            if not self.parse_header():
                # not enough data to parse the header yet
                return len(data)

        if self.payload_length >= 0 and len(self.data) >= self.header_size + self.payload_length:
            # do we have more data than we need?
            if len(self.data) > self.header_size + self.payload_length:
                more_data_size = len(self.data) - (self.header_size + self.payload_length)
                self.data = self.data[:self.header_size + self.payload_length]
                return len(data) - more_data_size
        return len(data)

    def parse_header(self):
        if self.data is None or len(self.data) < 16:
            return False
        self.protocol_version = struct.unpack('<B', self.data[2:3])[0]
        self.session_id = struct.unpack('<B', self.data[3:4])[0]
        self.message_type = struct.unpack('<H', self.data[4:6])[0]
        self.command_type = struct.unpack('<B', self.data[6:7])[0]
        self.subsystem_number = struct.unpack('<B', self.data[7:8])[0]
        self.channel = struct.unpack('<B', self.data[8:9])[0]
        self.sequence_number = struct.unpack('<B', self.data[9:10])[0]
        self.reserved = struct.unpack('<H', self.data[10:12])[0]
        self.payload_length = struct.unpack('<i', self.data[12:16])[0]
        return True
    
    def payload(self):
        if self.need_more_data():
            return None
        if self.data is not None and self.message_type in self.message_types:
            return self.message_types[self.message_type](self.data[self.header_size:])
        return None


class EdgeTechSonar(Node):
    def __init__(self, node_name='edgetech_sonar', **kwargs):
        super().__init__(node_name, **kwargs)

        self.publisher_port_low = self.create_publisher(RawSonarImage, 'sonar_image_port_low', 10)
        self.publisher_port_high = self.create_publisher(RawSonarImage, 'sonar_image_port_high', 10)
        self.publisher_starboard_low = self.create_publisher(RawSonarImage, 'sonar_image_starboard_low', 10)
        self.publisher_starboard_high = self.create_publisher(RawSonarImage, 'sonar_image_starboard_high', 10)

        self.declare_parameter('host', '192.9.0.100')
        self.declare_parameter('data_port', 1901)


        host = self.get_parameter('host').get_parameter_value().string_value
        data_port = self.get_parameter('data_port').get_parameter_value().integer_value

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((host, data_port))

        self.socket.settimeout(5.0)

        self.message = JSFMessage()

        self.timer = self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        try:
            data = self.socket.recv(1024)
            if not data:
                return
            while len(data) > 0:
                if self.message.need_more_data():
                    consumed = self.message.add_data(data)
                    data = data[consumed:]
                if not self.message.need_more_data():
                    payload = self.message.payload()
                    if payload is not None:
                        print(payload)
                        ros_msg = payload.to_ros_message()
                        if self.message.subsystem_number == 20: # low frequency
                            if self.message.channel == 0: # port
                                self.publisher_port_low.publish(ros_msg)
                            elif self.message.channel == 1: # starboard
                                self.publisher_starboard_low.publish(ros_msg)
                        elif self.message.subsystem_number == 21: # high frequency
                            if self.message.channel == 0: # port
                                self.publisher_port_high.publish(ros_msg)
                            elif self.message.channel == 1: # starboard
                                self.publisher_starboard_high.publish(ros_msg)
                    self.message = JSFMessage()
        except Exception as e:
            self.get_logger().error(f'Error receiving data: {e}')




def main(args=None):

    rclpy.init(args=args)

    sonar = EdgeTechSonar()
    rclpy.spin(sonar)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

