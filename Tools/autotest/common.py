from __future__ import print_function

import abc
import copy
import errno
import math
import os
import re
import shutil
import sys
import time
import traceback
import pexpect
import fnmatch
import operator
import numpy
import socket
import struct

from MAVProxy.modules.lib import mp_util

from pymavlink import mavwp, mavutil, DFReader
from pymavlink import mavextra

from pysim import util, vehicleinfo

# a list of pexpect objects to read while waiting for
# messages. This keeps the output to stdout flowing
expect_list = []

# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))

# Check python version for abstract base class
if sys.version_info[0] >= 3 and sys.version_info[1] >= 4:
    ABC = abc.ABC
else:
    ABC = abc.ABCMeta('ABC', (), {})

try:
    from itertools import izip as zip
except ImportError:
    # probably python2
    pass

class ErrorException(Exception):
    """Base class for other exceptions"""
    pass


class AutoTestTimeoutException(ErrorException):
    pass


class WaitModeTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given mode change."""
    pass


class WaitAltitudeTimout(AutoTestTimeoutException):
    """Thrown when fails to achieve given altitude range."""
    pass


class WaitGroundSpeedTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given ground speed range."""
    pass


class WaitRollTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given roll in degrees."""
    pass


class WaitPitchTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given pitch in degrees."""
    pass


class WaitHeadingTimeout(AutoTestTimeoutException):
    """Thrown when fails to achieve given heading."""
    pass


class WaitDistanceTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain distance"""
    pass


class WaitLocationTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain location"""
    pass


class WaitWaypointTimeout(AutoTestTimeoutException):
    """Thrown when fails to attain waypoint ranges"""
    pass


class SetRCTimeout(AutoTestTimeoutException):
    """Thrown when fails to send RC commands"""
    pass


class MsgRcvTimeoutException(AutoTestTimeoutException):
    """Thrown when fails to receive an expected message"""
    pass


class NotAchievedException(ErrorException):
    """Thrown when fails to achieve a goal"""
    pass


class PreconditionFailedException(ErrorException):
    """Thrown when a precondition for a test is not met"""
    pass

class ArmedAtEndOfTestException(ErrorException):
    """Created when test left vehicle armed"""
    pass

class Context(object):
    def __init__(self):
        self.parameters = []
        self.sitl_commandline_customised = False

# https://stackoverflow.com/questions/616645/how-do-i-duplicate-sys-stdout-to-a-log-file-in-python
class TeeBoth(object):
    def __init__(self, name, mode, mavproxy_logfile):
        self.file = open(name, mode)
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        self.mavproxy_logfile = mavproxy_logfile
        self.mavproxy_logfile.set_fh(self)
        sys.stdout = self
        sys.stderr = self
    def close(self):
        sys.stdout = self.stdout
        sys.stderr = self.stderr
        self.mavproxy_logfile.set_fh(None)
        self.mavproxy_logfile = None
        self.file.close()
        self.file = None
    def write(self, data):
        self.file.write(data)
        self.stdout.write(data)
    def flush(self):
        self.file.flush()

class MAVProxyLogFile(object):
    def __init__(self):
        self.fh = None
    def close(self):
        pass
    def set_fh(self, fh):
        self.fh = fh
    def write(self, data):
        if self.fh is not None:
            self.fh.write(data)
        else:
            sys.stdout.write(data)
    def flush(self):
        if self.fh is not None:
            self.fh.flush()
        else:
            sys.stdout.flush()

class Telem(object):
    def __init__(self, destination_address):
        self.destination_address = destination_address

        self.buffer = bytes()
        self.connected = False
        self.port = None

    def connect(self):
        try:
            self.connected = False
            self.progress("Connecting to (%s:%u)" % self.destination_address)
            if self.port is not None:
                try:
                    self.port.close() # might be reopening
                except Exception as e:
                    pass
            self.port = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.port.connect(self.destination_address)
            self.port.setblocking(False)
            self.port.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
            self.connected = True
            self.progress("Connected")
        except IOError as e:
            self.progress("Failed to connect: %s" % str(e))
            time.sleep(0.5)
            return False
        return True

    def do_read(self):
        try:
            data = self.port.recv(1024)
        except socket.error as e:
            if e.errno not in [ errno.EAGAIN, errno.EWOULDBLOCK ]:
                self.progress("Exception: %s" % str(e))
                self.connected = False
            return bytes()
        if len(data) == 0:
            self.progress("EOF")
            self.connected = False
            return bytes()
#        self.progress("Read %u bytes" % len(data))
        return data

    def update(self):
        if not self.connected:
            if not self.connect():
                return
        self.update_read()

class LTM(Telem):
    def __init__(self, destination_address):
        super(LTM, self).__init__(destination_address)

        self.HEADER1 = 0x24
        self.HEADER2 = 0x54

        self.FRAME_G = 0x47
        self.FRAME_A = 0x41
        self.FRAME_S = 0x53

        self.frame_lengths = {
            self.FRAME_G: 18,
            self.FRAME_A: 10,
            self.FRAME_S: 11,
        }
        self.frame_lengths = {
            self.FRAME_G: 18,
            self.FRAME_A: 10,
            self.FRAME_S: 11,
        }

        self.data_by_id = {}
        self.frames = {}

    def g(self):
        return self.frames.get(self.FRAME_G, None)
    def a(self):
        return self.frames.get(self.FRAME_A, None)
    def s(self):
        return self.frames.get(self.FRAME_S, None)

    def progress(self, message):
        print("LTM: %s" % message)

    def handle_data(self, dataid, value):
        self.progress("%u=%u" % (dataid, value))
        self.data_by_id[dataid] = value

    def consume_frame(self):
        b2 = self.buffer[2]
        if sys.version_info.major < 3:
            b2 = ord(b2)
        frame_type = b2
        frame_length = self.frame_lengths[frame_type]
        # check frame CRC
        crc = 0
        count = 0
        for c in self.buffer[3:frame_length-1]:
            if sys.version_info.major < 3:
                c = ord(c)
            old = crc
            crc ^= c
            count += 1
        buffer_crc = self.buffer[frame_length-1]
        if sys.version_info.major < 3:
            buffer_crc = ord(buffer_crc)
        if crc != buffer_crc:
            raise NotAchievedException("Invalid checksum on frame type %s" % str(chr(frame_type)))
#        self.progress("Received valid %s frame" % str(chr(frame_type)))

        class Frame(object):
            def __init__(self, buffer):
                self.buffer = buffer
            def intn(self, offset, count):
                ret = 0
                for i in range(offset, offset+count):
                    print("byte: %02x" % ord(self.buffer[i]))
                    ret = ret | (ord(self.buffer[i]) << ((i-offset)*8))
                return ret
            def int32(self, offset):
                t = struct.unpack("<i", self.buffer[offset:offset+4])
                return t[0]
#                return self.intn(offset, 4)
            def int16(self, offset):
                t = struct.unpack("<h", self.buffer[offset:offset+2])
                return t[0]
#                return self.intn(offset, 2)

        class FrameG(Frame):
            def __init__(self, buffer):
                super(FrameG, self,).__init__(buffer)
            def lat(self):
                return self.int32(3)
            def lon(self):
                return self.int32(7)
            def gndspeed(self):
                ret = self.buffer[11]
                if sys.version_info.major < 3:
                    ret = ord(ret)
                return ret
            def alt(self):
                return self.int32(12)
            def sats(self):
                s = self.buffer[16]
                if sys.version_info.major < 3:
                    s = ord(s)
                return (s>>2)
            def fix_type(self):
                s = self.buffer[16]
                if sys.version_info.major < 3:
                    s = ord(s)
                return s & 0b11

        class FrameA(Frame):
            def __init__(self, buffer):
                super(FrameA, self,).__init__(buffer)
            def pitch(self):
                return self.int16(3)
            def roll(self):
                return self.int16(5)
            def hdg(self):
                return self.int16(7)
        class FrameS(Frame):
            def __init__(self, buffer):
                super(FrameS, self,).__init__(buffer)

        if frame_type == self.FRAME_G:
            frame = FrameG(self.buffer[0:frame_length-1])
        elif frame_type == self.FRAME_A:
            frame = FrameA(self.buffer[0:frame_length-1])
        elif frame_type == self.FRAME_S:
            frame = FrameS(self.buffer[0:frame_length-1])
        else:
            raise NotAchievedException("Bad frame?!?!?!")
        self.buffer = self.buffer[frame_length:]
        self.frames[frame_type] = frame

    def update_read(self):
        self.buffer += self.do_read()
        while len(self.buffer):
            if len(self.buffer) == 0:
                break
            b0 = self.buffer[0]
            if sys.version_info.major < 3:
                b0 = ord(b0)
            if b0 != self.HEADER1:
                self.bad_chars += 1
                self.buffer = self.buffer[1:]
                continue
            b1 = self.buffer[1]
            if sys.version_info.major < 3:
                b1 = ord(b1)
            if b1 != self.HEADER2:
                self.bad_chars += 1
                self.buffer = self.buffer[1:]
                continue
            b2 = self.buffer[2]
            if sys.version_info.major < 3:
                b2 = ord(b2)
            if b2 not in [self.FRAME_G, self.FRAME_A, self.FRAME_S]:
                self.bad_chars += 1
                self.buffer = self.buffer[1:]
                continue
            frame_len = self.frame_lengths[b2]
            if len(self.buffer) < frame_len:
                continue
            self.consume_frame()

    def get_data(self, dataid):
        try:
            return self.data_by_id[dataid]
        except KeyError as e:
            pass
        return None

class FRSky(Telem):
    def __init__(self, destination_address):
        super(FRSky, self).__init__(destination_address)

        self.dataid_GPS_ALT_BP          = 0x01
        self.dataid_TEMP1               = 0x02
        self.dataid_FUEL                = 0x04
        self.dataid_TEMP2               = 0x05
        self.dataid_GPS_ALT_AP          = 0x09
        self.dataid_BARO_ALT_BP         = 0x10
        self.dataid_GPS_SPEED_BP        = 0x11
        self.dataid_GPS_LONG_BP         = 0x12
        self.dataid_GPS_LAT_BP          = 0x13
        self.dataid_GPS_COURS_BP        = 0x14
        self.dataid_GPS_SPEED_AP        = 0x19
        self.dataid_GPS_LONG_AP         = 0x1A
        self.dataid_GPS_LAT_AP          = 0x1B
        self.dataid_BARO_ALT_AP         = 0x21
        self.dataid_GPS_LONG_EW         = 0x22
        self.dataid_GPS_LAT_NS          = 0x23
        self.dataid_CURRENT             = 0x28
        self.dataid_VFAS                = 0x39

class FRSkyD(FRSky):
    def __init__(self, destination_address):
        super(FRSkyD, self).__init__(destination_address)

        self.state_WANT_START_STOP_D = 16,
        self.state_WANT_ID = 17,
        self.state_WANT_BYTE1 = 18,
        self.state_WANT_BYTE2 = 19,

        self.START_STOP_D = 0x5E
        self.BYTESTUFF_D = 0x5D

        self.state = self.state_WANT_START_STOP_D

        self.data_by_id = {}
        self.bad_chars = 0

    def progress(self, message):
        print("FRSkyD: %s" % message)

    def handle_data(self, dataid, value):
        self.progress("%u=%u" % (dataid, value))
        self.data_by_id[dataid] = value

    def update_read(self):
        self.buffer += self.do_read()
        consume = None
        while len(self.buffer):
            if consume is not None:
                self.buffer = self.buffer[consume:]
            if len(self.buffer) == 0:
                break
            consume = 1
            if sys.version_info.major >= 3:
                b = self.buffer[0]
            else:
                b = ord(self.buffer[0])
            if self.state == self.state_WANT_START_STOP_D:
                if b != self.START_STOP_D:
                    # we may come into a stream mid-way, so we can't judge
                    self.bad_chars += 1
                    continue
                self.state = self.state_WANT_ID
                continue
            elif self.state == self.state_WANT_ID:
                self.dataid = b
                self.state = self.state_WANT_BYTE1
                continue
            elif self.state in [self.state_WANT_BYTE1, self.state_WANT_BYTE2]:
                if b == 0x5D:
                    # byte-stuffed
                    if len(self.buffer) < 2:
                        # try again in a little while
                        consume = 0
                        return
                    if ord(self.buffer[1]) == 0x3E:
                        b = self.START_STOP_D
                    elif ord(self.buffer[1]) == 0x3D:
                        b = self.BYTESTUFF_D;
                    else:
                        raise ValueError("Unknown stuffed byte")
                    consume = 2
                if self.state == self.state_WANT_BYTE1:
                    self.b1 = b
                    self.state = self.state_WANT_BYTE2
                    continue

                data = self.b1 | b << 8
                self.handle_data(self.dataid, data)
                self.state = self.state_WANT_START_STOP_D

    def get_data(self, dataid):
        try:
            return self.data_by_id[dataid]
        except KeyError as e:
            pass
        return None


class FRSkySPort(FRSky):
    def __init__(self, destination_address):
        super(FRSkySPort, self).__init__(destination_address)

        self.state_SEND_POLL = 34
        self.state_WANT_FRAME_TYPE = 35
        self.state_WANT_ID1 = 36,
        self.state_WANT_ID2 = 37,
        self.state_WANT_DATA = 38,
        self.state_WANT_CRC = 39,

        self.START_STOP_SPORT = 0x7E
        self.BYTESTUFF_SPORT  = 0x7D
        self.SPORT_DATA_FRAME = 0x10

        self.SENSOR_ID_VARIO             = 0x00 # Sensor ID  0
        self.SENSOR_ID_FAS               = 0x22 # Sensor ID  2
        self.SENSOR_ID_GPS               = 0x83 # Sensor ID  3
        self.SENSOR_ID_SP2UR             = 0xC6 # Sensor ID  6
        self.SENSOR_ID_28                = 0x1B # Sensor ID 28

        self.state = self.state_WANT_FRAME_TYPE

        self.data_by_id = {}
        self.bad_chars = 0

        self.poll_sent = 0

        self.id_descriptions = {
            0x5000: "status text (dynamic)",
            0x5006: "Attitude and range (dynamic)",
            0x800: "GPS lat or lon (600 with 1 sensor)",
            0x5005: "Vel and Yaw",
            0x5001: "AP status",
            0x5002: "GPS Status",
            0x5004: "Home",
            0x5008: "Battery 2 status",
            0x5003: "Battery 1 status",
            0x5007: "parameters",

            # SPort non-passthrough:
            0x02: "Temp1",
            0x04: "Fuel",
            0x05: "Temp2",
            0x10: "Baro Alt BP",
            0x21: "BARO_ALT_AP",
            0x30: "VARIO",
            0x39: "VFAS",
            # 0x800: "GPS", ## comments as duplicated dictrionary key
        }

        self.sensors_to_poll = [
            self.SENSOR_ID_VARIO,
            self.SENSOR_ID_FAS,
            self.SENSOR_ID_GPS,
            self.SENSOR_ID_SP2UR,
        ]
        self.next_sensor_id_to_poll = 0 # offset into sensors_to_poll

    def progress(self, message):
        print("FRSkySPort: %s" % message)

    def handle_data(self, dataid, value):
        self.progress("%s (0x%x)=%u" % (self.id_descriptions[dataid], dataid, value))
        self.data_by_id[dataid] = value

    def read_bytestuffed_byte(self):
        if sys.version_info.major >= 3:
            b = self.buffer[0]
        else:
            b = ord(self.buffer[0])
        if b == 0x7D:
            # byte-stuffed
            if len(self.buffer) < 2:
                self.consume = 0
                return None
            self.consume = 2
            if sys.version_info.major >= 3:
                b2 = self.buffer[1]
            else:
                b2 = ord(self.buffer[1])
            if b2 == 0x5E:
                return self.START_STOP_SPORT
            if b2 == 0x5D:
                return self.BYTESTUFF_SPORT
            raise ValueError("Unknown stuffed byte (0x%02x)" % b2)
        return b

    def calc_crc(self, byte):
        self.crc += byte
        self.crc += self.crc >> 8
        self.crc &= 0xFF

    def next_sensor(self):
        ret =  self.sensors_to_poll[self.next_sensor_id_to_poll]
        self.next_sensor_id_to_poll += 1
        if self.next_sensor_id_to_poll >= len(self.sensors_to_poll):
            self.next_sensor_id_to_poll = 0
        return ret

    def check_poll(self):
        now = time.time()
        if now - self.poll_sent > 2:
            if self.state != self.state_WANT_FRAME_TYPE:
                raise ValueError("Expected to be wanting a frame type when repolling")
            self.progress("Re-polling")
            self.state = self.state_SEND_POLL

        if self.state == self.state_SEND_POLL:
            sensor_id = self.next_sensor()
            self.progress("Sending poll for 0x%02x" % sensor_id)
            buf = struct.pack('<BB', self.START_STOP_SPORT, sensor_id)
            self.port.sendall(buf)
            self.state = self.state_WANT_FRAME_TYPE
            self.poll_sent = now

    def update(self):
        if not self.connected:
            if not self.connect():
                return
        self.check_poll()
        self.do_sport_read()

    def do_sport_read(self):
        self.buffer += self.do_read()
        self.consume = None
        while len(self.buffer):
            if self.consume is not None:
                self.buffer = self.buffer[self.consume:]
            if len(self.buffer) == 0:
                break
            self.consume = 1
            if sys.version_info.major >= 3:
                b = self.buffer[0]
            else:
                b = ord(self.buffer[0])
#            self.progress("Have (%s) bytes state=%s b=0x%02x" % (str(len(self.buffer)), str(self.state), b));
            if self.state == self.state_WANT_FRAME_TYPE:
                if b != self.SPORT_DATA_FRAME:
                    # we may come into a stream mid-way, so we can't judge
                    self.progress("############# Bad char %x" % b)
                    raise ValueError("Bad char (0x%02x)" % b)
                    self.bad_chars += 1
                    continue
                self.crc = 0
                self.calc_crc(b)
                self.state = self.state_WANT_ID1
                continue
            elif self.state == self.state_WANT_ID1:
                self.id1 = self.read_bytestuffed_byte()
                if self.id1 is None:
                    break
                self.calc_crc(self.id1)
                self.state = self.state_WANT_ID2
                continue
            elif self.state == self.state_WANT_ID2:
                self.id2 = self.read_bytestuffed_byte()
                if self.id2 is None:
                    break
                self.calc_crc(self.id2)
                self.state = self.state_WANT_DATA
                self.data_byte_count = 0
                self.data = 0
                continue
            elif self.state == self.state_WANT_DATA:
                data_byte = self.read_bytestuffed_byte()
                if data_byte is None:
                    break
                self.calc_crc(data_byte)
                self.data = self.data | (data_byte << (8*(self.data_byte_count)))
                self.data_byte_count += 1
                if self.data_byte_count == 4:
                    self.state = self.state_WANT_CRC
                continue
            elif self.state == self.state_WANT_CRC:
                crc = self.read_bytestuffed_byte()
                if crc is None:
                    break
                self.crc = 0xFF - self.crc
                dataid = (self.id2 << 8) | self.id1
                if self.crc != crc:
                    self.progress("Incorrect frsky checksum (received=%02x calculated=%02x id=0x%x)" % (crc, self.crc, dataid))
#                    raise ValueError("Incorrect frsky checksum (want=%02x got=%02x id=0x%x)" % (crc, self.crc, dataid))
                else:
                    self.handle_data(dataid, self.data)
                self.state = self.state_SEND_POLL
            else:
                raise ValueError("Unknown state (%u)" % self.state)

    def get_data(self, dataid):
        try:
            return self.data_by_id[dataid]
        except KeyError as e:
            pass
        return None

class FRSkyPassThrough(FRSkySPort):
    def __init__(self, destination_address):
        super(FRSkyPassThrough, self).__init__(destination_address)

        self.sensors_to_poll = [self.SENSOR_ID_28]

    def progress(self, message):
        print("FRSkyPassthrough: %s" % message)

class AutoTest(ABC):
    """Base abstract class.
    It implements the common function for all vehicle types.
    """
    def __init__(self,
                 binary,
                 valgrind=False,
                 gdb=False,
                 speedup=8,
                 frame=None,
                 params=None,
                 gdbserver=False,
                 lldb=False,
                 breakpoints=[],
                 disable_breakpoints=False,
                 viewerip=None,
                 use_map=False,
                 _show_test_timings=False):

        if binary is None:
            raise ValueError("Should always have a binary")

        self.binary = binary
        self.valgrind = valgrind
        self.gdb = gdb
        self.lldb = lldb
        self.frame = frame
        self.params = params
        self.gdbserver = gdbserver
        self.breakpoints = breakpoints
        self.disable_breakpoints = disable_breakpoints
        self.speedup = speedup

        self.mavproxy = None
        self.mav = None
        self.viewerip = viewerip
        self.use_map = use_map
        self.contexts = []
        self.context_push()
        self.buildlog = None
        self.copy_tlog = False
        self.logfile = None
        self.max_set_rc_timeout = 0
        self.last_wp_load = 0
        self.forced_post_test_sitl_reboots = 0
        self.skip_list = []
        self.run_tests_called = False
        self._show_test_timings = _show_test_timings
        self.test_timings = dict()
        self.total_waiting_to_arm_time = 0
        self.waiting_to_arm_count = 0

    @staticmethod
    def progress(text):
        """Display autotest progress text."""
        print("AUTOTEST: " + text)

    # following two functions swiped from autotest.py:
    @staticmethod
    def buildlogs_dirpath():
        return os.getenv("BUILDLOGS", util.reltopdir("../buildlogs"))

    def sitl_home(self):
        HOME = self.sitl_start_location()
        return "%f,%f,%u,%u" % (HOME.lat,
                                HOME.lng,
                                HOME.alt,
                                HOME.heading)

    def mavproxy_version(self):
        '''return the current version of mavproxy as a tuple e.g. (1,8,8)'''
        return util.MAVProxy_version()

    def mavproxy_version_gt(self, major, minor, point):
        if os.getenv("AUTOTEST_FORCE_MAVPROXY_VERSION", None) is not None:
            return True
        (got_major, got_minor, got_point) = self.mavproxy_version()
        print("Got: %s.%s.%s" % (got_major, got_minor, got_point))
        if got_major > major:
            return True
        elif got_major < major:
            return False
        if got_minor > minor:
            return True
        elif got_minor < minor:
            return False
        return got_point > point

    def open_mavproxy_logfile(self):
        return MAVProxyLogFile()

    def buildlogs_path(self, path):
        """Return a string representing path in the buildlogs directory."""
        bits = [self.buildlogs_dirpath()]
        if isinstance(path, list):
            bits.extend(path)
        else:
            bits.append(path)
        return os.path.join(*bits)

    def sitl_streamrate(self):
        """Allow subclasses to override SITL streamrate."""
        return 10

    def autotest_connection_hostport(self):
        '''returns host and port of connection between MAVProxy and autotest,
        colon-separated'''
        return "127.0.0.1:19550"

    def autotest_connection_string_from_mavproxy(self):
        return "tcpin:" + self.autotest_connection_hostport()

    def autotest_connection_string_to_mavproxy(self):
        return "tcp:" + self.autotest_connection_hostport()

    def mavproxy_options(self):
        """Returns options to be passed to MAVProxy."""
        ret = ['--sitl=127.0.0.1:5501',
               '--out=' + self.autotest_connection_string_from_mavproxy(),
               '--streamrate=%u' % self.sitl_streamrate(),
               '--cmd="set heartbeat %u"' % self.speedup]
        if self.viewerip:
            ret.append("--out=%s:14550" % self.viewerip)
        if self.use_map:
            ret.append('--map')

        return ret

    def vehicleinfo_key(self):
        return self.log_name()

    def repeatedly_apply_parameter_file(self, filepath):
        '''keep applying a parameter file until no parameters changed'''
        for i in range(0, 3):
            self.mavproxy.send("param load %s\n" % filepath)
            while True:
                line = self.mavproxy.readline()
                match = re.match(".*Loaded [0-9]+ parameters.*changed ([0-9]+)",
                                 line)
                if match is not None:
                    if int(match.group(1)) == 0:
                        return
                    break
        raise NotAchievedException()

    def apply_defaultfile_parameters(self):
        """Apply parameter file."""
        self.progress("Applying default parameters file")
        # setup test parameters
        vinfo = vehicleinfo.VehicleInfo()
        if self.params is None:
            frames = vinfo.options[self.vehicleinfo_key()]["frames"]
            self.params = frames[self.frame]["default_params_filename"]
        if not isinstance(self.params, list):
            self.params = [self.params]
        for x in self.params:
            self.repeatedly_apply_parameter_file(os.path.join(testdir, x))
        self.set_parameter('LOG_REPLAY', 1)
        self.set_parameter('LOG_DISARMED', 1)
        self.reboot_sitl()
        self.fetch_parameters()

    def count_lines_in_filepath(self, filepath):
        return len([i for i in open(filepath)])

    def count_expected_fence_lines_in_filepath(self, filepath):
        count = 0
        is_qgc = False
        for i in open(filepath):
            i = re.sub("#.*", "", i) # trim comments
            if i.isspace():
                # skip empty lines
                continue
            if re.match("QGC", i):
                # skip QGC header line
                is_qgc = True
                continue
            count += 1
        if is_qgc:
            count += 2 # file doesn't include return point + closing point
        return count

    def load_fence_using_mavproxy(self, filename):
        self.set_parameter("FENCE_TOTAL", 0)
        filepath = os.path.join(testdir, self.current_test_name_directory, filename)
        count = self.count_expected_fence_lines_in_filepath(filepath)
        self.mavproxy.send('fence load %s\n' % filepath)
#        self.mavproxy.expect("Loaded %u (geo-)?fence" % count)
        tstart = self.get_sim_time_cached()
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to do load")
            newcount = self.get_parameter("FENCE_TOTAL")
            self.progress("fence total: %u want=%u" % (newcount, count))
            if count == newcount:
                break
            self.delay_sim_time(1)

    def load_fence(self, filename):
        self.load_fence_using_mavproxy(filename)

    def fetch_parameters(self):
        self.mavproxy.send("param fetch\n")
        self.mavproxy.expect("Received [0-9]+ parameters")

    def reboot_sitl_mav(self, required_bootcount=None):
        """Reboot SITL instance using mavlink and wait for it to reconnect."""
        old_bootcount = self.get_parameter('STAT_BOOTCNT')
        # ardupilot SITL may actually NAK the reboot; replace with
        # run_cmd when we don't do that.
        self.mav.mav.command_long_send(self.sysid_thismav(),
                                       1,
                                       mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       1,  # confirmation
                                       1, # reboot autopilot
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0)
        self.detect_and_handle_reboot(old_bootcount, required_bootcount=required_bootcount)

    def send_cmd_enter_cpu_lockup(self):
        """Poke ArduPilot to stop the main loop from running"""
        self.mav.mav.command_long_send(self.sysid_thismav(),
                                       1,
                                       mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       1,  # confirmation
                                       42, # lockup autopilot
                                       24, # no, really, we mean it
                                       71, # seriously, we're not kidding
                                       93, # we know exactly what we're
                                       0,
                                       0,
                                       0)

    def reboot_sitl(self, required_bootcount=None):
        """Reboot SITL instance and wait for it to reconnect."""
        self.progress("Rebooting SITL")
        self.reboot_sitl_mav(required_bootcount=required_bootcount)
        self.assert_simstate_location_is_at_startup_location()

    def reboot_sitl_mavproxy(self, required_bootcount=None):
        """Reboot SITL instance using MAVProxy and wait for it to reconnect."""
        old_bootcount = self.get_parameter('STAT_BOOTCNT')
        self.mavproxy.send("reboot\n")
        self.detect_and_handle_reboot(old_bootcount, required_bootcount=required_bootcount)

    def detect_and_handle_reboot(self, old_bootcount, required_bootcount=None, timeout=10):
        tstart = time.time()
        if required_bootcount is None:
            required_bootcount = old_bootcount + 1
        while True:
            if time.time() - tstart > timeout:
                raise AutoTestTimeoutException("Did not detect reboot")
            try:
                current_bootcount = self.get_parameter('STAT_BOOTCNT', timeout=1)
                print("current=%s required=%u" % (str(current_bootcount), required_bootcount))
                if current_bootcount == required_bootcount:
                    break
            except NotAchievedException:
                pass

        # empty mav to avoid getting old timestamps:
        self.drain_mav()

        self.initialise_after_reboot_sitl()

    def set_streamrate(self, streamrate):
        tstart = time.time()
        while True:
            if time.time() - tstart > 10:
                raise AutoTestTimeoutException("stream rate change failed")

            self.mavproxy.send("set streamrate %u\n" % (streamrate))
            self.mavproxy.send("set streamrate\n")
            self.mavproxy.expect('.*streamrate ((?:-)?[0-9]+)', timeout=1)
            rate = self.mavproxy.match.group(1)
            print("rate: %s" % str(rate))
            if int(rate) == int(streamrate):
                break

        if streamrate <= 0:
            return

        self.drain_mav()
        m = self.mav.recv_match(type='SYSTEM_TIME',
                                blocking=True,
                                timeout=10)
        print("Received (%s)" % str(m))
        if m is None:
            raise NotAchievedException("Did not get SYSTEM_TIME")
        self.drain_mav()

    def htree_from_xml(self, xml_filepath):
        '''swiped from mavproxy_param.py'''
        xml = open(xml_filepath,'rb').read()
        from lxml import objectify
        objectify.enable_recursive_str()
        tree = objectify.fromstring(xml)
        htree = {}
        for p in tree.vehicles.parameters.param:
            n = p.get('name').split(':')[1]
            htree[n] = p
        for lib in tree.libraries.parameters:
            for p in lib.param:
                n = p.get('name')
                htree[n] = p
        return htree

    def test_parameter_documentation_get_all_parameters(self):
        xml_filepath = os.path.join(self.rootdir(), "apm.pdef.xml")
        param_parse_filepath = os.path.join(self.rootdir(), 'Tools', 'autotest', 'param_metadata', 'param_parse.py')
        try:
            os.unlink(xml_filepath)
        except OSError:
            pass
        vehicle = self.log_name()
        if vehicle == "HeliCopter":
            vehicle = "ArduCopter"
        if vehicle == "QuadPlane" or vehicle =="Soaring":
            vehicle = "ArduPlane"
        cmd = [param_parse_filepath, '--vehicle', vehicle]
#        cmd.append("--verbose")
        if util.run_cmd(cmd,
                        directory=util.reltopdir('.')) != 0:
            print("Failed param_parse.py (%s)" % vehicle)
            return False
        htree = self.htree_from_xml(xml_filepath)

        target_system = self.sysid_thismav()
        target_component = 1

        self.customise_SITL_commandline([
            "--unhide-groups"
        ])

        (parameters, seq_id) = self.download_parameters(target_system, target_component)
        fail = False
        for param in parameters.keys():
            if param.startswith("SIM_"):
                # too many of these to worry about
                continue
            if param not in htree:
                print("%s not in XML" % param)
                fail = True
        if fail:
            raise NotAchievedException("Downloaded parameters missing in XML")

        # FIXME: this should be doable if we filter out e.g BRD_* and CAN_*?
#        self.progress("Checking no extra parameters present in XML")
#        fail = False
#        for param in htree:
#            if param.startswith("SIM_"):
#                # too many of these to worry about
#                continue
#            if param not in parameters:
#                print("%s not in downloaded parameters but in XML" % param)
#                fail = True
#        if fail:
#            raise NotAchievedException("Extra parameters in XML")

    def find_format_defines(self, filepath):
        ret = {}
        for line in open(filepath,'rb').readlines():
            if type(line) == bytes:
                line = line.decode("utf-8")
            m = re.match('#define (\w+_(?:LABELS|FMT|UNITS|MULTS))\s+(".*")', line)
            if m is None:
                continue
            (a, b) = (m.group(1), m.group(2))
            if a in ret:
                raise NotAchievedException("Duplicate define for (%s)" % a)
            ret[a] = b
        return ret

    def vehicle_code_dirpath(self):
        '''returns path to vehicle-specific code directory e.g. ~/ardupilot/Rover'''
        dirname = self.log_name()
        if dirname == "QuadPlane":
            dirname = "ArduPlane"
        elif dirname == "HeliCopter":
            dirname = "ArduCopter"
        elif dirname == "Soaring":
            dirname = "ArduPlane"
        return os.path.join(self.rootdir(), dirname)

    def all_log_format_ids(self):
        ids = {}
        filepath = os.path.join(self.rootdir(), 'libraries', 'AP_Logger', 'LogStructure.h')
        state_outside = 0
        state_inside = 1
        state = state_outside

        defines = self.find_format_defines(filepath)

        linestate_none = 45
        linestate_within = 46
        linestate = linestate_none
        message_infos = []
        for line in open(filepath,'rb').readlines():
#            print("line: %s" % line)
            if type(line) == bytes:
                line = line.decode("utf-8")
            line = re.sub("//.*", "", line) # trim comments
            if re.match("\s*$", line):
                # blank line
                continue
            if state == state_outside:
                if "#define LOG_BASE_STRUCTURES" in line:
#                    self.progress("Moving inside")
                    state = state_inside
                continue
            if state == state_inside:
                if "#define LOG_COMMON_STRUCTURES" in line:
#                    self.progress("Moving outside")
                    state = state_outside
                    break
                if linestate == linestate_none:
                    if "#define LOG_SBP_STRUCTURES" in line:
                        continue
                    m = re.match("\s*{(.*)},\s*", line)
                    if m is not None:
                        # complete line
#                        print("Complete line: %s" % str(line))
                        message_infos.append(m.group(1))
                        continue
                    m = re.match("\s*{(.*)[\\\]", line)
                    if m is None:
                        raise NotAchievedException("Bad line %s" % line)
                    partial_line = m.group(1)
                    linestate = linestate_within
                    continue
                if linestate == linestate_within:
                    m = re.match("(.*)}", line)
                    if m is None:
                        raise NotAchievedException("Bad closing line (%s)" % line)
                    message_infos.append(partial_line + m.group(1))
                    linestate = linestate_none
                    continue
                raise NotAchievedException("Bad line (%s)")

        if linestate != linestate_none:
            raise NotAchievedException("Must be linestate-none at end of file")
        if state == state_inside:
            raise NotAchievedException("Must be outside at end of file")

        # now look in the vehicle-specific logfile:
        filepath = os.path.join(self.vehicle_code_dirpath(), "Log.cpp")
        state_outside = 67
        state_inside = 68
        state = state_outside
        linestate_none = 89
        linestate_within = 90
        linestate = linestate_none
        for line in open(filepath,'rb').readlines():
            if type(line) == bytes:
                line = line.decode("utf-8")
            line = re.sub("//.*", "", line) # trim comments
            if re.match("\s*$", line):
                # blank line
                continue
            if state == state_outside:
                if ("const LogStructure" in line or
                    "const struct LogStructure" in line):
                    state = state_inside;
                continue
            if state == state_inside:
                if re.match("};", line):
                    state = state_outside;
                    break;
                if linestate == linestate_none:
                    if "#if FRAME_CONFIG == HELI_FRAME" in line:
                        continue
                    if "#if PRECISION_LANDING == ENABLED" in line:
                        continue
                    if "#end" in line:
                        continue
                    if "LOG_COMMON_STRUCTURES" in line:
                        continue
                    m = re.match("\s*{(.*)},\s*", line)
                    if m is not None:
                        # complete line
#                        print("Complete line: %s" % str(line))
                        message_infos.append(m.group(1))
                        continue
                    m = re.match("\s*{(.*)", line)
                    if m is None:
                        raise NotAchievedException("Bad line %s" % line)
                    partial_line = m.group(1)
                    linestate = linestate_within
                    continue
                if linestate == linestate_within:
                    m = re.match("(.*)}", line)
                    if m is None:
                        raise NotAchievedException("Bad closing line (%s)" % line)
                    message_infos.append(partial_line + m.group(1))
                    linestate = linestate_none
                    continue
                raise NotAchievedException("Bad line (%s)")

        if state == state_inside:
            raise NotAchievedException("Should not be in state_inside at end")


        for message_info in message_infos:
            for define in defines:
                message_info = re.sub(define, defines[define], message_info)
            m = re.match('\s*LOG_\w+\s*,\s*sizeof\([^)]+\)\s*,\s*"(\w+)"\s*,\s*"(\w+)"\s*,\s*"([\w,]+)"\s*,\s*"([^"]+)"\s*,\s*"([^"]+)"\s*$', message_info)
            if m is None:
                raise NotAchievedException("Failed to match (%s)" % message_info)
            (name, fmt, labels, units, multipliers) = (m.group(1), m.group(2), m.group(3), m.group(4), m.group(5))
            if name in ids:
                raise NotAchievedException("Already seen a (%s) message" % name)
            ids[name] = {
                "name": name,
                "format": fmt,
                "labels": labels,
                "units": units,
                "multipliers": multipliers,
            }

        # now look for Log_Write(...) messages:
        base_directories = [
            os.path.join(self.rootdir(), 'libraries'),
            self.vehicle_code_dirpath(),
        ]
        log_write_statements = []
        for base_directory in base_directories:
            for root, dirs, files in os.walk(base_directory):
                state_outside = 37
                state_inside = 38
                state = state_outside
                for f in files:
                    if not re.search("[.]cpp$", f):
                        continue
                    filepath = os.path.join(root, f)
                    count = 0
                    for line in open(filepath,'rb').readlines():
                        if type(line) == bytes:
                            line = line.decode("utf-8")
                        if state == state_outside:
                            if re.match("\s*AP::logger\(\)[.]Write\(", line):
                                state = state_inside
                                log_write_statement = line
                            continue
                        if state == state_inside:
                            log_write_statement += line
                            if re.match(".*\);", line):
                                log_write_statements.append(log_write_statement)
                                state = state_outside
                        count += 1
                    if state != state_outside:
                        raise NotAchievedException("Expected to be outside at end of file")
#                    print("%s has %u lines" % (f, count))
        # change all whitespace to single space
        log_write_statements = [re.sub("\s+", " ", x) for x in log_write_statements]
#        print("Got log-write-statements: %s" % str(log_write_statements))
        results = []
        for log_write_statement in log_write_statements:
            for define in defines:
                log_write_statement = re.sub(define, defines[define], log_write_statement)
            my_re = ' AP::logger\(\)[.]Write\(\s*"(\w+)"\s*,\s*"([\w,]+)".*\);'
            m = re.match(my_re, log_write_statement)
            if m is None:
                if "TimeUS,C,Cnt,IMUMin,IMUMax,EKFMin" in log_write_statement:
                    # special-case for logging ekf timing:
                    for name in ["NKT", "XKT"]:
                        x = re.sub(" name,", '"'+name+'",', log_write_statement)
                        m = re.match(my_re, x)
                        if m is None:
                            raise NotAchievedException("Did not match (%s)", x)
                        results.append((m.group(1), m.group(2)))
                    continue
                raise NotAchievedException("Did not match (%s)" % (log_write_statement))
            else:
                results.append((m.group(1), m.group(2)))

        for result in results:
            (name, labels) = result
            if name in ids:
                raise Exception("Already have id for (%s)" % name)
#            self.progress("Adding Log_Write result (%s)" % name)
            ids[name] = {
                "name": name,
                "labels": labels,
            }

        if len(ids) == 0:
            raise NotAchievedException("Did not get any ids")

        return ids

    def test_onboard_logging_generation(self):
        '''just generates, as we can't do a lot of testing'''
        xml_filepath = os.path.join(self.rootdir(), "LogMessages.xml")
        parse_filepath = os.path.join(self.rootdir(), 'Tools', 'autotest', 'logger_metadata', 'parse.py')
        try:
            os.unlink(xml_filepath)
        except OSError:
            pass
        vehicle = self.log_name()
        vehicle_map = {
            "ArduCopter": "Copter",
            "HeliCopter": "Copter",
            "ArduPlane": "Plane",
            "QuadPlane": "Plane",
            "Soaring": "Plane",
            "Rover": "Rover",
            "AntennaTracker": "Tracker",
            "ArduSub": "Sub",
        }
        vehicle = vehicle_map[vehicle]

        cmd = [parse_filepath, '--vehicle', vehicle]
#        cmd.append("--verbose")
        if util.run_cmd(cmd,
                        directory=util.reltopdir('.')) != 0:
            print("Failed parse.py (%s)" % vehicle)
            return False
        length = os.path.getsize(xml_filepath)
        min_length = 1024
        if length < min_length:
            raise NotAchievedException("short xml file (%u < %u)" %
                                       (length, min_length))
        self.progress("xml file length is %u" % length)

        from lxml import objectify
        xml = open(xml_filepath,'rb').read()
        objectify.enable_recursive_str()
        tree = objectify.fromstring(xml)

        docco_ids = {}
        for thing in tree.logformat:
            name = str(thing.get("name"))
            docco_ids[name] = {
                "name": name,
                "labels": [],
            }
            for field in thing.fields.field:
#                print("field: (%s)" % str(field))
                fieldname = field.get("name")
#                print("Got (%s.%s)" % (name,str(fieldname)))
                docco_ids[name]["labels"].append(fieldname)

        code_ids = self.all_log_format_ids()
        print("Code ids: (%s)" % str(code_ids.keys()))
        print("Docco ids: (%s)" % str(docco_ids.keys()))

        for name in sorted(code_ids.keys()):
            if name not in docco_ids:
                self.progress("Undocumented message: %s" % name)
                continue
            seen_labels = {}
            for label in code_ids[name]["labels"].split(","):
                if label in seen_labels:
                    raise NotAchievedException("%s.%s is duplicate label" %
                                               (name, label))
                seen_labels[label] = True
                if label not in docco_ids[name]["labels"]:
                    raise NotAchievedException("%s.%s not in documented fields (have (%s))" %
                                               (name, label, ",".join(docco_ids[name]["labels"])))
        for name in sorted(docco_ids):
            if name not in code_ids:
                raise NotAchievedException("Documented message (%s) not in code" % name)
            for label in docco_ids[name]["labels"]:
                if label not in code_ids[name]["labels"].split(","):
                    raise NotAchievedException("documented field %s.%s not found in code" %
                                               (name, label))

    def initialise_after_reboot_sitl(self):

        # after reboot stream-rates may be zero.  Prompt MAVProxy to
        # send a rate-change message by changing away from our normal
        # stream rates and back again:
        self.set_streamrate(self.sitl_streamrate()+1)
        self.set_streamrate(self.sitl_streamrate())
        self.progress("Reboot complete")

    def customise_SITL_commandline(self, customisations, model=None, defaults_filepath=None):
        '''customisations could be "--uartF=sim:nmea" '''
        self.contexts[-1].sitl_commandline_customised = True
        self.stop_SITL()
        self.start_SITL(model=model,
                        defaults_filepath=defaults_filepath,
                        customisations=customisations,
                        wipe=False)
        self.wait_heartbeat(drain_mav=True)
        # MAVProxy only checks the streamrates once every 15 seconds.
        # Encourage it:
        self.set_streamrate(self.sitl_streamrate()+1)
        self.set_streamrate(self.sitl_streamrate())
        # we also need to wait for MAVProxy to requests streams again
        # - in particular, RC_CHANNELS.
        m = self.mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=15)
        if m is None:
            raise NotAchievedException("No RC_CHANNELS message after restarting SITL")

    def reset_SITL_commandline(self):
        self.progress("Resetting SITL commandline to default")
        self.stop_SITL()
        self.start_SITL(wipe=False)
        self.set_streamrate(self.sitl_streamrate()+1)
        self.set_streamrate(self.sitl_streamrate())
        self.progress("Reset SITL commandline to default")

    def stop_SITL(self):
        self.progress("Stopping SITL")
        util.pexpect_close(self.sitl)

    def close(self):
        """Tidy up after running all tests."""
        if self.use_map:
            self.mavproxy.send("module unload map\n")
            self.mavproxy.expect("Unloaded module map")

        self.mav.close()
        util.pexpect_close(self.mavproxy)
        self.stop_SITL()

        valgrind_log = util.valgrind_log_filepath(binary=self.binary,
                                                  model=self.frame)
        if os.path.exists(valgrind_log):
            os.chmod(valgrind_log, 0o644)
            shutil.copy(valgrind_log,
                        self.buildlogs_path("%s-valgrind.log" %
                                            self.log_name()))

    def start_test(self, description):
        self.progress("#")
        self.progress("########## %s  ##########" % description)
        self.progress("#")

    def try_symlink_tlog(self):
        self.buildlog = self.buildlogs_path(self.log_name() + "-test.tlog")
        self.progress("buildlog=%s" % self.buildlog)
        if os.path.exists(self.buildlog):
            os.unlink(self.buildlog)
        try:
            os.link(self.logfile, self.buildlog)
        except OSError as error:
            self.progress("OSError [%d]: %s" % (error.errno, error.strerror))
            self.progress("WARN: Failed to create symlink: %s => %s, "
                          "will copy tlog manually to target location" %
                          (self.logfile, self.buildlog))
            self.copy_tlog = True

    #################################################
    # GENERAL UTILITIES
    #################################################
    def expect_list_clear(self):
        """clear the expect list."""
        global expect_list
        for p in expect_list[:]:
            expect_list.remove(p)

    def expect_list_extend(self, list_to_add):
        """Extend the expect list."""
        global expect_list
        expect_list.extend(list_to_add)

    def drain_all_pexpects(self):
        global expect_list
        for p in expect_list:
            util.pexpect_drain(p)

    def idle_hook(self, mav):
        """Called when waiting for a mavlink message."""
        self.drain_all_pexpects()

    def message_hook(self, mav, msg):
        """Called as each mavlink msg is received."""
        self.idle_hook(mav)

    def expect_callback(self, e):
        """Called when waiting for a expect pattern."""
        global expect_list
        for p in expect_list:
            if p == e:
                continue
            util.pexpect_drain(p)

    def drain_mav_unparsed(self, quiet=False):
        count = 0
        tstart = time.time()
        while True:
            this = self.mav.recv(1000000)
            if len(this) == 0:
                break
            count += len(this)
        if quiet:
            return
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count/float(tdelta),)
        self.progress("Drained %u bytes from mav (%s).  These were unparsed." % (count, rate))

    def drain_mav(self, mav=None):
        if mav is None:
            mav = self.mav
        count = 0
        tstart = time.time()
        while mav.recv_match(blocking=False) is not None:
            count += 1
        tdelta = time.time() - tstart
        if tdelta == 0:
            rate = "instantly"
        else:
            rate = "%f/s" % (count/float(tdelta),)

        self.progress("Drained %u messages from mav (%s)" % (count, rate))

    #################################################
    # SIM UTILITIES
    #################################################
    def get_sim_time(self, timeout=60):
        """Get SITL time in seconds."""
        m = self.mav.recv_match(type='SYSTEM_TIME', blocking=True, timeout=timeout)
        if m is None:
            raise AutoTestTimeoutException("Did not get SYSTEM_TIME message")
        return m.time_boot_ms * 1.0e-3

    def get_sim_time_cached(self):
        """Get SITL time in seconds."""
        x = self.mav.messages.get("SYSTEM_TIME", None)
        if x is None:
            raise NotAchievedException("No cached time available")
        return x.time_boot_ms * 1.0e-3

    def sim_location(self):
        """Return current simulator location."""
        m = self.mav.recv_match(type='SIMSTATE', blocking=True)
        return mavutil.location(m.lat*1.0e-7,
                                m.lng*1.0e-7,
                                0,
                                math.degrees(m.yaw))

    def save_wp(self, ch=7):
        """Trigger RC Aux to save waypoint."""
        self.set_rc(ch, 1000)
        self.delay_sim_time(1)
        self.set_rc(ch, 2000)
        self.delay_sim_time(1)
        self.set_rc(ch, 1000)
        self.delay_sim_time(1)

    def clear_wp(self, ch=8):
        """Trigger RC Aux to clear waypoint."""
        self.progress("Clearing waypoints")
        self.set_rc(ch, 1000)
        self.delay_sim_time(0.5)
        self.set_rc(ch, 2000)
        self.delay_sim_time(0.5)
        self.set_rc(ch, 1000)
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting 0 waypoints')

    def log_download(self, filename, timeout=360, upload_logs=False):
        """Download latest log."""
        self.wait_heartbeat()
        self.mavproxy.send("module load log\n")
        self.mavproxy.expect("Loaded module log")
        self.mavproxy.send("log list\n")
        self.mavproxy.expect("numLogs")
        self.wait_heartbeat()
        self.wait_heartbeat()
        self.mavproxy.send("set shownoise 0\n")
        self.mavproxy.send("log download latest %s\n" % filename)
        self.mavproxy.expect("Finished downloading", timeout=timeout)
        self.mavproxy.send("module unload log\n")
        self.mavproxy.expect("Unloaded module log")
        self.drain_mav_unparsed()
        self.wait_heartbeat()
        self.wait_heartbeat()
        if upload_logs and os.getenv("AUTOTEST_UPLOAD"):
            # optionally upload logs to server so we can see travis failure logs
            import datetime
            import glob
            import subprocess
            logdir = os.path.dirname(filename)
            datedir = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
            flist = glob.glob("logs/*.BIN")
            for e in ['BIN', 'bin', 'tlog']:
                flist += glob.glob(os.path.join(logdir, '*.%s' % e))
            print("Uploading %u logs to https://firmware.ardupilot.org/CI-Logs/%s" % (len(flist), datedir))
            cmd = ['rsync', '-avz'] + flist + ['cilogs@autotest.ardupilot.org::CI-Logs/%s/' % datedir]
            subprocess.call(cmd)

    def show_gps_and_sim_positions(self, on_off):
        """Allow to display gps and actual position on map."""
        if on_off is True:
            # turn on simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 1\n')
            self.mavproxy.send('map set showsimpos 1\n')
        else:
            # turn off simulator display of gps and actual position
            self.mavproxy.send('map set showgpspos 0\n')
            self.mavproxy.send('map set showsimpos 0\n')

    @staticmethod
    def mission_count(filename):
        """Load a mission from a file and return number of waypoints."""
        wploader = mavwp.MAVWPLoader()
        wploader.load(filename)
        return wploader.count()

    def install_message_hook(self, hook):
        self.mav.message_hooks.append(hook)

    def remove_message_hook(self, hook):
        oldlen = len(self.mav.message_hooks)
        self.mav.message_hooks = list(filter(lambda x : x != hook,
                                        self.mav.message_hooks))
        if len(self.mav.message_hooks) == oldlen:
            raise NotAchievedException("Failed to remove hook")

    def rootdir(self):
        this_dir = os.path.dirname(__file__)
        return os.path.realpath(os.path.join(this_dir, "../.."))

    def assert_mission_files_same(self, file1, file2):
        self.progress("Comparing (%s) and (%s)" % (file1, file2, ))
        f1 = open(file1)
        f2 = open(file2)
        for l1, l2 in zip(f1, f2):
            l1 = l1.rstrip("\r\n")
            l2 = l2.rstrip("\r\n")
            if l1 == l2:
                # e.g. the first "QGC WPL 110" line
                continue
            if re.match(r"0\s", l1):
                # home changes...
                continue
            l1 = l1.rstrip()
            l2 = l2.rstrip()
            fields1 = re.split(r"\s+", l1)
            fields2 = re.split(r"\s+", l2)
            # line = int(fields1[0])
            t = int(fields1[3]) # mission item type
            for (count, (i1, i2)) in enumerate(zip(fields1, fields2)):
                if count == 2: # frame
                    if t in [mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                             mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                             mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                             mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                             mavutil.mavlink.MAV_CMD_DO_JUMP,
                             mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                             ]:
                        # ardupilot doesn't remember frame on these commands
                        if int(i1) == 3:
                            i1 = 0
                        if int(i2) == 3:
                            i2 = 0
                if count == 6: # param 3
                    if t in [mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME]:
                        # ardupilot canonicalises this to -1 for ccw or 1 for cw.
                        if float(i1) == 0:
                            i1 = 1.0
                        if float(i2) == 0:
                            i2 = 1.0
                if count == 7: # param 4
                    if t == mavutil.mavlink.MAV_CMD_NAV_LAND:
                        # ardupilot canonicalises "0" to "1" param 4 (yaw)
                        if int(float(i1)) == 0:
                            i1 = 1
                        if int(float(i2)) == 0:
                            i2 = 1
                if 0 <= count <= 3 or 11 <= count <= 11:
                    if int(i1) != int(i2):
                        raise ValueError("Files have different content: (%s vs %s) (%s vs %s) (%d vs %d) (count=%u)" %
                                         (file1, file2, l1, l2, int(i1), int(i2), count))  # NOCI
                    continue
                if 4 <= count <= 10:
                    f_i1 = float(i1)
                    f_i2 = float(i2)
                    delta = abs(f_i1 - f_i2)
                    max_allowed_delta = 0.000009
                    if delta > max_allowed_delta:
                        raise ValueError(
                            ("Files have different (float) content: " +
                             "(%s) and (%s) " +
                             "(%s vs %s) " +
                             "(%f vs %f) " +
                             "(%.10f) " +
                             "(count=%u)") %
                            (file1, file2,
                             l1, l2,
                             f_i1, f_i2,
                             delta,
                             count)) # NOCI
                    continue
                raise ValueError("count %u not handled" % count)
        self.progress("Files same")

    def assert_rally_files_same(self, file1, file2):
        self.progress("Comparing (%s) and (%s)" % (file1, file2, ))
        f1 = open(file1)
        f2 = open(file2)
        lines_f1 = f1.readlines()
        lines_f2 = f2.readlines()
        self.assert_rally_content_same(lines_f1, lines_f2)

    def assert_rally_filepath_content(self, file1, content):
        f1 = open(file1)
        lines_f1 = f1.readlines()
        lines_content = content.split("\n")
        print("lines content: %s" % str(lines_content))
        self.assert_rally_content_same(lines_f1, lines_content)

    def assert_rally_content_same(self, f1, f2):
        '''check each line in f1 matches one-to-one with f2'''
        for l1, l2 in zip(f1, f2):
            print("l1: %s" % l1)
            print("l2: %s" % l2)
            l1 = l1.rstrip("\n")
            l2 = l2.rstrip("\n")
            l1 = l1.rstrip("\r")
            l2 = l2.rstrip("\r")
            if l1 == l2:
                # e.g. the first "QGC WPL 110" line
                continue
            if re.match(r"0\s", l1):
                # home changes...
                continue
            l1 = l1.rstrip()
            l2 = l2.rstrip()
            print("al1: %s" % str(l1))
            print("al2: %s" % str(l2))
            fields1 = re.split(r"\s+", l1)
            fields2 = re.split(r"\s+", l2)
            # line = int(fields1[0])
            t = int(fields1[3]) # mission item type
            for (count, (i1, i2)) in enumerate(zip(fields1, fields2)):
                # if count == 2: # frame
                #     if t in [mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                #              mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                #              mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                #              mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                #              mavutil.mavlink.MAV_CMD_DO_JUMP,
                #              mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL,
                #              ]:
                #         # ardupilot doesn't remember frame on these commands
                #         if int(i1) == 3:
                #             i1 = 0
                #         if int(i2) == 3:
                #             i2 = 0
                # if count == 6: # param 3
                #     if t in [mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME]:
                #         # ardupilot canonicalises this to -1 for ccw or 1 for cw.
                #         if float(i1) == 0:
                #             i1 = 1.0
                #         if float(i2) == 0:
                #             i2 = 1.0
                # if count == 7: # param 4
                #     if t == mavutil.mavlink.MAV_CMD_NAV_LAND:
                #         # ardupilot canonicalises "0" to "1" param 4 (yaw)
                #         if int(float(i1)) == 0:
                #             i1 = 1
                #         if int(float(i2)) == 0:
                #             i2 = 1
                if 0 <= count <= 3 or 11 <= count <= 11:
                    if int(i1) != int(i2):
                        raise ValueError("Rally points different: (%s vs %s) (%d vs %d) (count=%u)" %
                                         (l1, l2, int(i1), int(i2), count))  # NOCI
                    continue
                if 4 <= count <= 10:
                    f_i1 = float(i1)
                    f_i2 = float(i2)
                    delta = abs(f_i1 - f_i2)
                    max_allowed_delta = 0.000009
                    if delta > max_allowed_delta:
                        raise ValueError(
                            ("Rally has different (float) content: " +
                             "(%s vs %s) " +
                             "(%f vs %f) " +
                             "(%.10f) " +
                             "(count=%u)") %
                            (l1, l2,
                             f_i1, f_i2,
                             delta,
                             count)) # NOCI
                    continue
                raise ValueError("count %u not handled" % count)
        self.progress("Rally content same")

    def load_rally(self, filename):
        """Load rally points from a file to flight controller."""
        self.progress("Loading rally points (%s)" % filename)
        path = os.path.join(testdir, self.current_test_name_directory, filename)
        self.mavproxy.send('rally load %s\n' % path)
        self.mavproxy.expect("Loaded")

    def load_sample_mission(self):
        self.load_mission(self.sample_mission_filename())

    def load_mission(self, filename):
        return self.load_mission_from_filepath(self.current_test_name_directory, filename)

    def load_mission_from_filepath(self, filepath, filename):
        """Load a mission from a file to flight controller."""
        self.progress("Loading mission (%s)" % filename)
        path = os.path.join(testdir, filepath, filename)
        tstart = self.get_sim_time_cached()
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to do waypoint thing")
            # the following hack is to get around MAVProxy statustext deduping:
            while time.time() - self.last_wp_load < 3:
                self.progress("Waiting for MAVProxy de-dupe timer to expire")
                time.sleep(1)
            self.mavproxy.send('wp load %s\n' % path)
            self.mavproxy.expect('Loaded ([0-9]+) waypoints from')
            load_count = self.mavproxy.match.group(1)
            self.last_wp_load = time.time()
            self.mavproxy.expect("Flight plan received")
            self.mavproxy.send('wp list\n')
            self.mavproxy.expect('Requesting ([0-9]+) waypoints')
            request_count = self.mavproxy.match.group(1)
            if load_count != request_count:
                self.progress("request_count=%s != load_count=%s" %
                              (request_count, load_count))
                continue
            self.mavproxy.expect('Saved ([0-9]+) waypoints to (.+?way.txt)')
            save_count = self.mavproxy.match.group(1)
            if save_count != request_count:
                raise NotAchievedException("request count != load count")
            # warning: this assumes MAVProxy was started in the CWD!
            # on the autotest server we invoke autotest.py one-up from
            # the git root, like this:
            # timelimit 32000 APM/Tools/autotest/autotest.py --timeout=30000 > buildlogs/autotest-output.txt 2>&1
            # that means the MAVProxy log files are not reltopdir!
            saved_filepath = self.mavproxy.match.group(2)
            saved_filepath = saved_filepath.rstrip()
            self.assert_mission_files_same(path, saved_filepath)
            break
        self.mavproxy.send('wp status\n')
        self.mavproxy.expect('Have (\d+) of (\d+)')
        status_have = self.mavproxy.match.group(1)
        status_want = self.mavproxy.match.group(2)
        if status_have != status_want:
            raise ValueError("status count mismatch")
        if status_have != save_count:
            raise ValueError("status have not equal to save count")

        wploader = mavwp.MAVWPLoader()
        wploader.load(path)
        num_wp = wploader.count()
        if num_wp != int(status_have):
            raise ValueError("num_wp=%u != status_have=%u" %
            (num_wp, int(status_have)))
        if num_wp == 0:
            raise ValueError("No waypoints loaded?!")
        return num_wp

    def save_mission_to_file(self, filename):
        """Save a mission to a file"""
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        self.mavproxy.send('wp save %s\n' % filename)
        self.mavproxy.expect('Saved ([0-9]+) waypoints')
        num_wp = int(self.mavproxy.match.group(1))
        self.progress("num_wp: %d" % num_wp)
        return num_wp

    def rc_defaults(self):
        return {
            1: 1500,
            2: 1500,
            3: 1500,
            4: 1500,
            5: 1500,
            6: 1500,
            7: 1500,
            8: 1500,
            9: 1500,
            10: 1500,
            11: 1500,
            12: 1500,
            13: 1500,
            14: 1500,
            15: 1500,
            16: 1500,
        }

    def set_rc_from_map(self, _map, timeout=2000):
        map_copy = _map.copy()
        tstart = self.get_sim_time()
        while len(map_copy.keys()):
            if self.get_sim_time_cached() - tstart > timeout:
                raise SetRCTimeout("Failed to set RC channels")
            for chan in map_copy:
                value = map_copy[chan]
                self.send_set_rc(chan, value)
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            self.progress("m: %s" % m)
            new = dict()
            for chan in map_copy:
                chan_pwm = getattr(m, "chan" + str(chan) + "_raw")
                if chan_pwm != map_copy[chan]:
                    new[chan] = map_copy[chan]
            map_copy = new

    def set_rc_default(self):
        """Setup all simulated RC control to 1500."""
        _defaults = self.rc_defaults()
        self.set_rc_from_map(_defaults)

    def check_rc_defaults(self):
        """Ensure all rc outputs are at defaults"""
        _defaults = self.rc_defaults()
        m = self.mav.recv_match(type='RC_CHANNELS', blocking=True, timeout=5)
        if m is None:
            raise NotAchievedException("No RC_CHANNELS messages?!")
        need_set = {}
        for chan in _defaults:
            default_value = _defaults[chan]
            current_value = getattr(m, "chan" + str(chan) + "_raw")
            if default_value != current_value:
                self.progress("chan=%u needs resetting is=%u want=%u" %
                              (chan, current_value, default_value))
                need_set[chan] = default_value
        self.set_rc_from_map(need_set)

    def send_set_rc(self, chan, pwm):
        self.mavproxy.send('rc %u %u\n' % (chan, pwm))

    def set_rc(self, chan, pwm, timeout=2000):
        """Setup a simulated RC control to a PWM value"""
        self.drain_mav()
        tstart = self.get_sim_time()
        wclock = time.time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.send_set_rc(chan, pwm)
            m = self.mav.recv_match(type='RC_CHANNELS', blocking=True)
            chan_pwm = getattr(m, "chan" + str(chan) + "_raw")
            wclock_delta = time.time() - wclock
            sim_time_delta = self.get_sim_time_cached()-tstart
            if sim_time_delta == 0:
                time_ratio = None
            else:
                time_ratio = wclock_delta / sim_time_delta
            self.progress("set_rc (wc=%s st=%s r=%s): ch=%u want=%u got=%u" %
                          (wclock_delta,
                           sim_time_delta,
                           time_ratio,
                           chan,
                           pwm,
                           chan_pwm))
            if chan_pwm == pwm:
                delta = self.get_sim_time_cached() - tstart
                if delta > self.max_set_rc_timeout:
                    self.max_set_rc_timeout = delta
                return True
        raise SetRCTimeout("Failed to send RC commands to channel %s" % str(chan))

    def location_offset_ne(self, location, north, east):
        '''move location in metres'''
        print("old: %f %f" % (location.lat, location.lng))
        (lat, lng) = mp_util.gps_offset(location.lat, location.lng, east, north)
        location.lat = lat
        location.lng = lng
        print("new: %f %f" % (location.lat, location.lng))

    def zero_throttle(self):
        """Set throttle to zero."""
        if self.is_rover():
            self.set_rc(3, 1500)
        else:
            self.set_rc(3, 1000)

    def set_output_to_max(self, chan):
        """Set output to max with RC Radio taking into account REVERSED parameter."""
        is_reversed = self.get_parameter("RC%u_REVERSED" % chan)
        out_max = int(self.get_parameter("RC%u_MAX" % chan))
        out_min = int(self.get_parameter("RC%u_MIN" % chan))
        if is_reversed == 0:
            self.set_rc(chan, out_max)
        else:
            self.set_rc(chan, out_min)

    def set_output_to_min(self, chan):
        """Set output to min with RC Radio taking into account REVERSED parameter."""
        is_reversed = self.get_parameter("RC%u_REVERSED" % chan)
        out_max = int(self.get_parameter("RC%u_MAX" % chan))
        out_min = int(self.get_parameter("RC%u_MIN" % chan))
        if is_reversed == 0:
            self.set_rc(chan, out_min)
        else:
            self.set_rc(chan, out_max)

    def set_output_to_trim(self, chan):
        """Set output to trim with RC Radio."""
        out_trim = int(self.get_parameter("RC%u_TRIM" % chan))
        self.set_rc(chan, out_trim)

    def get_stick_arming_channel(self):
        """Return the Rudder channel number as set in parameter."""
        raise ErrorException("Rudder parameter is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    def get_disarm_delay(self):
        """Return disarm delay value."""
        raise ErrorException("Disarm delay is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    def arming_test_mission(self):
        """Load arming test mission.
        This mission is used to allow to change mode to AUTO. For each vehicle
        it get an unlimited wait waypoint and the starting takeoff if needed."""
        if self.is_rover() or self.is_plane() or self.is_sub():
            return os.path.join(testdir, self.current_test_name_directory + "test_arming.txt")
        else:
            return None

    def armed(self):
        """Return true if vehicle is armed and safetyoff"""
        return self.mav.motors_armed()

    def send_mavlink_arm_command(self):
        target_sysid = 1
        target_compid = 1
        self.mav.mav.command_long_send(
            target_sysid,
            target_compid,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1, # confirmation
            1,  # ARM
            0,
            0,
            0,
            0,
            0,
            0)

    def set_analog_rangefinder_parameters(self):
        self.set_parameter("RNGFND1_TYPE", 1)
        self.set_parameter("RNGFND1_MIN_CM", 0)
        self.set_parameter("RNGFND1_MAX_CM", 4000)
        self.set_parameter("RNGFND1_SCALING", 12.12)
        self.set_parameter("RNGFND1_PIN", 0)

    def send_debug_trap(self, timeout=6000):
        self.progress("Sending trap to autopilot")
        self.run_cmd(mavutil.mavlink.MAV_CMD_DEBUG_TRAP,
                     32451, # magic number to trap
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)

    def arm_vehicle(self, timeout=20):
        """Arm vehicle with mavlink arm message."""
        self.progress("Arm motors with MAVLink cmd")
        self.drain_mav()
        tstart = self.get_sim_time()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        while self.get_sim_time_cached() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("Motors ARMED")
                return True
        raise AutoTestTimeoutException("Failed to ARM with mavlink")

    def disarm_vehicle(self, timeout=60, force=False):
        """Disarm vehicle with mavlink disarm message."""
        self.progress("Disarm motors with MAVLink cmd")
        self.drain_mav_unparsed()
        p2 = 0
        if force:
            p2 = 21196 # magic force disarm value
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     0,  # DISARM
                     p2,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        return self.wait_disarmed()

    def wait_disarmed_default_wait_time(self):
        return 30

    def wait_disarmed(self, timeout=None, tstart=None):
        if timeout is None:
            timeout = self.wait_disarmed_default_wait_time()
        self.progress("Waiting for DISARM")
        if tstart is None:
            tstart = self.get_sim_time()
        while True:
            delta = self.get_sim_time_cached() - tstart
            if delta > timeout:
                raise AutoTestTimeoutException("Failed to DISARM")
            self.wait_heartbeat()
            self.progress("Got heartbeat")
            if not self.mav.motors_armed():
                self.progress("DISARMED after %.2f seconds (allowed=%.2f)" %
                              (delta, timeout))
                return True

    def CPUFailsafe(self):
        '''Most vehicles just disarm on failsafe'''
        # customising the SITL commandline ensures the process will
        # get stopped/started at the end of the test
        if self.frame is None:
            raise ValueError("Frame is none?")
        self.customise_SITL_commandline([])
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Sending enter-cpu-lockup")
        # when we're in CPU lockup we don't get SYSTEM_TIME messages,
        # so get_sim_time breaks:
        tstart = self.get_sim_time()
        self.send_cmd_enter_cpu_lockup()
        self.wait_disarmed(timeout=5, tstart=tstart)


    def cpufailsafe_wait_servo_channel_value(self, channel, value, timeout=30):
        '''we get restricted messages while doing cpufailsafe, this working then'''
        start = time.time()
        while True:
            if time.time() - start > timeout:
                raise NotAchievedException("Did not achieve value")
            m = self.mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1)

            if m is None:
                raise NotAchievedException("Did not get SERVO_OUTPUT_RAW")
            channel_field = "servo%u_raw" % channel
            m_value = getattr(m, channel_field, None)
            self.progress("Servo%u=%u want=%u" % (channel, m_value, value))
            if m_value == value:
                break

    def plane_CPUFailsafe(self):
        '''In lockup Plane should copy RC inputs to RC outputs'''
        # customising the SITL commandline ensures the process will
        # get stopped/started at the end of the test
        self.customise_SITL_commandline([])
        self.wait_ready_to_arm()
        self.arm_vehicle()
        self.progress("Sending enter-cpu-lockup")
        # when we're in CPU lockup we don't get SYSTEM_TIME messages,
        # so get_sim_time breaks:
        self.send_cmd_enter_cpu_lockup()
        start_time = time.time() # not sim time!
        while True:
            want = "Initialising ArduPilot"
            if time.time() - start_time > 30:
                raise NotAchievedException("Did not get %s" % want)
            # we still need to parse the incoming messages:
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=0.1)
            x = self.mav.messages.get("STATUSTEXT", None)
            if x is not None and want in x.text:
                break
        # Different scaling for RC input and servo output means the
        # servo output value isn't the rc input value:
        self.progress("Setting RC to 1200")
        self.send_set_rc(2, 1200)
        self.progress("Waiting for servo of 1260")
        self.cpufailsafe_wait_servo_channel_value(2, 1260)
        self.send_set_rc(2, 1700)
        self.cpufailsafe_wait_servo_channel_value(2, 1660)

    def mavproxy_arm_vehicle(self):
        """Arm vehicle with mavlink arm message send from MAVProxy."""
        self.progress("Arm motors with MavProxy")
        self.mavproxy.send('arm throttle\n')
        self.mav.motors_armed_wait()
        self.progress("ARMED")
        return True

    def mavproxy_disarm_vehicle(self):
        """Disarm vehicle with mavlink disarm message send from MAVProxy."""
        self.progress("Disarm motors with MavProxy")
        self.mavproxy.send('disarm\n')
        self.wait_disarmed()
        return True

    def arm_motors_with_rc_input(self, timeout=20):
        """Arm motors with radio."""
        self.progress("Arm motors with radio")
        self.set_output_to_max(self.get_stick_arming_channel())
        tstart = self.get_sim_time()
        while True:
            self.wait_heartbeat()
            tdelta = self.get_sim_time_cached() - tstart
            if self.mav.motors_armed():
                self.progress("MOTORS ARMED OK WITH RADIO")
                self.set_output_to_trim(self.get_stick_arming_channel())
                self.progress("Arm in %ss" % tdelta)  # TODO check arming time
                return True
            print("Not armed after %f seconds" % (tdelta))
            if tdelta > timeout:
                break
        self.progress("Failed to ARM with radio")
        self.set_output_to_trim(self.get_stick_arming_channel())
        return False

    def disarm_motors_with_rc_input(self, timeout=20):
        """Disarm motors with radio."""
        self.progress("Disarm motors with radio")
        self.set_output_to_min(self.get_stick_arming_channel())
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_delay = self.get_sim_time_cached() - tstart
                self.progress("MOTORS DISARMED OK WITH RADIO")
                self.set_output_to_trim(self.get_stick_arming_channel())
                self.progress("Disarm in %ss" % disarm_delay)  # TODO check disarming time
                return True
        self.progress("Failed to DISARM with radio")
        self.set_output_to_trim(self.get_stick_arming_channel())
        return False

    def arm_motors_with_switch(self, switch_chan, timeout=20):
        """Arm motors with switch."""
        self.progress("Arm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 2000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < timeout:
            self.wait_heartbeat()
            if self.mav.motors_armed():
                self.progress("MOTORS ARMED OK WITH SWITCH")
                return True
        self.progress("Failed to ARM with switch")
        return False

    def disarm_motors_with_switch(self, switch_chan, timeout=20):
        """Disarm motors with switch."""
        self.progress("Disarm motors with switch %d" % switch_chan)
        self.set_rc(switch_chan, 1000)
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                self.progress("MOTORS DISARMED OK WITH SWITCH")
                return True
        self.progress("Failed to DISARM with switch")
        return False

    def disarm_wait(self, timeout=10):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not disarm")
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                return

    def wait_autodisarm_motors(self):
        """Wait for Autodisarm motors within disarm delay
        this feature is only available in copter (DISARM_DELAY) and plane (LAND_DISARMDELAY)."""
        self.progress("Wait autodisarming motors")
        disarm_delay = self.get_disarm_delay()
        tstart = self.get_sim_time()
        timeout = disarm_delay * 2
        while self.get_sim_time_cached() < tstart + timeout:
            self.wait_heartbeat()
            if not self.mav.motors_armed():
                disarm_time = self.get_sim_time_cached() - tstart
                self.progress("MOTORS AUTODISARMED")
                self.progress("Autodisarm in %ss, expect less than %ss" % (disarm_time, disarm_delay))
                return disarm_time <= disarm_delay
        raise AutoTestTimeoutException("Failed to AUTODISARM")

    def set_autodisarm_delay(self, delay):
        """Set autodisarm delay"""
        raise ErrorException("Auto disarm is not supported by vehicle %s frame %s", (self.vehicleinfo_key(), self.frame))

    @staticmethod
    def should_fetch_all_for_parameter_change(param_name):
        if fnmatch.fnmatch(param_name, "*_ENABLE") or fnmatch.fnmatch(param_name, "*_ENABLED"):
            return True
        if param_name in ["ARSPD_TYPE",
                          "ARSPD2_TYPE",
                          "BATT2_MONITOR",
                          "CAN_DRIVER",
                          "COMPASS_PMOT_EN",
                          "OSD_TYPE",
                          "RSSI_TYPE",
                          "WENC_TYPE"]:
            return True
        return False

    def set_parameter(self, name, value, add_to_context=True, epsilon=0.0002, retries=10):
        """Set parameters from vehicle."""
        self.progress("Setting %s to %f" % (name, value))
        old_value = self.get_parameter(name, retry=2)
        for i in range(1, retries+2):
            self.mavproxy.send("param set %s %s\n" % (name, str(value)))
            returned_value = self.get_parameter(name)
            delta = float(value) - returned_value
            if abs(delta) < epsilon:
                # yes, near-enough-to-equal.
                if add_to_context:
                    self.context_get().parameters.append((name, old_value))
                if self.should_fetch_all_for_parameter_change(name.upper()) and value != 0:
                    self.fetch_parameters()
                return
        raise ValueError("Param fetch returned incorrect value (%s) vs (%s)"
                         % (returned_value, value))

    def get_parameter(self, name, retry=1, timeout=60):
        """Get parameters from vehicle."""
        for i in range(0, retry):
            self.mavproxy.send("param fetch %s\n" % name)
            try:
                self.mavproxy.expect("%s = ([-0-9.]*)\r\n" % (name,), timeout=timeout/retry)
                try:
                    # sometimes race conditions garble the MAVProxy output
                    ret = float(self.mavproxy.match.group(1))
                except ValueError as e:
                    continue
                return ret
            except pexpect.TIMEOUT:
                pass
        raise NotAchievedException("Failed to retrieve parameter (%s)" % name)

    def context_get(self):
        """Get Saved parameters."""
        return self.contexts[-1]

    def context_push(self):
        """Save a copy of the parameters."""
        self.contexts.append(Context())

    def context_pop(self):
        """Set parameters to origin values in reverse order."""
        dead = self.contexts.pop()

        dead_parameters = dead.parameters
        dead_parameters.reverse()
        for p in dead_parameters:
            (name, old_value) = p
            self.set_parameter(name,
                               old_value,
                               add_to_context=False)

    def sysid_thismav(self):
        return 1

    def run_cmd_int(self,
                    command,
                    p1,
                    p2,
                    p3,
                    p4,
                    x,
                    y,
                    z,
                    want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    timeout=10,
                    target_sysid=None,
                    target_compid=None,
                    frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT):

        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        if target_compid is None:
            target_compid = 1

        """Send a MAVLink command int."""
        self.mav.mav.command_int_send(target_sysid,
                                      target_compid,
                                      frame,
                                      command,
                                      0, # current
                                      0, # autocontinue
                                      p1,
                                      p2,
                                      p3,
                                      p4,
                                      x,
                                      y,
                                      z)
        self.run_cmd_get_ack(command, want_result, timeout)

    def send_cmd(self,
                 command,
                 p1,
                 p2,
                 p3,
                 p4,
                 p5,
                 p6,
                 p7,
                 target_sysid=None,
                 target_compid=None,
                 ):
        """Send a MAVLink command long."""
        if target_sysid is None:
            target_sysid = self.sysid_thismav()
        if target_compid is None:
            target_compid = 1
        try:
            command_name = mavutil.mavlink.enums["MAV_CMD"][command].name
        except KeyError as e:
            command_name = "UNKNOWN=%u" % command
        self.progress("Sending COMMAND_LONG to (%u,%u) (%s) (p1=%f p2=%f p3=%f p4=%f p5=%f p6=%f  p7=%f)" %
                      (
                          target_sysid,
                          target_compid,
                          command_name,
                          p1,
                          p2,
                          p3,
                          p4,
                          p5,
                          p6,
                          p7))
        self.mav.mav.command_long_send(target_sysid,
                                       target_compid,
                                       command,
                                       1,  # confirmation
                                       p1,
                                       p2,
                                       p3,
                                       p4,
                                       p5,
                                       p6,
                                       p7)

    def run_cmd(self,
                command,
                p1,
                p2,
                p3,
                p4,
                p5,
                p6,
                p7,
                want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                target_sysid=None,
                target_compid=None,
                timeout=10,
                quiet=False):
        self.send_cmd(command,
                      p1,
                      p2,
                      p3,
                      p4,
                      p5,
                      p6,
                      p7,
                      target_sysid=target_sysid,
                      target_compid=target_compid,
        )
        self.run_cmd_get_ack(command, want_result, timeout, quiet=quiet)

    def run_cmd_get_ack(self, command, want_result, timeout, quiet=False):
        tstart = self.get_sim_time_cached()
        while True:
            delta_time = self.get_sim_time_cached() - tstart
            if delta_time > timeout:
                raise AutoTestTimeoutException("Did not get good COMMAND_ACK within %fs" % timeout)
            m = self.mav.recv_match(type='COMMAND_ACK',
                                    blocking=True,
                                    timeout=0.1)
            if m is None:
                continue
            if not quiet:
                self.progress("ACK received: %s (%fs)" % (str(m), delta_time))
            if m.command == command:
                if m.result != want_result:
                    raise ValueError("Expected %s got %s" % (want_result,
                                                             m.result))
                break

    def verify_parameter_values(self, parameter_stuff):
        bad = ""
        for param in parameter_stuff:
            fetched_value = self.get_parameter(param)
            wanted_value = parameter_stuff[param]
            max_delta = 0.0
            if type(wanted_value) == tuple:
                max_delta = wanted_value[1]
                wanted_value = wanted_value[0]
            if abs(fetched_value - wanted_value) > max_delta:
                bad += "%s=%f (want=%f +/-%f)    " % (param, fetched_value, wanted_value, max_delta)
        if len(bad):
            raise NotAchievedException("Bad parameter values: %s" %
                                       (bad,))

    #################################################
    # UTILITIES
    #################################################
    @staticmethod
    def longitude_scale(lat):
        ret = math.cos(lat * (math.radians(1)));
        print("scale=%f" % ret)
        return ret

    @staticmethod
    def get_distance(loc1, loc2):
        """Get ground distance between two locations."""
        return AutoTest.get_distance_accurate(loc1, loc2)
        # dlat = loc2.lat - loc1.lat
        # try:
        #     dlong = loc2.lng - loc1.lng
        # except AttributeError:
        #     dlong = loc2.lon - loc1.lon

        # return math.sqrt((dlat*dlat) + (dlong*dlong)*AutoTest.longitude_scale(loc2.lat)) * 1.113195e5

    @staticmethod
    def get_distance_accurate(loc1, loc2):
        """Get ground distance between two locations."""
        try:
            lon1 = loc1.lng
            lon2 = loc2.lng
        except AttributeError:
            lon1 = loc1.lon
            lon2 = loc2.lon

        return mp_util.gps_distance(loc1.lat, lon1, loc2.lat, lon2)

    @staticmethod
    def get_latlon_attr(loc, attrs):
        '''return any found latitude attribute from loc'''
        ret = None
        for attr in attrs:
            if hasattr(loc, attr):
                ret = getattr(loc, attr)
                break
        if ret is None:
            raise ValueError("None of %s in loc(%s)" % (str(attrs), str(loc)))
        return ret

    @staticmethod
    def get_lat_attr(loc):
        '''return any found latitude attribute from loc'''
        return AutoTest.get_latlon_attr(loc, ["lat", "latitude"])

    @staticmethod
    def get_lon_attr(loc):
        '''return any found latitude attribute from loc'''
        return AutoTest.get_latlon_attr(loc, ["lng", "lon", "longitude"])

    @staticmethod
    def get_distance_int(loc1, loc2):
        """Get ground distance between two locations in the normal "int" form
        - lat/lon multiplied by 1e7"""
        loc1_lat = AutoTest.get_lat_attr(loc1)
        loc2_lat = AutoTest.get_lat_attr(loc2)
        loc1_lon = AutoTest.get_lon_attr(loc1)
        loc2_lon = AutoTest.get_lon_attr(loc2)

        return AutoTest.get_distance_accurate(
            mavutil.location(loc1_lat*1e-7, loc1_lon*1e-7),
            mavutil.location(loc2_lat*1e-7, loc2_lon*1e-7))

        # dlat = loc2_lat - loc1_lat
        # dlong = loc2_lon - loc1_lon
        #
        # dlat /= 10000000.0
        # dlong /= 10000000.0
        #
        # return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    @staticmethod
    def get_bearing(loc1, loc2):
        """Get bearing from loc1 to loc2."""
        off_x = loc2.lng - loc1.lng
        off_y = loc2.lat - loc1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def change_mode(self, mode, timeout=60):
        '''change vehicle flightmode'''
        self.progress("Changing mode to %s" % mode)
        self.mavproxy.send('mode %s\n' % mode)
        tstart = self.get_sim_time()
        while not self.mode_is(mode):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                    self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    self.get_sim_time_cached() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
            self.mavproxy.send('mode %s\n' % mode)
        self.progress("Got mode %s" % mode)

    def capable(self, capability):
        return self.get_autopilot_capabilities() & capability

    def assert_capability(self, capability):
        if not self.capable(capability):
            name = mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"][capability].name
            raise NotAchievedException("AutoPilot does not have capbility %s" % (name,))

    def assert_no_capability(self, capability):
        if self.capable(capability):
            name = mavutil.mavlink.enums["MAV_PROTOCOL_CAPABILITY"][capability].name
            raise NotAchievedException("AutoPilot has feature %s (when it shouln't)" % (name,))

    def get_autopilot_capabilities(self):
        # Cannot use run_cmd otherwise the respond is lost during the wait for ACK
        self.mav.mav.command_long_send(self.sysid_thismav(),
                                       1,
                                       mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                                       0,  # confirmation
                                       1,  # 1: Request autopilot version
                                       0,
                                       0,
                                       0,
                                       0,
                                       0,
                                       0)
        m = self.mav.recv_match(type='AUTOPILOT_VERSION',
                                blocking=True,
                                timeout=10)
        if m is None:
            raise NotAchievedException("Did not get AUTOPILOT_VERSION")
        return m.capabilities

    def test_get_autopilot_capabilities(self):
        self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT)
        self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION)

    def get_mode_from_mode_mapping(self, mode):
        """Validate and return the mode number from a string or int."""
        mode_map = self.mav.mode_mapping()
        if mode_map is None:
            mav_type = self.mav.messages['HEARTBEAT'].type
            mav_autopilot = self.mav.messages['HEARTBEAT'].autopilot
            raise ErrorException("No mode map for (mav_type=%s mav_autopilot=%s)" % (mav_type, mav_autopilot))
        if isinstance(mode, str):
            if mode in mode_map:
                return mode_map.get(mode)
        if mode in mode_map.values():
            return mode
        self.progress("Available modes '%s'" % mode_map)
        raise ErrorException("Unknown mode '%s'" % mode)

    def run_cmd_do_set_mode(self,
                            mode,
                            timeout=30,
                            want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED):
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     base_mode,
                     custom_mode,
                     0,
                     0,
                     0,
                     0,
                     0,
                     want_result=want_result,
                     timeout=timeout
        )

    def do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message."""
        tstart = self.get_sim_time()
        want_custom_mode = self.get_mode_from_mode_mapping(mode)
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise AutoTestTimeoutException("Failed to change mode")
            self.run_cmd_do_set_mode(mode, timeout=10)
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=5)
            if m is None:
                raise ErrorException("Heartbeat not received")
            self.progress("Got mode=%u want=%u" % (m.custom_mode, want_custom_mode))
            if m.custom_mode == want_custom_mode:
                return

    def mavproxy_do_set_mode_via_command_long(self, mode, timeout=30):
        """Set mode with a command long message with Mavproxy."""
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        custom_mode = self.get_mode_from_mode_mapping(mode)
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise AutoTestTimeoutException("Failed to change mode")
            self.mavproxy.send("long DO_SET_MODE %u %u\n" %
                               (base_mode, custom_mode))
            m = self.mav.recv_match(type='HEARTBEAT',
                                    blocking=True,
                                    timeout=5)
            if m is None:
                raise ErrorException("Did not receive heartbeat")
            if m.custom_mode == custom_mode:
                return True

    def reach_heading_manual(self, heading, turn_right=True):
        """Manually direct the vehicle to the target heading."""
        if self.is_copter():
            self.mavproxy.send('rc 4 1580\n')
            self.wait_heading(heading)
            self.set_rc(4, 1500)
        if self.is_plane():
            self.progress("NOT IMPLEMENTED")
        if self.is_rover():
            steering_pwm = 1700
            if not turn_right:
                steering_pwm = 1300
            self.mavproxy.send('rc 1 %u\n' % steering_pwm)
            self.mavproxy.send('rc 3 1550\n')
            self.wait_heading(heading)
            self.set_rc(3, 1500)
            self.set_rc(1, 1500)

    def assert_vehicle_location_is_at_startup_location(self, dist_max=1):
        here = self.mav.location()
        start_loc = self.sitl_start_location()
        dist = self.get_distance(here, start_loc)
        data = "dist=%f max=%f (here: %s start-loc: %s)" % (dist, dist_max, here, start_loc)

        if dist > dist_max:
            raise NotAchievedException("Far from startup location: %s" % data)
        self.progress("Close to startup location: %s" % data)

    def assert_simstate_location_is_at_startup_location(self, dist_max=1):
        simstate_loc = self.sim_location()
        start_loc = self.sitl_start_location()
        dist = self.get_distance(simstate_loc, start_loc)
        data = "dist=%f max=%f (simstate: %s start-loc: %s)" % (dist, dist_max, simstate_loc, start_loc)

        if dist > dist_max:
            raise NotAchievedException("simstate from startup location: %s" % data)
        self.progress("Simstate Close to startup location: %s" % data)

    def reach_distance_manual(self, distance):
        """Manually direct the vehicle to the target distance from home."""
        if self.is_copter():
            self.set_rc(2, 1350)
            self.wait_distance(distance, accuracy=5, timeout=60)
            self.set_rc(2, 1500)
        if self.is_plane():
            self.progress("NOT IMPLEMENTED")
        if self.is_rover():
            self.mavproxy.send('rc 3 1700\n')
            self.wait_distance(distance, accuracy=2)
            self.set_rc(3, 1500)

    def guided_achieve_heading(self, heading):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > 200:
                raise NotAchievedException("Did not achieve heading")
            self.run_cmd(mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                         heading,  # target angle
                         10,  # degrees/second
                         1,  # -1 is counter-clockwise, 1 clockwise
                         0,  # 1 for relative, 0 for absolute
                         0,  # p5
                         0,  # p6
                         0,  # p7
            )
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            self.progress("heading=%d want=%d" % (m.heading, int(heading)))
            if m.heading == int(heading):
                return

    def do_set_relay(self, relay_num, on_off, timeout=10):
        """Set relay with a command long message."""
        self.progress("Set relay %d to %d" % (relay_num, on_off))
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_SET_RELAY,
                     relay_num,
                     on_off,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)

    def do_set_relay_mavproxy(self, relay_num, on_off):
        """Set relay with mavproxy."""
        self.progress("Set relay %d to %d" % (relay_num, on_off))
        self.mavproxy.send('module load relay\n')
        self.mavproxy.expect("Loaded module relay")
        self.mavproxy.send("relay set %d %d\n" % (relay_num, on_off))
    #################################################
    # WAIT UTILITIES
    #################################################
    def delay_sim_time(self, seconds_to_wait):
        """Wait some second in SITL time."""
        tstart = self.get_sim_time()
        tnow = tstart
        self.progress("Delaying %f seconds" % (seconds_to_wait,))
        while tstart + seconds_to_wait > tnow:
            tnow = self.get_sim_time()

    def wait_altitude(self, altitude_min, altitude_max, relative=False, timeout=30, **kwargs):
        """Wait for a given altitude range."""
        assert altitude_min <= altitude_max, "Minimum altitude should be less than maximum altitude."

        def get_altitude(alt_relative=False, timeout2=30):
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout2)
            if msg:
                if alt_relative:
                    return msg.relative_alt / 1000.0  # mm -> m
                else:
                    return msg.alt / 1000.0  # mm -> m
            raise MsgRcvTimeoutException("Failed to get Global Position")

        def validator(value2, target2=None):
            if altitude_min <= value2 <= altitude_max:
                return True
            else:
                return False

        self.wait_and_maintain(value_name="Altitude", target=altitude_min, current_value_getter=lambda: get_altitude(relative, timeout), accuracy=(altitude_max - altitude_min), validator=lambda value2, target2: validator(value2, target2), timeout=timeout, **kwargs)

    def wait_groundspeed(self, speed_min, speed_max, timeout=30, **kwargs):
        """Wait for a given ground speed range."""
        assert speed_min <= speed_max, "Minimum speed should be less than maximum speed."

        def get_groundspeed(timeout2):
            msg = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=timeout2)
            if msg:
                return msg.groundspeed
            raise MsgRcvTimeoutException("Failed to get Groundspeed")

        def validator(value2, target2=None):
            if speed_min <= value2 <= speed_max:
                return True
            else:
                return False

        self.wait_and_maintain(value_name="Groundspeed", target=speed_min, current_value_getter=lambda: get_groundspeed(timeout), accuracy=(speed_max - speed_min), validator=lambda value2, target2: validator(value2, target2), timeout=timeout, **kwargs)

    def wait_roll(self, roll, accuracy, timeout=30, **kwargs):
        """Wait for a given roll in degrees."""
        def get_roll(timeout2):
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=timeout2)
            if msg:
                p = math.degrees(msg.pitch)
                r = math.degrees(msg.roll)
                self.progress("Roll %d Pitch %d" % (r, p))
                return r
            raise MsgRcvTimeoutException("Failed to get Roll")

        def validator(value2, target2):
            return math.fabs((value2 - target2 + 180) % 360 - 180) <= accuracy

        self.wait_and_maintain(value_name="Roll", target=roll, current_value_getter=lambda: get_roll(timeout), validator=lambda value2, target2: validator(value2, target2), accuracy=accuracy, timeout=timeout, **kwargs)

    def wait_pitch(self, pitch, accuracy, timeout=30, **kwargs):
        """Wait for a given pitch in degrees."""
        def get_pitch(timeout2):
            msg = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=timeout2)
            if msg:
                p = math.degrees(msg.pitch)
                r = math.degrees(msg.roll)
                self.progress("Pitch %d Roll %d" % (p, r))
                return p
            raise MsgRcvTimeoutException("Failed to get Pitch")

        def validator(value2, target2):
            return math.fabs((value2 - target2 + 180) % 360 - 180) <= accuracy

        self.wait_and_maintain(value_name="Pitch", target=pitch, current_value_getter=lambda: get_pitch(timeout), validator=lambda value2, target2: validator(value2, target2), accuracy=accuracy, timeout=timeout, **kwargs)

    def wait_and_maintain(self, value_name, target, current_value_getter, validator=None, accuracy=2.0, timeout=30, **kwargs):
        tstart = self.get_sim_time()
        achieving_duration_start = None
        sum_of_achieved_values = 0.0
        last_value = 0.0
        count_of_achieved_values = 0
        called_function = kwargs.get("called_function", None)
        minimum_duration = kwargs.get("minimum_duration", 0)
        self.progress("Waiting for %s %.02f with accuracy %.02f" % (value_name, target, accuracy))
        last_print_time = 0
        while self.get_sim_time_cached() < tstart + timeout:  # if we failed to received message with the getter the sim time isn't updated
            last_value = current_value_getter()
            if called_function is not None:
                called_function(last_value, target)
            if self.get_sim_time_cached() - last_print_time > 1:
                self.progress("%s=%0.2f (want %f +- %f)" %
                              (value_name, last_value, target, accuracy))
                last_print_time = self.get_sim_time_cached()
            if validator is not None:
                is_value_valid = validator(last_value, target)
            else:
                is_value_valid = math.fabs(last_value - target) <= accuracy
            if is_value_valid:
                sum_of_achieved_values += last_value
                count_of_achieved_values += 1.0
                if achieving_duration_start is None:
                    achieving_duration_start = self.get_sim_time_cached()
                if self.get_sim_time_cached() - achieving_duration_start >= minimum_duration:
                    self.progress("Attained %s=%f" % (value_name, sum_of_achieved_values / count_of_achieved_values))
                    return True
            else:
                achieving_duration_start = None
                sum_of_achieved_values = 0.0
                count_of_achieved_values = 0
        raise AutoTestTimeoutException("Failed to attain %s %f, reach %f" % (value_name, target, (sum_of_achieved_values / count_of_achieved_values) if count_of_achieved_values != 0 else last_value))

    def wait_heading(self, heading, accuracy=5, timeout=30, **kwargs):
        """Wait for a given heading."""
        def get_heading_wrapped(timeout2):
            msg = self.mav.recv_match(type='VFR_HUD', blocking=True, timeout=timeout2)
            if msg:
                return msg.heading
            raise MsgRcvTimeoutException("Failed to get heading")

        def validator(value2, target2):
            return math.fabs((value2 - target2 + 180) % 360 - 180) <= accuracy

        self.wait_and_maintain(value_name="Heading", target=heading, current_value_getter=lambda: get_heading_wrapped(timeout), validator=lambda value2, target2: validator(value2, target2), accuracy=accuracy, timeout=timeout, **kwargs)

    def wait_distance(self, distance, accuracy=2, timeout=30, **kwargs):
        """Wait for flight of a given distance."""
        start = self.mav.location()

        def get_distance():
            return self.get_distance(start, self.mav.location())

        def validator(value2, target2):
            return (value2 - target2) >= accuracy

        self.wait_and_maintain(value_name="Distance", target=distance, current_value_getter=lambda: get_distance(), validator=lambda value2, target2: validator(value2, target2), accuracy=accuracy, timeout=timeout, **kwargs)

    def wait_distance_to_location(self, location, distance_min, distance_max, timeout=30, **kwargs):
        """Wait for flight of a given distance."""
        assert distance_min <= distance_max, "Distance min should be less than distance max."

        def get_distance():
            return self.get_distance(location, self.mav.location())

        def validator(value2, target2=None):
            return distance_min <= value2 <= distance_max

        self.wait_and_maintain(value_name="Distance", target=distance_min, current_value_getter=lambda: get_distance(), validator=lambda value2, target2: validator(value2, target2), accuracy=(distance_max - distance_min), timeout=timeout, **kwargs)

    def wait_servo_channel_value(self, channel, value, timeout=2, comparator=operator.eq):
        """wait for channel value comparison (default condition is equality)"""
        channel_field = "servo%u_raw" % channel
        opstring = ("%s" % comparator)[-3:-1]
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel value condition not met")
            m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field, None)
            self.progress("want SERVO_OUTPUT_RAW.%s=%u %s %u" %
                          (channel_field, m_value, opstring, value))
            if m_value is None:
                raise ValueError("message (%s) has no field %s" %
                                 (str(m), channel_field))
            if comparator(m_value, value):
                return

    def wait_rc_channel_value(self, channel, value, timeout=2):
        """wait for channel to hit value"""
        channel_field = "chan%u_raw" % channel
        tstart = self.get_sim_time()
        while True:
            remaining = timeout - (self.get_sim_time_cached() - tstart)
            if remaining <= 0:
                raise NotAchievedException("Channel never achieved value")
            m = self.mav.recv_match(type='RC_CHANNELS',
                                    blocking=True,
                                    timeout=remaining)
            if m is None:
                continue
            m_value = getattr(m, channel_field)
            self.progress("RC_CHANNELS.%s=%u want=%u" %
                          (channel_field, m_value, value))
            if m_value is None:
                raise ValueError("message (%s) has no field %s" %
                                 (str(m), channel_field))
            if m_value == value:
                return

    def wait_location(self,
                      loc,
                      accuracy=5.0,
                      timeout=30,
                      target_altitude=None,
                      height_accuracy=-1,
                      **kwargs):
        """Wait for arrival at a location."""
        def get_distance_to_loc():
            return self.get_distance(self.mav.location(), loc)

        def validator(value2, empty=None):
            if value2 <= accuracy:
                if target_altitude is not None:
                    height_delta = math.fabs(self.mav.location().alt - target_altitude)
                    if height_accuracy != -1 and height_delta > height_accuracy:
                        return False
                return True
            else:
                return False
        debug_text = "Distance to Location (%.4f, %.4f) " % (loc.lat, loc.lng)
        if target_altitude is not None:
            debug_text += "at altitude %.1f height_accuracy=%.1f" % (target_altitude, height_accuracy)
        self.wait_and_maintain(value_name=debug_text, target=0, current_value_getter=lambda: get_distance_to_loc(), accuracy=accuracy, validator=lambda value2, target2: validator(value2, None), timeout=timeout, **kwargs)

    def wait_current_waypoint(self, wpnum, timeout=60):
        tstart = self.get_sim_time()
        while self.get_sim_time() < tstart + timeout:
            seq = self.mav.waypoint_current()
            self.progress("Waiting for wp=%u current=%u" % (wpnum, seq))
            if seq == wpnum:
                break

    def wait_waypoint(self,
                      wpnum_start,
                      wpnum_end,
                      allow_skip=True,
                      max_dist=2,
                      timeout=400):
        """Wait for waypoint ranges."""
        tstart = self.get_sim_time()
        # this message arrives after we set the current WP
        start_wp = self.mav.waypoint_current()
        current_wp = start_wp
        mode = self.mav.flightmode

        self.progress("wait for waypoint ranges start=%u end=%u"
                      % (wpnum_start, wpnum_end))
        # if start_wp != wpnum_start:
        #    raise WaitWaypointTimeout("test: Expected start waypoint %u "
        #                              "but got %u" %
        #                  (wpnum_start, start_wp))

        last_wp_msg = 0
        while self.get_sim_time_cached() < tstart + timeout:
            seq = self.mav.waypoint_current()
            m = self.mav.recv_match(type='NAV_CONTROLLER_OUTPUT',
                                    blocking=True)
            wp_dist = m.wp_dist
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)

            # if we changed mode, fail
            if self.mav.flightmode != mode:
                raise WaitWaypointTimeout('Exited %s mode' % mode)

            if self.get_sim_time_cached() - last_wp_msg > 1:
                self.progress("WP %u (wp_dist=%u Alt=%.02f), current_wp: %u,"
                              "wpnum_end: %u" %
                              (seq, wp_dist, m.alt, current_wp, wpnum_end))
                last_wp_msg = self.get_sim_time_cached()
            if seq == current_wp+1 or (seq > current_wp+1 and allow_skip):
                self.progress("test: Starting new waypoint %u" % seq)
                tstart = self.get_sim_time()
                current_wp = seq
                # the wp_dist check is a hack until we can sort out
                # the right seqnum for end of mission
            # if current_wp == wpnum_end or (current_wp == wpnum_end-1 and
            #                                wp_dist < 2):
            if current_wp == wpnum_end and wp_dist < max_dist:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq >= 255:
                self.progress("Reached final waypoint %u" % seq)
                return True
            if seq > current_wp+1:
                raise WaitWaypointTimeout(("Skipped waypoint! Got wp %u expected %u"
                                           % (seq, current_wp+1)))
        raise WaitWaypointTimeout("Timed out waiting for waypoint %u of %u" %
                                  (wpnum_end, wpnum_end))

    def mode_is(self, mode, cached=False, drain_mav=True):
        if not cached:
            self.wait_heartbeat(drain_mav=drain_mav)
        try:
            return self.get_mode_from_mode_mapping(self.mav.flightmode) == self.get_mode_from_mode_mapping(mode)
        except Exception as e:
            pass
        # assume this is a number....
        return self.mav.messages['HEARTBEAT'].custom_mode == mode

    def wait_mode(self, mode, timeout=60):
        """Wait for mode to change."""
        self.progress("Waiting for mode %s" % mode)
        tstart = self.get_sim_time()
        while not self.mode_is(mode, drain_mav=False):
            custom_num = self.mav.messages['HEARTBEAT'].custom_mode
            self.progress("mav.flightmode=%s Want=%s custom=%u" % (
                    self.mav.flightmode, mode, custom_num))
            if (timeout is not None and
                    self.get_sim_time_cached() > tstart + timeout):
                raise WaitModeTimeout("Did not change mode")
        self.progress("Got mode %s" % mode)

    def wait_gps_sys_status_not_present_or_enabled_and_healthy(self, timeout=30):
        self.progress("Waiting for GPS health")
        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise AutoTestTimeoutException("GPS status bits did not become good")
            m = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
            if m is None:
                continue
            if (not (m.onboard_control_sensors_present & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not present")
                return
            if (not (m.onboard_control_sensors_enabled & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not enabled")
                continue
            if (not (m.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)):
                self.progress("GPS not healthy")
                continue
            self.progress("GPS healthy")
            return

    def wait_ready_to_arm(self, timeout=None, require_absolute=True):
        # wait for EKF checks to pass
        self.progress("Waiting for ready to arm")
        start = self.get_sim_time()
        self.wait_ekf_happy(timeout=timeout, require_absolute=require_absolute)
        if require_absolute:
            self.wait_gps_sys_status_not_present_or_enabled_and_healthy()
        armable_time = (self.get_sim_time()-start)
        self.progress("Took %u seconds to become armable" % armable_time)
        self.total_waiting_to_arm_time += armable_time
        self.waiting_to_arm_count += 1

    def wait_heartbeat(self, drain_mav=True, *args, **x):
        '''as opposed to mav.wait_heartbeat, raises an exception on timeout'''
        if drain_mav:
            self.drain_mav()
        m = self.mav.wait_heartbeat(*args, **x)
        if m is None:
            raise AutoTestTimeoutException("Did not receive heartbeat")

    def wait_ekf_happy(self, timeout=30, require_absolute=True):
        """Wait for EKF to be happy"""

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            return True

        # all of these must be set for arming to happen:
        required_value = (mavutil.mavlink.EKF_ATTITUDE |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ |
                          mavutil.mavlink.ESTIMATOR_VELOCITY_VERT |
                          mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL |
                          mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_REL)
        # none of these bits must be set for arming to happen:
        error_bits = (mavutil.mavlink.ESTIMATOR_CONST_POS_MODE |
                      mavutil.mavlink.ESTIMATOR_ACCEL_ERROR)
        if require_absolute:
            required_value |= (mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS |
                               mavutil.mavlink.ESTIMATOR_POS_VERT_ABS |
                               mavutil.mavlink.ESTIMATOR_PRED_POS_HORIZ_ABS)
            error_bits |= mavutil.mavlink.ESTIMATOR_GPS_GLITCH
        self.wait_ekf_flags(required_value, error_bits, timeout=timeout)

    def wait_ekf_flags(self, required_value, error_bits, timeout=30):
        self.progress("Waiting for EKF value %u" % required_value)
        last_print_time = 0
        tstart = self.get_sim_time()
        while timeout is None or self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            errors = current & error_bits
            everything_ok = (errors == 0 and
                             current & required_value == required_value)
            if everything_ok or self.get_sim_time_cached() - last_print_time > 1:
                self.progress("Wait EKF.flags: required:%u current:%u errors=%u" %
                              (required_value, current, errors))
                last_print_time = self.get_sim_time_cached()
            if everything_ok:
                self.progress("EKF Flags OK")
                return True
        raise AutoTestTimeoutException("Failed to get EKF.flags=%u" %
                                       required_value)

    def wait_gps_disable(self, timeout=30):
        """Disable GPS and wait for EKF to report the end of assistance from GPS."""
        self.set_parameter("SIM_GPS_DISABLE", 1)
        tstart = self.get_sim_time()

        """ if using SITL estimates directly """
        if (int(self.get_parameter('AHRS_EKF_TYPE')) == 10):
            self.progress("GPS disable skipped")
            return 

        # all of these must NOT be set for arming NOT to happen:
        not_required_value = mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL
        self.progress("Waiting for EKF not having bits %u" % not_required_value)
        last_print_time = 0
        while timeout is None or self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=timeout)
            if m is None:
                continue
            current = m.flags
            if self.get_sim_time_cached() - last_print_time > 1:
                self.progress("Wait EKF.flags: not required:%u current:%u" %
                              (not_required_value, current))
                last_print_time = self.get_sim_time_cached()
            if current & not_required_value != not_required_value:
                self.progress("GPS disable OK")
                return
        raise AutoTestTimeoutException("Failed to get EKF.flags=%u disabled" % not_required_value)

    def wait_text(self, *args, **kwargs):
        self.wait_statustext(*args, **kwargs)

    def wait_statustext(self, text, timeout=20, the_function=None):
        """Wait a specific STATUS_TEXT."""
        self.progress("Waiting for text : %s" % text.lower())
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() < tstart + timeout:
            if the_function is not None:
                the_function()
            m = self.mav.recv_match(type='STATUSTEXT', blocking=True, timeout=0.1)
            if m is None:
                continue
            if text.lower() in m.text.lower():
                self.progress("Received expected text : %s" % m.text.lower())
                return True
        raise AutoTestTimeoutException("Failed to received text : %s" %
                                       text.lower())

    def get_mavlink_connection_going(self):
        # get a mavlink connection going
        connection_string = self.autotest_connection_string_to_mavproxy()
        try:
            self.mav = mavutil.mavlink_connection(connection_string,
                                                  robust_parsing=True,
                                                  source_component=250)
        except Exception as msg:
            self.progress("Failed to start mavlink connection on %s: %s" %
                          (connection_string, msg,))
            raise
        self.mav.message_hooks.append(self.message_hook)
        self.mav.idle_hooks.append(self.idle_hook)

    def show_test_timings_key_sorter(self, t):
        (k, v) = t
        return ((v, k))

    def show_test_timings(self):
        longest = 0
        for desc in self.test_timings.keys():
            if len(desc) > longest:
                longest = len(desc)
        for desc, test_time in sorted(self.test_timings.items(),
                                      key=self.show_test_timings_key_sorter):
            fmt = "%" + str(longest) + "s: %.2fs"
            self.progress(fmt % (desc, test_time))

    def send_statustext(self, text):
        if sys.version_info.major >= 3 and not isinstance(text, bytes):
            text = bytes(text, "ascii")
        self.mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, text)

    def get_exception_stacktrace(self, e):
        if sys.version_info[0] >= 3:
            ret = "%s\n" % e
            ret += ''.join(traceback.format_exception(etype=type(e),
                                                      value=e,
                                                      tb=e.__traceback__))
            return ret
        return traceback.format_exc(e)

    def run_one_test(self, name, desc, test_function, interact=False):
        '''new-style run-one-test used by run_tests'''
        test_output_filename = self.buildlogs_path("%s-%s.txt" %
                                                   (self.log_name(), name))
        tee = TeeBoth(test_output_filename, 'w', self.mavproxy_logfile)

        prettyname = "%s (%s)" % (name, desc)
        self.send_statustext(prettyname)
        self.start_test(prettyname)
        self.set_current_test_name(name)
        self.context_push()

        start_time = time.time()

        ex = None
        try:
            self.check_rc_defaults()
            self.change_mode(self.default_mode())
            self.drain_mav()
            self.drain_all_pexpects()

            test_function()
        except Exception as e:
            self.progress("Exception caught: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.test_timings[desc] = time.time() - start_time
        if self.contexts[-1].sitl_commandline_customised:
            self.reset_SITL_commandline()
        self.context_pop()

        passed = True
        if ex is not None:
            passed = False

        self.wait_heartbeat()
        if self.armed() and not self.is_tracker():
            if ex is None:
                ex = ArmedAtEndOfTestException("Still armed at end of test")
            self.progress("Armed at end of test; force-rebooting SITL")
            self.disarm_vehicle(force=True)
            self.forced_post_test_sitl_reboots += 1
            self.progress("Force-resetting SITL")
            self.reboot_sitl() # that'll learn it
            passed = False

        if passed:
            self.progress('PASSED: "%s"' % prettyname)
        else:
            self.progress('FAILED: "%s": %s (see %s)' %
                          (prettyname, repr(ex), test_output_filename))
            self.fail_list.append((prettyname, ex, test_output_filename))
            if interact:
                self.progress("Starting MAVProxy interaction as directed")
                self.mavproxy.interact()

        self.clear_mission_using_mavproxy()

        tee.close()

    def check_test_syntax(self, test_file):
        """Check mistake on autotest function syntax."""
        self.start_test("Check for syntax mistake in autotest lambda")
        if not os.path.isfile(test_file):
            self.progress("File %s does not exist" % test_file)
        test_file = test_file.rstrip('c')
        try:
            with open(test_file) as f:
                # check for lambda: test_function without paranthesis
                faulty_strings = re.findall(r"lambda\s*:\s*\w+.\w+\s*\)", f.read())
                if faulty_strings:
                    desc = "Syntax error in autotest lambda at : \n"
                    for x in range(len(faulty_strings)):
                        desc += faulty_strings[x] + "\n"
                    raise ErrorException(desc)
        except ErrorException as msg:
            self.progress("FAILED: Check for syntax mistake in autotest lambda. \n" + str(msg))
            exit(1)
        self.progress("PASSED: Check for syntax mistake in autotest lambda")

    def uses_vicon(self):
        return False

    def defaults_filepath(self):
        return None

    def start_mavproxy(self):
        self.progress("Starting MAVProxy")
        self.mavproxy = util.start_MAVProxy_SITL(
            self.vehicleinfo_key(),
            logfile=self.mavproxy_logfile,
            options=self.mavproxy_options())
        self.mavproxy.expect('Telemetry log: (\S+)\r\n')
        self.logfile = self.mavproxy.match.group(1)
        self.progress("LOGFILE %s" % self.logfile)
        self.try_symlink_tlog()

        self.progress("Waiting for Parameters")
        self.mavproxy.expect('Received [0-9]+ parameters')

    def start_SITL(self, **sitl_args):
        start_sitl_args = {
            "breakpoints": self.breakpoints,
            "disable_breakpoints": self.disable_breakpoints,
            "gdb": self.gdb,
            "gdbserver": self.gdbserver,
            "lldb": self.lldb,
            "home": self.sitl_home(),
            "speedup": self.speedup,
            "valgrind": self.valgrind,
            "vicon": self.uses_vicon(),
            "wipe": True,
        }
        start_sitl_args.update(**sitl_args)
        if ("defaults_filepath" not in start_sitl_args or
            start_sitl_args["defaults_filepath"] is None):
            start_sitl_args["defaults_filepath"] = self.defaults_filepath()

        if "model" not in start_sitl_args or start_sitl_args["model"] is None:
            start_sitl_args["model"] = self.frame
        self.progress("Starting SITL")
        self.sitl = util.start_SITL(self.binary, **start_sitl_args)

    def sitl_is_running(self):
        return self.sitl.is_alive()

    def init(self):
        """Initilialize autotest feature."""
        self.check_test_syntax(test_file=self.test_filepath())

        self.mavproxy_logfile = self.open_mavproxy_logfile()

        if self.frame is None:
            self.frame = self.default_frame()

        if self.frame is None:
            raise ValueError("frame must not be None")

        self.progress("Starting simulator")
        self.start_SITL()

        self.start_mavproxy()

        self.progress("Starting MAVLink connection")
        self.get_mavlink_connection_going()

        self.apply_defaultfile_parameters()

        util.expect_setup_callback(self.mavproxy, self.expect_callback)

        self.expect_list_clear()
        self.expect_list_extend([self.sitl, self.mavproxy])

        self.progress("Ready to start testing!")

    def upload_using_mission_protocol(self, mission_type, items):
        '''mavlink2 required'''
        target_system = 1
        target_component = 1
        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        len(items),
                                        mission_type)
        tstart = self.get_sim_time_cached()
        remaining_to_send = set(range(0, len(items)))
        sent = set()
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("timeout uploading %s" % str(mission_type))
            if len(remaining_to_send) == 0:
                self.progress("All sent")
                break
            m = self.mav.recv_match(type=['MISSION_REQUEST', 'MISSION_ACK'],
                                    blocking=True,
                                    timeout=1)
            if m is None:
                continue

            if m.get_type() == 'MISSION_ACK':
                if (m.target_system == 255 and
                    m.target_component == 0 and
                    m.type == 1 and
                    m.mission_type == 0):
                    # this is just MAVProxy trying to screw us up
                    continue
                else:
                    raise NotAchievedException("Received unexpected mission ack %s" % str(m))

            self.progress("Handling request for item %u/%u" % (m.seq, len(items)-1))
            self.progress("Item (%s)" % str(items[m.seq]))
            if m.seq in sent:
                self.progress("received duplicate request for item %u" % m.seq)
                continue

            if m.seq not in remaining_to_send:
                raise NotAchievedException("received request for unknown item %u" % m.seq)

            if m.mission_type != mission_type:
                raise NotAchievedException("received request for item from wrong mission type")

            if items[m.seq].mission_type != mission_type:
                raise NotAchievedException("supplied item not of correct mission type")
            if items[m.seq].target_system != target_system:
                raise NotAchievedException("supplied item not of correct target system")
            if items[m.seq].target_component != target_component:
                raise NotAchievedException("supplied item not of correct target component")
            if items[m.seq].seq != m.seq:
                raise NotAchievedException("supplied item has incorrect sequence number (%u vs %u)" % (items[m.seq].seq, m.seq))

            items[m.seq].pack(self.mav.mav)
            self.mav.mav.send(items[m.seq])
            remaining_to_send.discard(m.seq)
            sent.add(m.seq)
        m = self.mav.recv_match(type='MISSION_ACK',
                                blocking=True,
                                timeout=1)
        if m is None:
            raise NotAchievedException("Did not receive MISSION_ACK")
        if m.mission_type != mission_type:
            raise NotAchievedException("Mission ack not of expected mission type")
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Mission upload failed (%s)" %
                                       (mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name),)
        self.progress("Upload of all %u items succeeded" % len(items))

    def download_using_mission_protocol(self, mission_type, verbose=False, timeout=10):
        '''mavlink2 required'''
        target_system = 1
        target_component = 1
        self.drain_mav_unparsed()
        self.progress("Sending mission_request_list")
        self.mav.mav.mission_request_list_send(target_system,
                                               target_component,
                                               mission_type)

        tstart = self.get_sim_time_cached()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Did not get MISSION_COUNT packet")
            m = self.mav.recv_match(blocking=True, timeout=0.1)
            if verbose:
                self.progress(str(m))
            if m.get_type() == 'MISSION_ACK':
                if m.target_system == 255 and m.target_component == 0:
                    # this was for MAVProxy
                    continue
                self.progress("Mission ACK: %s" % str(m))
                raise NotAchievedException("Received MISSION_ACK while waiting for MISSION_COUNT")
            if m.get_type() != 'MISSION_COUNT':
                continue
            if m is None:
                raise NotAchievedException("Did not get MISSION_COUNT response")
            if m.target_component != 250: # FIXME: constant?!
                continue
            if m.mission_type != mission_type:
                raise NotAchievedException("Mission count response of incorrect type")
            break

        items = []
        tstart = self.get_sim_time_cached()
        remaining_to_receive = set(range(0, m.count))
        next_to_request = 0
        while True:
            if self.get_sim_time_cached() - tstart > 10:
                raise NotAchievedException("timeout downloading type=%s" %
                                           (mavutil.mavlink.enums["MAV_MISSION_TYPE"][mission_type].name))
            if len(remaining_to_receive) == 0:
                self.progress("All received")
                return items
            self.progress("Requesting item %u" % next_to_request)
            self.mav.mav.mission_request_int_send(target_system,
                                                  target_component,
                                                  next_to_request,
                                                  mission_type)
            m = self.mav.recv_match(type='MISSION_ITEM_INT',
                                    blocking=True,
                                    timeout=5,
                                    condition='MISSION_ITEM_INT.mission_type==%u' % mission_type)
            self.progress("Got (%s)" % str(m))
            if m is None:
                raise NotAchievedException("Did not receive MISSION_ITEM_INT")
            if m.mission_type != mission_type:
                raise NotAchievedException("Received waypoint of wrong type")
            if m.seq != next_to_request:
                raise NotAchievedException("Received waypoint is out of sequence")
            self.progress("Item %u OK" % m.seq)
            items.append(m)
            next_to_request += 1
            remaining_to_receive.discard(m.seq)

    def poll_home_position(self, quiet=False, timeout=30):
        old = self.mav.messages.get("HOME_POSITION", None)
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                raise NotAchievedException("Failed to poll home position")
            if not quiet:
                self.progress("Sending MAV_CMD_GET_HOME_POSITION")
            try:
                self.run_cmd(
                    mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    quiet=quiet)
            except ValueError as e:
                continue
            m = self.mav.messages.get("HOME_POSITION", None)
            if m is None:
                continue
            if old is None:
                break
            if m._timestamp != old._timestamp:
                break
        return m

    def distance_to_home(self, use_cached_home=False):
        m = self.mav.messages.get("HOME_POSITION", None)
        if use_cached_home is False or m is None:
            m = self.poll_home_position(quiet=True)
        here = self.mav.recv_match(type='GLOBAL_POSITION_INT',
                                   blocking=True)
        return self.get_distance_int(m, here)

    def home_position_as_mav_location(self):
        m = self.poll_home_position()
        return mavutil.location(m.latitude*1.0e-7, m.longitude*1.0e-7, m.altitude*1.0e-3, 0)

    def offset_location_ne(self, location, metres_north, metres_east):
        '''return a new location offset from passed-in location'''
        (target_lat, target_lng) = mavextra.gps_offset(location.lat,
                                                       location.lng,
                                                       metres_east,
                                                       metres_north)
        return mavutil.location(target_lat,
                                target_lng,
                                location.alt,
                                location.heading)

    def monitor_groundspeed(self, want, tolerance=0.5, timeout=5):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time_cached() - tstart > timeout:
                break
            m = self.mav.recv_match(type='VFR_HUD', blocking=True)
            if m.groundspeed > want+tolerance:
                raise NotAchievedException("Too fast (%f > %f)" %
                                           (m.groundspeed, want))
            if m.groundspeed < want-tolerance:
                raise NotAchievedException("Too slow (%f < %f)" %
                                           (m.groundspeed, want))
            self.progress("GroundSpeed OK (got=%f) (want=%f)" %
                          (m.groundspeed, want))

    def fly_test_set_home(self):
        if self.is_tracker():
            # tracker starts armed...
            self.disarm_vehicle(force=True)
        self.reboot_sitl()

        # HOME_POSITION is used as a surrogate for origin until we
        # start emitting GPS_GLOBAL_ORIGIN
        orig_home = self.poll_home_position()
        if orig_home is None:
            raise AutoTestTimeoutException()
        self.progress("Original home: %s" % str(orig_home))
        # original home should be close to SITL home...
        start_loc = self.sitl_start_location()
        self.progress("SITL start loc: %s" % str(start_loc))
        delta = abs(orig_home.latitude * 1.0e-7 - start_loc.lat)
        if delta > 0.0000001:
            raise ValueError("homes differ in lat got=%f vs want=%f delta=%f" %
                             (orig_home.latitude * 1.0e-7, start_loc.lat, delta))
        delta = abs(orig_home.longitude * 1.0e-7 - start_loc.lng)
        if delta > 0.0000001:
            raise ValueError("homes differ in lon  got=%f vs want=%f delta=%f" %
                             (orig_home.longitude * 1.0e-7, start_loc.lng, delta))
        if self.is_rover():
            self.progress("### Rover skipping altitude check unti position fixes in")
        else:
            home_alt_m = orig_home.altitude * 1.0e-3
            if abs(home_alt_m - start_loc.alt) > 2: # metres
                raise ValueError("homes differ in alt got=%fm want=%fm" %
                                 (home_alt_m, start_loc.alt))
        new_x = orig_home.latitude + 1000
        new_y = orig_home.longitude + 2000
        new_z = orig_home.altitude + 300000 # 300 metres
        print("new home: %s %s %s" % (str(new_x), str(new_y), str(new_z)))
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         new_x,
                         new_y,
                         new_z/1000.0, # mm => m
                         )

        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        got_home_latitude = home.latitude
        got_home_longitude = home.longitude
        got_home_altitude = home.altitude
        if (got_home_latitude != new_x or
            got_home_longitude != new_y or
            abs(got_home_altitude - new_z) > 100): # float-conversion issues
            self.reboot_sitl()
            raise NotAchievedException(
                "Home mismatch got=(%f, %f, %f) set=(%f, %f, %f)" %
                (got_home_latitude, got_home_longitude, got_home_altitude,
                new_x, new_y, new_z))

        self.progress("monitoring home to ensure it doesn't drift at all")
        tstart = self.get_sim_time()
        while self.get_sim_time_cached() - tstart < 10:
            home = self.poll_home_position(quiet=True)
            self.progress("home: %s" % str(home))
            if (home.latitude != got_home_latitude or
                home.longitude != got_home_longitude or
                home.altitude != got_home_altitude): # float-conversion issues
                self.reboot_sitl()
                raise NotAchievedException("home is drifting")

        self.progress("Waiting for EKF to start")
        self.wait_ready_to_arm()
        self.progress("now use lat=0, lon=0 to reset home to current location")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         0, # lat
                         0, # lon
                         new_z/1000.0, # mm => m
                         )
        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        if self.distance_to_home(use_cached_home=True) > 1:
            raise NotAchievedException("Setting home to current location did not work")

        self.progress("Setting home elsewhere again")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         0, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         new_x,
                         new_y,
                         new_z/1000.0, # mm => m
                         )
        if self.distance_to_home() < 10:
            raise NotAchievedException("Setting home to location did not work")

        self.progress("use param1=1 to reset home to current location")
        self.run_cmd_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                         1, # p1,
                         0, # p2,
                         0, # p3,
                         0, # p4,
                         37, # lat
                         21, # lon
                         new_z/1000.0, # mm => m
                         )
        home = self.poll_home_position()
        self.progress("home: %s" % str(home))
        if self.distance_to_home() > 1:
            raise NotAchievedException("Setting home to current location did not work")

        if self.is_tracker():
            # tracker starts armed...
            self.disarm_vehicle(force=True)
        self.reboot_sitl()

    def test_fixed_yaw_calibration(self):
        self.context_push()
        ex = None
        try:
            wanted = {
                "COMPASS_OFS_X": (100, 3.0),
                "COMPASS_OFS_Y": (200, 3.0),
                "COMPASS_OFS_Z": (300, 3.0),
                "COMPASS_DIA_X": 1,
                "COMPASS_DIA_Y": 1,
                "COMPASS_DIA_Z": 1,
                "COMPASS_ODI_X": 0,
                "COMPASS_ODI_Y": 0,
                "COMPASS_ODI_Z": 0,

                "COMPASS_OFS2_X": (100, 3.0),
                "COMPASS_OFS2_Y": (200, 3.0),
                "COMPASS_OFS2_Z": (300, 3.0),
                "COMPASS_DIA2_X": 1,
                "COMPASS_DIA2_Y": 1,
                "COMPASS_DIA2_Z": 1,
                "COMPASS_ODI2_X": 0,
                "COMPASS_ODI2_Y": 0,
                "COMPASS_ODI2_Z": 0,

                "COMPASS_OFS3_X": (100, 3.0),
                "COMPASS_OFS3_Y": (200, 3.0),
                "COMPASS_OFS3_Z": (300, 3.0),
                "COMPASS_DIA3_X": 1,
                "COMPASS_DIA3_Y": 1,
                "COMPASS_DIA3_Z": 1,
                "COMPASS_ODI2_X": 0,
                "COMPASS_ODI2_Y": 0,
                "COMPASS_ODI2_Z": 0,
            }
            self.set_parameter("SIM_MAG_OFS_X", 100)
            self.set_parameter("SIM_MAG_OFS_Y", 200)
            self.set_parameter("SIM_MAG_OFS_Z", 300)

            # set to some sensible-ish initial values.  If your initial
            # offsets are way, way off you can get some very odd effects.
            for param in wanted:
                value = 0.0
                if "DIA" in param:
                    value = 1.001
                elif "ODI" in param:
                    value = 0.001
                self.set_parameter(param, value)
            # zero the parameters:
            self.progress("zeroing parameters")
            self.drain_mav()  # these two lines are odd....
            self.get_sim_time()
            self.run_cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS,
                         2, # param1 (compass0)
                         0, # param2
                         0, # param3
                         0, # param4
                         0, # param5
                         0, # param6
                         0 # param7
            )
            self.progress("zeroed parameters")
            # ensure these are all zero; note that we don't set the DIA or
            # ODI in SET_SENSOR_OFFSETS...
            for axis in "X", "Y", "Z":
                name = "COMPASS_OFS_%s" % axis
                value = self.get_parameter(name)
                if value != 0.0:
                    raise NotAchievedException("Failed to zero %s; got %f" %
                                               (name, value))
            self.change_mode('LOITER')
            self.wait_ready_to_arm() # so we definitely have position
            ss = self.mav.recv_match(type='SIMSTATE', blocking=True, timeout=1)
            if ss is None:
                raise NotAchievedException("Did not get SIMSTATE")
            self.progress("Got SIMSTATE (%s)" % str(ss))

            self.run_cmd(mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW,
                         math.degrees(ss.yaw), # param1
                         0, # param2
                         0, # param3
                         0, # param4

                         0, # param5
                         0, # param6
                         0 # param7
                         )
            self.verify_parameter_values(wanted)

            self.progress("Rebooting and making sure we could arm with these values")
            self.reboot_sitl()
            self.wait_ready_to_arm(timeout=60)

        except Exception as e:
            ex = e

        self.context_pop()

        if ex is not None:
            raise ex

    def test_dataflash_over_mavlink(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("LOG_BACKEND_TYPE", 2)
            self.reboot_sitl()
            self.wait_ready_to_arm()
            self.mavproxy.send('arm throttle\n')
            self.mavproxy.expect('PreArm: Logging failed')
            self.mavproxy.send("module load dataflash_logger\n")
            self.mavproxy.send("dataflash_logger set verbose 1\n")
            self.mavproxy.expect('logging started')
            self.mavproxy.send("dataflash_logger set verbose 0\n")
            self.delay_sim_time(1)
            self.drain_mav() # hopefully draining COMMAND_ACK from that failed arm
            self.arm_vehicle()
            tstart = self.get_sim_time()
            last_status = 0
            while True:
                now = self.get_sim_time()
                if now - tstart > 60:
                    break
                if now - last_status > 5:
                    last_status = now
                    self.mavproxy.send('dataflash_logger status\n')
                    # seen on autotest: Active Rate(3s):97.790kB/s Block:164 Missing:0 Fixed:0 Abandoned:0
                    self.mavproxy.expect("Active Rate\([0-9]s\):([0-9]+[.][0-9]+)")
                    rate = float(self.mavproxy.match.group(1))
                    self.progress("Rate: %f" % rate)
                    if rate < 50:
                        raise NotAchievedException("Exceptionally low transfer rate")
            self.disarm_vehicle()
        except Exception as e:
            self.disarm_vehicle()
            self.progress("Exception (%s) caught" % str(e))
            ex = e
        self.context_pop()
        self.mavproxy.send("module unload dataflash_logger\n")
        self.mavproxy.expect("Unloaded module dataflash_logger")
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_dataflash_sitl(self):
        self.context_push()
        ex = None
        try:
            self.set_parameter("LOG_BACKEND_TYPE", 5)
            self.reboot_sitl()
            self.drain_mav() # hopefully draining COMMAND_ACK from that failed arm
            self.mavproxy.send("module load log\n")
#            self.mavproxy.expect("Loaded module log\n")
            self.mavproxy.send("log erase\n")
            self.mavproxy.send("log list\n")
            self.mavproxy.expect("Log 1  numLogs 1 lastLog 1 size")
            self.reboot_sitl()
            self.mavproxy.send("log list\n")
            self.mavproxy.expect("Log 1  numLogs 2 lastLog 2 size")
        except Exception as e:
            self.progress("Exception (%s) caught" % str(e))
            ex = e
        self.mavproxy.send("module unload log\n")
        self.context_pop()
        self.reboot_sitl()
        if ex is not None:
            raise ex

    def test_arm_feature(self):
        """Common feature to test."""
        # TEST ARMING/DISARM
        if self.get_parameter("ARMING_CHECK") != 1.0 and not self.is_sub():
            raise ValueError("Arming check should be 1")
        if not self.is_sub() and not self.is_tracker():
            self.set_parameter("ARMING_RUDDER", 2)  # allow arm and disarm with rudder on first tests
        if self.is_copter():
            interlock_channel = 8  # Plane got flighmode_ch on channel 8
            if not self.is_heli():  # heli don't need interlock option
                interlock_channel = 9
                self.set_parameter("RC%u_OPTION" % interlock_channel, 32)
            self.set_rc(interlock_channel, 1000)
        self.zero_throttle()
        # Disable auto disarm for next tests
        # Rover and Sub don't have auto disarm
        if self.is_copter() or self.is_plane():
            self.set_autodisarm_delay(0)
        self.start_subtest("Test normal arm and disarm features")
        self.wait_ready_to_arm()
        self.progress("default arm_vehicle() call")
        if not self.arm_vehicle():
            raise NotAchievedException("Failed to ARM")
        self.progress("default disarm_vehicle() call")
        if not self.disarm_vehicle():
            raise NotAchievedException("Failed to DISARM")
        self.progress("arm with mavproxy")
        if not self.mavproxy_arm_vehicle():
            raise NotAchievedException("Failed to ARM")
        self.progress("disarm with mavproxy")
        if not self.mavproxy_disarm_vehicle():
            raise NotAchievedException("Failed to DISARM")

        if not self.is_sub():
            self.start_subtest("Test arm with rc input")
            if not self.arm_motors_with_rc_input():
                raise NotAchievedException("Failed to arm with RC input")
            self.progress("disarm with rc input")
            if self.is_balancebot():
                self.progress("balancebot can't disarm with RC input")
                self.disarm_vehicle()
            else:
                if not self.disarm_motors_with_rc_input():
                    raise NotAchievedException("Failed to disarm with RC input")

            self.start_subtest("Test arm and disarm with switch")
            arming_switch = 7
            self.set_parameter("RC%d_OPTION" % arming_switch, 41)
            self.set_rc(arming_switch, 1000)
            # delay so a transition is seen by the RC switch code:
            self.delay_sim_time(0.5)
            if not self.arm_motors_with_switch(arming_switch):
                raise NotAchievedException("Failed to arm with switch")
            if not self.disarm_motors_with_switch(arming_switch):
                raise NotAchievedException("Failed to disarm with switch")
            self.set_rc(arming_switch, 1000)

            if self.is_copter():
                self.start_subtest("Test arming failure with throttle too high")
                self.set_rc(3, 1800)
                try:
                    if self.arm_vehicle():
                        raise NotAchievedException("Armed when throttle too high")
                except ValueError:
                    pass
                if self.arm_motors_with_rc_input():
                    raise NotAchievedException(
                        "Armed via RC when throttle too high")
                if self.arm_motors_with_switch(arming_switch):
                    raise NotAchievedException("Armed via RC when switch too high")
                self.zero_throttle()
                self.set_rc(arming_switch, 1000)

            # Sub doesn't have 'stick commands'
            self.start_subtest("Test arming failure with ARMING_RUDDER=0")
            self.set_parameter("ARMING_RUDDER", 0)
            if self.arm_motors_with_rc_input():
                raise NotAchievedException(
                    "Armed with rudder when ARMING_RUDDER=0")
            self.start_subtest("Test disarming failure with ARMING_RUDDER=0")
            self.arm_vehicle()
            if self.disarm_motors_with_rc_input():
                raise NotAchievedException(
                    "Disarmed with rudder when ARMING_RUDDER=0")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.start_subtest("Test disarming failure with ARMING_RUDDER=1")
            self.set_parameter("ARMING_RUDDER", 1)
            self.arm_vehicle()
            if self.disarm_motors_with_rc_input():
                raise NotAchievedException(
                    "Disarmed with rudder with ARMING_RUDDER=1")
            self.disarm_vehicle()
            self.wait_heartbeat()
            self.set_parameter("ARMING_RUDDER", 2)

            if self.is_copter():
                self.start_subtest("Test arming failure with interlock enabled")
                self.set_rc(interlock_channel, 2000)
                if self.arm_motors_with_rc_input():
                    raise NotAchievedException(
                        "Armed with RC input when interlock enabled")
                if self.arm_motors_with_switch(arming_switch):
                    raise NotAchievedException(
                        "Armed with switch when interlock enabled")
                self.disarm_vehicle()
                self.wait_heartbeat()
                self.set_rc(arming_switch, 1000)
                self.set_rc(interlock_channel, 1000)
                if self.is_heli():
                    self.start_subtest("Test motor interlock enable can't be set while disarmed")
                    self.set_rc(interlock_channel, 2000)
                    channel_field = "servo%u_raw" % interlock_channel
                    interlock_value = self.get_parameter("SERVO%u_MIN" % interlock_channel)
                    tstart = self.get_sim_time()
                    while True:
                        if self.get_sim_time_cached() - tstart > 20:
                            self.set_rc(interlock_channel, 1000)
                            break # success!
                        m = self.mav.recv_match(type='SERVO_OUTPUT_RAW',
                                                blocking=True,
                                                timeout=2)
                        if m is None:
                            continue
                        m_value = getattr(m, channel_field, None)
                        if m_value is None:
                            self.set_rc(interlock_channel, 1000)
                            raise ValueError("Message has no %s field" %
                                             channel_field)
                        self.progress("SERVO_OUTPUT_RAW.%s=%u want=%u" %
                                      (channel_field, m_value, interlock_value))
                        if m_value != interlock_value:
                            self.set_rc(interlock_channel, 1000)
                            raise NotAchievedException("Motor interlock was changed while disarmed")
                self.set_rc(interlock_channel, 1000)

        self.start_subtest("Test all mode arming")
        if self.arming_test_mission() is not None:
            self.load_mission(self.arming_test_mission())

        for mode in self.mav.mode_mapping():
            self.drain_mav()
            self.start_subtest("Mode : %s" % mode)
            if mode == "FOLLOW":
                self.set_parameter("FOLL_ENABLE", 1)
            if mode in self.get_normal_armable_modes_list():
                self.progress("Armable mode : %s" % mode)
                self.change_mode(mode)
                self.arm_vehicle()
                if not self.disarm_vehicle():
                    raise NotAchievedException("Failed to DISARM")
                self.progress("PASS arm mode : %s" % mode)
            if mode in self.get_not_armable_mode_list():
                if mode in self.get_not_disarmed_settable_modes_list():
                    self.progress("Not settable mode : %s" % mode)
                    try:
                        self.change_mode(mode)
                    except AutoTestTimeoutException:
                        self.progress("PASS not able to set mode : %s disarmed" % mode)
                    except ValueError:
                        self.progress("PASS not able to set mode : %s disarmed" % mode)
                else:
                    self.progress("Not armable mode : %s" % mode)
                    self.change_mode(mode)
                    self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 1,  # ARM
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 0,
                                 want_result=mavutil.mavlink.MAV_RESULT_FAILED
                                 )
                self.progress("PASS not able to arm in mode : %s" % mode)
            if mode in self.get_position_armable_modes_list():
                self.progress("Armable mode needing Position : %s" % mode)
                self.wait_ekf_happy()
                self.change_mode(mode)
                self.arm_vehicle()
                self.wait_heartbeat()
                if not self.disarm_vehicle():
                    raise NotAchievedException("Failed to DISARM")
                self.progress("PASS arm mode : %s" % mode)
                self.progress("Not armable mode without Position : %s" % mode)
                self.wait_gps_disable()
                self.change_mode(mode)
                self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                             1,  # ARM
                             0,
                             0,
                             0,
                             0,
                             0,
                             0,
                             want_result=mavutil.mavlink.MAV_RESULT_FAILED
                             )
                self.set_parameter("SIM_GPS_DISABLE", 0)
                self.wait_ekf_happy() # EKF may stay unhappy for a while
                self.progress("PASS not able to arm without Position in mode : %s" % mode)
            if mode in self.get_no_position_not_settable_modes_list():
                self.progress("Setting mode need Position : %s" % mode)
                self.wait_ekf_happy()
                self.wait_gps_disable()
                try:
                    self.change_mode(mode)
                except AutoTestTimeoutException:
                    self.set_parameter("SIM_GPS_DISABLE", 0)
                    self.progress("PASS not able to set mode without Position : %s" % mode)
                except ValueError:
                    self.set_parameter("SIM_GPS_DISABLE", 0)
                    self.progress("PASS not able to set mode without Position : %s" % mode)
            if mode == "FOLLOW":
                self.set_parameter("FOLL_ENABLE", 0)
        self.change_mode(self.default_mode())
        if self.armed():
            if not self.disarm_vehicle():
                raise NotAchievedException("Failed to DISARM")

        # we should find at least one Armed event and one disarmed
        # event, and at least one ARM message for arm and disarm
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="EV", condition="EV.Id==10") # armed
        if m is None:
            raise NotAchievedException("Did not find an Armed EV message")

        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="EV", condition="EV.Id==11") # disarmed
        if m is None:
            raise NotAchievedException("Did not find a disarmed EV message")

        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="ARM", condition="ARM.ArmState==1")
        if m is None:
            raise NotAchievedException("Did not find a armed ARM message")

        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type="ARM", condition="ARM.ArmState==0")
        if m is None:
            raise NotAchievedException("Did not find a disarmed ARM message")

        self.progress("ALL PASS")
    # TODO : Test arming magic;

    def get_message_rate(self, victim_message, timeout):
        tstart = self.get_sim_time()
        count = 0
        while self.get_sim_time_cached() < tstart + timeout:
            m = self.mav.recv_match(type=victim_message,
                                    blocking=True,
                                    timeout=0.1
                                    )
            if m is not None:
                count += 1
        time_delta = self.get_sim_time_cached() - tstart
        self.progress("%s count after %f seconds: %u" %
                      (victim_message, time_delta, count))
        return count/time_delta

    def rate_to_interval_us(self, rate):
        return 1/float(rate)*1000000.0

    def set_message_rate_hz(self, id, rate_hz):
        '''set a message rate in Hz; 0 for original, -1 to disable'''
        if rate_hz == 0 or rate_hz == -1:
            set_interval = rate_hz
        else:
            set_interval = self.rate_to_interval_us(rate_hz)
        self.run_cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     id,
                     set_interval,
                     0,
                     0,
                     0,
                     0,
                     0)

    def test_rate(self, desc, in_rate, expected_rate):
        self.progress("###### %s" % desc)
        self.progress("Setting rate to %u" % in_rate)
        # SET_MESSAGE_INTERVAL rates are given in microseconds
        if in_rate == 0 or in_rate == -1:
            set_interval = in_rate
        else:
            set_interval = self.rate_to_interval_us(in_rate)

        self.mavproxy.send("long SET_MESSAGE_INTERVAL %u %d\n" %
                           (self.victim_message_id, set_interval))
        self.mav.recv_match(type='COMMAND_ACK', blocking=True)
        new_measured_rate = self.get_message_rate(self.victim_message, 10)
        self.progress("Measured rate: %f (want %u)" %
                      (new_measured_rate, expected_rate))
        if round(new_measured_rate) != expected_rate:
            raise NotAchievedException("Rate not achieved (got %f want %u)" %
                                       (new_measured_rate, expected_rate))

        # make sure get_message_interval works:
        self.mavproxy.send("long GET_MESSAGE_INTERVAL %u\n" %
                           (self.victim_message_id))
        m = self.mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)
        want = set_interval
        if set_interval == 0:
            want = self.rate_to_interval_us(expected_rate)

        if m.interval_us != want:
            raise NotAchievedException("Did not read same interval back from autopilot: want=%d got=%d)" %
            (want, m.interval_us))
        m = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
        if m.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise NotAchievedException("Expected ACCEPTED for reading message interval")

    def test_set_message_interval(self):
        self.victim_message = 'VFR_HUD'
        self.victim_message_id = mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD
        try:
            # tell MAVProxy to stop stuffing around with the rates:
            self.mavproxy.send("set streamrate -1\n")

            rate = round(self.get_message_rate(self.victim_message, 20))
            self.progress("Initial rate: %u" % rate)

            self.test_rate("Test set to %u" % (rate/2,), rate/2, rate/2)
            # this assumes the streamrates have not been played with:
            self.test_rate("Resetting original rate using 0-value", 0, rate)
            self.test_rate("Disabling using -1-value", -1, 0)
            self.test_rate("Resetting original rate", rate, rate)

            self.progress("try getting a message which is not ordinarily streamed out")
            rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
            if rate != 0:
                raise PreconditionFailedException("Already getting CAMERA_FEEDBACK")
            self.progress("try various message rates")
            for want_rate in range(5, 14):
                self.mavproxy.send(
                    "long SET_MESSAGE_INTERVAL %u %d\n" %
                    (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK,
                     self.rate_to_interval_us(want_rate)))
                self.drain_mav()
                rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
                self.progress("Want=%u got=%u" % (want_rate, rate))
                if rate != want_rate:
                    raise NotAchievedException("Did not get expected rate")


            self.progress("try at the main loop rate")
            # have to reset the speedup as MAVProxy can't keep up otherwise
            old_speedup = self.get_parameter("SIM_SPEEDUP")
            self.set_parameter("SIM_SPEEDUP", 1.0)
            # ArduPilot currently limits message rate to 80% of main loop rate:
            want_rate = self.get_parameter("SCHED_LOOP_RATE") * 0.8
            self.mavproxy.send("long SET_MESSAGE_INTERVAL %u %d\n" %
                               (mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK,
                                self.rate_to_interval_us(want_rate)))
            rate = round(self.get_message_rate("CAMERA_FEEDBACK", 20))
            self.set_parameter("SIM_SPEEDUP", old_speedup)
            self.progress("Want=%f got=%f" % (want_rate, rate))
            if abs(rate - want_rate) > 2:
                raise NotAchievedException("Did not get expected rate")

            self.drain_mav()

            self.progress("Resetting CAMERA_FEEDBACK rate to zero")
            self.set_message_rate_hz(mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_FEEDBACK, -1)

            non_existant_id = 145
            self.mavproxy.send("long GET_MESSAGE_INTERVAL %u\n" %
                               (non_existant_id))
            m = self.mav.recv_match(type='MESSAGE_INTERVAL', blocking=True)
            if m.interval_us != 0:
                raise NotAchievedException("Supposed to get 0 back for unsupported stream")
            m = self.mav.recv_match(type='COMMAND_ACK', blocking=True)
            if m.result != mavutil.mavlink.MAV_RESULT_FAILED:
                raise NotAchievedException("Getting rate of unsupported message is a failure")

            sr = self.sitl_streamrate()
            self.mavproxy.send("set streamrate %u\n" % sr)

        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            # tell MAVProxy to start stuffing around with the rates:
            sr = self.sitl_streamrate()
            self.mavproxy.send("set streamrate %u\n" % sr)
            raise e

    def test_request_message(self, timeout=60):
        rate = round(self.get_message_rate("CAMERA_FEEDBACK", 10))
        if rate != 0:
            raise PreconditionFailedException("Receving camera feedback")
        # temporarily use a constant in place of
        # mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE until we have a
        # pymavlink release:
        self.run_cmd(512,
                     180,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=timeout)
        m = self.mav.recv_match(type='CAMERA_FEEDBACK', blocking=True, timeout=1)
        if m is None:
            raise NotAchievedException("Requested CAMERA_FEEDBACK did not arrive")

    def clear_mission(self, mission_type, target_system=1, target_component=1):
        '''clear mision_type from autopilot.  Note that this does NOT actually
        send a MISSION_CLEAR_ALL message
        '''
        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_ALL:
            # recurse
            if not self.is_tracker() and not self.is_plane():
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
            self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            if not self.is_sub() and not self.is_tracker():
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_RALLY)
            self.last_wp_load = time.time()
            return

        self.mav.mav.mission_count_send(target_system,
                                        target_component,
                                        0,
                                        mission_type)
        m = self.mav.recv_match(type='MISSION_ACK',
                                blocking=True,
                                timeout=5)
        if m is None:
            raise NotAchievedException("Expected ACK for clearing mission")
        if m.target_system != self.mav.mav.srcSystem:
            raise NotAchievedException("ACK not targetted at correct system want=%u got=%u" %
                                       (self.mav.mav.srcSystem, m.target_system))
        if m.target_component != self.mav.mav.srcComponent:
            raise NotAchievedException("ACK not targetted at correct component want=%u got=%u" %
                                       (self.mav.mav.srcComponent, m.target_component))
        if m.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise NotAchievedException("Expected MAV_MISSION_ACCEPTED got %s" %
                                       (mavutil.mavlink.enums["MAV_MISSION_RESULT"][m.type].name,))

        if mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            self.last_wp_load = time.time()


    def clear_fence_using_mavproxy(self, timeout=10):
        self.mavproxy.send("fence clear\n")
        tstart = self.get_sim_time_cached()
        while True:
            now = self.get_sim_time_cached()
            if now - tstart > timeout:
                raise AutoTestTimeoutException("FENCE_TOTAL did not go to zero")
            if self.get_parameter("FENCE_TOTAL") == 0:
                break

    def clear_fence(self):
        self.clear_fence_using_mavproxy()

    def clear_mission_using_mavproxy(self):
        self.mavproxy.send("wp clear\n")
        self.mavproxy.send('wp list\n')
        self.mavproxy.expect('Requesting [0-9]+ waypoints')
        num_wp = mavwp.MAVWPLoader().count()
        if num_wp != 0:
            raise NotAchievedException("Failed to clear mission")
        self.last_wp_load = time.time()

    def test_config_error_loop(self):
        '''test the sensor config error loop works and that parameter sets are persistent'''
        parameter_name = "SERVO8_MIN"
        old_parameter_value = self.get_parameter(parameter_name)
        old_sim_baro_count = self.get_parameter("SIM_BARO_COUNT")
        new_parameter_value = old_parameter_value + 5
        ex = None
        try:
            self.set_parameter("STAT_BOOTCNT", 0)
            self.set_parameter("SIM_BARO_COUNT", 0)

            if self.is_tracker():
                # starts armed...
                self.progress("Disarming tracker")
                self.disarm_vehicle(force=True)

            self.reboot_sitl(required_bootcount=1);
            self.progress("Waiting for 'Config error'")
            self.mavproxy.expect("Config error");
            self.progress("Setting %s to %f" % (parameter_name, new_parameter_value))
            self.set_parameter(parameter_name, new_parameter_value)
        except Exception as e:
            ex = e

        self.progress("Resetting SIM_BARO_COUNT")
        self.set_parameter("SIM_BARO_COUNT", old_sim_baro_count)

        if self.is_tracker():
            # starts armed...
            self.progress("Disarming tracker")
            self.disarm_vehicle(force=True)

        self.progress("Calling reboot-sitl ")
        self.reboot_sitl(required_bootcount=2);

        if ex is not None:
            raise ex

        if self.get_parameter(parameter_name) != new_parameter_value:
            raise NotAchievedException("Parameter value did not stick")

    def test_gripper(self):
        self.context_push()
        self.set_parameter("GRIP_ENABLE", 1)
        self.fetch_parameters()
        self.set_parameter("GRIP_GRAB", 2000)
        self.set_parameter("GRIP_RELEASE", 1000)
        self.set_parameter("GRIP_TYPE", 1)
        self.set_parameter("SIM_GRPS_ENABLE", 1)
        self.set_parameter("SIM_GRPS_PIN", 8)
        self.set_parameter("SERVO8_FUNCTION", 28)
        self.set_parameter("SERVO8_MIN", 1000)
        self.set_parameter("SERVO8_MAX", 2000)
        self.set_parameter("SERVO9_MIN", 1000)
        self.set_parameter("SERVO9_MAX", 2000)
        self.set_parameter("RC9_OPTION", 19)
        self.set_rc(9, 1500)
        self.reboot_sitl()
        self.progress("Waiting for ready to arm")
        self.wait_ready_to_arm()
        self.progress("Test gripper with RC9_OPTION")
        self.progress("Releasing load")
        # non strict string matching because of catching text issue....
        self.wait_text("Gripper load releas", the_function=lambda: self.send_set_rc(9, 1000))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb", the_function=lambda: self.send_set_rc(9, 2000))
        self.progress("Releasing load")
        self.wait_text("Gripper load releas", the_function=lambda: self.send_set_rc(9, 1000))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb", the_function=lambda: self.send_set_rc(9, 2000))
        self.progress("Test gripper with Mavlink cmd")
        self.progress("Releasing load")
        self.wait_text("Gripper load releas",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_RELEASE,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_GRAB,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Releasing load")
        self.wait_text("Gripper load releas",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_RELEASE,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.progress("Grabbing load")
        self.wait_text("Gripper load grabb",
                       the_function=lambda: self.mav.mav.command_long_send(1,
                                                                           1,
                                                                           mavutil.mavlink.MAV_CMD_DO_GRIPPER,
                                                                           0,
                                                                           1,
                                                                           mavutil.mavlink.GRIPPER_ACTION_GRAB,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           0,
                                                                           ))
        self.context_pop()
        self.reboot_sitl()

    def is_copter(self):
        return False

    def is_sub(self):
        return False

    def is_plane(self):
        return False

    def is_rover(self):
        return False

    def is_balancebot(self):
        return False

    def is_heli(self):
        return False

    def is_tracker(self):
        return False

    def initial_mode(self):
        '''return mode vehicle should start in with no RC inputs set'''
        return None

    def initial_mode_switch_mode(self):
        '''return mode vehicle should start in with default RC inputs set'''
        return None

    def upload_fences_from_locations(self,
                                     vertex_type,
                                     list_of_list_of_locs,
                                     target_system=1,
                                     target_component=1):
        seq = 0
        items = []
        for locs in list_of_list_of_locs:
            if type(locs) == dict:
                # circular fence
                if vertex_type == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
                    v = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
                else:
                    v = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    v,
                    0, # current
                    0, # autocontinue
                    locs["radius"], # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(locs["loc"].lat *1e7), # latitude
                    int(locs["loc"].lng *1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                seq += 1
                items.append(item)
                continue
            count = len(locs)
            for loc in locs:
                item = self.mav.mav.mission_item_int_encode(
                    target_system,
                    target_component,
                    seq, # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL,
                    vertex_type,
                    0, # current
                    0, # autocontinue
                    count, # p1
                    0, # p2
                    0, # p3
                    0, # p4
                    int(loc.lat *1e7), # latitude
                    int(loc.lng *1e7), # longitude
                    33.0000, # altitude
                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                seq += 1
                items.append(item)

        self.upload_using_mission_protocol(mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
                                           items)

    def wait_for_initial_mode(self):
        '''wait until we get a heartbeat with an expected initial mode (the
one specified in the vehicle constructor)'''
        want = self.initial_mode()
        if want is None:
            return
        self.progress("Waiting for initial mode %s" % want)
        self.wait_mode(want)

    def wait_for_mode_switch_poll(self):
        '''look for a transition from boot-up-mode (e.g. the flightmode
specificied in Copter's constructor) to the one specified by the mode
switch value'''
        want = self.initial_mode_switch_mode()
        if want is None:
            return
        self.progress("Waiting for mode-switch mode %s" % want)
        self.wait_mode(want)

    def start_subtest(self, description):
        self.progress("-")
        self.progress("---------- %s  ----------" % description)
        self.progress("-")

    def start_subsubtest(self, description):
        self.progress(".")
        self.progress(".......... %s  .........." % description)
        self.progress(".")

    def end_subtest(self, description):
        '''TODO: sanity checks?'''
        pass

    def end_subsubtest(self, description):
        '''TODO: sanity checks?'''
        pass

    def test_skipped(self, test, reason):
        (name, desc, func) = test
        self.progress("##### %s is skipped: %s" % (name, reason))
        self.skip_list.append((test, reason))

    def last_onboard_log(self):
        '''return number of last onboard log'''
        self.mavproxy.send("module load log\n")
        self.mavproxy.expect("Loaded module log")
        self.mavproxy.send("log list\n")
        self.mavproxy.expect("lastLog ([0-9]+)")
        num_log = int(self.mavproxy.match.group(1))
        self.mavproxy.send("module unload log\n")
        self.mavproxy.expect("Unloaded module log")
        return num_log

    def current_onboard_log_filepath(self):
        '''return filepath to currently open dataflash log'''
        return os.path.join("logs/%08u.BIN" % self.last_onboard_log())

    def dfreader_for_current_onboard_log(self):
        return DFReader.DFReader_binary(self.current_onboard_log_filepath(),
                                        zero_time_base=True);

    def current_onboard_log_contains_message(self, messagetype):
        dfreader = self.dfreader_for_current_onboard_log()
        m = dfreader.recv_match(type=messagetype)
        print("m=%s" % str(m))
        return m is not None

    def run_tests(self, tests):
        """Autotest vehicle in SITL."""
        if self.run_tests_called:
            raise ValueError("run_tests called twice")
        self.run_tests_called = True

        self.init()

        self.fail_list = []

        try:
            self.progress("Waiting for a heartbeat with mavlink protocol %s"
                          % self.mav.WIRE_PROTOCOL_VERSION)
            self.wait_heartbeat()
            self.wait_for_initial_mode()
            self.progress("Setting up RC parameters")
            self.set_rc_default()
            self.wait_for_mode_switch_poll()
            if not self.is_tracker(): # FIXME - more to the point, fix Tracker's mission handling
                self.clear_mission(mavutil.mavlink.MAV_MISSION_TYPE_ALL)

            for test in tests:
                (name, desc, func) = test
                self.run_one_test(name, desc, func)

        except pexpect.TIMEOUT:
            self.progress("Failed with timeout")
            self.fail_list.append(["Failed with timeout", None, None])
        self.close()

        if len(self.skip_list):
            self.progress("Skipped tests:")
            for skipped in self.skip_list:
                (test, reason) = skipped
                (name, desc, func) = test
                print("  %s (see %s)" % (name, reason))

        if len(self.fail_list):
            self.progress("Failing tests:")
            for failure in self.fail_list:
                (desc, exception, debug_filename) = failure
                print("  %s (%s) (see %s)" % (desc, exception, debug_filename))
            return False

        return True

    def dictdiff(self, dict1, dict2):
        fred = copy.copy(dict1)
        for key in dict2.keys():
            try:
                del fred[key]
            except:
                pass
        return fred

    def wait_distance_to_home(self, distance_min, distance_max, timeout=10, use_cached_home=True):
        tstart = self.get_sim_time()
        while True:
            if self.get_sim_time() - tstart > timeout:
                raise NotAchievedException("Did not achieve distance from home")
            distance = self.distance_to_home(use_cached_home)
            self.progress("Distance from home: now=%f %f<want<%f" %
                          (distance, distance_min, distance_max))
            if distance >= distance_min and distance <= distance_max:
                return

    # download parameters tries to cope with its download being
    # interrupted or broken by simply retrying the download a few
    # times.
    def download_parameters(self, target_system, target_component):
        # try a simple fetch-all:
        tstart = self.get_sim_time_cached()
        last_parameter_received = 0
        attempt_count = 0
        start_done = False
        # make flake8 happy:
        count = 0
        expected_count = 0
        seen_ids = {}
        self.progress("Downloading parameters")
        while True:
            now = self.get_sim_time_cached()
            if not start_done or now - last_parameter_received > 10:
                start_done = True
                if attempt_count > 3:
                    raise AutoTestTimeoutException("Failed to download parameters  (have %s/%s) (seen_ids-count=%u)" %
                                                   (str(count), str(expected_count), len(seen_ids.keys())))
                elif attempt_count != 0:
                    self.progress("Download failed; retrying")
                self.drain_mav()
                self.mav.mav.param_request_list_send(target_system, target_component)
                attempt_count += 1
                count = 0
                expected_count = None
                seen_ids = {}
                id_seq = {}
            m = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
            if m is None:
                raise AutoTestTimeoutException("tardy PARAM_VALUE (have %s/%s)" % (
                    str(count), str(expected_count)))
            if m.param_index == 65535:
                self.progress("volunteered parameter: %s" % str(m))
                continue
            if False:
                self.progress("  received (%4u/%4u %s=%f" %
                              (m.param_index, m.param_count, m.param_id, m.param_value))
            if m.param_index >= m.param_count:
                raise ValueError("parameter index (%u) gte parameter count (%u)" %
                                 (m.param_index, m.param_count))
            if expected_count is None:
                expected_count = m.param_count
            else:
                if m.param_count != expected_count:
                    raise ValueError("expected count changed")
            if m.param_id not in seen_ids:
                count += 1
                seen_ids[m.param_id] = m.param_value
                last_parameter_received = now
                if count == expected_count:
                    break

        self.progress("Downloaded %u parameters OK (attempt=%u)" %
                      (count, attempt_count))
        return (seen_ids, id_seq)

    def test_parameters_download(self):
        self.start_subtest("parameter download")
        target_system = self.sysid_thismav()
        target_component = 1
        (parameters, seq_id) = self.download_parameters(target_system, target_component)
        self.reboot_sitl()
        (parameters2, seq2_id) = self.download_parameters(target_system, target_component)

        delta = self.dictdiff(parameters, parameters2)
        if len(delta) != 0:
            raise ValueError("Got %u fewer parameters when downloading second time (before=%u vs after=%u) (delta=%s)" %
                             (len(delta), len(parameters), len(parameters2), str(delta.keys())))

        delta = self.dictdiff(parameters2, parameters)
        if len(delta) != 0:
            raise ValueError("Got %u extra parameters when downloading second time (before=%u vs after=%u) (delta=%s)" %
                             (len(delta), len(parameters), len(parameters2), str(delta.keys())))

        self.end_subsubtest("parameter download")

    def test_enable_parameter(self):
        self.start_subtest("enable parameters")
        target_system = 1
        target_component = 1
        parameters = self.download_parameters(target_system, target_component)
        enable_parameter = self.sample_enable_parameter()
        if enable_parameter is None:
            self.progress("Skipping enable parameter check as no enable parameter supplied")
            return
        self.set_parameter(enable_parameter, 1)
        parameters2 = self.download_parameters(target_system, target_component)
        if len(parameters) == len(parameters2):
            raise NotAchievedException("Enable parameter did not increase no of parameters downloaded")
        self.end_subsubtest("enable download")

    def test_parameters_mis_total(self):
        self.start_subsubtest("parameter mis_total")
        if self.is_tracker():
            # uses CMD_TOTAL not MIS_TOTAL, and it's in a scalr not a
            # group and it's generally all bad.
            return
        self.start_subtest("Ensure GCS is not be able to set MIS_TOTAL")
        old_mt = self.get_parameter("MIS_TOTAL")
        ex = None
        try:
            self.set_parameter("MIS_TOTAL", 17, retries=0)
        except ValueError as e:
            ex = e
        if ex is None:
            raise NotAchievedException("Set parameter when I shouldn't have")
        if old_mt != self.get_parameter("MIS_TOTAL"):
            raise NotAchievedException("Total has changed")

        self.start_subtest("Ensure GCS is able to set other MIS_ parameters")
        self.set_parameter("MIS_OPTIONS", 1)
        if self.get_parameter("MIS_OPTIONS") != 1:
            raise NotAchievedException("Failed to set MIS_OPTIONS")

    def test_parameter_documentation(self):
        '''ensure parameter documentation is valid'''
        self.start_subsubtest("Check all parameters are documented")
        self.test_parameter_documentation_get_all_parameters()

    def test_parameters(self):
        '''general small tests for parameter system'''
        self.test_parameter_documentation();
        self.test_parameters_mis_total()
        self.test_parameters_download()

    def disabled_tests(self):
        return {}

    def test_parameter_checks_poscontrol(self, param_prefix):
        self.wait_ready_to_arm()
        self.context_push()
        self.set_parameter("%s_POSXY_P" % param_prefix, -1)
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=4,
                     want_result=mavutil.mavlink.MAV_RESULT_FAILED)
        self.context_pop()
        self.run_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                     1,  # ARM
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     timeout=4,
                     want_result=mavutil.mavlink.MAV_RESULT_ACCEPTED)
        self.disarm_vehicle()

    def test_pid_tuning(self):
        self.progress("making sure we're not getting PID_TUNING messages")
        m = self.mav.recv_match(type='PID_TUNING', blocking=True, timeout=5)
        if m is not None:
            raise PreconditionFailedException("Receiving PID_TUNING already")
        self.set_parameter("GCS_PID_MASK", 1)
        self.progress("making sure we are now getting PID_TUNING messages")
        m = self.mav.recv_match(type='PID_TUNING', blocking=True, timeout=5)
        if m is None:
            raise PreconditionFailedException("Did not start to get PID_TUNING message")

    def sample_mission_filename(self):
        return "flaps.txt"

    def test_advanced_failsafe(self):
        self.context_push()
        ex = None
        try:
            self.drain_mav()
            self.assert_no_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)
            self.set_parameter("AFS_ENABLE", 1)
            self.drain_mav()
            self.assert_capability(mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION)
            self.set_parameter("AFS_TERM_ACTION", 42)
            self.load_sample_mission()
            messages = []
            def my_message_hook(mav, m):
                if m.get_type() != 'STATUSTEXT':
                    return
                messages.append(m)
            self.install_message_hook(my_message_hook)
            try:
                self.change_mode("AUTO") # must go to auto for AFS to latch on
            finally:
                self.remove_message_hook(my_message_hook)

            if "AFS State: AFS_AUTO" not in [x.text for x in messages]:
                self.wait_statustext("AFS State: AFS_AUTO")
            self.change_mode("MANUAL")
            self.start_subtest("RC Failure")
            self.set_parameter("AFS_RC_FAIL_TIME", 1)
            self.set_parameter("SIM_RC_FAIL", 1)
            self.wait_statustext("Terminating due to RC failure")
            self.set_parameter("AFS_RC_FAIL_TIME", 0)
            self.set_parameter("SIM_RC_FAIL", 0)
            self.set_parameter("AFS_TERMINATE", 0)

            if not self.is_plane():
                # plane requires a polygon fence...
                self.start_subtest("Altitude Limit breach")
                self.set_parameter("AFS_AMSL_LIMIT", 100)
                self.mavproxy.send("fence enable\n")
                self.wait_statustext("Terminating due to fence breach")
                self.set_parameter("AFS_AMSL_LIMIT", 0)
                self.set_parameter("AFS_TERMINATE", 0)
                self.mavproxy.send("fence disable\n")

            self.start_subtest("GPS Failure")
            self.set_parameter("AFS_MAX_GPS_LOSS", 1)
            self.set_parameter("SIM_GPS_DISABLE", 1)
            self.wait_statustext("AFS State: GPS_LOSS")
            self.set_parameter("SIM_GPS_DISABLE", 0)
            self.set_parameter("AFS_MAX_GPS_LOSS", 0)
            self.set_parameter("AFS_TERMINATE", 0)

            self.send_cmd(mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
                          1,  # terminate
                          0,
                          0,
                          0,
                          0,
                          0,
                          0,
            )
            self.wait_statustext("Terminating due to GCS request")

        except Exception as e:
            ex = e
        self.mavproxy.send("fence disable\n")
        if ex is not None:
            raise ex

    def drain_mav_seconds(self, seconds):
        tstart = self.get_sim_time_cached()
        while self.get_sim_time_cached() - tstart < seconds:
            self.drain_mav();
            self.delay_sim_time(0.5)

    def nmea_output(self):
        self.set_parameter("SERIAL5_PROTOCOL", 20) # serial5 is NMEA output
        self.set_parameter("GPS_TYPE2", 5) # GPS2 is NMEA
        self.customise_SITL_commandline([
            "--uartE=tcp:6735", # GPS2 is NMEA....
            "--uartF=tcpclient:127.0.0.1:6735", # serial5 spews to localhost:6735
        ])
        gps1 = self.mav.recv_match(type="GPS_RAW_INT", blocking=True, timeout=10)
        self.progress("gps1=(%s)" % str(gps1))
        if gps1 is None:
            raise NotAchievedException("Did not receive GPS_RAW_INT")
        tstart = self.get_sim_time()
        while True:
            now = self.get_sim_time()
            if now - tstart > 20:
                raise NotAchievedException("NMEA output not updating?!")
            gps2 = self.mav.recv_match(type="GPS2_RAW", blocking=True, timeout=1)
            self.progress("gps2=%s" % str(gps2))
            if gps2 is None:
                continue
            if gps2.time_usec != 0:
                break
        if self.get_distance_int(gps1, gps2) > 1:
            raise NotAchievedException("NMEA output inaccurate")

    def mavproxy_load_module(self, module):
        self.mavproxy.send("module load %s\n" % module)
        self.mavproxy.expect("Loaded module %s" % module)

    def mavproxy_unload_module(self, module):
        self.mavproxy.send("module unload %s\n" % module)
        self.mavproxy.expect("Unloaded module %s" % module)

    def accelcal(self):
        ex = None
        try:
            self.customise_SITL_commandline(["-M", "calibration"])
            self.mavproxy_load_module("sitl_calibration")
            self.mavproxy_load_module("calibration")
            self.mavproxy_load_module("relay")
            self.mavproxy.send("sitl_accelcal\n")
            self.mavproxy.send("accelcal\n")
            self.mavproxy.expect("Init Gyro")
            self.mavproxy.expect("Calibrated")
            for wanted in [ "level",
                            "on its LEFT side",
                            "on its RIGHT side",
                            "nose DOWN",
                            "nose UP",
                            "on its BACK",
            ]:
                timeout = 2
                self.mavproxy.expect("Place vehicle %s and press any key." % wanted, timeout=timeout)
                self.mavproxy.expect("sitl_accelcal: sending attitude, please wait..", timeout=timeout)
                self.mavproxy.expect("sitl_accelcal: attitude detected, please press any key..", timeout=timeout)
                self.mavproxy.send("\n")
            self.mavproxy.expect("APM: Calibration successful", timeout=timeout)
        except Exception as e:
            self.progress("Caught exception: %s" %
                          self.get_exception_stacktrace(e))
            ex = e
        self.mavproxy_unload_module("relay")
        self.mavproxy_unload_module("calibration")
        self.mavproxy_unload_module("sitl_calibration")
        if ex is not None:
            raise ex

    def test_button(self):
        self.set_parameter("SIM_PIN_MASK", 0)
        self.set_parameter("BTN_ENABLE", 1)
        btn = 2
        pin = 3
        self.drain_mav()
        self.set_parameter("BTN_PIN%u" % btn, pin)
        m = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        self.progress("m: %s" % str(m))
        if m is None:
            raise NotAchievedException("Did not get BUTTON_CHANGE event")
        mask = 1<<btn
        if m.state & mask:
            raise NotAchievedException("Bit incorrectly set in mask (got=%u dontwant=%u)" % (m.state, mask))
        # SITL instantly reverts the pin to its old value
        m2 = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        self.progress("m2: %s" % str(m2))
        if m2 is None:
            raise NotAchievedException("Did not get repeat message")
        # wait for messages to stop coming:
        self.drain_mav_seconds(15)

        self.set_parameter("SIM_PIN_MASK", 0)
        m3 = self.mav.recv_match(type='BUTTON_CHANGE', blocking=True, timeout=1)
        self.progress("m3: %s" % str(m3))
        if m3 is None:
            raise NotAchievedException("Did not get new message")
        if m.last_change_ms == m3.last_change_ms:
            raise NotAchievedException("last_change_ms same as first message")
        if m3.state != 0:
            raise NotAchievedException("Didn't get expected mask back in message (mask=0 state=%u" % (m3.state))

    def tfp_validate_vel_and_yaw(self, value):
        self.progress("validating vel_and_yaw(0x%02x)" % value)
        VELANDYAW_XYVEL_OFFSET = 9
        VELANDYAW_YAW_LIMIT = 0x7FF
        VELANDYAW_YAW_OFFSET = 16
        yaw = value >> VELANDYAW_YAW_OFFSET
        xy_vel = value >> VELANDYAW_XYVEL_OFFSET & 0xFF
        z_vel_dm_per_second = value & 0xFFFF

        gpi = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if gpi is None:
            return
        self.progress(" yaw=%u gpi=%u" % (yaw, gpi.hdg))
        self.progress(" xy_vel=%u" % xy_vel)
        self.progress(" z_vel_dm_per_second=%u" % z_vel_dm_per_second)
        if int(round(yaw/10.0)) == int(round(gpi.hdg/100.0)):
            self.progress("Yaw match")
            return True
              # FIXME: need to be under way to check the velocities, really....
        return False

    def tfp_validate_battery1(self, value):
        self.progress("validating battery1 (0x%02x)" % value)
        BATT_VOLTAGE_LIMIT            = 0x1FF
        BATT_CURRENT_OFFSET           = 9
        BATT_TOTALMAH_LIMIT           = 0x7FFF
        BATT_TOTALMAH_OFFSET          = 17
        voltage = (value & BATT_VOLTAGE_LIMIT)/10.0
        batt = self.mav.recv_match(
            type='BATTERY_STATUS',
            blocking=True,
            timeout=5,
            condition="BATTERY_STATUS.id==0"
        )
        if batt is None:
            raise NotAchievedException("Did not get BATTERY_STATUS message")
        battery_status_value = batt.voltages[0]/1000.0
        self.progress("BATTERY_STATUS==%f frsky==%f" % (battery_status_value, voltage))
        if abs(battery_status_value - voltage) > 0.1:
            return False
        return True

    def test_frsky_passthrough(self):
        self.set_parameter("SERIAL5_PROTOCOL", 10) # serial5 is FRSky passthrough
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        frsky = FRSkyPassThrough(("127.0.0.1", 6735))

        # waiting until we are ready to arm should ensure our wanted
        # statustext doesn't get blatted out of the ArduPilot queue by
        # random messages.
        self.wait_ready_to_arm()

        # test we get statustext strings.  This relies on ArduPilot
        # emitting statustext strings when we fetch parameters.
        self.mavproxy.send("param fetch\n")
        tstart = self.get_sim_time_cached()
        old_data = None
        text = ""
        while True:
            now = self.get_sim_time()
            if now - tstart > 60: # it can take a *long* time to get these messages down!
                raise NotAchievedException("Did not get statustext in time")
            frsky.update()
            data = frsky.get_data(0x5000) # no timestamping on this data, so we can't catch legitimate repeats.
            if data is None:
                continue
            # frsky sends each quartet three times; skip the suplicates.
            if old_data is not None and old_data == data:
                continue
            old_data = data
            self.progress("Got (0x%x)" % data)
            severity = 0
            last = False
            for i in 3, 2, 1, 0:
                x = (data >> i*8) & 0xff
                text += chr(x & 0x7f)
                self.progress("  x=0x%02x" % x)
                if x & 0x80:
                    severity += 1 << i
                self.progress("Text sev=%u: %s" % (severity, str(text)))
                if (x & 0x7f) == 0x00:
                    last = True
            if last:
                m = re.match("Ardu(Plane|Copter|Rover|Tracker|Sub) V[345]", text)
                if m is not None:
                    want_sev = mavutil.mavlink.MAV_SEVERITY_INFO
                    if severity != want_sev:
                        raise NotAchievedException("Incorrect severity; want=%u got=%u" % (want_sev, severity))
                    self.progress("Got statustext (%s)" % m.group(0))
                    break
                text = ""

        self.wait_ready_to_arm()
        self.drain_mav_unparsed()
        # anything with a lambda in here needs a proper test written.
        # This, at least makes sure we're getting some of each
        # message.  These are ordered according to the wfq scheduler
        wants = {
            0x5000: lambda xx : True,
            0x5006: lambda xx : True,
            0x800: lambda xx : True,
            0x5005: self.tfp_validate_vel_and_yaw,
            0x5001: lambda xx : True,
            0x5002: lambda xx : True,
            0x5004: lambda xx : True,
            #            0x5008: lambda x : True, # no second battery, so this doesn't arrive
            0x5003: self.tfp_validate_battery1,
            0x5007: lambda xx : True,
        }
        tstart = self.get_sim_time_cached()
        while len(wants):
            self.progress("Still wanting (%s)" % ",".join([ ("0x%02x" % x) for x in wants.keys()]))
            wants_copy = copy.copy(wants)
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to get frsky data")
            frsky.update()
            for want in wants_copy:
                data = frsky.get_data(want)
                if data is None:
                    continue
                self.progress("Checking %s" % str(want))
                if wants[want](data):
                    self.progress("  Fulfilled")
                    del wants[want]

    def test_frsky_sport(self):
        self.set_parameter("SERIAL5_PROTOCOL", 4) # serial5 is FRSky sport
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        frsky = FRSkySPort(("127.0.0.1", 6735))
        self.wait_ready_to_arm()
        self.drain_mav_unparsed()
        # anything with a lambda in here needs a proper test written.
        # This, at least makes sure we're getting some of each
        # message.
        wants = {
            0x02:  lambda x : True,
            0x04:  lambda x : True,
            0x05:  lambda x : True,
            0x10:  lambda x : True,
            0x21:  lambda x : True,
            0x30:  lambda x : True,
            0x39:  lambda x : True,
            0x800: lambda x : True,
        }
        tstart = self.get_sim_time_cached()
        last_wanting_print = 0
        while len(wants):
            now = self.get_sim_time()
            if now - last_wanting_print > 1:
                self.progress("Still wanting (%s)" % ",".join([ ("0x%02x" % x) for x in wants.keys()]))
                last_wanting_print = now
            wants_copy = copy.copy(wants)
            if now - tstart > 10:
                raise AutoTestTimeoutException("Failed to get frsky data")
            frsky.update()
            for want in wants_copy:
                data = frsky.get_data(want)
                if data is None:
                    continue
                self.progress("Checking %s" % str(want))
                if wants[want](data):
                    self.progress("  Fulfilled")
                    del wants[want]

    def test_frsky_d(self):
        self.set_parameter("SERIAL5_PROTOCOL", 3) # serial5 is FRSky output
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        frsky = FRSkyD(("127.0.0.1", 6735))
        self.wait_ready_to_arm()
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if m is None:
            raise NotAchievedException("Did not receive GLOBAL_POSITION_INT")
        gpi_abs_alt = int(m.alt / 1000) # mm -> m
        tstart = self.get_sim_time_cached()
        while True:
            t2 = self.get_sim_time_cached()
            if t2 - tstart > 10:
                raise AutoTestTimeoutException("Failed to get frsky data")
            frsky.update()
            alt = frsky.get_data(frsky.dataid_GPS_ALT_BP)
            self.progress("Got alt (%s) mav=%s" % (str(alt), str(gpi_abs_alt)))
            if alt is None:
                continue
            self.drain_mav_unparsed()
            if alt == gpi_abs_alt:
                break

    def test_ltm_g(self, ltm):
        g = ltm.g()
        if g is None:
            return
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print("m: %s" % str(m))

        print("g.lat=%s m.lat=%s" % (str(g.lat()), str(m.lat)))
        if abs(m.lat - g.lat()) > 10:
            return False

        print("g.lon:%s m.lon:%s" % (str(g.lon()), str(m.lon)))
        if abs(m.lon - g.lon()) > 10:
            return False

        print("gndspeed: %s" % str(g.gndspeed()))
        if g.gndspeed() != 0:
            # FIXME if we start the vehicle moving.... check against VFR_HUD?
            return False

        print("g.alt=%s m.alt=%s" % (str(g.alt()/100.0), str(m.relative_alt/1000.0)))
        if abs(m.relative_alt/1000.0 - g.alt()/100.0) > 1:
            return False

        print("sats: %s" % str(g.sats()))
        m = self.mav.recv_match(type='GPS_RAW_INT', blocking=True)
        if m.satellites_visible != g.sats():
            return False

        constrained_fix_type = m.fix_type
        if constrained_fix_type > 3:
            constrained_fix_type = 3
        print("fix_type: %s" % g.fix_type())
        if constrained_fix_type != g.fix_type():
            return False

        return True

    def test_ltm_a(self, ltm):
        a = ltm.a()
        if a is None:
            return
        m = self.mav.recv_match(type='ATTITUDE', blocking=True)

        pitch = a.pitch()
        print("pitch: %s" % str(pitch))
        if abs(math.degrees(m.pitch) - pitch) > 1:
            return False

        roll = a.roll()
        print("roll: %s" % str(roll))
        if abs(math.degrees(m.roll) - roll) > 1:
            return False

        hdg = a.hdg()
        myaw = math.degrees(m.yaw)
        myaw += 360
        myaw %= 360
        print("a.hdg=%s m.hdg=%s" % (str(hdg), str(myaw)))
        if abs(myaw - hdg) > 1:
            return False

        return True

    def test_ltm_s(self, ltm):
        s = ltm.s()
        if s is None:
            return
        # FIXME.  Actually check the field values are correct :-)
        return True

    def test_ltm(self):
        self.set_parameter("SERIAL5_PROTOCOL", 25) # serial5 is LTM output
        self.customise_SITL_commandline([
            "--uartF=tcp:6735" # serial5 spews to localhost:6735
        ])
        ltm = LTM(("127.0.0.1", 6735))
        self.wait_ready_to_arm()
        self.drain_mav_unparsed()
        m = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if m is None:
            raise NotAchievedException("Did not receive GLOBAL_POSITION_INT")
        gpi_abs_alt = int(m.alt / 1000) # mm -> m

        wants = {
            "g": self.test_ltm_g,
            "a": self.test_ltm_a,
            "s": self.test_ltm_s,
        }

        tstart = self.get_sim_time_cached()
        while True:
            self.progress("Still wanting (%s)" %
                          ",".join([ ("%s" % x) for x in wants.keys()]))
            if len(wants) == 0:
                break
            now = self.get_sim_time_cached()
            if now - tstart > 10:
                raise AutoTestTimeoutException("Failed to get ltm data")

            ltm.update()

            wants_copy = copy.copy(wants)
            for want in wants_copy:
                self.progress("Checking %s" % str(want))
                if wants[want](ltm):
                    self.progress("  Fulfilled")
                    del wants[want]

    def tests(self):
        return [
            ("PIDTuning",
             "Test PID Tuning",
             self.test_pid_tuning),

            ("ArmFeatures", "Arm features", self.test_arm_feature),

            ("SetHome",
            "Test Set Home",
             self.fly_test_set_home),

            ("ConfigErrorLoop",
             "Test Config Error Loop",
             self.test_config_error_loop),

            ("CPUFailsafe",
             "Ensure we do something appropriate when the main loop stops",
             self.CPUFailsafe),

            ("Parameters",
             "Test Parameter Set/Get",
             self.test_parameters),

            ("LoggerDocumentation",
             "Test Onboard Logging Generation",
             self.test_onboard_logging_generation),

            ("GetCapabilities",
             "Get Capabilities",
             self.test_get_autopilot_capabilities),
        ]

    def post_tests_announcements(self):
        if self._show_test_timings:
            if self.waiting_to_arm_count == 0:
                avg = None
            else:
                avg = self.total_waiting_to_arm_time/self.waiting_to_arm_count
            self.progress("Spent %f seconds waiting to arm. count=%u avg=%f" %
                          (self.total_waiting_to_arm_time,
                          self.waiting_to_arm_count,
                          avg))
            self.show_test_timings()
        if self.forced_post_test_sitl_reboots != 0:
            print("Had to force-reset SITL %u times" %
                  (self.forced_post_test_sitl_reboots,))

    def autotest(self):
        """Autotest used by ArduPilot autotest CI."""
        all_tests = self.tests()
        disabled = self.disabled_tests()
        tests = []
        for test in all_tests:
            (name, desc, func) = test
            if name in disabled:
                self.test_skipped(test, disabled[name])
                continue
            tests.append(test)

        ret = self.run_tests(tests)
        self.post_tests_announcements()
        return ret

    def mavfft_fttd(self, sensor_type, sensor_instance, since, until):
        '''display fft for raw ACC data in current logfile'''

        '''object to store data about a single FFT plot'''
        class MessageData(object):
            def __init__(self, ffth):
                self.seqno = -1
                self.fftnum = ffth.N
                self.sensor_type = ffth.type
                self.instance = ffth.instance
                self.sample_rate_hz = ffth.smp_rate
                self.multiplier = ffth.mul
                self.sample_us = ffth.SampleUS
                self.data = {}
                self.data["X"] = []
                self.data["Y"] = []
                self.data["Z"] = []
                self.holes = False
                self.freq = None

            def add_fftd(self, fftd):
                self.seqno += 1
                self.data["X"].extend(fftd.x)
                self.data["Y"].extend(fftd.y)
                self.data["Z"].extend(fftd.z)

        mlog = self.dfreader_for_current_onboard_log()

        # see https://holometer.fnal.gov/GH_FFT.pdf for a description of the techniques used here
        messages = []
        messagedata = None
        while True:
            m = mlog.recv_match()
            if m is None:
                break
            msg_type = m.get_type()
            if msg_type == "ISBH":
                if messagedata is not None:
                    if messagedata.sensor_type == sensor_type and messagedata.instance == sensor_instance and messagedata.sample_us > since and messagedata.sample_us < until:
                        messages.append(messagedata)
                messagedata = MessageData(m)
                continue

            if msg_type == "ISBD":
                if messagedata is not None and messagedata.sensor_type == sensor_type and messagedata.instance == sensor_instance:
                    messagedata.add_fftd(m)

        fft_len = len(messages[0].data["X"])
        sum_fft = {
                "X": numpy.zeros(int(fft_len/2+1)),
                "Y": numpy.zeros(int(fft_len/2+1)),
                "Z": numpy.zeros(int(fft_len/2+1)),
            }
        sample_rate = 0
        counts = 0
        window = numpy.hanning(fft_len)
        freqmap = numpy.fft.rfftfreq(fft_len, 1.0 / messages[0].sample_rate_hz)

        # calculate NEBW constant
        S2 = numpy.inner(window, window)

        for message in messages:
            for axis in [ "X","Y","Z" ]:
                # normalize data and convert to dps in order to produce more meaningful magnitudes
                if message.sensor_type == 1:
                    d = numpy.array(numpy.degrees(message.data[axis])) / float(message.multiplier)
                else:
                    d = numpy.array(message.data[axis]) / float(message.multiplier)

                # apply window to the input
                d *= window
                # perform RFFT
                d_fft = numpy.fft.rfft(d)
                # convert to squared complex magnitude
                d_fft = numpy.square(abs(d_fft))
                # remove DC component
                d_fft[0] = 0
                d_fft[-1] = 0
                # accumulate the sums
                sum_fft[axis] += d_fft

            sample_rate = message.sample_rate_hz
            counts += 1

        numpy.seterr(divide = 'ignore')
        psd = {}
        for axis in [ "X","Y","Z" ]:
            # normalize output to averaged PSD
            psd[axis] = 2 * (sum_fft[axis] / counts) / (sample_rate * S2)
            psd[axis] = 10 * numpy.log10 (psd[axis])

        psd["F"] = freqmap

        return psd
