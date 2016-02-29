from RTBox_utils import find_ports
from uuid import getnode as get_mac
from psychopy import core
import serial
import platform
import os
import struct
import time
import warnings
import numpy as np


class RTBox(object):
    """
    This class serves as a handler for all RTBox operations. If initialized
    with a port, it will attempt to connect to that port only. If initialized
    without one, it will connect to the first RTBox it finds from the
    find_ports method.
    """
    def __init__(self, port=None):
        super(RTBox, self).__init__()
        ports = find_ports(port)
        if not ports:
            raise Exception('Unable to connect to RTBox')
        else:
            self.port, self.ver = ports[0]
        self.ser = serial.Serial(port=self.port, baudrate=115200,
                                 timeout=1.0)
        self.info = {}
        self.info['events'] = ['1', '2', '3', '4']
        self.info['clkRatio'] = 1.0
        self.info['clockUnit'] = 1.0/115200
        self.info['enabled'] = np.array([True, False, False, False, False, False])
        self.info['tpreOffset'] = [9.2306e-5, 9.2306e-5]

        if self.ver >= 5.0:
            self.info['tpreOffset'][1] += 10.0/115200

        if self.ver > 4.3:
            b = self._readEEPROM(224, 6)
            self.info['TTLWidth'] = (255 - b[0]) / 7200.0
            self.info['TTLresting'] = get_bits(b[1], range(0, 2))
            self.info['threshold'] = np.array(get_bits(b[1], [3, 6])).dot([1, 2]) + 1
            self.info['debounceInterval'] = tuple((256**i * b[i + 2]) / 921600. for i in range(0, 4))
        else:
            raise NotImplementedError('Older RTBox not supported currently')
        mac = "%0.2X" % get_mac()
        self.info['MAC'] = [0] + [int(mac[i:i+2], 16) for i in range(0, len(mac), 2)]
        for i in range(16):
            b14 = self._readEEPROM(i*14, 14)
            if np.equal(b14[:6], 255).all():  # EEPROM not written
                break
            if np.equal(b14[8:14], self.info['MAC'][1:7]).all():
                break
        if i == 15:
            i = 0
        self.info['MAC'][0] = i*14
        if not np.equal(np.diff(b14[:6]), 0).all():
            ratio = np.array(b14[:8], dtype=np.uint8).view(np.float64)[0]
            if np.abs(ratio - 1) < 0.01:
                self.info['clkRatio'] = ratio
        if self.info['clkRatio'] == 1:
            warnings.warn('Clock ratio not corrected. Run RTBox.set_clock() before timing responses.')

    def close(self):
        """
        Closes the connection to the RTBox. Call this before terminating your
        program.
        """
        self.ser.close

    def set_clock(self, trials=30):
        """
        Measures and sets clock ratio. This only needs to be run once per
        machine per RTBox.
        """
        interval = 1  # interval between trials
        ntrial = max(10, trials/interval)  # at least 10 trials
        print 'Measuring clock ratio. ESC to stop. Trials remaining:%4.i' % ntrial
        self._enable_byte(0)  # disable all
        t = np.zeros((ntrial, 2))
        for i in range(ntrial):
            if ntrial >= 20 and self.info['clkRatio'] == 1.0:
                if i < 10:
                    self._syncClocks(10)
                    print '\b\b\b\b%4.i' % ntrial - i
                    t[i, :] = self.info['sync'][:2]
                    core.wait(interval)

    def _enable_byte(self, enByte):
        if self.ver < 4.1:
            raise NotImplementedError('Older RTBox not supported currently')
        else:
            for i in range(4):
                self._purge()
                self.ser.write('e' + struct.pack('!B', enByte))
                if self.ser.read(1) == 'e':
                    break
                if i == 3:
                    raise Exception('RTBox not responding')

    def _syncClocks(self, nr, enableInd=None):
        if np.any(self.info['enabled']):
            self._enable_byte(0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        t = np.zeros(nr, 3)
        for i in range(nr):
            for iTry in range(4):
                time.sleep(np.random.rand()/1000)
                t[i, 0] = core.getTime()
                self.ser.write('Y')
                t[i, 1] = core.getTime()
                b7 = unpack_bytes(self.ser.read(7))
                if len(b7) == 7 and b7[0] == 89:
                    break
                if i == 3:
                    raise Exception('RTBox not responding')
            t[i, 2] = self._bytes2secs(b7[1:])
        t[:, 0] += self.info['tpreOffset'][0]
        tdiff = (t[:, 0] - t[:, 2]).max()
        i = (t[:, 0] - t[:, 2]).argmax()
        twin = t[i, 1] - t[i, 0]
        tbox = t[i, 2]
        tdiff_ub = (t[:, 1] - t[:, 2]).min() - tdiff
        i = (t[:, 1] - t[:, 2]).argmin()
        method3_1 = t[i, 0:2].mean() - t[i, 2] - tdiff
        self.info.sync = [tdiff, tbox, tdiff_ub, method3_1]
        if twin > 0.003 and tdiff_ub > 0.001:
            warnings.warn("USB Overload")
        if enableInd is not None:
            foo = (2**np.arange(6)[enableInd]).dot(
                   self.info['enabled'][enableInd])
            self._enable_byte(foo)  # restore enable

    def _bytes2secs(self, b6, ratio=None):
        if ratio is None:
            ratio = self.info['clkRatio']
        return (256**np.arange(5, -1, -1)).dot(b6) * self.info['clockUnit'] * ratio

    def _purge(self):
        byte = self.ser.inWaiting()
        tout = core.getTime() + 1
        # check to make sure RTBox is idle
        while True:
            time.sleep(0.001)
            byte1 = self.ser.inWaiting()
            if byte1 == byte:
                break
            if core.getTime() > tout:
                raise Exception('RTBox not responding')
            byte = byte1
        # purge buffers
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def _readEEPROM(self, addr, n_bytes):
        self.ser.write(struct.pack('!B', 17))
        self.ser.write(struct.pack('!B', addr) + struct.pack('!B', n_bytes))
        time.sleep(0.01)
        return unpack_bytes(self.ser.read(n_bytes))

    def _get_latency(self, msecs):
        system = platform.system()
        if system == 'Darwin':
            folder = '/System/Library/Extensions'
            mac_ver = int(platform.mac_ver()[0].split('.')[1])
            if mac_ver >= 9:
                vendor = '/AppleUSBFTDI.kext'
            else:
                vendor = '/FTDIUSBSerialDriver.kext'
            fname = os.path.join(folder, vendor, '/Contents/Info.plist')
            with open(fname) as f:
                contents = f.read()
            ind = contents.find('<key>FTDI2XXB')
            if ind < 0:
                ind = contents.find('<key>FT2XXB')
                if ind < 0:
                    return (None, 'Failed to detect product key')

        elif system == 'Windows':
            raise NotImplementedError('Windows not supported yet')
        elif system == 'Linux':
            raise NotImplementedError('Linux not supported yet')
        else:
            raise('System not recognized')


def get_bits(byteval, locs):
    """
    From: http://stackoverflow.com/questions/2591483/getting-a-specific-bit-value-in-a-byte-string
    """
    return tuple(((byteval & (1 << i)) != 0) for i in locs)


def unpack_bytes(b_str):
    """
    Takes a byte string and unpacks into tuple of ints
    """
    return tuple(struct.unpack('!B', i)[0] for i in b_str)
