from RTBox_utils import find_ports
from RTBox_utils import get_latency
from uuid import getnode as get_mac
from psychopy import core
from scipy import stats
import serial
import struct
import time
import warnings
import numpy as np

"""
todo:
handle fake mode?
"""


class RTBox(object):
    """
    This class serves as a handler for all RTBox operations. If initialized
    with a port, it will attempt to connect to that port only. If initialized
    without one, it will connect to the first RTBox it finds from the
    find_ports method.
    """
    def __init__(self, port=None, test=False):
        super(RTBox, self).__init__()
        if (test):
            return
        ports = find_ports(port)
        if not ports:
            raise Exception('Unable to connect to RTBox')
        else:
            self.port, self.ver = ports[0]
        self.ser = serial.Serial(port=self.port, baudrate=115200,
                                 timeout=1.0)
        self.info = {}
        self.info['events'] = ['1', '2', '3', '4']
        self.info['clockRatio'] = 1.0
        self.info['clockRatioLoaded'] = False
        self.info['clockUnit'] = 1.0/115200
        self.info['enabled'] = np.array([True, False, False, False, False, False])
        self.info['tpreOffset'] = [9.2306e-5, 9.2306e-5]
        self.info['latencyTimer'] = get_latency();

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
        self.info['MAC'] = [int(mac[i:i+2], 16) for i in range(0, len(mac), 2)]

        eeprom_addr_ind = 0
        eeprom_addr_found = False
        # Try to find eeprom address with corresponding MAC address
        for i in range(16):
            b14 = self._readEEPROM(i*14, 14)
            if np.equal(b14[8:14], self.info['MAC']).all():
                eeprom_addr_found = True
                self.info['clockRatioLoaded'] = True
                break
        # If fail, try to find an empty slot
        if not eeprom_addr_found:
            for i in range(16):
                b14 = self._readEEPROM(i*14, 14)
                if np.equal(b14[:6], 255).all():  # EEPROM not written
                    eeprom_addr_ind = i
                    break
        self.info['eeprom_addr'] = eeprom_addr_ind*14

        if not np.equal(np.diff(b14[:6]), 0).all():
            ratio = np.array(b14[:8], dtype=np.uint8).view(np.float64)[0]
            if np.abs(ratio - 1) < 0.01:
                self.info['clockRatio'] = ratio
        if not self.info['clockRatioLoaded']:
            warnings.warn('Clock ratio not corrected. Run RTBox.set_clock() before timing responses.')

    def close(self):
        """
        Closes the connection to the RTBox. Call this before terminating your
        program.
        """
        self.ser.close

    def set_clock(self, trials_per_iteration=20, iterations=3, interval=1):
        """
        Measures and sets clock ratio. This only needs to be run once per
        machine per RTBox.
        The default parameters in general should be used and not modified.
        """

        if trials_per_iteration < 1 or iterations < 1:
            raise Exception('trials_per_iteration and iterations need to be positive integers')
        if trials_per_iteration < 10 or iterations < 3:
            warnings.warn('Recommend trials_per_iteration >= 10 and iterations >= 3')
        if interval < 1:
            warnings.warn('Recommend interval time = 1s')

        print 'Measuring clock ratio. Progress:\n'

        self._enable_byte(0)  # disable all

        for iteration in range(0, iterations):
            results = np.zeros((trials_per_iteration, 2))
            # conduct trials
            for i in range(trials_per_iteration):
                sync_stats = self._syncClocks(10)
                results[i, :] = sync_stats[:2]
                print '%d / %d trials' % (i + trials_per_iteration*iteration, trials_per_iteration*iterations)
                core.wait(interval)
            # get new ratio
            t_receive = results[:,1]
            t_diff = results[:,0]
            [slope, intercept, r_value, p_value, std_error] = \
                stats.linregress(t_receive, t_diff)
            self.info['clockRatio'] = self.info['clockRatio'] * (1 + slope)
            if (i+1 == trials_per_iteration):
                slope_final = slope
                std_error_final = std_error

        print '\n'
        print 'Clock ratio (computer/box): %.8f += %.8f\n' % (self.info['clockRatio'], std_error_final)

        if std_error_final > 1e-4:
            ratioBigSE_warning = 'The slope std error is large (%2g).  Try longer time for clock ratio' % std_error_final
            warnings.warn(ratioBigSE_warning)
        if abs(slope_final) > 0.01:
            self.info['clockRatio'] = 1.0;
            ratioErr = 'The clock ratio differenece is very high (%2g).  ' % slope_final
            ratioErr = ratioErr + 'Your computer timing probably has a problem?'
            raise Exception(ratioErr)

        if self.ver < 4.2:
            raise NotImplementedError('Older RTBox not supported currently')
        else:
            # Store ratio in EEPROM
            ratio_nparray = np.array([self.info['clkRatio']], dtype=np.float64)
            ratio_bytes = ratio_nparray.view(np.uint8)
            mac_info_array = np.array(self.info['MAC'], dtype=np.uint8)
            eeprom_data = np.concatenate([ratio_bytes, mac_info_array])
            self._writeEEPROM(self.info['eeprom_addr'], eeprom_data)

    def get_button_names(self):
        return self.info['events']

    def set_button_names(self, names):
        if len(names) is not 4:
            raise Exception('Array needs to be length 4; need to set names for 4 buttons')
        for name in names:
            if type(name) is not str:
                raise Exception('Names need to be strings')
        self.info['events'] = names

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

    def _syncClocks(self, num_samples, enableInd=None):
        if np.any(self.info['enabled']):
            self._enable_byte(0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        t_pre = np.zeros(num_samples)
        t_post = np.zeros(num_samples)
        t_receive = np.zeros(num_samples)
        for i in range(num_samples):
            # try to get correct response in 4 tries
            for tries in range(4):
                # 0~1 ms random interval
                time.sleep(np.random.rand()/1000)
                t_pre[i] = core.getTime()
                # send signal to obtain t_receive
                self.ser.write('Y')
                t_post[i] = core.getTime()
                b7 = unpack_bytes(self.ser.read(7))
                if len(b7) == 7 and b7[0] == 89:
                    break
                if tries == 3:
                    raise Exception('RTBox not responding')
            t_receive[i] = self._bytes2secs(b7[1:])
        # t_pre += self.info['tpreOffset'][0] ???
        # the latest tpre is the closest to real write
        t_diff_method1 = (t_pre - t_receive).max()
        i_method1 = (t_pre - t_receive).argmax()
        # tpost-tpre for the selected sample: upper bound
        twin = t_post[i_method1] - t_pre[i_method1]
        # used for linear fit and time check
        t_receive_method1 = t_receive[i_method1]
        t_diff_method2 = (t_post - t_receive).min()
        # earliest tpost - lastest tpre
        tdiff_ub = t_diff_method2 - t_diff_method1

        """
        #ok find minwin index
        i_method2 = (t_post - t_receive).argmin()
        # diff between methods 3 and 1
        pre_post_mean = np.mean(np.concatenate((t_pre, t_post), axis=0), axis=0)
        # t_diff_method3 = pre_post_mean - t_receive[i_method2] ????
        # should actually be...
        t_diff_method3 = (pre_post_mean - t_receive).min()
        method3_1 = t_diff_method3 - t_diff_method1
        """

        if twin > 0.003 and tdiff_ub > 0.001:
            warnings.warn("USB Overload")
        if enableInd is not None:
            foo = (2**np.arange(6)[enableInd]).dot(
                   self.info['enabled'][enableInd])
            self._enable_byte(foo)  # restore enable
        return [t_diff_method1, t_receive_method1]

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
        self.ser.write(pack_bytes([17]))
        self.ser.write(pack_bytes([addr, n_bytes]))
        time.sleep(0.01)
        return unpack_bytes(self.ser.read(n_bytes))

    def _writeEEPROM(self, addr, bytes):
        n_bytes = len(bytes)
        self.ser.write(pack_bytes([16]))
        self.ser.write(pack_bytes([addr, n_bytes]))
        data_string = pack_bytes(bytes)
        self.ser.write(data_string)


def get_bits(byteval, locs):
    """
    From: http://stackoverflow.com/questions/2591483/getting-a-specific-bit-value-in-a-byte-string
    """
    return tuple(((byteval & (1 << i)) != 0) for i in locs)

def pack_bytes(byte_array):
    """
    Takes a tuple of ints and packs into byte string
    """
    num_ints = len(byte_array)
    byte_string = ""
    for i in range(0, num_ints):
        byte_string += struct.pack('!B', byte_array[i])
    return byte_string

def unpack_bytes(byte_string):
    """
    Takes a byte string and unpacks into tuple of ints
    """
    return tuple(struct.unpack('!B', i)[0] for i in byte_string)


