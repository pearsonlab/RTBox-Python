from RTBox_utils import find_ports
from uuid import getnode as get_mac
from psychopy import core
from scipy import stats
import serial
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

    EVENT_TYPE_BIT_ORDER = {
        'press': 0,
        'release': 1,
        'sound': 2,
        'light': 4,
        'tr': 5,
        'aux': 6
    }

    BUTTON_DOWN_EVENTS = ['1_down', '2_down', '3_down', '4_down']
    BUTTON_UP_EVENTS = ['1_up', '2_up', '3_up', '4_up']
    BUTTON_EVENTS = BUTTON_DOWN_EVENTS + BUTTON_UP_EVENTS
    OTHER_EVENTS_WO_SERIAL = ['sound', 'light', 'tr', 'aux']
    EVENTS = BUTTON_EVENTS + OTHER_EVENTS_WO_SERIAL + ['serial']

    EVENT_CODES = {
        49: '1_down',
        51: '2_down',
        53: '3_down',
        55: '4_down',
        50: '1_up',
        52: '2_up',
        54: '3_up',
        56: '4_up',
        97: 'sound',
        48: 'light',
        57: 'tr',
        98: 'aux',
        89: 'serial'
    }

    def __init__(self, port=None):
        """
        Construct a new RTBox object.  Will call warning to run set_clock if RTBox does not have clock ratio between
        itself and the computer set in memory.

        :param port: If initialized, will attempt to connect to specified port
        :return: Nothing
        """

        super(RTBox, self).__init__()

        ports = find_ports(port)
        if not ports:
            raise Exception('Unable to connect to RTBox')
        else:
            self.port, self.ver = ports[0]
        self.ser = serial.Serial(port=self.port, baudrate=115200,
                                 timeout=1.0)
        self.clockRatio = 1.0
        self.clockRatioLoaded = False
        self.clockUnit = 1.0/115200
        self.enabled_event_types = {
            'press': True,
            'release': False,
            'sound': False,
            'light': False,
            'tr': False,
            'aux': False
        }
        self.tpreOffset = [9.2306e-5, 9.2306e-5]
        self.latency_timer = 1.0
        self.num_events_to_read = 1

        if self.ver >= 5.0:
            self.tpreOffset[1] += 10.0/115200

        if self.ver > 4.3:
            b = self._readEEPROM(224, 6)
            self.TTLWidth = (255 - b[0]) / 7200.0
            self.TTLresting = get_bits(b[1], range(0, 2))
            self.threshold = np.array(get_bits(b[1], [3, 6])).dot([1, 2]) + 1
            self.debounceInterval = tuple((256**i * b[i + 2]) / 921600. for i in range(0, 4))
        else:
            raise NotImplementedError('Older RTBox not supported currently')

        mac = "%0.2X" % get_mac()
        self.MAC = [int(mac[i:i+2], 16) for i in range(0, len(mac), 2)]

        # default eeprom address index is 0
        eeprom_addr_ind = 0
        eeprom_addr_found = False
        # Try to find eeprom address with corresponding MAC address
        for i in range(16):
            b14 = self._readEEPROM(i*14, 14)
            if np.equal(b14[8:14], self.MAC).all():
                # set eeprom address index
                eeprom_addr_ind = i
                eeprom_addr_found = True
                # set clock ratio
                self.clockRatio = np.array(b14[:8], dtype=np.uint8).view(np.float64)[0]
                self.clockRatioLoaded = True
                break
        # If fail, try to find an empty slot
        if not eeprom_addr_found:
            print 'MAC not found'
            for i in range(16):
                b14 = self._readEEPROM(i*14, 14)
                if np.equal(b14[:6], 255).all():  # EEPROM not written
                    eeprom_addr_ind = i
                    break

        self.eeprom_addr = eeprom_addr_ind*14

        if not self.clockRatioLoaded:
            warnings.warn('Clock ratio not corrected. Run RTBox.set_clock() before timing responses.')

    def close(self):
        """
        Closes the connection to the RTBox. Call this before terminating your program.

        :return: nothing
        """
        self.ser.close()

    def get_clock_ratio(self):
        """
        Returns current clock ratio variable

        :return: current clock ratio
        """

        return self.clockRatio

    def get_t_diff(self):
        """
        Returns current t_diff variable (in computer time (secs))

        :return: current t_diff
        """

        return self.t_diff

    def prep(self, samples=20):
        """
        Sets t_diff to current difference between computer time and the RTBox time.  Run every time RTBox is plugged in
        to the USB port.  Make sure that set_clock has been run at least once between the computer and the RTBox, and
        the RTBox memory has the clock ratio.

        :return: nothing
        """

        [t_diff, t_receive] = self._syncClocks(samples)
        self.t_diff = t_diff
        # self.t_receive = t_receive ??
        # if self.enabled_events['light'] or self.enabled_events['tr'] or self.enabled_events['aux']: ??
        #     self._update_enabled_events() ??
        self._purge()

    def read(self, secs=0.1):
        """
        Read all enabled input responses from the time read is called for time specified in secs.

        :param secs: length of time to read in seconds

        :return: Returns a list [events, event_times].  events contains a list of the string label of events detected.
        event_times returns a list of times the events were recorded relative to the psychopy core module.  Element in
        index i for events and event_times refer to the same event.
        """
        self._purge()
        # Wait for time
        curr_time = core.getTime()
        timeout = curr_time + secs

        is_reading = False
        num_bytes_prev = self.ser.inWaiting()
        while curr_time < timeout or is_reading:
            time.sleep(self.latency_timer/1000)
            curr_time = core.getTime()
            num_bytes_curr = self.ser.inWaiting()
            # Wait if reading
            if num_bytes_curr > num_bytes_prev:
                is_reading = True
            else:
                is_reading = False
            num_bytes_prev = num_bytes_curr
        num_events = num_bytes_curr / 7
        # Each event contains 7 bytes
        events_bytes = self.ser.read(num_events * 7)
        events = []
        event_times = []
        for i in range(0, num_events):
            events.append(RTBox.EVENT_CODES[unpack_bytes(events_bytes[i*7])[0]])
            event_times.append(self._bytes2secs(unpack_bytes(events_bytes[i*7 + 1: (i+1)*7])) + self.t_diff)
        return [events, event_times]

    def wait_press(self, secs=0.1):
        """
        Wait for input up to the time specified in secs.

        :param secs: length of max time to read in seconds

        :return: Returns a tuple (event, event_time).  'event' is the string label of the event detected.
        'event_time' is the timestamp relative to the psychopy core module.
        """
        self._purge()
        # Wait for time
        curr_time = core.getTime()
        timeout = curr_time + secs

        is_reading = False
        num_bytes_prev = self.ser.inWaiting()
        while curr_time < timeout or is_reading:
            time.sleep(self.latency_timer/1000)
            curr_time = core.getTime()
            num_bytes_curr = self.ser.inWaiting()
            # Wait if reading
            if num_bytes_curr > num_bytes_prev:
                is_reading = True
            else:
                if is_reading:  # if finished reading one event
                    break
                is_reading = False
            num_bytes_prev = num_bytes_curr
        num_events = num_bytes_curr / 7
        # Each event contains 7 bytes
        events_bytes = self.ser.read(num_events * 7)
        if len(events_bytes) > 0:
            event = RTBox.EVENT_CODES[unpack_bytes(events_bytes[0])[0]]
            event_time = self._bytes2secs(unpack_bytes(events_bytes[1:7])) + self.t_diff
        else:
            event = None
            event_time = core.getTime()
        return (event, event_time)

    def set_clock(self, samples_per_trial=10, trials_per_iteration=20, iterations=3, interval=1):
        """
        Measures and sets clock ratio, stores clock ratio in RTBox memory. This only needs to be run once for each
        computer and RTBox pair, but make sure to check that RTBox still has clock ratio stored in memory (__init__ will
        warn to run set_clock if the clock ratio is not in memory.  The default parameters in general should not be
        modified.

        :param samples_per_trial: number of samples t_receive and t_diff to obtain from _syncClocks for a trial
        :param trials_per_iteration: number of t_receive and t_diff pairs to obtain for linear regression in an iteration
        :param iterations: number of times to calculate clock ratio to converge to actual clock ratio

        :return: nothing
        """

        if trials_per_iteration < 1 or iterations < 1:
            raise Exception('trials_per_iteration and iterations need to be positive integers')
        if samples_per_trial < 10 or trials_per_iteration < 10 or iterations < 3:
            warnings.warn('Recommend samples_per_trial >= 10, trials_per_iteration >= 10, and iterations >= 3')
        if interval < 1:
            warnings.warn('Recommend interval time = 1s')

        print 'Measuring clock ratio. Progress:\n'

        for iteration in range(0, iterations):
            results = np.zeros((trials_per_iteration, 2))
            # conduct trials
            for i in range(trials_per_iteration):
                sync_stats = self._syncClocks(samples_per_trial)
                results[i, :] = sync_stats[:2]
                print '%d / %d trials' % (i + trials_per_iteration*iteration, trials_per_iteration*iterations)
                core.wait(interval)
            # get new ratio
            t_receive = results[:,1]
            t_diff = results[:,0]
            [slope, intercept, r_value, p_value, std_error] = \
                stats.linregress(t_receive, t_diff)
            self.clockRatio = self.clockRatio * (1 + slope)
            if (i+1 == trials_per_iteration):
                slope_final = slope
                std_error_final = std_error

        print '\n'
        print 'Clock ratio (computer/box): %.8f += %.8f\n' % (self.clockRatio, std_error_final)

        if std_error_final > 1e-4:
            ratioBigSE_warning = 'The slope std error is large (%2g).  Try longer time for clock ratio' % std_error_final
            warnings.warn(ratioBigSE_warning)
        if abs(slope_final) > 0.01:
            self.clockRatio = 1.0;
            ratioErr = 'The clock ratio differenece is very high (%2g).  ' % slope_final
            ratioErr = ratioErr + 'Your computer timing probably has a problem?'
            raise Exception(ratioErr)

        if self.ver < 4.2:
            raise NotImplementedError('Older RTBox not supported currently')
        else:
            # Store ratio in EEPROM
            print 'Storing clock ratio (' + str(self.clockRatio) + ')'
            ratio_nparray = np.array([self.clockRatio], dtype=np.float64)
            ratio_bytes = ratio_nparray.view(np.uint8)
            mac_info_array = np.array(self.MAC, dtype=np.uint8)
            eeprom_data = np.concatenate([ratio_bytes, mac_info_array])
            self._writeEEPROM(self.eeprom_addr, eeprom_data)

    def enable_event_types(self, event_types):
        """
        Enable event types specified in string list events_to_enable.  Valid event types are: press, release,
        sound, pulse, light, tr, aux

        :param event_types: A list typed in string format of event types to be enabled.

        :return: nothing
        """
        self._enable_disable_events(event_types, True)

    def disable_event_types(self, event_types):
        """
        Disable event types specified in string list events_to_enable.  Valid event types are: press, release,
        sound, pulse, light, tr, aux

        :param event_types: A list typed in string format of event types to be disabled.

        :return: nothing
        """

        self._enable_disable_events(event_types, False)

    def _enable_disable_events(self, event_types, enable_disable):
        # Check for valid input
        for event_type in event_types:
            if type(event_type) is not 'str':
                raise Exception('Events need to be in string format')
            if event_type.lower() == 'all':
                events = list(self.enabled_event_types.keys())
            if event_type.lower() not in self.enabled_event_types.keys():
                raise Exception('Events needs to be one of the following:\n'
                                'press, release, sound, pulse, light, tr, aux\n'
                                'In addition, input \'all\' will enable all of the events above')

        for event_type in event_types:
            self.enabled_event_types[event_type] = enable_disable

        self._update_enabled_event_types()

    def _update_enabled_event_types(self):
        byte_int_to_send = self._get_event_type_byte_string()
        self._enable_byte(byte_int_to_send)

    def _get_event_type_byte_string(self):
        byteInt = 0
        for event_type in self.enabled_event_types:
            if self.enabled_event_types[event_type]:
                bit_index = RTBox.EVENT_TYPE_BIT_ORDER[event_type]
                byteInt += pow(2,bit_index)
        return byteInt

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
        # disable all inputs
        self._enable_byte(0)
        # clear serial buffers
        self._purgeBuffers()

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

        # reenable inputs
        self._update_enabled_event_types()

        return [t_diff_method1, t_receive_method1]

    def _bytes2secs(self, b6, ratio=None):
        if ratio is None:
            ratio = self.clockRatio
        return (256**np.arange(5, -1, -1)).dot(b6) * self.clockUnit * ratio

    def _purge(self):
        byte = self.ser.inWaiting()
        tout = core.getTime() + 1
        # check to make sure RTBox is idle
        while True:
            time.sleep(0.0001)
            byte1 = self.ser.inWaiting()
            if byte1 == byte:
                break
            if core.getTime() > tout:
                raise Exception('RTBox not responding')
            byte = byte1
        # purge buffers
        self._purgeBuffers()

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

    def _purgeBuffers(self):
        if (float(serial.VERSION) >= 3):
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        else:
            self.ser.flushInput()
            self.ser.flushOutput()

# From: http://stackoverflow.com/questions/2591483/getting-a-specific-bit-value-in-a-byte-string
def get_bits(byteval, locs):
    return tuple(((byteval & (1 << i)) != 0) for i in locs)

# Takes a tuple of ints and packs into byte string
def pack_bytes(byte_array):
    num_ints = len(byte_array)
    byte_string = ""
    for i in range(0, num_ints):
        byte_string += struct.pack('!B', byte_array[i])
    return byte_string

# Takes a byte string and unpacks into tuple of ints
def unpack_bytes(byte_string):
    return tuple(struct.unpack('!B', i)[0] for i in byte_string)


