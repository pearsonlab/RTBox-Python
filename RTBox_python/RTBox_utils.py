import platform
import glob
import serial
import time
import os


def find_ports(port=None):
    '''
    Finds names of all ports with RTBoxes connected and returns the port name
    along with the version number

    Params:
    port (str) - If this is set, the function verifies that the port is an
                 RTBox and it returns the port name with the RTBox version
                 number. If not set, the function will close all RTBoxes
                 and return a list of the ones available.

    Returns:
    valid_ports (list of 2 element tuples) - List of (port_name, version) for
                                             each valid RTBox port.
    '''
    valid_ports = []
    if port is None:
        ### CLOSE ALL RTBoxes
        system = platform.system()
        if system == 'Windows':
            ports = ['\\\\.\\COM%i'%i for i in range(1, 257)]
        elif system == 'Darwin':
            ports = glob.glob('/dev/cu.usbserialRTBox*')
            if not ports:
                ports = glob.glob('/dev/cu.usbserial*')
        elif system == 'Linux':
            ports = glob.glob('/dev/ttyUSB*')
        else:
            raise('System not supported')
    else:
        ports = [port]
    for p in ports:
        try:
            ser = serial.Serial(port=p, baudrate=115200, timeout=0.3)
        except OSError:  # could not connect as serial device
            continue
        ser.write(bytes('X'))
        idn = ser.read(21)
        if len(idn) > 0 and idn[0] == '?':
            ser.write('R')
            time.sleep(0.5)
            ser.write('X')
            idn = ser.read(21)
        ser.close()
        if 'USTCRTBOX' in idn:
            if len(idn) < 21:
                ver = -1.0
            else:
                ver = float(idn[18:])
                if ver > 10:
                    ver = ver/100
            valid_ports.append((p, ver))

    return valid_ports


def get_latency():
        system = platform.system()
        if system == 'Darwin':
            _get_latency_mac()
        elif system == 'Windows':
            raise NotImplementedError('Windows not supported yet')
        elif system == 'Linux':
            raise NotImplementedError('Linux not supported yet')
        else:
            raise('System not recognized')

# returns latency in ms as a float and error message
def _get_latency_mac():
    # find FTDI driver
    vendor = 'FTDIUSBSerialDriver.kext/'
    mac_ver = int(platform.mac_ver()[0].split('.')[1])
    if mac_ver >= 9:
        folder = '/Library/Extensions/'
    else:
        folder = '/System/Library/Extensions/'
    fname = os.path.join(folder, vendor, 'Contents/Info.plist')
    # find FTDI2XXB inside FTDI driver file
    try:
        f = open(fname)
        contents = f.read()
        f.close()
    except (OSError, IOError):
        return (None, 'Failed to find FTDI driver; check correct installation')
    ind = contents.find('<key>FTDI2XXB')
    if ind < 0:
        return (None, 'Failed to detect product key')
    # find first LatencyTimer key after FTDI2XXB key
    ind += contents[ind:].find('<key>LatencyTimer</key>')
    ind += contents[ind:].find('<integer>')
    # note indices for latency timer value
    i0 = ind + 9 # skip <integer>
    i1 = ind + contents[ind:].find('</integer>')
    latency = float(contents[i0:i1])
    return (latency, None)
