import platform
import glob
import serial
import time

def find_ports(allPorts):
    '''
    Finds names of all ports with RTBoxes connected and returns the port name
    along with the version number

    Params:
    allPorts (bool) - If set to True, this function will close any opened RTBox
                      and return all RTBox ports.

    Returns:
    valid_ports (list of 2 element tuples) - List of (port_name, version) for
                                             each valid RTBox port.
    '''
    if allPorts:
        ### CLOSE ALL RTBoxes
        pass
    system = platform.system()
    if system == 'Windows':
        ports = ['\\\\.\\COM%i'%i for i in range(1,257)]
    elif system == 'Darwin':
        ports = glob.glob('/dev/cu.usbserialRTBox*')
        if not ports:
            ports = glob.glob('/dev/cu.usbserial*')
    elif system == 'Linux':
        ports = glob.glob('/dev/ttyUSB*')
    else:
        raise('System not supported')
    valid_ports = []
    for port in ports:
        try:
            ser = serial.Serial(port=port, baudrate=115200, timeout=0.3)
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
            valid_ports.append((port, ver))

    return valid_ports