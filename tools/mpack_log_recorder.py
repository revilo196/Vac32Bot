"""
python tool for loggin the serial msgpack data from the robot
"""

import msgpack
import serial
import sys
import pandas as pd
import numpy as np
from datetime import datetime
from typing import Dict

BAUD = 2000000
PORT = "COM11"


if __name__ == '__main__':
    ser = serial.serial_for_url(PORT , do_not_open=True)
    ser.baudrate = BAUD
    now = datetime.now()
    dt_string = now.strftime("%d%m%Y%H%M%S")
    store = pd.HDFStore("tools/logs/vac_bot_log_" + dt_string + ".hdf")

    try:
        ser.open()
    except serial.SerialException as e:
        sys.stderr.write('Could not open serial port {}: {}\n'.format(ser.name, e))
        sys.exit(1)

    log = {}

    running = True
    while running:
        data = ser.read_until(b'\xffMP\xff')
        data = data[:-4]
        try:
            if len(data)-2 == int(data[0]):
                try:
                    pack = data[2:]
                    unpack = msgpack.unpackb(pack)

                    for k in unpack:
                        kd = k.decode("UTF-8") #decode bytes
                        package = unpack[k]  # log package is allways contains key-value pairs
                        package = dict(zip([k.decode("UTF-8") for k in package.keys()], list(package.values()))) #decode keys bytes

                        if kd not in log: # add new log stream if needed
                            log[kd] = []
                        print(package)
                        log[kd].append(package)

                    if b'logger' in unpack:
                        print(unpack)
                        running = False
                except:
                    print(data)
            else:
                print(data)
        except:
             print(data)
    
    for k in log:
        df = pd.DataFrame(log[k])
        store.put(k, df)

