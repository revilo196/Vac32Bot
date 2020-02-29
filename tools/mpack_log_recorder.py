"""
python tool for loggin the serial msgpack data from the robot
"""

import msgpack
import serial
import sys
import pandas as pd
import threading
from datetime import datetime


BAUD = 2000000
PORT = "COM11"


class LoggerThread(threading.Thread):
    """
    Serial Logger Thread
    logs all msgpack packages from a serial port
    can be interacted from the user
    """
    def __init__(self, port, baud):
        """
        :param port: Serial port name e.g COM11 or /dev/ttySC1
        :param baud: baud rate of the communication
        """
        threading.Thread.__init__(self)
        self.port = port
        self.baud = baud
        self.running = False
        self.pause = False
        self.printing = False

    def stop(self):
        """
        stop the logger
        this will stop the logging loop and save logged data
        """
        self.running = False
    
    def pause(self):
        """ pause the logging loop """
        self.pause = True

    def resume(self):
        """ continue the logging loop"""
        self.pause = False

    def toggle_print(self):
        """
        toggle printing the logging loop data to the console
        """
        self.printing = not self.printing

    def print(self, *args):
        if self.printing:
            print(args)

    def run(self):
        """ setup an run the logging loop"""
        self.running = True
        ser = serial.serial_for_url(self.port , do_not_open=True)
        ser.baudrate = self.baud
        now = datetime.now()
        dt_string = now.strftime("%d%m%Y%H%M%S")
        store = pd.HDFStore("tools/logs/vac_bot_log_" + dt_string + ".hdf")
        ser.timeout = 1
        try:
            ser.open()
        except serial.SerialException as e:
            sys.stderr.write('Could not open serial port {}: {}\n'.format(ser.name, e))
            sys.exit(1)

        log = {}

        print("logger started")

        while self.running:
            if self.pause:
                continue

            data = ser.read_until(b'\xffMP\xff')  # each msgpack starts with this 3 bytes

            if len(data) <= 4:
                continue

            data = data[:-4]
            try:
                if len(data)-2 == int(data[0]):
                    try:
                        pack = data[2:]
                        unpack = msgpack.unpackb(pack)

                        for k in unpack:
                            kd = k.decode("UTF-8") # decode bytes
                            package = unpack[k]  # log package is allways contains key-value pairs
                            package = dict(zip([k.decode("UTF-8") for k in package.keys()], list(package.values()))) #decode keys bytes

                            if kd not in log: # add new log stream if needed
                                log[kd] = []
                            self.print(package)
                            log[kd].append(package)

                        if b'logger' in unpack:
                            self.print(unpack)
                            self.running = False
                    except:
                        self.print(data)
                else:
                    self.print(data)
            except:
                self.print(data)
        
        for k in log:
            df = pd.DataFrame(log[k])
            store.put(k, df)


if __name__ == '__main__':
    # logger is running in background
    logger = LoggerThread(PORT, BAUD)
    logger.start()

    # user interface
    while True:
        input1 = input() 
        print(input1)

        if str(input1).upper() == 'E':
            logger.stop()
            logger.join()
            break

        if str(input1).upper() == 'S':
            logger.resume()

        if str(input1).upper() == 'P':
            logger.pause()

        if str(input1).upper() == 'V':
            logger.toggle_print()




