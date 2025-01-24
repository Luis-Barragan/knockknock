<<<<<<< HEAD
#!/usr/bin/python
## Serial Port Logger Application

# We can't do any thing without Serial
try:
    import serial
except (ImportError):
    msg = """ERROR: pyserial library not found
    Install pyserial library
    pip install pyserial"""
    print(msg)
    exit(1)

# Other Imports
import logging, time
from signal import signal, SIGINT
from sys import exit
from queue import Queue,Empty

# Port Configuration
PORT = "COM3"
BAUD = 115200
logFileName='serial.log'
q = Queue(2)

def setupLogger(filename):    
    logger = logging.getLogger('Serial Logger')
    logger.setLevel(logging.DEBUG)
    # create file handler which logs even debug messages
    fh = logging.FileHandler(filename)
    fh.setLevel(logging.DEBUG)
    # create console handler with a higher log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s.%(msecs)03d, %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    # add the handlers to the logger
    logger.addHandler(fh)
    logger.addHandler(ch)
    # Return the Created logger
    return logger

def main(qSignal):
    ## Logger Configuration
    global logFileName
    log = setupLogger(logFileName)
    ## Begin
    log.info("Program Started")
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        log.error("Got Fatal error - {}".format(e))
        exit(4)
    # Loop for Reception
    while 1:
        ## Ctrl+C signal
        try:
            squit = qSignal.get(block=False, timeout=0.1)
        except Empty as e:
            squit = False            
        if squit == True:
            log.info("Exiting")
            ser.close()
            exit(0)
        # Get Data
        try:
            data = ser.readline().decode('utf-8').strip()
            if data.startswith("b'"):
                data = data[2:-1]
            if len(data) > 0:
                log.info(data)
        except KeyboardInterrupt as e:
            q.put(True)
            log.info("Ctrl + C pressed")
    
    
def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    q.put(True)
    

if __name__ == "__main__":
    signal(SIGINT, handler)
    print('Running. Press CTRL-C to exit.')
=======
#!/usr/bin/python
## Serial Port Logger Application

# We can't do any thing without Serial
try:
    import serial
except (ImportError):
    msg = """ERROR: pyserial library not found
    Install pyserial library
    pip install pyserial"""
    print(msg)
    exit(1)

# Other Imports
import logging, time
from signal import signal, SIGINT
from sys import exit
from queue import Queue,Empty

# Port Configuration
PORT = "COM3"
BAUD = 115200
logFileName='serial.log'
q = Queue(2)

def setupLogger(filename):    
    logger = logging.getLogger('Serial Logger')
    logger.setLevel(logging.DEBUG)
    # create file handler which logs even debug messages
    fh = logging.FileHandler(filename)
    fh.setLevel(logging.DEBUG)
    # create console handler with a higher log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # create formatter and add it to the handlers
    formatter = logging.Formatter('%(asctime)s.%(msecs)03d, %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    # add the handlers to the logger
    logger.addHandler(fh)
    logger.addHandler(ch)
    # Return the Created logger
    return logger

def main(qSignal):
    ## Logger Configuration
    global logFileName
    log = setupLogger(logFileName)
    ## Begin
    log.info("Program Started")
    ser = None
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        log.error("Got Fatal error - {}".format(e))
        exit(4)
    # Loop for Reception
    while 1:
        ## Ctrl+C signal
        try:
            squit = qSignal.get(block=False, timeout=0.1)
        except Empty as e:
            squit = False            
        if squit == True:
            log.info("Exiting")
            ser.close()
            exit(0)
        # Get Data
        try:
            data = ser.readline().decode('utf-8').strip()
            if data.startswith("b'"):
                data = data[2:-1]
            if len(data) > 0:
                log.info(data)
        except KeyboardInterrupt as e:
            q.put(True)
            log.info("Ctrl + C pressed")
    
    
def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    q.put(True)
    

if __name__ == "__main__":
    signal(SIGINT, handler)
    print('Running. Press CTRL-C to exit.')
>>>>>>> master
    main(q)    