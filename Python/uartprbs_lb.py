import logging
import serial
import time

###################################################################################


class CommUART(object):
    def __init__(self, address):
        self.address = address
        self.sc = None

    def connect(self):
        logging.debug("Opening COM port : {0}".format(self.address))
        self.sc = None
        while self.sc is None:
            try:
                self.sc = serial.Serial(port=self.address, baudrate=576000, rtscts=True)
            except serial.serialutil.SerialException as se:
                if 'Device or resource busy:' in se.__str__():
                    logging.info('Opening COM port is taking a little while, please stand by...')
                else:
                    logging.error('se: {0}'.format(se))
                time.sleep(1)
        logging.debug("COM port open successfully.")
        self.sc.flushInput()

    def disconnect(self):
        logging.debug("Closing COM port : {0}".format(self.address))
        self.sc.close()

    def receivedPacket(self, length):
        if self.sc is None:
            raise Exception('COM port is not opened.')
        packet = b''
        received = 0
        while received < length:
            serialByte = self.sc.read(1)
            if serialByte is None:
                raise Exception('Bad character.')
            elif len(serialByte) == 0:
                break
            elif received < length:
                received += 1
                packet += serialByte
        return packet
    
    def send(self, data):
        self.sc.write(bytes([data]))

    def prbs8(self, curval):
        newbit = (((curval >> 6) ^ (curval >> 5)) & 1)
        return ((curval << 1) | newbit) & 0x7f
###################################################################################

def main():
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s : %(message)s')

    #comm = CommUART("/dev/cu.usbserial-A8UKQC7S")
    comm = CommUART("/dev/cu.usbmodem1422")
    comm.connect()

    curval = 0
    #packet = comm.receivedPacket(1)
    #curval = int.from_bytes(packet, byteorder = 'little')
    val = comm.prbs8(0xff)
    byteCount = 0
    dropcnt = 0
    deltatime = 0
    drop = False
    while True:
        try:
            comm.send(val)
            startTime = time.time()
            packet = comm.receivedPacket(1)
            endTime = time.time()
            deltatime += endTime - startTime
            curval = int.from_bytes(packet, byteorder = 'little')
            if curval != val:
                dropcnt += 1
            val = comm.prbs8(curval)
            byteCount += 1

            bytesPerSec = byteCount / deltatime #(endTime - startTime)

            #print("Bytes : {0}".format(bytes))
            #if drop:
            #    print("Dropped.... Bytes/sec : {0}".format(bytesPerSec))
            #else:
            if (byteCount & 0xff) == 0:
                print("Bytes/sec : %.2f, drop %d " %(bytesPerSec, dropcnt))
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Exiting.")
            break

    comm.disconnect()

###################################################################################


if __name__ == '__main__':
    main()
