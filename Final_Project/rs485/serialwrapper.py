import serial
import rs485
from typing import Callable

class SerialWrapper(rs485.RS485):
    '''Wrapper for using a pySerial object with RS485.'''

    def __init__(self, serial):
        self.serial = serial
        super(SerialWrapper, self).__init__(fWrite=self._write, fAvailable=self._available, fRead=self._read)

    def _write(self, aByte):
        return self.serial.write(chr(aByte))

    def _available(self):
        return self.serial.inWaiting()

    def _read(self):
        data = self.serial.read(1)
        return str(data[0])
