#!/usr/bin/python3
import serial, curses, time
from serial.serialutil import SerialException
from getch import getch, pause

def pbar (window):
    sample_delays = []
    sample_history = []

    while True:
        try:
            setupSerialAndLoop(window)
            break
        except SerialException as e:
            window.clear()
            window.addstr(0, 0, "Something went wrong with the serial connection: {}".format(str(e)))
            window.addstr(1, 0, "Press Enter to try again...")
            window.refresh()
            pause('')
            continue

def setupSerialAndLoop (window):
    with serial.Serial('/dev/ttyUSB1', 115200) as ser:
        while True:
            try:
                window.clear()
                window.addstr(0,0, "Waiting for start-of-message byte")
                readByteFromController(ser)
                window.refresh()
            except EndOfMessageException:
                break

        frame = 0
        while True:
            window.clear()
            try:
                emptymsg = readNByteBinaryString(ser, 4)
                buttonmsg = readNByteBinaryString(ser, 4)
                fullmsg = readNByteBinaryString(ser, 4)
            except EndOfMessageException:
                continue
            window.addstr(3, 3, " Empty msg: {}".format(emptymsg))
            window.addstr(4, 3, "Button msg: {}".format(buttonmsg))
            window.addstr(5, 3, "  Full msg: {}".format(fullmsg))
            window.addstr(7, 3, "Frame: {}".format(frame))
            window.refresh()

            frame += 1

def readNByteBinaryString(serial, n):
    input_bytes = bytearray()
    for i in range (0, n):
        input_bytes.append(readByteFromController(serial))

def numberArrayToBinaryString (arr):
    result = '0b'
    for i in range(0, len(arr)):
        result += format(arr[i], '08b')
    return result

def readByteFromController(serial):
    b = ord(serial.read())
    if b not in [255, 128]:
        return readByteFromController(serial)
    if b == 255:
        raise EndOfMessageException
    return ord(serial.read())

class EndOfMessageException(Exception):
    pass

curses.wrapper(pbar)
