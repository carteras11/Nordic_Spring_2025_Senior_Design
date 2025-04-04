#!/usr/bin/python
'''
    Log File Format:
    distance[m],RSSI[dBm],SNR[dB]
'''
import serial
import serial.tools.list_ports
import time
from datetime import datetime

DK_PORT_NAME = '/dev/ttyACM0'
DK_PORT_BAUDRATE = 115200

def port_exists(name : int) -> bool:
    available_ports = list(serial.tools.list_ports.comports())
    for port in available_ports:
        if port.device == name:
            return True
    return False

if __name__=="__main__":

    try:
        while True:
            print("Checking for DK at port " + DK_PORT_NAME + "...")
            while not port_exists(DK_PORT_NAME):
                time.sleep(1)
            print("Found DK at port " + DK_PORT_NAME)

            time.sleep(2) # let the serial port fully setup before attempting access
            ser = serial.Serial(DK_PORT_NAME,
                            DK_PORT_BAUDRATE)
            
            filename = input("Enter the filename to write to: ")
            try:
                f = open(str(filename), "a")
            except e:
                print('Exception:', e)

            date = datetime.now()
            f.write('Entries from ' + str(date) + '\r\n')

            while True:
                try:
                    line = ser.readline().decode('utf-8')
                except serial.SerialException as e:
                    print("Exception:", e)
                    ser.close()
                    # let the serial port fully close before trying to access again
                    time.sleep(3)
                    break

                distance = input("New RX, enter distance: ")
                try:
                    print('   Logging', line.strip(), 'at distance', distance)
                    f.write(str(distance) + ',' + line.strip() + '\r\n')
                except e:
                    print("Error writing to file", e);
            
            print("Serial connection closed")
            f.close()

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        f.write('\r\n')
        f.close()
        ser.close()
