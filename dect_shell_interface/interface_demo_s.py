#!/usr/bin/python
'''
    Log File Format:

    NOTES
        - Do separate ping test and perf test
    

    On Server Side (stationary):
    Begin pinging (dect ping -s)

    On Client Side
    Loop
        Wait for confirmation
        Wait for new GPS coordinate read

        dect ping -c 
        Read resulting data
        Log data along with the current GPS position

        


'''
import serial
import serial.tools.list_ports
import time
from datetime import datetime
import pynmea2


DK_PORT_NAME = '/dev/ttyACM0'
DK_PORT_BAUDRATE = 115200


TX_PWR = str("10")
TX_MCS = str("4")


def port_exists(name : str) -> bool:
    available_ports = list(serial.tools.list_ports.comports())
    for port in available_ports:
        if port.device == name:
            return True
    return False

# automatically adds the leading 0x03 and trailing /n needed
def desh_msg(msg : str) -> str:
    return (chr(3)+msg+"\n").encode('utf-8')


if __name__=="__main__":

    try:
        while True:

            print("Checking for DK at port " + DK_PORT_NAME + "...")
            while not port_exists(DK_PORT_NAME):
                time.sleep(1)
            print("Found DK at port " + DK_PORT_NAME)

            time.sleep(1) # let the serial ports fully setup before attempting access
            
            dk_ser = serial.Serial(DK_PORT_NAME, DK_PORT_BAUDRATE, timeout=1)
            print("Set up PySerial ports")

            time.sleep(1)
        
            dk_ser.write(desh_msg("dect sett --reset"))
            time.sleep(0.5)
            dk_ser.write(desh_msg("dect sett --tx_pwr -10"))
            time.sleep(0.5)
            dk_ser.write(desh_msg("dect sett -b 9"))
            time.sleep(0.5)
            dk_ser.write(desh_msg("dect sett -t 39"))
            time.sleep(0.5)
            dk_ser.write(desh_msg("dect ping stop"))
            time.sleep(0.5)
            dk_ser.write(desh_msg("dect perf stop"))
            time.sleep(0.5)
            print("Configured dect shell")



            while True:
                # Wait for user to trigger new read
                #input(">>> Press ENTER to begin test")
                print(">>> STARTING NEW SERVERS")
                for mcs in range(0,5):
                    #input(">>> Press ENTER to begin test")

                    print(">>> MCS" + str(mcs))

                    print("Starting ping server...",end=" ")
                    dk_ser.write(desh_msg("dect ping -s --channel 1711"))

                    resp = ""
                    while "PING_RESULT_RESP sent" not in resp:
                        resp = dk_ser.readline().decode('UTF-8')
                        if len(resp) > 0: print(resp.strip())

                    print("ping server done")

                    # Stop prev server
                    dk_ser.write(desh_msg("dect ping stop"))
                    time.sleep(0.5)
                    
                    print("Starting perf server...",end=" ")
                    dk_ser.write(desh_msg("dect perf -s -t -1 --channel 1711"))

                    start = time.perf_counter()
                    resp = ""
                    while "RESULT_RESP sent" not in resp:
                        resp = dk_ser.readline().decode('UTF-8')
                        if len(resp) > 0: print(resp.strip())
                        current_time = time.perf_counter()
                        if (current_time - start > 10):
                            break;


                    time.sleep(1.5)
                    print("perf server done")

                    # Stop prev server
                    dk_ser.write(desh_msg("dect perf stop"))
                    time.sleep(0.5)



    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        dk_ser.write(desh_msg("dect ping stop"))
        time.sleep(1)
        dk_ser.write(desh_msg("dect perf stop"))
        time.sleep(1)
        dk_ser.close()
