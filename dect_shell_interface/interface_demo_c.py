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


DK_PORT_NAME = '/dev/ttyACM1'
DK_PORT_BAUDRATE = 115200

GPS_PORT_NAME = '/dev/ttyACM0'
GPS_PORT_BAUDRATE = 9600

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

            print("Checking for GPS at port " + GPS_PORT_NAME + "...")
            while not port_exists(GPS_PORT_NAME):
                time.sleep(1)
            print("Found GPS at port " + GPS_PORT_NAME)

            time.sleep(2) # let the serial ports fully setup before attempting access
            
            dk_ser = serial.Serial(DK_PORT_NAME, DK_PORT_BAUDRATE, timeout=1)
            gps_ser = serial.Serial(GPS_PORT_NAME, GPS_PORT_BAUDRATE, timeout=1)
            print("Set up PySerial ports")

            time.sleep(1)

            '''
            dk_ser.write(desh_msg("dect sett --reset"))
            time.sleep(0.5)
            dk_ser.write(desh_msg("dect sett --tx_pwr -10"))
            time.sleep(0.5)
            dk_ser.write(desh_msg("dect sett -b 9"))
            time.sleep(0.5)
            '''
            print("Configured dect shell")


            filename = "test.txt" #input("Enter the filename to write to: ")
            try:
                f = open("test_logs/"+str(filename), "+xa")
            except:
                print('ERROR OPENING FILE')

            while True:
                # Wait for user to trigger new read
                input("Press ENTER to begin test")
                print("Waiting for new GPS fix...")

                # Wait for GPS fix (type GGA)
                line = gps_ser.readline()
                gps_msg = pynmea2.parse(line.decode())
                while gps_msg.sentence_type != "GGA":
                    line = gps_ser.readline()
                    gps_msg = pynmea2.parse(line.decode())

                print("New GPS fix, beginning test")
                f.write("GPS:" + str(gps_msg.timestamp) + ',' + str(gps_msg.lat) + ',' +  str(gps_msg.lon) + '\n')
                print(str(gps_msg).strip()+'\n')

                for tx_mcs in range(0, 12): # 0 - 11
                    f.write(">>> MCS" + str(tx_mcs) + '\n')
                    print(">>> MCS" + str(tx_mcs))

                    # Initiate non-harq ping test
                    print("Starting ping...")
                    dk_ser.write(desh_msg("dect ping -c --s_tx_id 39 --c_tx_pwr " + TX_PWR + " -i 3 -t 2000 -l 4 \
                        --c_tx_mcs " + str(tx_mcs) + " --c_count 3 --channel 1677"))
                
                    resp = ""
                    while "ping command done" not in resp:
                        resp = dk_ser.readline().decode('UTF-8')
                        #print(resp.strip())
                        f.write(resp.strip() + '\n')

                    print("ping completed")

                    time.sleep(1)

                    print("Starting perf...")
                    dk_ser.write(desh_msg("dect perf -c --c_gap_subslots 3 --c_tx_mcs "+ str(tx_mcs) +"" --c_slots 4 --s_tx_id 39 -t 3 \
                                        --channel 1677"))
                    
                    resp = ""
                    while "perf command done" not in resp:
                        #print(resp.strip())
                        resp = dk_ser.readline().decode('UTF-8')
                        f.write(resp.strip() + '\n')

                    print("perf completed")

                    time.sleep(2.5)


    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        dk_ser.close()
        gps_ser.close()
        f.close()