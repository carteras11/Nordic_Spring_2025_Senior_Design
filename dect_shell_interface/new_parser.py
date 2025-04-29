LOG_FNAME = 'test_logstest.txt'
OUTPUT_FNAME = 'log_parsed.txt'

'''
ISSUES: ACCIDENTALLY SET MCS4 FOR ALL PERF OPERATIONS!!!
'''

def get_line_after(line_in, substring):
    return str(line_in[line_in.index(substring)+len(substring):]).strip()

def get_line_between(line_in, s1, s2):
    return str(line_in[line_in.index(s1)+len(s1):line_in.index(s2)]).strip()

if __name__=="__main__":
    
    try:
        f_in = open(LOG_FNAME, "r")
    except:
        print('ERROR OPENING IN FILE')

    try:
        f_out = open(OUTPUT_FNAME, 'w')
    except:
        print('ERROR OPENING OUT FILE')

    f_out.write("test_num,time,lat,long,mcs,min_rssi,max_rssi\n")

    state = 'get new gps or mcs'
    gps = ''
    mcs = ''
    min_rssi = ''
    max_rssi = ''

    test_num = 0

    for line_in in f_in:
        # state update
        if state == 'get new gps or mcs':
            if 'GPS' in line_in:
                state = 'get new gps or mcs'
                gps = get_line_after(line_in, 'GPS:')
                test_num += 1
            elif '>>> MCS' in line_in:
                mcs = get_line_after(line_in, 'MCS')
                state = 'wait for server results'
        
        elif state == 'wait for server results':
            if 'ping server operation' in line_in:
                state = 'get rssi'

        elif state == 'get rssi':
            if 'rx: min RSSI' in line_in:
                min_rssi = get_line_between(line_in, 'min RSSI', ',')
                max_rssi = get_line_after(line_in, 'max RSSI')
                f_out.write(str(test_num)+','+gps+','+mcs+','+min_rssi+','+max_rssi+'\n')
                state = 'get new gps or mcs'

    print("Exiting...")

    f_in.close()
    f_out.close()