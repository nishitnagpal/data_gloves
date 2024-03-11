
import serial
import keyboard
import time
import numpy as np
import struct
import csv
import datetime
import matplotlib.pyplot as plt
import statistics

dataCount = 1000000

sensors = ['wrist', 'palm', 'thumb', 'index', 'mid', 'ring', 'pinky']

path = 'C:/Users/NISHIT/.spyder-py3./Recordings/'

# Write CSV file
def write_CSV(filename, sensorNames, data, mode=0):
    try:
        with open(filename, 'w') as csvfile:
           # i = 0
           # for sName in sensorNames:
            #    if mode == 0:
             #       csvfile.write("Sensor," + str(i) + ",4," + sName +",Measured Time\n")
              #  elif mode == 1:
               #     csvfile.write("Sensor," + str(i) + ",7," + sName +",OrientationAcceleration\n")
                #i += 1
                
            csv_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
            csv_writer.writerows(data)
    except Exception as e:
        print('Error in write_CSV:')
        print(e)
        return 0
    return 1

# Read CSV file with all experimental data, return time series of values
def parse_CSV_raw(filename):
    try:
        with open(filename, 'r') as csvfile:
            creader = csv.reader(csvfile, delimiter=',', lineterminator='\n')
            data = [r for r in creader]
    except:
        return 0
    #names = [item[-2] for item in data[:20] if item[-1] == 'Orientation']
    numSensors = len(sensors)
    data = data[numSensors:]
   
    #data2 = [[item[1], Quaternion(item[2], item[3], item[4], item[5])] for item in data]
   
    #data is: id, timestamp, measured time as (x, y, z, w) [, position as (x, y, z)]
    ts = [np.array([int(item[1])/1000 for item in data if int(item[0]) == i]) for i in range(numSensors)]
    data = [np.array([((item[2])) for item in data if int(item[0]) == i]) for i in range(numSensors)]
    data = np.array(data, dtype="object")
    ts = np.array(ts, dtype="object")
    ts = ts.T
    for i in range(numSensors):
        plt.figure()
        plt.plot(data[i], color="r")
        plt.plot(ts.T, color="y")
        
    # get rid of empty sensor entries
    #names = [names[i] for i in range(numSensors) if len(data[i]) > 0]
    #ts = [ts[i] for i in range(numSensors) if len(data[i]) > 0]
    #data = [data[i] for i in range(numSensors) if len(data[i]) > 0]
   
    


mode = 0

showData = 1


# object to store all data
data = np.zeros((dataCount,6))

if mode == 1:
    data = np.zeros((dataCount,9))

toRead = 10
if mode == 1:
    toRead = 16


print("press space to stop recording, Esc to abort recording\n")

filename = input('Enter filename (default: date + time):\n') or datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
filename += '.csv'

# serial port
ser = serial.Serial()
ser.baudrate = 500000
ser.port = 'COM3'
ser.open()

count = 0
nextTs = 1000


a = 1


try:
    ser.flushInput()
    
    #ser.write(b'hello!');
    #time.sleep(0.1)
    #ser.write(b'there');
    
    # try to find beginning of incoming stream
    while 1:
        if ser.in_waiting >= toRead - 1:
            bData = ser.read(toRead - 1)
            pos1 = bData.find(b'\x01\x01')
            pos2 = bData.find(b'\x02\x06')
            
            if pos1 > 0:
                ser.read(pos1 + 1)
                break
            if pos2 > 0:
                ser.read(pos2 + 1)
                break
        #if ser.in_waiting > 0:
        #    print(ser.read(ser.in_waiting))
        #if keyboard.is_pressed('s'):# and a == 1:
        #    a = 0
        #    ser.write(b'1234567890');
        #if not keyboard.is_pressed('s'):
        #    a = 1
        if keyboard.is_pressed('space') or keyboard.is_pressed('escape'):
            ser.close();
            break
    
    # flush the input in a controlled way
    while ser.is_open and ser.in_waiting >= toRead:
        bData = ser.read(toRead)
    
    
    t = time.perf_counter()
    
    
    while ser.is_open and not keyboard.is_pressed('space'):
        ts = int((time.perf_counter() - t)*1000)
        
        while ser.in_waiting >= toRead:
            bData = ser.read(toRead)
            
            if mode == 0:
                quat = np.array(struct.unpack('<4h', bData[2:]))
            elif mode == 1:
                rcvd = np.array(struct.unpack('<7h', bData[2:]))
                quat = rcvd[:4]
                acc = rcvd[4:] / 100
            
            #quat = quat/16384
            #quat = (quat/(8000000)*1000000)
            quat = ((quat/32768)*1000000)
            
            data[count][0] = bData[0] + bData[1] - 2 # create unique ID
            data[count][1] = ts
            
            #quat is in format wxyz -> change IMU coordinates to correct format xyzw
            # also in quaternion coordinates z is up, in screen coordinates y is up -> switch axes to (x, -z, y, w)
            #data[count][2] = quat[1] # x
            #data[count][3] = -quat[3] # y
            #data[count][4] = quat[2] # z
            data[count][5] = quat[0] # w
            
            
            if mode == 1:
                data[count][6] = -acc[0] # x
                data[count][7] = acc[2] # y
                data[count][8] = -acc[1] # z
                
                #plt.plot(ts,acc[0],'o',color="r")
                #plt.plot(ts,acc[1],'o',color="g")
                #plt.plot(ts,acc[2],'o',color="b")
            
            count += 1
            
            #data.extend(ser.read_all())
        
        if ts >= nextTs:
            print('time: ' + str(ts/1000) + '  count: ' + str(count) + '  last: ' + str(bData[0] + bData[1] - 2))
            nextTs += 1000
        
        if keyboard.is_pressed('escape'):
            if count > 0:
                # adjust data for displaying purposes
                data = data[:count]
                data[:,1] -= np.min(data[:,1])
            
            # set count to zero not write to a file
            count = 0
            break
except:
    ser.close()
    print('An error occured during recording')

ser.close()


if count > 0:
    # adjust data to write
    data = data[:count]
    data[:,1] -= np.min(data[:,1])
    
    
    print('\nwriting to file...')
    
    if mode == 0:
        dataToWrite = [[int(d[0]), int(round(d[1])), d[5]] for d in data]
    elif mode == 1:
        dataToWrite = [[int(d[0]), int(round(d[1])), d[2], d[3], d[4], d[5], d[6], d[7], d[8]] for d in data]
    
    # write data to file (otherwise recording was aborted)
    if write_CSV(path+filename, sensors, dataToWrite, mode):
        print('written to ' + path + filename)
        parse_CSV_raw(path+filename);
        
else:
    print('recording aborted or no data available')  

#if mode == 0 and showData == 1:
    #plt.figure()
    #plt.plot(data[:,1]/10000,data[:,1],color="r")
    #plt.plot(data[:,1]/10000,data[:,3],color="g")
    #plt.plot(data[:,1]/10000,data[:,5],color="b")
    
    
    
