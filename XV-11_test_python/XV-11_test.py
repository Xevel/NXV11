# XV-11 Test
# Copyright 2010 Nicolas "Xevel" Saugnier
#
#Licensed under the Apache License, Version 2.0 (the "License");
#you may not use this file except in compliance with the License.
#You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS,
#WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#See the License for the specific language governing permissions and
#limitations under the License.
#
# Version 0.2.2

# Requires VPython
# and for communication with the robot : PySerial 


from visual import *
import thread, time, sys, traceback


use_real_robot = True # DEMO MODE IS BROKEN RIGHT NOW, USE THE REAL ROBOT OF SPOOF THE DATA FROM AN ARDUINO!
com_port = 3  # 5 = COM6 on Windows, tty5 on Linux
baudrate = 115200


if use_real_robot:
    import serial

#setup the window, view, etc
scene.forward = (0,1,0)
scene.background=(0.1,0.1,0.2)
ser = None


# setup display
point=points(pos=[(0,0,0) for i in range(360)], size=5, color=(1,0,0))
pointb=points(pos=[(0,0,0) for i in range(360)], size=5, color=(0.4,0,0))

point2=points(pos=[(0,0,0) for i in range(360)], size=2, color=(1,1,0))
point2b=points(pos=[(0,0,0) for i in range(360)], size=2, color=(0.4,0.4,0))

#point3=points(pos=[(0,0,0) for i in range(360)], size=5, color = (1,1,1))
#point3b=points(pos=[(0,0,0) for i in range(360)], size=5, color = (0.4,0.4,0.4))

offset = 50
ring(pos=(0,0,0), axis=(0,1,0), radius=offset-1, thickness=1, color = color.yellow)
# plot ring every meter
ring(pos=(0,0,0), axis=(0,1,0), radius=6000, thickness=5, color = color.green)
ring(pos=(0,0,0), axis=(0,1,0), radius=1000, thickness=1, color = color.green)
ring(pos=(0,0,0), axis=(0,1,0), radius=2000, thickness=1, color = color.green)
ring(pos=(0,0,0), axis=(0,1,0), radius=3000, thickness=1, color = color.green)
ring(pos=(0,0,0), axis=(0,1,0), radius=4000, thickness=1, color = color.green)
ring(pos=(0,0,0), axis=(0,1,0), radius=5000, thickness=1, color = color.green)
# draw a line shows 0 degree
curve(pos=[(0,offset,0), (6000,offset,0)], radius=10)

label(pos = (0,2000,6000), text="Red : distance , Gray line: 0 degree\nYellow : quality + 50, yellow ring materializes 'quality == 0'\ndarker colors when quality is subpar")
label_speed  = label(pos = (0,500,-5000) , xoffset =1)
label_errors = label(pos = (0,800,-4000) , xoffset =1, text="errors: 0")

# update function, takes the angle (an int, from 0 to 359) and the four bytes of data
def update_view( angle, x, x1, x2, x3):
    global offset
    point.pos[angle] = vector( 0, 0, 0)
    pointb.pos[angle] = vector( 0, 0, 0)
    point2.pos[angle] = vector( 0, 0, 0)
    point2b.pos[angle] = vector( 0, 0, 0)
    #point3.pos[angle] = vector( 0, 0, 0)
    #point3b.pos[angle] = vector( 0, 0, 0)
    
    angle_rad = angle * pi / 180.0
    c = cos(angle_rad)
    s = sin(angle_rad)

    dist_mm = x | (( x1 & 0x1f) << 8) # data on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8) # data on 10 bits or more ?
    
    # display the sample
    if x1 & 0x80: # flag for "bad data" ?
        # yes it's bad data
        point.pos[angle] = vector( 0,0,0)
    else:
        # no, it's cool
        if not x1 & 0x40:
            # X+1:6 not set : quality is OK
            point.pos[angle] = vector( dist_mm*c,0, dist_mm*s)
            point2.pos[angle] = vector( (quality + offset)*c,0, (quality + offset)*s)
            #point3.pos[angle] = vector( dist_mm, (quality + offset), 0)
        else:
            # X+1:6 set : Warning, the quality is not as good as expected
            point.pos[angle] = vector( dist_mm*c,0, dist_mm*s)
            point2.pos[angle] = vector( (quality + offset)*c,0, (quality + offset)*s)
           #point3.pos[angle] = vector( dist_mm, (quality + offset), 0)
            


# Demo Mode if you don't have a hacked robot # BROKEN RIGHT NOW
if not use_real_robot:
    f = open("OneRun_no_shield.txt", 'r')
    data_str = f.readlines()[6:]
    data = [ int(d,16) for d in data_str]

    #print len(data_str)
    for i in range(360):
        update_view(i, data[4*i], data[4*i+1], data[4*i+2], data[4*i+3])
else:
    ser = serial.Serial(com_port, baudrate)

init_level = 0
index = 0
speed_rpm = 0.0


def checksum(data):
    # group the data my word, little endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum.
    chk32 = 0
    for data in data_list:
        chk32 = (chk32 << 1) + data

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )

def read_in():
    global in_frame, init_level, angle, index, speed_rpm

    nb_errors = 0
    while True:
        try:
            time.sleep(0.00001) # do not hog the processor power

            if init_level == 0 :
                # start byte
                b = ord(ser.read(1))
                if b == 0xFA : 
                    init_level = 1
                else:
                    init_level = 0
            elif init_level == 1:
                # position index 
                b = ord(ser.read(1))
                if b >= 0xA0 and b <= 0xF9  : 
                    index = b - 0xA0
                    init_level = 2
                else:
                    init_level = 0
            elif init_level == 2 :
                # speed
                b_speed = [ ord(b) for b in ser.read(2)]
                
                # data
                b_data0 = [ ord(b) for b in ser.read(4)]
                b_data1 = [ ord(b) for b in ser.read(4)]
                b_data2 = [ ord(b) for b in ser.read(4)]
                b_data3 = [ ord(b) for b in ser.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion... 
                all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3 

                # checksum  
                b_checksum = [ ord(b) for b in ser.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    speed_rpm = float( b_speed[0] | (b_speed[1] << 8) ) / 64.0
                    label_speed.text = "RPM : " + str(speed_rpm)
                    
                    update_view(index * 4 + 0, b_data0[0], b_data0[1], b_data0[2], b_data0[3])
                    update_view(index * 4 + 1, b_data1[0], b_data1[1], b_data1[2], b_data1[3])
                    update_view(index * 4 + 2, b_data2[0], b_data2[1], b_data2[2], b_data2[3])
                    update_view(index * 4 + 3, b_data3[0], b_data3[1], b_data3[2], b_data3[3])
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1
                    label_errors.text = "errors: "+str(nb_errors)
                    
                    # display the samples in an error state
                    update_view(index * 4 + 0, 0, 0x80, 0, 0)
                    update_view(index * 4 + 1, 0, 0x80, 0, 0)
                    update_view(index * 4 + 2, 0, 0x80, 0, 0)
                    update_view(index * 4 + 3, 0, 0x80, 0, 0)
                    

                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
        except :
            traceback.print_exc(file=sys.stdout)



if use_real_robot:
    th = thread.start_new_thread(read_in, ())
    


# this is the main loop, keeps the application alive and refreshes the window
while True:
    rate(60) # max FPS of the 3D display
    pass







