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
import thread


use_real_robot = True
com_port = 25  # 5 = COM6 on Windows, tty5 on Linux
baudrate = 115200


if use_real_robot:
    import serial

#setup the window, view, etc
scene.forward = (0,-1,1)
scene.background=(0.1,0.1,0.2)
ser = None


# setup display
point=points(pos=[(0,0,0) for i in range(360)], size=5, color=color.red)
point2=points(pos=[(0,0,0) for i in range(360)], size=2, color=color.yellow)

# update function, takes the angle (an int, from 0 to 359) and the four bytes of data
def update_view( angle, x, x1, x2, x3):
    point2.pos[angle] = vector( 0,0, 0)

    angle_rad = angle * pi / 180.0
    c = cos(angle_rad)
    s = sin(angle_rad)

    dist_raw = x | (( x1 & 0x3f) << 8) # data on 13 bits ? 14 bits ?
    second_value_raw = x2 | (x3 << 8) # data on 10 bits or more ?

    # compute the position of the sample
    if x1 & 0x80: # flag for "bad data" ?
        # yes it's bad data
        point.pos[angle] = vector( 0,0,0)
    else:
        # no, it's cool
        #TODO determine what the bit 6 of x1 means...
        point.pos[angle] = vector( dist_raw*c,0, dist_raw*s)
        point2.pos[angle] = vector( second_value_raw*c,0, second_value_raw*s)


# Demo Mode if you don't have a hacked robot
if not use_real_robot:
    f = open("OneRun_no_shield.txt", 'r')
    data_str = f.readlines()[6:]
    data = [ int(d,16) for d in data_str]

    #print len(data_str)
    for i in range(360):
        update_view(i, data[4*i], data[4*i+1], data[4*i+2], data[4*i+3])
else:
    ser = serial.Serial(com_port, baudrate)

in_frame = False
init_level = 0
angle = 0

def read_in():
    global in_frame, init_level, angle
    while True:
        time.sleep(0.00001)
        if not in_frame:
            # wait for the start marker
            b = ord(ser.read(1))
                
            if init_level == 0 and b == 0x5A :
                init_level = 1
            elif init_level == 1 and b == 0xA5 :
                init_level = 2
            elif init_level == 2 and b == 0x00 :
                init_level = 3
            elif init_level == 3 and b == 0xC0 :
                init_level = 4
            elif init_level == 4 :
                init_level = 5
            elif init_level == 5 :
                in_frame = True
                init_level = 0
            else:
                init_level = 0
        else:
            # process input, 4 byte at a time
            bs = ser.read(4)
            update_view(angle, ord(bs[0]), ord(bs[1]), ord(bs[2]), ord(bs[3]))
            angle += 1
            if angle == 360 :
                in_frame = False
                angle = 0


if use_real_robot:
    th = thread.start_new_thread(read_in, ())
    


# this is the main loop, keeps the application alive and refreshes the window
while True:
    rate(60) # max FPS of the 3D display
    pass







