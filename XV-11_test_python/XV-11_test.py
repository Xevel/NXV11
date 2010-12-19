# XV-11 Lidar Test
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
# Version 0.3.6


# Requires VPython
# and for communication with the robot : PySerial 

from visual import *
import thread, time, sys, traceback

##---------- SETTINGS -------------- 
use_real_robot = True # Set to True to use data from the COM port, False to use demo data.

com_port = "COM25"  # 5 = COM6 on Windows, tty5 on Linux
baudrate = 115200
##---------------------------------

ser = None

#setup the window, view, etc
scene.forward = (1, -1, 0)
scene.background = (0.1, 0.1, 0.2)
scene.title = "Neato XV-11 Laser Distance Sensor - OpenLidarMap"

## setup display

# points
point     = points(pos=[(0,0,0) for i in range(360)], size=5, color=(1  , 0, 0))
pointb    = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0.4, 0, 0))

point2  = points(pos=[(0,0,0) for i in range(360)], size=3, color=(1  , 1,   0))
point2b = points(pos=[(0,0,0) for i in range(360)], size=3, color=(0.4, 0.4, 0))

offset = 140

outer_line= curve (pos=[(0,0,0) for i in range(360)], size=5, color=(1  , 0, 0))
lines=[curve(pos=[(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0)),(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0))]) for i in range(360)]

zero_intensity_ring = ring(pos=(0,0,0), axis=(0,1,0), radius=offset-1, thickness=1, color = color.yellow)

#draw the robot to indicate
box(pos = (44,10,0), width=100, length = 50, height = 20)
cylinder(pos=(19,0,0), axis = (0,20,0), radius=50)
cylinder(axis= (0,28,0),radius=20)

#
label(pos = (0,2000,6000), text="Red : distance\nYellow : quality + 50, yellow ring materializes 'quality == 0'\ndarker colors when quality is subpar")
label_speed = label(pos = (0,500,-5000) , xoffset =1)
label_errors = label(pos = (0,800,-4000) , xoffset =1, text="errors: 0")

use_points = True
use_outer_line = False
use_lines = False
use_intensity = True

def update_view( angle, data ):
    """Updates the view of a sample.

    Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
    """
    global offset, use_outer_line, use_line

    #reset the point display
    point.pos[angle] = vector( 0, 0, 0 )
    pointb.pos[angle] = vector( 0, 0, 0 )
    point2.pos[angle] = vector( 0, 0, 0 )
    point2b.pos[angle] = vector( 0, 0, 0 )

    
    #unpack data using the denomination used during the discussions
    x = data[0]
    x1= data[1]
    x2= data[2]
    x3= data[3]
    
    angle_rad = angle * pi / 180.0
    c = cos(angle_rad)
    s = -sin(angle_rad)

    dist_mm = x | (( x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8) # quality is on 16 bits

    dist_x = dist_mm*c
    dist_y = dist_mm*s

    if not use_lines : lines[angle].pos[1]=(offset*c,0,offset*s)
    if not use_outer_line : outer_line.pos[angle]=(offset*c,0,offset*s)
    
    
    # display the sample
    if x1 & 0x80: # is the flag for "bad data" set?
        # yes it's bad data
        lines[angle].pos[1]=(offset*c,0,offset*s)
        outer_line.pos[angle]=(offset*c,0,offset*s)
    else:
        # no, it's cool
        if not x1 & 0x40:
            # X+1:6 not set : quality is OK
            if use_points : point.pos[angle] = vector( dist_x,0, dist_y)
            if use_intensity : point2.pos[angle] = vector( (quality + offset)*c,0, (quality + offset)*s)
            if use_lines : lines[angle].color = (1,0,0)
            if use_outer_line : outer_line.color[angle] = (1,0,0)
        else:
            # X+1:6 set : Warning, the quality is not as good as expected
            if use_points : pointb.pos[angle] = vector( dist_x,0, dist_y)
            if use_intensity : point2b.pos[angle] = vector( (quality + offset)*c,0, (quality + offset)*s)
            if use_lines : lines[angle].color = (0.4,0,0)
            if use_outer_line : outer_line.color[angle] = (0.4,0,0)
        if use_lines : lines[angle].pos[1]=( dist_x, 0, dist_y)
        if use_outer_line : outer_line.pos[angle]=( dist_x, 0, dist_y)
        


def checksum(data):
    """Compute and return the checksum as an int.

    data -- list of 20 bytes (as ints), in the order they arrived in.
    """
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )



class pid_controler:
    def __init__(self, kp, ki, kd, ):
        self.set_coeffs( kp, ki, kd)
        self.reset()

    def set_coeffs(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def set_max_input(self, max_in):
        self.max_input = max_in
        
    def set_max_output(self, max_out):
        self.max_output = max_out
        
    def set_max_integral(self, max_i):
        self.max_I = max_i
        
    def set_output_ratio(self, ratio):
        self.out_ratio = r

    def reset(self):
        self.integral = 0.0
        self.prev_D = 0.0
        self.prev_sample = 0.0

    def process(error):
        ##  /* derivate value                                             
        ##  *             f(t+h) - f(t)        with f(t+h) = current value
        ##  *  derivate = -------------             f(t)   = previous value
        ##  *                    h
        ##  * so derivate = current error - previous error
        ##  *
        ##  * We can apply a filter to reduce noise on the derivate term,
        ##  * by using a bigger period.
        ##  */
        derivate = error - self.prev_sample

        if self.max_input :
            S_MAX(error, self.max_input) #saturate input before calculating integral
            
        ##   /* 
        ##   * Integral value : the integral become bigger with time .. (think
        ##   * to area of graph, we add one area to the previous) so, 
        ##   * integral = previous integral + current value
        ##   */
        self.integral += error ;

        if self.max_I:
            S_MAX(self.integral, self.max_I); # saturate integrale term

        output =  (long)(self.Kp * ( error + self.Ki * self.integral + (self.Kd * derivate) / PID_DERIVATE_FILTER_SIZE ));

        output = self.out_ratio * output;
  
        if(self.max_output):
            S_MAX(output, self.max_output);  # saturate output
    
        # backup values for the next calcul of derivate */
        self.prev_sample = error;
        self.prev_D = derivate
  
        return output

init_level = 0
index = 0

#rpm_setpoint = 300.0
#controler = pid_controler()
#controler.set_out_ratio(255)


def motor_control( speed ):
    global ser, controler, rpm_setpoint
    val = controler.process( speed - rpm_setpoint)
    ser.write(chr(val))



def gui_update_speed(speed_rpm):
    label_speed.text = "RPM : " + str(speed_rpm)

def compute_speed(data):
    speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
    return speed_rpm

def read_v_2_4():
    global init_level, angle, index

    nb_errors = 0
    while True:
        try:
            time.sleep(0.00001) # do not hog the processor power

            if init_level == 0 :
                b = ord(ser.read(1))
                # start byte
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
                elif b != 0xFA:
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
                    speed_rpm = compute_speed(b_speed)
                    gui_update_speed(speed_rpm)

                    #motor_control(speed_rpm)
                    
                    update_view(index * 4 + 0, b_data0)
                    update_view(index * 4 + 1, b_data1)
                    update_view(index * 4 + 2, b_data2)
                    update_view(index * 4 + 3, b_data3)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1
                    label_errors.text = "errors: "+str(nb_errors)
                    
                    # display the samples in an error state
                    update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    update_view(index * 4 + 3, [0, 0x80, 0, 0])
                    
                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
        except :
            traceback.print_exc(file=sys.stdout)


           
# Demo Mode if you don't have a hacked robot
if not use_real_robot:
    f = open("OneRun_no_shield.txt", 'r')
    #the data is in v2.1 mode, just show distances
    data_str = f.readlines()[6:]
    data = [ int(d,16) for d in data_str]
    for i in range(360):
        update_view(i, data[4*i:4*i+4])
else: 
    import serial
    ser = serial.Serial(com_port, baudrate)
    th = thread.start_new_thread(read_v_2_4, ())
    
drag_pos=None
pick = False
# this is the main loop, keeps the application alive and refreshes the window
while True:
    rate(60) # synchonous repaint at 60fps

    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info

        if s == "s": # stop motor
            ser.write(chr(0))
        elif s=="r": # run motor
            ser.write(chr(130))

        elif s=="o": # Toggle outer line
            use_outer_line = not use_outer_line
        elif s=="l": # Toggle rays
            use_lines = not use_lines
        elif s=="p": # Toggle points
            use_points = not use_points
        elif s=="i": # Toggle intensity
            use_intensity = not use_intensity
            zero_intensity_ring.visible = use_intensity


        
            


