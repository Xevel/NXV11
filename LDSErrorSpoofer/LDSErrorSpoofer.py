# LDS Error Spoofer
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
# Version 0.1

# Requires : PySerial 



#spoof error data to the robot.
# it sends "code * 4" times the error code, so by looking
# at the logs before and after is enough to know which error code is which.
# Just hope that sending invalid error codes will not crash the robot...

#An other approach would be to begin spoofing the codes we know of (02,03,21,25,35,50...)
# and then do this to figure out the other ones...



import serial, time


com_port = 27  # 5 == "COM6" == "/dev/tty5"
baudrate = 115200

ser = serial.Serial(com_port, baudrate)

def checksum(data):
    # group the data by word, little endian
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



index=0xA0
for err_code in range(255): #for each possible error code
    for k in range(err_code): # send 'err_code' times a packet containing four times the error code 
        if index > 0xF9: #wrap the index
            index=0xA0

        # <start byte> <index> <speed (300rpm)>
        data= "\xFA" + chr(index) + "\x00\x4B"

        for j in range(4): # 4 (invalid) range samples
            data += chr(err_code) + "\x80\x00\x00"    # error code, error flag is set, intensity is zero
        
        chksm = checksum([ord(d) for d in data]) #compute the checksum
        data += chr(chksm & 0xFF) + chr(chksm>>8) # append the checksum, little endian

        #print [ hex(ord(d2)) for d2 in data]
        ser.write(data) # send the packet to the robot
        time.sleep(0.001) # don't flood too much the robot... May need to be adjusted
        
print "DONE !"
