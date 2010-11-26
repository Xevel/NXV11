
f = open("hand_in_box.txt", 'rb')
data_bin = f.read(40000)
f_str= open("hand_in_box_str.txt", 'w')
for i in range(len(data_bin)):
    tmp = "," + hex(ord(data_bin[i])) + '\n'
    f_str.write( tmp )
f_str.close()
