XV-11 test
Copyright 2010 Nicolas "Xevel" Saugnier

This repository contains the python visualizer, as well as the arduino sketch that spoofs a lidar (using a few full revolutions, because I only had an old Duemilanove(with 16k of flash) available).

How to make the program work in demo mode :

- install VPython
- open XV-11_test.py in VIDLE (installed with VPython)
- set use_real_robot to False
- F5 to run. Maintain left+right mouse button + move the mouse up and down to zoom.

How to make the program work in live mode :

- install VPython
- install PySerial
- open XV-11_test.py in VIDLE (installed with VPython)
- set your COM port
- F5 to run. Maintain left+right mouse button + move the mouse up and down to zoom.

How to make an arduino spoof the lidar:

- program LidarSpoofing on your arduino (works even with older Atmega168 based boards), and update your com port

How to extract your data to give them to the spoofer (from a Saleae Logic Scan, the long and secure way):
- Open the session in Saleae Logic
- Choose "Analyser" > "Async Serial", set the correct channel and check "Use Autobaud"
- click the "Export Analyser Data" of the analyser you just created, save to a file
- copy the content (or just part of it) into an OpenOffice Calc spreadsheet, choose the comma as separator
- discard the column containing the time, and the headers if any
- save the spreadsheet in .CSV format
- select the part you want to use, copy it into the LidarSpoofing/data.pde file, replacing the current data, and update the number of sample (equal to the number of lines you selected). Be careful not to copy too much, the arduino has limited flash memory.

- program the arduino with this sketch, then run the python script. The last scan will probably be messed up, but it should go back to normal at the next revolution.





Licensing

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
