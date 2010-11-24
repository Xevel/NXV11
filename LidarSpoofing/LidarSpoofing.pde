/*
Lidar Spoofing

Copyright 2010 Nicolas "Xevel" Saugnier

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// This arduino sketch outputs recorded data to the serial port, so to any
// external application listening to the erial port, it looks like the Piccolo
// Lidar is talking.

#include <avr/pgmspace.h>

extern unsigned int nb_data ;
extern PROGMEM prog_uint8_t data[];

void setup() {
  // initialize the serial communication:
  Serial.begin(115200);
  Serial.println("start"); // TODO replace with the right message... for now it
  // is treated as garbage anyway.
}

int k;
byte c;
void loop() {
  c = pgm_read_byte_near(data+k);
  Serial.print(c);
  k++;
  if (k == nb_data){
    k = 0;
  }
  
  delayMicroseconds(10); // delay for debug, remov to have full speed
}

