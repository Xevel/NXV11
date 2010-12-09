#include <pololu/orangutan.h>

/*
LidarBluetooth motor controller


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

/*
This program is intended for the Pololu Baby Orangutan B.


v0.1: 2010/12/09 Functional motor controller
Sets the motor speed with value received from the serial port.
For example, receiving 0x00 will stop the motor, and 0xFF will
push the motor to it's max speed.

*/



// RX is on PD0
// TX is on PD1
// baudrate : 115200


int main(void){

	// blink the leds just so we know it's running
	red_led(0);
	delay_ms(500);
	red_led(1);
	delay_ms(200);
	red_led(0);
    delay_ms(500);
	red_led(1);
	delay_ms(200);
	red_led(0);
    delay_ms(500);
	red_led(1);

	serial_set_mode(SERIAL_AUTOMATIC);
    serial_set_baud_rate(115200);

	unsigned char c = 0;
    while(1)  
    {  
		if (!serial_receive_blocking(&c, 1, 100)){
			// Set the motor speed with the value received.  
	    	set_m2_speed((int)c);

			// resend the value, for debug
			serial_send(c, 1);
		}
    }
}
