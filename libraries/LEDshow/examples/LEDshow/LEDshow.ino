#include <LEDshow.h>

LED led1(10,9,8);//The first for DIO(Data I/O),the second for RCLK (Storage register clock),the third for SCLK (Serial clock)

void setup(){}

void loop(){

led1.ledshow(1);
delay(1000);
led1.ledshow(2);
delay(1000);
}
