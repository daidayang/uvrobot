
#include <SoftwareSerialParityHalfDuplex.h>
SoftwareSerialParityHalfDuplex mySerial(2, 2); // RX, TX can be the same now!

void setup() {
  // put your setup code here, to run once:
  mySerial.begin(9600,ODD);
  //mySerial.begin(9600,EVEN);
  //mySerial.begin(9600,NONE);
   
}

void loop() {
  // put your main code here, to run repeatedly:
  mySerial.println("Hello Worlds");
  delay(1000);
}
