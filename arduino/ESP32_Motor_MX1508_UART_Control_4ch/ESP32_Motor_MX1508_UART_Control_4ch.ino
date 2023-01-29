//  https://www.instructables.com/ESP32-Mecanum-Wheels-Robot-and-Bluetooth-Gamepad-C/
//  https://compendium.readthedocs.io/en/latest/tasks/drivetrains/mecanum.html

#include <Arduino.h>

#define RightFrontBWD 12  //  INT4
#define RightFrontFWD 13  //  INT3
#define RightBackBWD  27  //  INT2
#define RightBackFWD  26  //  INT1

#define LeftFrontBWD  25  //  INT4
#define LeftFrontFWD  33  //  INT3
#define LeftBackBWD   32  //  INT2
#define LeftBackFWD   23  //  INT1

#define LeftStartLevel  104
#define RightStartLevel 104

int motorspeed = 128;

int v_left_front_Old;
int v_left_back_Old;
int v_right_front_Old;
int v_right_back_Old;

int v_left_front;
int v_left_back;
int v_right_front;
int v_right_back;

unsigned long LastSpeedChangeTimeInMs = 0;
unsigned long LastSpeedCommandReceivedInMs = 0;
unsigned long LastBatteryCheckTimeInMs = 0;

String Serial1InputString = "";          // a String to hold incoming data
char Serial1InputBytes[20];
byte Serial1InputPos = 0;
bool Serial1InputComplete = false;     // whether the string is complete

#define RXD2 21     //  Color: Blue    ESP32 Pin 21   Jetson Nano Pin 8  
#define TXD2 22     //  Color: White   ESP32 Pin 22   Jetson Nano Pin 10

#define BatteryVIN_Pin 35
#define led_pin        34

void setup() {
  // pinMode(led_pin, OUTPUT);

  pinMode(RightFrontFWD, OUTPUT);
  analogWrite(RightFrontFWD, 0);

  pinMode(RightFrontBWD, OUTPUT);
  analogWrite(RightFrontBWD, 0);

  pinMode(RightBackFWD, OUTPUT);
  analogWrite(RightBackFWD, 0);

  pinMode(RightBackBWD, OUTPUT);
  analogWrite(RightBackBWD, 0);

  pinMode(LeftFrontFWD, OUTPUT);
  analogWrite(LeftFrontFWD, 0);

  pinMode(LeftFrontBWD, OUTPUT);
  analogWrite(LeftFrontBWD, 0);

  pinMode(LeftBackFWD, OUTPUT);
  analogWrite(LeftBackFWD, 0);

  pinMode(LeftBackBWD, OUTPUT);
  analogWrite(LeftBackBWD, 0);

  // put your setup code here, to run once:
  Serial.begin(115200);      // make sure your Serial Monitor is also set at this baud rate.
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);

//  Dabble.begin("MyEsp32");       //set bluetooth name of your device

  LastSpeedChangeTimeInMs = millis();
  LastSpeedCommandReceivedInMs = LastSpeedChangeTimeInMs;
  LastBatteryCheckTimeInMs = LastSpeedChangeTimeInMs;
}

void loop() {
  unsigned long NowInMs;

  ReadSerial0CommandInput();

  NowInMs = millis();
  if ( (NowInMs - LastSpeedCommandReceivedInMs) > 5000 ) {
    LastSpeedCommandReceivedInMs = NowInMs;
    v_left_front = 0;
    v_left_back = 0;
    v_right_front = 0;
    v_right_back = 0;    
//    Serial.println(NowInMs - LastSpeedCommandReceivedInMs);

    if ( (NowInMs - LastBatteryCheckTimeInMs) > 10000 ) {
      LastBatteryCheckTimeInMs = NowInMs;
      int BatVal = analogRead(BatteryVIN_Pin);
      Serial1.print("Bat: ");
      Serial1.println(BatVal * 7.4 / 2905);

      Serial.print("Bat: ");
      Serial.println(BatVal * 7.4 / 2905);
    }
  }

/*
  if ( ( v_right_front != v_right_front_Old ) ||
       ( v_right_back != v_right_back_Old ) ||
       ( v_left_front != v_left_front_Old ) ||
       ( v_left_back != v_left_back_Old ) ) {

        Serial.print("lf=");
        Serial.print(v_left_front);
        Serial.print(", lb=");
        Serial.print(v_left_back);
        Serial.print(", rf=");
        Serial.print(v_right_front);
        Serial.print(", rb=");
        Serial.println(v_right_back);
  }
*/
  
  if ( v_right_front != v_right_front_Old ) {
    v_right_front_Old = v_right_front;

    if ( v_right_front > 0 ) {
      analogWrite(RightFrontFWD, v_right_front);
      analogWrite(RightFrontBWD, 0);      

/*
      Serial.print("RF_F ");
      Serial.print(RightFrontFWD);
      Serial.print(" 1, ");
      Serial.print("RF_B ");
      Serial.print(RightFrontBWD);
      Serial.print(" 0, ");
*/
    }
    else {
      analogWrite(RightFrontFWD, 0);
      analogWrite(RightFrontBWD, -v_right_front);      
/*    
      Serial.print("RF_F ");
      Serial.print(RightFrontFWD);
      Serial.print(" 0, ");
      Serial.print("RF_B ");
      Serial.print(RightFrontBWD);
      Serial.print(" 1, ");
*/      
    }
  }


  if ( v_right_back != v_right_back_Old ) {
    v_right_back_Old = v_right_back;

    if ( v_right_back > 0 ) {
      analogWrite(RightBackFWD, v_right_back);
      analogWrite(RightBackBWD, 0);      
/*
      Serial.print("RB_F ");
      Serial.print(RightBackFWD);
      Serial.print(" 1, ");
      Serial.print("RB_B ");
      Serial.print(RightBackBWD);
      Serial.print(" 0, ");
*/
    }
    else {
      analogWrite(RightBackFWD, 0);
      analogWrite(RightBackBWD, -v_right_back);      
/*
      Serial.print("RB_F ");
      Serial.print(RightBackFWD);
      Serial.print(" 0, ");
      Serial.print("RB_B ");
      Serial.print(RightBackBWD);
      Serial.print(" 1, ");
*/
    }
  }

  if ( v_left_front != v_left_front_Old ) {
    v_left_front_Old = v_left_front;

    if ( v_left_front > 0 ) {
      analogWrite(LeftFrontFWD, v_left_front);
      analogWrite(LeftFrontBWD, 0);      
/*
      Serial.print("LF_F ");
      Serial.print(LeftFrontFWD);
      Serial.print(" 1, ");
      Serial.print("LF_B ");
      Serial.print(LeftFrontBWD);
      Serial.print(" 0, ");
*/
    }
    else {
      analogWrite(LeftFrontFWD, 0);
      analogWrite(LeftFrontBWD, -v_left_front);      
/*
      Serial.print("LF_F ");
      Serial.print(LeftFrontFWD);
      Serial.print(" 0, ");
      Serial.print("LF_B ");
      Serial.print(LeftFrontBWD);
      Serial.print(" 1, ");
*/      
    }    
  }

  if ( v_left_back != v_left_back_Old ) {
    v_left_back_Old = v_left_back;

    if ( v_left_back > 0 ) {
      analogWrite(LeftBackFWD, v_left_back);
      analogWrite(LeftBackBWD, 0);      
/*
      Serial.print("LB_F ");
      Serial.print(LeftBackFWD);
      Serial.print(" 1, ");
      Serial.print("LB_B ");
      Serial.print(LeftBackBWD);
      Serial.println(" 0");
*/      
    }
    else {
      analogWrite(LeftBackFWD, 0);
      analogWrite(LeftBackBWD, -v_left_back);      
/*
      Serial.print("LB_F ");
      Serial.print(LeftBackFWD);
      Serial.print(" 0, ");
      Serial.print("LB_B ");
      Serial.print(LeftBackBWD);
      Serial.println(" 1");
*/      
    }
  }
}

void moveForward() {
  analogWrite(RightFrontFWD, motorspeed);
  analogWrite(RightFrontBWD, 0);
  analogWrite(RightBackFWD, motorspeed);
  analogWrite(RightBackBWD, 0);

  analogWrite(LeftFrontFWD, motorspeed);
  analogWrite(LeftFrontBWD, 0);
  analogWrite(LeftBackFWD, motorspeed);
  analogWrite(LeftBackBWD, 0);
}

void moveBackward() {
  analogWrite(RightFrontFWD, 0);
  analogWrite(RightFrontBWD, motorspeed);
  analogWrite(RightBackFWD, 0);
  analogWrite(RightBackBWD, motorspeed);

  analogWrite(LeftFrontFWD, 0);
  analogWrite(LeftFrontBWD, motorspeed);
  analogWrite(LeftBackFWD, 0);
  analogWrite(LeftBackBWD, motorspeed);
}

void rotateRight() {
  analogWrite(RightFrontFWD, 0);
  analogWrite(RightFrontBWD, motorspeed);
  analogWrite(RightBackFWD, 0);
  analogWrite(RightBackBWD, motorspeed);

  analogWrite(LeftFrontFWD, motorspeed);
  analogWrite(LeftFrontBWD, 0);
  analogWrite(LeftBackFWD, motorspeed);
  analogWrite(LeftBackBWD, 0);
}

void rotateLeft() {
  analogWrite(RightFrontFWD, motorspeed);
  analogWrite(RightFrontBWD, 0);
  analogWrite(RightBackFWD, motorspeed);
  analogWrite(RightBackBWD, 0);

  analogWrite(LeftFrontFWD, 0);
  analogWrite(LeftFrontBWD, motorspeed);
  analogWrite(LeftBackFWD, 0);
  analogWrite(LeftBackBWD, motorspeed);
}

void stopMoving() {
  analogWrite(RightFrontFWD, 0);
  analogWrite(RightFrontBWD, 0);
  analogWrite(RightBackFWD, 0);
  analogWrite(RightBackBWD, 0);

  analogWrite(LeftFrontFWD, 0);
  analogWrite(LeftFrontBWD, 0);
  analogWrite(LeftBackFWD, 0);
  analogWrite(LeftBackBWD, 0);
}

void moveSidewaysRight() {
  analogWrite(RightFrontFWD, 0);
  analogWrite(RightFrontBWD, motorspeed);
  analogWrite(RightBackFWD, motorspeed);
  analogWrite(RightBackBWD, 0);

  analogWrite(LeftFrontFWD, motorspeed);
  analogWrite(LeftFrontBWD, 0);
  analogWrite(LeftBackFWD, 0);
  analogWrite(LeftBackBWD, motorspeed);
}

void moveSidewaysLeft() {
  analogWrite(RightFrontFWD, motorspeed);
  analogWrite(RightFrontBWD, 0);
  analogWrite(RightBackFWD, 0);
  analogWrite(RightBackBWD, motorspeed);

  analogWrite(LeftFrontFWD, 0);
  analogWrite(LeftFrontBWD, motorspeed);
  analogWrite(LeftBackFWD, motorspeed);
  analogWrite(LeftBackBWD, 0);
}

void moveRightForward() {
  analogWrite(RightFrontFWD, 0);
  analogWrite(RightFrontBWD, 0);
  analogWrite(RightBackFWD, motorspeed);
  analogWrite(RightBackBWD, 0);

  analogWrite(LeftFrontFWD, motorspeed);
  analogWrite(LeftFrontBWD, 0);
  analogWrite(LeftBackFWD, 0);
  analogWrite(LeftBackBWD, 0);
}

void moveLeftForward() {
  analogWrite(RightFrontFWD, motorspeed);
  analogWrite(RightFrontBWD, 0);
  analogWrite(RightBackFWD, 0);
  analogWrite(RightBackBWD, 0);

  analogWrite(LeftFrontFWD, 0);
  analogWrite(LeftFrontBWD, 0);
  analogWrite(LeftBackFWD, motorspeed);
  analogWrite(LeftBackBWD, 0);
}


/*
 * ROS => ESP32 Drive Command
 *   Drive command packet starts with 0x81 and end with 0x80. 
 *   [0x81, LeftFront_msb, LeftFront_lsb, LeftBack_msb, LeftBack_lsb, RightFront_msb, RightFront_lsb, RightBack_msb, RightBack_lsb, 0x80]
 *   The msb and lsb bytes use 7 bits per byte.  The left most bit is alwary 0
 *   
 * RSP32 => ROS Battery Voltage
 *   Bat:7.4v 
 * 
 */
void ReadSerial0CommandInput(){
  int TmpInt;
  
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // Serial.print(inChar, HEX);
    // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    if ( inChar == 0x81 ) {
      Serial1InputComplete = false;
      Serial1InputPos = 0;
    }

    Serial1InputBytes[Serial1InputPos++] = inChar;

    if ( inChar == 0x80 ) {
      Serial1InputComplete = true;
    }
  }

  if ( Serial1InputComplete ) {
    Serial1InputComplete = false;

//    for(int idx=0; idx++; idx<Serial1InputPos ) {
//      Serial.print( (short) Serial1InputBytes[idx] );
//      Serial.print(" ");
//    }
//    Serial.println();

    int inputlen = Serial1InputPos;

//    Serial.print("inputString=");
//    Serial.println(Serial1InputString);
//    Serial.print("inputlen=");
//    Serial.println(inputlen);
    
    char c1 = Serial1InputBytes[0];
    char c2;
    switch ( c1 )
    {
      case '1':
        Serial.println("Steering SetZeroPos()");
        break;

      case '2':
        Serial.println("Steering ReadZeroPos()");
        break;

      case 0x81:
//        Serial.print("inputlen=");
//        Serial.println(inputlen);
        if ( inputlen != 10 ) {
          Serial1InputPos = 0;
          return;
        }

        // target value range -255 0 +255
        // actual value range 1 256 511
        
        c2 = Serial1InputBytes[1];
        TmpInt = c2 << 7;
        c2 = Serial1InputBytes[2];
        TmpInt += c2;
        v_left_front = TmpInt - 256;

        c2 = Serial1InputBytes[3];
        TmpInt = c2 << 7;
        c2 = Serial1InputBytes[4];
        TmpInt += c2;
        v_left_back = TmpInt - 256;

        c2 = Serial1InputBytes[5];
        TmpInt = c2 << 7;
        c2 = Serial1InputBytes[6];
        TmpInt += c2;
        v_right_front = TmpInt - 256;

        c2 = Serial1InputBytes[7];
        TmpInt = c2 << 7;
        c2 = Serial1InputBytes[8];
        TmpInt += c2;
        v_right_back = TmpInt - 256;

//        Serial.print(" lf=");
//        Serial.print(v_left_front);
//        Serial.print(" lb=");
//        Serial.print(v_left_back);
//        Serial.print(" rf=");
//        Serial.print(v_right_front);
//        Serial.print(" rb=");
//        Serial.println(v_right_back);

//        digitalWrite(led_pin, !digitalRead(led_pin));

        LastSpeedCommandReceivedInMs = millis();
        break;

      case 'S':
        if ( inputlen <= 2 ) {
          Serial1InputString = "";
          return;
        }
          
        c2 = Serial1InputString[1];
        String strValue = Serial1InputString.substring(2, inputlen);   
        int intValue = 0;
        switch( c2 ) {
          case 'B':
            intValue = strValue.toInt();
            Serial.print("SB = ");
            Serial.println(intValue);
          break;
          case 'A':
            intValue = strValue.toInt();
            Serial.print("SA = ");
            Serial.println(intValue);
          break;
        }
        break;
    }  
    Serial1InputString = "";
  }
}
