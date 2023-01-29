#include "Arduino.h"
#include <SPI.h>

#define VSPI_MISO   19
#define VSPI_MOSI   23
#define VSPI_SCLK   18

static const int spiClk = 1000000; // 1 MHz

uint16_t MotorControlValues[256];
  
#define PIN_OUT_UVLIGHT   4
#define PIN_OUT_FOGGER    2

#define PIN_OUT_SPI_DAC_CS  32
#define PIN_OUT_SPI_DAC_LD  33
#define PIN_OUT_MotorCtrlPowerPulse 22

#define PIN_DIN_HSU_R  25   //  J2-2
#define PIN_DIN_HSU_L  14   //  J1-4
#define PIN_DIN_HSV_R  26   //  J2-3
#define PIN_DIN_HSV_L  12   //  J1-2
#define PIN_DIN_HSW_R  27   //  J2-4
#define PIN_DIN_HSW_L  13   //  J1-3

#define PIN_D_TX2           //  J4-1
#define PIN_D_RX2           //  J4-2
                            //  J4-3 GND
                            
//  J3-1  (OUT) POT-OUT1                DAC-8
//  J3-2  (OUT) POT-OUT2                DAC-6
//  J3-3  GND
//  J3-4  (OUT) MotorCtrlPowerPulse     SSR-8
//  J3-5  (IN)  MtrCtrl Vlt             ESP-GPIO34


#define PIN_AIN_MtrCtrl   34
#define PIN_AIN_Battery   36
#define PIN_AIN_Charger   39


#define CW             1      // Assign a value to represent clock wise rotation
#define CCW           -1      // Assign a value to represent counter-clock wise rotation

bool HSU_L_Val = false;
bool HSV_L_Val = false;
bool HSW_L_Val = false;
bool HSU_R_Val = false;
bool HSV_R_Val = false;
bool HSW_R_Val = false;

int direct_L = 1;        // Integer variable to store BLDC rotation direction
int direct_R = 1;        // Integer variable to store BLDC rotation direction
int pulseCount_L;       // Integer variable to store the pulse count
int pulseCount_R;       // Integer variable to store the pulse count

unsigned long startTime_L;        // Float variable to store the start time of the current interrupt 
unsigned long prevTime_L;         // Float variable to store the start time of the previous interrupt 
unsigned long pulseTimeW_L;       // Float variable to store the elapsed time between interrupts for hall sensor W 
unsigned long pulseTimeU_L;       // Float variable to store the elapsed time between interrupts for hall sensor U 
unsigned long pulseTimeV_L;       // Float variable to store the elapsed time between interrupts for hall sensor V 
unsigned long AvPulseTime_L;      // Float variable to store the average elapsed time between all interrupts 

unsigned long startTime_R;        // Float variable to store the start time of the current interrupt 
unsigned long prevTime_R;         // Float variable to store the start time of the previous interrupt 
unsigned long pulseTimeW_R;       // Float variable to store the elapsed time between interrupts for hall sensor W 
unsigned long pulseTimeU_R;       // Float variable to store the elapsed time between interrupts for hall sensor U 
unsigned long pulseTimeV_R;       // Float variable to store the elapsed time between interrupts for hall sensor V 
unsigned long AvPulseTime_R;      // Float variable to store the average elapsed time between all interrupts 

int PPM_L;        // Float variable to store calculated pulses per minute
float RPM_L;        // Float variable to store calculated revolutions per minute

int PPM_R;        // Float variable to store calculated pulses per minute
float RPM_R;        // Float variable to store calculated revolutions per minute

unsigned long pulsePowerStartTimeInMs = 0;
bool inPowerPulsingMode = false;
bool MotorCtrlPoweredOn = false;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;     // whether the string is complete
bool reportBatteryInfo = false;  // need to send battery / charger voltage to ROS at 10 hz
int curDelay = 1000;

unsigned short ledFlashCounter = 1;
// unsigned long tenMilliTimer = 0;

long current_pulse_right = 0;
long current_pulse_left = 0;
long last_pulse_right = 0;
long last_pulse_left = 0;
int  force_factor = 1;

double x = 0.0;
double y = 0.0;
double theta = 0;
double linear_v = 0.0;
double angular_v = 0.0;

double wheel_track = 0.514;
double wheel_diameter = 0.254;  
double circle_enc = 40550;       //轮子每圈脉冲总数

bool first_get_encode_count = true;

unsigned long lastMotorCmd = 0;
unsigned long last100Millis = 0;
unsigned long last1second = 0;
unsigned long lastCalculationTimeInMs = 0;

unsigned int CntSecond = 0;

unsigned int cmd_left = 128; 
unsigned int cmd_right = 128;

int testCnt = 128;
int testDelta = 0;


SPIClass * vspi = NULL;

void setup() 
{
  pinMode(VSPI_SCLK, OUTPUT);
  pinMode(VSPI_MOSI, OUTPUT);
  pinMode(VSPI_MISO, INPUT);
  
  vspi = new SPIClass(VSPI);
  vspi->begin();

  pinMode (PIN_OUT_SPI_DAC_CS, OUTPUT);
  pinMode (PIN_OUT_SPI_DAC_LD, OUTPUT);

  digitalWrite(PIN_OUT_SPI_DAC_CS, HIGH);
  digitalWrite(PIN_OUT_SPI_DAC_LD, HIGH);
  
  pinMode(PIN_OUT_UVLIGHT,OUTPUT);        //  Pin for UV light switch
  digitalWrite (PIN_OUT_UVLIGHT, HIGH);   //  start UV light line HIGH  (Setting the relay off)

  pinMode(PIN_OUT_FOGGER,OUTPUT);         //  Pin for sprayer switch
  digitalWrite (PIN_OUT_FOGGER, HIGH);    //  start sprayer line HIGH  (Setting the relay off)


  prevTime_R = millis();
  prevTime_L = prevTime_R;
              
  pinMode(PIN_DIN_HSU_R,INPUT);  //  Pin for hall sensor encoder channel U for right
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_HSU_R), blink_U_R, CHANGE);

  pinMode(PIN_DIN_HSU_L,INPUT);  //  Pin for hall sensor encoder channel U for left
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_HSU_L), blink_U_L, CHANGE);

  pinMode(PIN_DIN_HSV_R,INPUT);  //  Pin for hall sensor encoder channel V for right
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_HSV_R), blink_V_R, CHANGE);

  pinMode(PIN_DIN_HSV_L,INPUT);  //  Pin for hall sensor encoder channel V for left
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_HSV_L), blink_V_L, CHANGE);

  pinMode(PIN_DIN_HSW_R,INPUT);  //  Pin for hall sensor encoder channel W for right
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_HSW_R), blink_W_R, CHANGE);

  pinMode(PIN_DIN_HSW_L,INPUT);  //  Pin for hall sensor encoder channel W for left
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_HSW_L), blink_W_L, CHANGE);

  HSU_L_Val = digitalRead(PIN_DIN_HSU_L);    // Set the U_L sensor value as boolean and read initial state
  HSV_L_Val = digitalRead(PIN_DIN_HSV_L);    // Set the V_L sensor value as boolean and read initial state 
  HSW_L_Val = digitalRead(PIN_DIN_HSW_L);    // Set the W_L sensor value as boolean and read initial state 
  HSU_R_Val = digitalRead(PIN_DIN_HSU_R);    // Set the U_R sensor value as boolean and read initial state
  HSV_R_Val = digitalRead(PIN_DIN_HSV_R);    // Set the V_R sensor value as boolean and read initial state 
  HSW_R_Val = digitalRead(PIN_DIN_HSW_R);    // Set the W_R sensor value as boolean and read initial state 

  current_pulse_right = 0;
  current_pulse_left = 0;
  last_pulse_right = 0;
  last_pulse_left = 0;
  force_factor = 1;

  lastMotorCmd = millis();
  last100Millis = lastMotorCmd;
  last1second = lastMotorCmd;
  lastCalculationTimeInMs = lastMotorCmd;

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  int MaxSpeedValue = 1340;
  int MinSpeedValue = 1880;
  for( int idx=1; idx<=127; idx++ )
  {
    MotorControlValues[idx] = MinSpeedValue-(MaxSpeedValue-MinSpeedValue)*(idx-127)/127;
//    Serial.print("MotorControlValues[");
//    Serial.print(idx);
//    Serial.print("] = ");
//    Serial.println(MotorControlValues[idx]);
  }
  MotorControlValues[0]=MotorControlValues[1];
  
  MotorControlValues[128]=2500;
//  Serial.print("MotorControlValues[128]=2500");
  
  MaxSpeedValue = 3500;   // 4095 is the max for MCP4822
  MinSpeedValue = 3120;
  for( int idx=129; idx<=255; idx++ )
  {
    MotorControlValues[idx] = MinSpeedValue+(MaxSpeedValue-MinSpeedValue)*(idx-129)/127;
//    Serial.print("MotorControlValues[");
//    Serial.print(idx);
//    Serial.print("] = ");
//    Serial.println(MotorControlValues[idx]);
  }
}


void blink_U_R() {
  startTime_R = millis();
  ]HSU_R_Val = digitalRead(PIN_DIN_HSU_R);
  HSW_R_Val = digitalRead(PIN_DIN_HSW_R);          // Read the current W (or V) hall sensor value    
  direct_R = (HSU_R_Val == HSW_R_Val) ? CW : CCW;
  pulseCount_R = pulseCount_R + (1 * direct_R);
  pulseTimeU_R = startTime_R - prevTime_R;        
  AvPulseTime_R = ((pulseTimeW_R + pulseTimeU_R + pulseTimeV_R)/3);   

  if ( AvPulseTime_R ==  0 ) AvPulseTime_R=10;
  
  PPM_R = (1000 / AvPulseTime_R) * 60;          
//  RPM_R = (float) PPM_R / 90;
  prevTime_R = startTime_R;
//  digitalWrite(PIN_OUT_FOGGER, !digitalRead(PIN_OUT_FOGGER));   // turn the LED on (HIGH is the voltage level)  
}

void blink_U_L() {
  startTime_L = millis();
  HSU_L_Val = digitalRead(PIN_DIN_HSU_L);
  HSW_L_Val = digitalRead(PIN_DIN_HSW_L);          // Read the current W (or V) hall sensor value    
  direct_L = (HSU_L_Val == HSW_L_Val) ? CW : CCW;
  pulseCount_L = pulseCount_L + (1 * direct_L);
  pulseTimeU_L = startTime_L - prevTime_L;        
  AvPulseTime_L = ((pulseTimeW_L + pulseTimeU_L + pulseTimeV_L)/3);   

  if ( AvPulseTime_L ==  0 ) AvPulseTime_L=10;

  PPM_L = (1000 / AvPulseTime_L) * 60;          
//  RPM_L = PPM_L / 90;
  prevTime_L = startTime_L;
}

void blink_V_R() {
  startTime_R = millis();
  HSV_R_Val = digitalRead(PIN_DIN_HSV_R);
  HSU_R_Val = digitalRead(PIN_DIN_HSU_R);          // Read the current U (or W) hall sensor value 
  direct_R = (HSV_R_Val == HSU_R_Val) ? CW : CCW;
  pulseCount_R = pulseCount_R + (1 * direct_R);
  pulseTimeV_R = startTime_R - prevTime_R;        
  AvPulseTime_R = ((pulseTimeW_R + pulseTimeU_R + pulseTimeV_R)/3);   

  if ( AvPulseTime_R ==  0 ) AvPulseTime_R=10;

  PPM_R = (1000 / AvPulseTime_R) * 60;          
//  RPM_R = PPM_R / 90;
  prevTime_R = startTime_R;
}

void blink_V_L() {
  startTime_L = millis();
  HSV_L_Val = digitalRead(PIN_DIN_HSV_L);
  HSU_L_Val = digitalRead(PIN_DIN_HSU_L);          // Read the current U (or W) hall sensor value 
  direct_L = (HSV_L_Val == HSU_L_Val) ? CW : CCW;
  pulseCount_L = pulseCount_L + (1 * direct_L);
  pulseTimeV_L = startTime_L - prevTime_L;        
  AvPulseTime_L = ((pulseTimeW_L + pulseTimeU_L + pulseTimeV_L)/3);   

  if ( AvPulseTime_L ==  0 ) AvPulseTime_L=10;
  
  PPM_L = (1000 / AvPulseTime_L) * 60;          
//  RPM_L = PPM_L / 90;
  prevTime_L = startTime_L;
}

void blink_W_R() {
  startTime_R = millis();                         // Set startTime to current microcontroller elapsed time value
  HSW_R_Val = digitalRead(PIN_DIN_HSW_R);         // Read the current W hall sensor value
  HSV_R_Val = digitalRead(PIN_DIN_HSV_R);         // Read the current V (or U) hall sensor value 
  direct_R = (HSW_R_Val == HSV_R_Val) ? CW : CCW; // Determine rotation direction (ternary if statement)
  pulseCount_R = pulseCount_R + (1 * direct_R);   // Add 1 to the pulse count
  pulseTimeW_R = startTime_R - prevTime_R;        // Calculate the current time between pulses
  AvPulseTime_R = ((pulseTimeW_R + pulseTimeU_R + pulseTimeV_R)/3); // Calculate the average time time between pulses

  if ( AvPulseTime_R ==  0 ) AvPulseTime_R=10;

  PPM_R = (1000 / AvPulseTime_R) * 60;            // Calculate the pulses per min (1000 millis in 1 second)
//  RPM_R = PPM_R / 90;                             // Calculate revs per minute based on 90 pulses per rev
  prevTime_R = startTime_R;                       // Remember the start time for the next interrupt
}

void blink_W_L() {
  startTime_L = millis();                         // Set startTime to current microcontroller elapsed time value
  HSW_L_Val = digitalRead(PIN_DIN_HSW_L);         // Read the current W hall sensor value
  HSV_L_Val = digitalRead(PIN_DIN_HSV_L);         // Read the current V (or U) hall sensor value 
  direct_L = (HSW_L_Val == HSV_L_Val) ? CW : CCW; // Determine rotation direction (ternary if statement)
  pulseCount_L = pulseCount_L + (1 * direct_L);   // Add 1 to the pulse count
  pulseTimeW_L = startTime_L - prevTime_L;        // Calculate the current time between pulses
  AvPulseTime_L = ((pulseTimeW_L + pulseTimeU_L + pulseTimeV_L)/3); // Calculate the average time time between pulses

  if ( AvPulseTime_L ==  0 ) AvPulseTime_L=10;

  PPM_L = (1000 / AvPulseTime_L) * 60;            // Calculate the pulses per min (1000 millis in 1 second)
//  RPM_L = PPM_L / 90;                             // Calculate revs per minute based on 90 pulses per rev
  prevTime_L = startTime_L;                       // Remember the start time for the next interrupt
}

void cal_odometry_pose(unsigned long curMs)
{
    //read latest pulse number
    //current_pulse_right = ...
    //current_pulse_right = ...
    
    double cur_delta_r = 0;
    double cur_delta_l = 0;
    double delta_time  = 0;
    
    if(first_get_encode_count)
    {
       cur_delta_r = 0;
       cur_delta_l = 0;
       first_get_encode_count = false;              
    }else
    {
        cur_delta_r = current_pulse_right - last_pulse_right;
        cur_delta_l = current_pulse_left - last_pulse_left;
    }

    
    delta_time = (double) (curMs - lastCalculationTimeInMs) /1000.0;
    lastCalculationTimeInMs = curMs;
   
    last_pulse_right = current_pulse_right;
    last_pulse_left = current_pulse_left;
  
    //last_time = nh.now();
  
  
    double dright = 0;
    double dleft  = 0;
    
  ///////////////test data/////////////////////////////////////////
    //cur_delta_r = 1000;
    //cur_delta_l = 1000;   
  
  ////////////////////////////////////////////////////////
    
    if(cur_delta_r == 0 && cur_delta_l == 0)
    {
      dright = 0;
      dleft  = 0;
    
    }else
    {
      //每次采集周期行进的长度 =（当前脉冲数 - 上次脉冲数）*3.1415926 *轮胎直径/(轮子每圈脉冲总数,单位m)
      //右轮前进距离
      dright = (cur_delta_r) * PI * wheel_diameter/(circle_enc);
      //左轮前进距离
      dleft = (cur_delta_l) * PI * wheel_diameter/(circle_enc);
    }  
  
    //机器人中心点的移动距离
    double dxy_ave = (dright + dleft)/2.0;
    //机器人中心点的转动角度
    double dth = (dright - dleft)/wheel_track;
    //速度
    double vxy = dxy_ave/delta_time;
    //角速度
    double vth = dth/delta_time;
      
    if(dxy_ave != 0)
    {
      double dx = cos(dth) * dxy_ave;
      double dy = -sin(dth) * dxy_ave;
      x += (cos(theta) * dx - sin(theta) * dy);
      y += (sin(theta) * dx + cos(theta) * dy);
    }  
  
    if(dth != 0)
    {
      theta += dth;
      if(theta > 3.1415926)
      {
        theta = -3.1415926;
      }
  
      if(theta < -3.1415926)
      {
        theta = 3.1415926;
      }
    }
  
  
    //串口数据发送         
    String  str_x(x,3);
    String  str_y(y,3);
    String  str_theta(theta,3);
    String  str_vxy(vxy,3);
    String  str_vth(vth,3);
  
    String  str_last = str_x+","+str_y+","+str_theta+","+str_vxy+","+str_vth;

    //  Sending it out to ROS   
    Serial.println("$"+str_last);

    Serial.print("P");
    Serial.print(current_pulse_left);
    Serial.print(",");
    Serial.println(current_pulse_right);
}

int cycleCount = 0;

void controlBLDCmotor(uint16_t left, uint16_t right)
{ 
  //  MCP4822 
  //    INPUT     => OUTPUT 
  //    0, 2048   => 0v, 4.096v
  //       0-1250 =>   0v-2.50v
  //    1250-2047 => 2.5v-4.01v

  //  The range for left and right is [0, 255]  Middle point is 128
  if ( left  > 255 ) left  = 255;
  if ( right > 255 ) right = 255;

  left = MotorControlValues[left];
  right = MotorControlValues[right];
    
  unsigned int b = 0;  
  vspi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

 //Prepare to output 2 bytes: SPI CS assert LOW, DAC latch set high.
  digitalWrite(PIN_OUT_SPI_DAC_LD, HIGH);
  digitalWrite(PIN_OUT_SPI_DAC_CS, LOW);

  //  A value
  b = left & 0x0FFF;
  b = b | 0b1101000000000000;
  vspi->transfer(b >> 8);      // High byte
  vspi->transfer((b & 0xFF));  //Low byte

//Assert CS high
  digitalWrite(PIN_OUT_SPI_DAC_CS,HIGH);
  delayMicroseconds(1);

  digitalWrite(PIN_OUT_SPI_DAC_LD,LOW);
  delayMicroseconds(1);
  digitalWrite(PIN_OUT_SPI_DAC_LD,HIGH);
  delayMicroseconds(5);//Set a delay to simplify scope analysis


 //Prepare to output 2 bytes: SPI CS assert LOW, DAC latch set high.
  digitalWrite(PIN_OUT_SPI_DAC_CS,LOW);

  //  B value
  b = right & 0x0FFF;
  b = b | 0b0101000000000000;
  vspi->transfer(b >> 8);      // High byte
  vspi->transfer((b & 0xFF));  // Low byte

//Assert CS high
  digitalWrite(PIN_OUT_SPI_DAC_CS,HIGH);
  delayMicroseconds(1);

  //Now set the DAC to load register to output (LD Low->High).
  digitalWrite(PIN_OUT_SPI_DAC_LD,LOW);
  delayMicroseconds(1);
  digitalWrite(PIN_OUT_SPI_DAC_LD,HIGH);
  delayMicroseconds(5);//Set a delay to simplify scope analysis

  vspi->endTransaction();  
}


void loop()
{
  unsigned long currentMillis = millis();

  if ( inPowerPulsingMode && (currentMillis-pulsePowerStartTimeInMs) > 500)
  {
    inPowerPulsingMode = false;
    digitalWrite (PIN_OUT_MotorCtrlPowerPulse, LOW);
    pinMode (PIN_OUT_MotorCtrlPowerPulse, INPUT);
  }
  
  // Continuously sending soft serial commands to motor every 100 ms
  if ((currentMillis-lastMotorCmd) >= 100)
  {    
    controlBLDCmotor(cmd_left, cmd_right);
    lastMotorCmd = currentMillis;
  
//    cmd_left+=testDelta;
//    if ( cmd_left >= 255 || cmd_left <= 0 )
//      testDelta = -testDelta;
  }

  if ((currentMillis-last1second) >= 1000)
  {
    last1second = currentMillis;
    cycleCount++;
    CntSecond++;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  
//    Serial.print("A0:");
//    Serial.print(analogRead(PIN_AIN_MtrCtrl));

//    Serial.print(" Cmd:");
//    Serial.println(cmd_left);
    if ( reportBatteryInfo )
    {
      int val = 0;  // variable to store the value read
      Serial.print("V");
      val = analogRead(PIN_AIN_Battery);  // read the battery voltage
      Serial.print(val);
      Serial.print(",");
      val = analogRead(PIN_AIN_Charger);  // read the charger voltage
      Serial.println(val);
    }
  }

  serialEvent();

  if (stringComplete) {
    int inputlen = inputString.length();
//    Serial.print("inputString=");
//    Serial.println(inputString);
//    Serial.print("inputlen=");
//    Serial.println(inputlen);
//    Serial.print("inputString[0]=");
//    Serial.println(inputString[0]);

    char c1 = inputString[0];
    char c2;
    char c3;

//    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

    if ( c1 == 'C' && inputlen > 2 )
    {
      c2 = inputString[2];

/*  Testing code
      if ( c2 == '1' )
        testDelta = 1;
      if ( c2 == '2' )
        testDelta = -1; 
//        force_factor = 2;
      if ( c2 == '3' )
        force_factor = 3;
      if ( c2 == '0' )
      {
        testDelta = 0;
        cmd_left = 128;
//        force_factor = 4;
      }

      if ( c2 == '9' )
      {
        current_pulse_right = 0;
        current_pulse_left = 0;
        x = 0;
        y = 0;
      }
*/

      if ( c2 == '1' )
        force_factor = 1;
      if ( c2 == '2' )
        force_factor = 2;
//      if ( c2 == '3' )
//        force_factor = 3;

      if ( c2 == 'A' )
      {
        MotorCtrlPoweredOn = true;
        //  Motor Control Power On voltage:  Off:  206,  On: 750
        if (analogRead(PIN_AIN_MtrCtrl)<800)  //  Currently in Off mode
        {
          inPowerPulsingMode = true;
          pulsePowerStartTimeInMs = currentMillis;
          pinMode (PIN_OUT_MotorCtrlPowerPulse, OUTPUT);
          digitalWrite (PIN_OUT_MotorCtrlPowerPulse, HIGH);          
        }
      }

      if ( c2 == 'Z' )
      {
        MotorCtrlPoweredOn = false;
        //  Motor Control Power On voltage:  Off:  206,  On: 750
        if (analogRead(PIN_AIN_MtrCtrl)>800)  //  Currently in On mode
        {
          inPowerPulsingMode = true;
          pulsePowerStartTimeInMs = currentMillis;
          pinMode (PIN_OUT_MotorCtrlPowerPulse, OUTPUT);
          digitalWrite (PIN_OUT_MotorCtrlPowerPulse, HIGH);          
        }
      }
      

      if ( c2 == 'B' )  //  Enable reporting battery/charger voltage
        reportBatteryInfo = true;
      if ( c2 == 'C' )  //  Disable reporting battery/charger voltage
        reportBatteryInfo = false;

      if ( c2 == 'D' )
        digitalWrite (PIN_OUT_UVLIGHT, HIGH);
      if ( c2 == 'E' )
        digitalWrite (PIN_OUT_UVLIGHT, LOW);

      if ( c2 == 'F' )
        digitalWrite (PIN_OUT_FOGGER, HIGH);
      if ( c2 == 'G' )
        digitalWrite (PIN_OUT_FOGGER, LOW);
    }

    if ( c1 == 'D' && inputlen > 3 )
    {
      unsigned int tmp_cmd_forward = 0;
      unsigned int tmp_cmd_turn = 0;

      c2 = inputString[2];
      c3 = inputString[3];

//      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)

     if ( c2 >= 48 && c2 <= 56 )
        tmp_cmd_forward = c2 - 48;
          
      if ( c2 >= 65 && c2 <= 70 ) 
        tmp_cmd_forward = c2 - 55;
        
      if ( c3 >= 48 && c3 <= 56 )
        tmp_cmd_turn = c3 - 48;
          
      if ( c3 >= 65 && c3 <= 70 ) 
        tmp_cmd_turn = c3 - 55;

      if ( tmp_cmd_forward >= 1 && tmp_cmd_forward <= 15 && tmp_cmd_turn >= 1 && tmp_cmd_turn <= 15 )
      { //  We have valid input
        tmp_cmd_forward  = (tmp_cmd_forward-8) << 3; 
        tmp_cmd_turn = (tmp_cmd_turn-8) << 3; 

        if (force_factor==2)
        {
          tmp_cmd_forward = tmp_cmd_forward << 1;
          tmp_cmd_turn = tmp_cmd_turn << 1;
        }

        tmp_cmd_forward = tmp_cmd_forward +  128;
        tmp_cmd_turn = tmp_cmd_turn +  128;
        
        cmd_left  = tmp_cmd_forward;
        cmd_right = tmp_cmd_turn;

        Serial.println(cmd_left);
        Serial.print(" ");
        Serial.println(cmd_right);
      }      
    }
    
//    Serial.print(inputlen);
//    Serial.print(" ");
//    Serial.println(inputString);
//  (-630) - (530) No turn

    // clear the string:
    inputString = "";
    stringComplete = false;
  }

//  currentMillis = millis();

  // Continuously sending odom pose via serial (115200) commands to ROS every 100 ms
//  if ((currentMillis-last100Millis) >= 100)
//  {
//    last100Millis = currentMillis;
//    cal_odometry_pose(currentMillis);
//  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
//  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
//    Serial.print(inChar, HEX);

    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n' || inChar == 0x0A) {
      stringComplete = true;
      Serial.print("A:");
      Serial.print(inputString);
    }
  }
}

int split(char dst[2][24], char* str, char* spl)
{
    int n = 0;
    char *result = NULL;
    result = strtok(str, spl);

    while( result != NULL )
    {
        strcpy(dst[n++], result);
        result = strtok(NULL, spl);
    }
    return n;
}

void receive_control_cmd()
{
    //串口读取数据
    int count = 0;
    String cmd_str = "";
    while (true)
    {      
        if (Serial.available() > 0)//串口接收到数据
        {
            char const_header = '$';
            char incomedate = Serial.read();
            if(incomedate == const_header)
            {
                if(count != 0)
                {
                    //剔除第一个字符后拆分
                    cmd_str = cmd_str.substring(1);
                    char strg[124];
                    char sim = ',';
                    strcpy(strg,cmd_str.c_str());
                    char odom_data[12][24];
                    int split_count = 0;
                    if(cmd_str.length()>1)
                    {
                      //Serial.println(strg);
                      split_count = split(odom_data,strg,&sim);
                    }
                    
                    //获取的控制数据在odom_data中
                    for(int i =0;i<split_count;++i)
                    {
                        Serial.println(odom_data[i]);
                    }
                }
          
                count = 0;     
                cmd_str = "";
            }
          
            cmd_str += incomedate;
            count++;
        }
        else
        {
          cmd_str.trim();
          if(cmd_str != "" && cmd_str.charAt(0)== '$')
          {
            //解析下发控制命令
            cmd_str = cmd_str.substring(1);
            char strg[124];
            char sim = ',';
            strcpy(strg,cmd_str.c_str());
            char odom_data[12][24];
            int split_count = 0;
            if(cmd_str.length()>1)
            {
              //Serial.println(strg);
              split_count = split(odom_data,strg,&sim);
            }
            
            for(int i =0;i<split_count;++i)
            {
                Serial.println(odom_data[i]);
            }            
          }  
          break;
        }
    }
}
