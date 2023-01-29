#include <EEPROM.h>
#include <SoftwareSerialParityHalfDuplex.h>

#define PIN_OUT_JAZZY   10
#define PIN_OUT_UVLIGHT 7
#define PIN_OUT_FOGGER  6

#define PIN_DIN_CH_A_R  2
#define PIN_DIN_CH_B_R  4
#define PIN_DIN_CH_A_L  3
#define PIN_DIN_CH_B_L  5

#define PIN_AIN_100V    A0
#define PIN_AIN_Battery A1

SoftwareSerialParityHalfDuplex mySerial(PIN_OUT_JAZZY, PIN_OUT_JAZZY);

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;     // whether the string is complete
bool reportBatteryInfo = false;  // need to send battery / charger voltage to ROS at 10 hz
int curDelay = 1000;

char RunMode = 'R';   //  'R' for Runing,  'C' for Charging
int RunSubMode = 0;
unsigned long charging_action_begin_time_ms;
int charging_begin_odom_r;
int charging_begin_odom_l;
int charging_battery_reading;
int charging_110v_reading;
int charging_battery_low_voltage;
int charging_fullycharged_voltage;
int charging_100v_low_voltage;    //  110v is engaged
int charging_100v_high_voltage;   //  110v is disengaged
int charging_fullycharged_min_holdingtime_ms;   //  The battery need to hold charging_fullycharged_voltage for at least charging_fullycharged_min_holdingtime_ms
uint16_t charging_battery_Values[16];
int charging_battery_Values_index = 0;
  
int current_pulse_right = 0;
int current_pulse_left = 0;
int last_pulse_right = 0;
int last_pulse_left = 0;
// int force_factor = 1;

double x = 0.0;
double y = 0.0;
double theta = 0;
double linear_v = 0.0;
double angular_v = 0.0;

double wheel_track = 0.514;
double wheel_diameter = 0.254;  
double circle_enc = 40550; //轮子每圈脉冲总数

bool first_get_encode_count = true;

unsigned long last10Millis = 0;
unsigned long last100Millis = 0;
unsigned long last1second = 0;
unsigned long lastCalculationTimeInMs = 0;

unsigned int CntSecond = 0;
unsigned long JazzySerialTimeInMs = 0;
unsigned long CntJazzySerial = 0;

int cycleCount = 0;
int cmd_forward = 0; 
int cmd_turn = 0;
int cmd_speed = 1;

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
   
  pinMode(PIN_OUT_JAZZY,OUTPUT);         //  Pin for the Software Serial to Jazzy main controller
  digitalWrite (PIN_OUT_JAZZY, LOW);     //  start serial line LOW

  pinMode(PIN_OUT_UVLIGHT,OUTPUT);       //  Pin for UV light switch
  digitalWrite (PIN_OUT_UVLIGHT, HIGH);   //  start UV light line HIGH  (Setting the relay off)

  pinMode(PIN_OUT_FOGGER,OUTPUT);        //  Pin for sprayer switch
  digitalWrite (PIN_OUT_FOGGER, HIGH);    //  start sprayer line HIGH  (Setting the relay off)

  pinMode(PIN_DIN_CH_A_R,INPUT);  //  Pin for rotary encoder channel A for right wheel
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_CH_A_R), blink_A_R, RISING);

  pinMode(PIN_DIN_CH_B_R,INPUT);  //  Pin for rotary encoder channel B for right wheel

  pinMode(PIN_DIN_CH_A_L,INPUT);  //  Pin for rotary encoder channel A for left wheel
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_CH_A_L), blink_A_L, RISING);

  pinMode(PIN_DIN_CH_B_L,INPUT);  //  Pin for rotary encoder channel B for left wheel  
  
  current_pulse_right = 0;
  current_pulse_left = 0;
  last_pulse_right = 0;
  last_pulse_left = 0;

  last10Millis = millis();
  last100Millis = last10Millis;
  last1second = last10Millis;
  lastCalculationTimeInMs = last10Millis;

  Serial.begin(57600);
//  Serial.begin(115200, SERIAL_8N2);
  delay(1000);
 
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  startupChair();

  charging_battery_low_voltage = 500;                   //  0x1F4
  charging_battery_low_voltage = EEPROM_Read_WORD(0);
  Serial.print("charging_battery_low_voltage=");
  Serial.println(charging_battery_low_voltage);

  charging_fullycharged_voltage = 630;                  //  0x276            590=0x24E  600=0x258
  charging_fullycharged_voltage = EEPROM_Read_WORD(2);
  Serial.print("charging_fullycharged_voltage=");
  Serial.println(charging_fullycharged_voltage);
  
  charging_100v_low_voltage = 800;                      //  0x320   110v is engaged
  charging_100v_low_voltage = EEPROM_Read_WORD(4);
  Serial.print("charging_100v_low_voltage=");
  Serial.println(charging_100v_low_voltage);
  
  charging_100v_high_voltage = 900;                     //  0x384   110v is disengaged
  charging_100v_high_voltage = EEPROM_Read_WORD(6);
  Serial.print("charging_100v_high_voltage=");
  Serial.println(charging_100v_high_voltage);
  
  charging_fullycharged_min_holdingtime_ms = 30000; //  The fully charged voltage need to hold for at least 30 sec.
  charging_battery_Values_index = 0;
}


void blink_A_R() {
  int valB = digitalRead(PIN_DIN_CH_B_R);
 
  if ( valB == HIGH )
    current_pulse_right = current_pulse_right + 1;
  else
    current_pulse_right = current_pulse_right - 1;

//  if ( abs(Wheel_Pulse_Count_Right - Wheel_Pulse_Count_Right_LastReport) > 10 ) 
//  {
//    Wheel_Pulse_Count_Right_LastReport = Wheel_Pulse_Count_Right;
//    Serial1.print("R ");
//    Serial1.println(Wheel_Pulse_Count_Right_LastReport);    
//  }
}

void blink_A_L() {
  int valB = digitalRead(PIN_DIN_CH_B_L);

  if ( valB == HIGH )
    current_pulse_left = current_pulse_left - 1;
  else
    current_pulse_left = current_pulse_left + 1;

//  if ( abs(Wheel_Pulse_Count_Left - Wheel_Pulse_Count_Left_LastReport) > 10 ) 
//  {
//    Wheel_Pulse_Count_Left_LastReport = Wheel_Pulse_Count_Left;
//    Serial1.print("L ");
//    Serial1.println(Wheel_Pulse_Count_Left_LastReport);    
//  }
}

void blink_B_R() {
}

void blink_B_L() {
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
  
//    String  str_last = str_x+","+str_y+","+str_theta+","+str_vxy+","+str_vth;

    //  Sending it out to ROS   
//    Serial.println("$"+str_last);

//    Serial.print("P");
//    Serial.print(current_pulse_left);
//    Serial.print(",");
//    Serial.println(current_pulse_right);
}

void startupChair()
{
  digitalWrite (PIN_OUT_JAZZY, LOW);     //  start serial line LOW
  delay(1000);
  
   // activate base with 110 ms pulse
  digitalWrite (PIN_OUT_JAZZY, HIGH); 
  delay(110);
 
  // start up software serial with even parity   
  mySerial.begin(38400, EVEN);

//  Serial.print("Started up base");

  // send init packet
  unsigned char buf2[2] = { 0x52, 0xad };
  mySerial.write(buf2, 2);
 
  // base will respond, but we don't care about the response
  delay(10);
}

void sendJoystickPacket(int forward, int turn, int speed)
{
  unsigned char buffer[6] = { 0x4a, 0x01, 0x03, 0x00, 0x00, 0x00 };

  buffer[2] = speed;
  buffer[3] = forward;
  buffer[4] = turn;
  buffer[5] = 0xff - (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4]);

  mySerial.write(buffer, 6);

//  Serial.print("f=");
//  Serial.print(forward);
//  Serial.print(" t=");
//  Serial.println(turn);
}

void loop()
{
  int tmp_cmd_forward = 0;
  int tmp_cmd_turn = 0;
  unsigned long currentMillis = millis();

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
    int tmpV1 = 0;
    int tmpV2 = 0;

//    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

    if ( c1 == 'C' && inputlen > 2 )
    {
      c2 = inputString[2];

      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

      if ( c2 == '1' )
        cmd_speed = 1;
      if ( c2 == '2' )
        cmd_speed = 2;
      if ( c2 == '3' )
        cmd_speed = 3;

      if ( c2 == '9' )
      {
        current_pulse_right = 0;
        current_pulse_left = 0;
        x = 0;
        y = 0;
      }

      if ( c2 == 'A' )
      {
        startupChair();
        RunMode = 'R';
        RunSubMode = 0;
        cmd_forward = 0;
        cmd_turn = 0;
        cmd_speed = 1;
        charging_action_begin_time_ms = millis();
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

      if ( c2 == 'J' ) {    //  Ready for chargin
        RunMode = 'C';
        RunSubMode = 1;        
      }

      if ( c2 == 'H' ) {    //  Exit charging mode
        RunMode = 'C';
        charging_action_begin_time_ms = currentMillis;
        cmd_forward = -14;
        RunSubMode = 7; 
      }

      if ( c2 == 'I' && inputlen > 6 ) {  //  Set the Battery voltage
        c1 = inputString[3];

        tmpV1 = 0;
        tmpV2 = 0;
        c2 = inputString[6];
        if ( c2 >= 48 && c2 <= 57 ) tmpV1 = c2 - 48;
        if ( c2 >= 65 && c2 <= 70 ) tmpV1 = c2 - 55;
        tmpV2 += tmpV1;
        
        c2 = inputString[5];
        if ( c2 >= 48 && c2 <= 57 ) tmpV1 = c2 - 48;
        if ( c2 >= 65 && c2 <= 70 ) tmpV1 = c2 - 55;
        tmpV2 += tmpV1 << 4;

        c2 = inputString[4];
        if ( c2 >= 48 && c2 <= 57 ) tmpV1 = c2 - 48;
        if ( c2 >= 65 && c2 <= 70 ) tmpV1 = c2 - 55;
        tmpV2 += tmpV1 << 8;
        
        // Serial.println(tmpV2);
        //  charging_battery_low_voltage
        if ( c1 == 'A' )
        {
          EEPROM_Write_WORD(0, tmpV2);
//          tmpV1 = EEPROM_Read_WORD(0);
        }
        //  charging_fullycharged_voltage
        if ( c1 == 'B' )
        {
          EEPROM_Write_WORD(2, tmpV2);
//          tmpV1 = EEPROM_Read_WORD(2);
        }
        //  charging_100v_low_voltage
        if ( c1 == 'C' )
        {
          EEPROM_Write_WORD(4, tmpV2);
//          tmpV1 = EEPROM_Read_WORD(4);
        }
        //  charging_100v_high_voltage
        if ( c1 == 'D' )
        {
          EEPROM_Write_WORD(6, tmpV2);
//          tmpV1 = EEPROM_Read_WORD(6);
        }        
//        Serial.println(tmpV1);
        if ( c1 == 'E' )
        {
          EEPROM_Write_WORD(8, tmpV2);
//          tmpV1 = EEPROM_Read_WORD(6);
        }        
      }      
    }

    if ( c1 == 'D' && inputlen > 5 && RunMode == 'R' )
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

      c2 = inputString[2];
      c3 = inputString[3];
      if ( c2 >= 48 && c2 <= 57 ) tmpV1 = c2 - 48;
      if ( c2 >= 65 && c2 <= 70 ) tmpV1 = c2 - 55;
      if ( c3 >= 48 && c3 <= 57 ) tmpV2 = c3 - 48;          
      if ( c3 >= 65 && c3 <= 70 ) tmpV2 = c3 - 55;
      tmp_cmd_forward = (tmpV1 << 4) + tmpV2;

//    Serial.print("tmpV1:");
//    Serial.print(tmpV1);
//    Serial.print(" tmpV2:");
//    Serial.print(tmpV2);
//    Serial.print(" tmp_cmd_forward:");
//    Serial.println(tmp_cmd_forward);
        
      tmpV1 = 0;
      tmpV2 = 0;
      c2 = inputString[4];
      c3 = inputString[5];
      if ( c2 >= 48 && c2 <= 57 ) tmpV1 = c2 - 48;
      if ( c2 >= 65 && c2 <= 70 ) tmpV1 = c2 - 55;
      if ( c3 >= 48 && c3 <= 57 ) tmpV2 = c3 - 48;          
      if ( c3 >= 65 && c3 <= 70 ) tmpV2 = c3 - 55;
      tmp_cmd_turn = (tmpV1 << 4) + tmpV2;

//    Serial.print("tmpV1:");
//    Serial.print(tmpV1);
//    Serial.print(" tmpV2:");
//    Serial.print(tmpV2);
//    Serial.print(" tmp_cmd_turn:");
//    Serial.println(tmp_cmd_turn);

      if ( tmp_cmd_forward >= 0 && tmp_cmd_forward <= 255 && tmp_cmd_turn >= 0 && tmp_cmd_turn <= 255 )
      { //  We have valid input
        cmd_forward = (tmp_cmd_forward-128); 
        cmd_turn    = (tmp_cmd_turn-128); 

//    Serial.print("cmd_forward:");
//    Serial.print(cmd_forward);
//    Serial.print(" cmd_turn:");
//    Serial.println(cmd_turn);

        //  So input of 1 or -1 has the smallest control speed
        if ( cmd_forward > 0 )
          cmd_forward = 11 + (cmd_forward >> 1);

        if ( cmd_forward < 0 )
          cmd_forward = -(11 + ((-cmd_forward) >> 1));

        if ( cmd_turn > 0 )
          cmd_turn = 13 + (cmd_turn >> 1);

        if ( cmd_turn < 0 )
          cmd_turn = -(13 + ((-cmd_turn) >> 1));

//    Serial.print("Cmd_F:");
//    Serial.print(cmd_forward);
//    Serial.print(" Cmd_T:");
//    Serial.println(cmd_turn);
      }
    }
    
//    Serial.print(inputlen);
//    Serial.print(" ");
//    Serial.println(inputString);

    // clear the string:
    inputString = "";
    stringComplete = false;
  }

//  long currentMillis = millis();

  // Continuously sending soft serial commands to jazzy every 10 ms
  if ((currentMillis-last10Millis) >= 10)
  {
    last10Millis = currentMillis;
    CntJazzySerial++;
    
    if (cycleCount <= 1)
    {  // don't drive for first second, otherwise it will not work
      sendJoystickPacket(0, 0, 7);
    }
    else
    {
      sendJoystickPacket(cmd_forward, cmd_turn, cmd_speed);      
//  RunSubMode :  1: Idle
//                     Receive CJ command. Switch to 2
//                2: Forward
//                     Keep on apply the forward motion unless the following happens
//                     a. Moving distance > x counts
//                     b. Not moving after y sec, under highest voltage
//                     c. R,L offset > z counts
//                     d. 110v voltage sensed
//                3: Charging1
            //  a. Stop moving forward at rate of m / .1 sec
            //  b. If lose 110v change to step 2
//                4: Charging2
//                     Normal charging mode
//                     a. Keeps in this mode until the battery voltage has reached to n voltage for 30 sec
//                        Apply backing command
//                5: Backing
//                     a. Keep backing for 5 sec
      if ( RunMode == 'E' ) {
        cmd_forward = 0;
        cmd_turn = 0;
        cmd_speed = 1;
      }

      if ( RunMode == 'R' ) {
        switch(RunSubMode) {
          case 0: // Normal running mode, Monitor battery voltage
            charging_battery_reading = analogRead(PIN_AIN_Battery);  // read the battery voltage
            if ( charging_battery_reading < charging_battery_low_voltage ) {
              if ((currentMillis - charging_action_begin_time_ms) > 30000) {
                //  Battery voltage has been lower than charging low voltage for more than 30 sec
                RunSubMode = 1;
                charging_action_begin_time_ms = currentMillis;              
              }         
            }
            else {
              charging_action_begin_time_ms = currentMillis;              
            }
            break;
            
          case 1:   //  Robot is still in running mode but need charging.  The status is sent to ROS by "SR1"  
          
            break;
        }        
      }

      if ( RunMode == 'C' ) {
        switch(RunSubMode) {
          case 0: //  Idle
            break;
            
          case 1: //  Setup for Forward moving

            for( charging_battery_Values_index = 0; charging_battery_Values_index < 16; charging_battery_Values_index++ )
              charging_battery_Values[charging_battery_Values_index] = 0;
            charging_battery_Values_index = 0;            
            
            cmd_forward = 16;
            cmd_turn = 0;
            cmd_speed = 1;
            charging_action_begin_time_ms = currentMillis;
            charging_begin_odom_r = current_pulse_right;
            charging_begin_odom_l = current_pulse_left;
            RunSubMode = 2;
            break;
             
          case 2: //  Forward            
            //  Keep on apply the forward motion unless the following happens
            //    a. Moving distance > x counts
            //    b. Not moving after y sec, under highest voltage
            //    c. R,L offset > z counts
            //    d. 110v voltage sensed
            charging_110v_reading = analogRead(PIN_AIN_100V);  // read the charger voltage
            if ( charging_110v_reading < charging_100v_low_voltage ) { //  110v is engaged
              RunSubMode = 3;
              charging_action_begin_time_ms = currentMillis;
            }
            else {
              if ( abs( current_pulse_right - charging_begin_odom_r ) > 750 || abs( current_pulse_left - charging_begin_odom_l ) > 750 ) {
                RunMode = 'E';    //  Enter error mode
                RunSubMode = 1;   //  error code 1
              }
              if ( abs((current_pulse_right-charging_begin_odom_r)-(current_pulse_left-charging_begin_odom_l)) > 250 ) {
                RunMode = 'E';    //  Enter error mode
                RunSubMode = 2;   //  error code 2
              }
              if ((currentMillis - charging_action_begin_time_ms) > 30000) {
                RunMode = 'E';    //  Enter error mode
                RunSubMode = 3;   //  error code 1
              }
            }
            break;

          case 3:  //  Charging1
            //  a. Stop moving forward at rate of m / .1 sec
            //  b. If lose 110v change to step 2
            charging_110v_reading = analogRead(PIN_AIN_100V);  // read the charger voltage
            if ( charging_110v_reading > charging_100v_high_voltage ) {   //  110v is disengaged
              RunSubMode = 2;
              charging_action_begin_time_ms = currentMillis;
            }
            else {
              if ((currentMillis - charging_action_begin_time_ms) > 250) {
                charging_action_begin_time_ms = currentMillis;
                if ( cmd_forward <= 11 ) {
                  RunSubMode = 4; 
                }
                else {
                  cmd_forward--;              
                }
              }              
            }
            break;
            
          case 4:  //  Charging2
            //  a. Keep on having 110v for 5 sec
            //  b. If lose 110v change to step 2
            charging_110v_reading = analogRead(PIN_AIN_100V);  // read the charger voltage
            if ( charging_110v_reading > charging_100v_high_voltage ) {   //  110v is disengaged
              RunSubMode = 2;
              charging_action_begin_time_ms = currentMillis;
            }
            else {
              if ((currentMillis - charging_action_begin_time_ms) > 5000) {
                charging_action_begin_time_ms = currentMillis;
                RunSubMode = 5; 
              }
            }
            break;
            
          case 5:  //  Normal charging mode
            //  a. Keeps in this mode until the battery voltage has reached to n voltage for 30 sec
            getAverageBatteryVoltage();  // read the battery voltage
            if ( charging_battery_reading > charging_fullycharged_voltage ) {
              charging_action_begin_time_ms = currentMillis;
              RunSubMode = 6; 
            }
            break;

          case 6:  //  Normal charging mode
            getAverageBatteryVoltage();  // read the battery voltage
            if ( charging_battery_reading < charging_fullycharged_voltage ) {
              RunSubMode = 5;             
            }
            else {  //  The fully charged voltage need to hold for at least 30 sec.
              if ((currentMillis - charging_action_begin_time_ms) > charging_fullycharged_min_holdingtime_ms) {
                charging_action_begin_time_ms = currentMillis;
                cmd_forward = -14;
                RunSubMode = 7; 
              }
            }
            break;

          case 7:  //  Backing
            if ((currentMillis - charging_action_begin_time_ms) > 10000) {
              cmd_forward = 0;
              cmd_turn = 0;
              cmd_speed = 1;
              RunMode = 'R';
              RunSubMode = 0; 
            }          
            break;                     
        }
      }
    }

    lastCalculationTimeInMs = currentMillis;
    currentMillis = millis();
    JazzySerialTimeInMs += (currentMillis-lastCalculationTimeInMs);
  }
  // chair will respond, but we don't care

  if ((currentMillis-last100Millis) >= 100)
  {
    last100Millis = currentMillis;
//    cal_odometry_pose(currentMillis);
  }

  if ((currentMillis-last1second) >= 1000)
  {
    last1second = currentMillis;
    cycleCount++;
    CntSecond++;
//    digitalWrite(PIN_OUT_UVLIGHT, !digitalRead(PIN_OUT_UVLIGHT));
//    digitalWrite(PIN_OUT_FOGGER, !digitalRead(PIN_OUT_FOGGER));

//    Serial.print(CntSecond);
//    Serial.print(" ");
//    Serial.print(CntJazzySerial);
//    Serial.print(" ");
//    Serial.println(JazzySerialTimeInMs);

//    Serial.print("Cmd: ");
//    Serial.print(cmd_forward);
//    Serial.print(", ");
//    Serial.print(cmd_turn);
//    Serial.print(", ");
//    Serial.println(cmd_speed);

    //  Send Robot run mode back to ROS
    Serial.print('S');
    Serial.print(RunMode);
    Serial.print(RunSubMode);
//    Serial.print(',');
//    Serial.print(cmd_forward);
    Serial.print('\n');

    //  Send robot odom back to ROS
    Serial.print("P");      
    Serial.print(current_pulse_left);
    Serial.print(",");
    Serial.print(current_pulse_right);
    Serial.print('\n');

    //  Send robot battery info back to ROS
    if ( reportBatteryInfo )
    {
        Serial.print("V");
        charging_110v_reading = analogRead(PIN_AIN_100V);       // read the 110v voltage
        Serial.print(charging_110v_reading);
        Serial.print(",");
//        charging_battery_reading = analogRead(PIN_AIN_Battery); // read the battery voltage           
        getAverageBatteryVoltage(); 
        Serial.print(charging_battery_reading);
        Serial.print('\n');
    }
  }
}

void getAverageBatteryVoltage()
{
  charging_battery_reading = analogRead(PIN_AIN_Battery); // read the battery voltage            
  charging_battery_Values[charging_battery_Values_index] = charging_battery_reading;
  if ( charging_battery_Values_index >= 7 )
    charging_battery_Values_index = 0;
  else 
    charging_battery_Values_index++;

  int VoltageTotal = 0;
  for(int idx=0; idx<8; idx++)
    VoltageTotal += charging_battery_Values[idx];
  charging_battery_reading = VoltageTotal>>3;
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
//    Serial.print(inChar, HEX);

    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n' || inChar == 0x0A) {

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  
    
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

void EEPROM_Write_WORD(int addr, int val1)
{
  EEPROM.write(addr+1, val1&0xff);
  EEPROM.write(addr, (val1>>8)&0xff);
}

int EEPROM_Read_WORD(int addr)
{
  int v1 = EEPROM.read(addr);
  int v2 = EEPROM.read(addr+1);
  return (v1<<8) + v2;
}
