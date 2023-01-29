#include <SoftwareSerialParityHalfDuplex.h>

#define PIN_OUT_JAZZY   32
#define PIN_OUT_UVLIGHT 33
#define PIN_OUT_FOGGER  37
#define PIN_DIN_CH_A_R  18
#define PIN_DIN_CH_B_R  19
#define PIN_DIN_CH_A_L  20
#define PIN_DIN_CH_B_L  21
#define PIN_AIN_Battery A0
#define PIN_AIN_Charger A1


SoftwareSerialParityHalfDuplex mySerial(PIN_OUT_JAZZY, PIN_OUT_JAZZY);

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
int force_factor = 1;

double x = 0.0;
double y = 0.0;
double theta = 0;
double linear_v = 0.0;
double angular_v = 0.0;

double wheel_track = 0.514;
double wheel_diameter = 0.254;  
double circle_enc = 40550; //轮子每圈脉冲总数

bool first_get_encode_count = true;

unsigned long last25Millis = 0;
unsigned long last100Millis = 0;
unsigned long last1second = 0;
unsigned long lastCalculationTimeInMs = 0;

unsigned int CntSecond = 0;
unsigned long JazzySerialTimeInMs = 0;
unsigned long CntJazzySerial = 0;

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
   
  pinMode(PIN_OUT_JAZZY,OUTPUT);         //  Pin for the Software Serial to Jazzy main controller
  digitalWrite (PIN_OUT_JAZZY, LOW);     //  start serial line LOW

  pinMode(PIN_OUT_UVLIGHT,OUTPUT);       //  Pin for UV light switch
  digitalWrite (PIN_OUT_UVLIGHT, HIGH);   //  start UV light line HIGH  (Setting the relay off)

  pinMode(PIN_OUT_FOGGER,OUTPUT);        //  Pin for sprayer switch
  digitalWrite (PIN_OUT_FOGGER, HIGH);    //  start sprayer line HIGH  (Setting the relay off)


//  pinMode(38,OUTPUT);         
//  digitalWrite (38, HIGH);    
//  pinMode(39,OUTPUT);         
//  digitalWrite (39, HIGH);    
//  pinMode(40,OUTPUT);         
//  digitalWrite (40, HIGH);    
//  pinMode(41,OUTPUT);         
//  digitalWrite (41, HIGH);    
  
  
  pinMode(PIN_DIN_CH_A_R,INPUT);  //  Pin for rotary encoder channel A for right wheel
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_CH_A_R), blink_A_R, RISING);

  pinMode(PIN_DIN_CH_B_R,INPUT);  //  Pin for rotary encoder channel B for right wheel
//  attachInterrupt(digitalPinToInterrupt(PIN_DIN_CH_B_R), blink_B_R, RISING);

  pinMode(PIN_DIN_CH_A_L,INPUT);  //  Pin for rotary encoder channel A for left wheel
  attachInterrupt(digitalPinToInterrupt(PIN_DIN_CH_A_L), blink_A_L, RISING);

  pinMode(PIN_DIN_CH_B_L,INPUT);  //  Pin for rotary encoder channel B for left wheel
//  attachInterrupt(digitalPinToInterrupt(PIN_DIN_CH_B_L), blink_B_L, RISING);

  current_pulse_right = 0;
  current_pulse_left = 0;
  last_pulse_right = 0;
  last_pulse_left = 0;
  force_factor = 1;

  last25Millis = millis();
  last100Millis = last25Millis;
  last1second = last25Millis;
  lastCalculationTimeInMs = last25Millis;

  Serial1.begin(115200);
  delay(1000);
 
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  startupChair();
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
  
    String  str_last = str_x+","+str_y+","+str_theta+","+str_vxy+","+str_vth;

    //  Sending it out to ROS   
    Serial1.println("$"+str_last);

    Serial1.print("P");
    Serial1.print(current_pulse_left);
    Serial1.print(",");
    Serial1.println(current_pulse_right);

    if ( reportBatteryInfo )
    {
        int val = 0;  // variable to store the value read
        Serial1.print("V");
        val = analogRead(A0);  // read the battery voltage
        Serial1.print(val);
        Serial1.print(",");
        val = analogRead(A1);  // read the charger voltage
        Serial1.println(val);
    }
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

  Serial1.println("Started up base");

  // send init packet
  unsigned char buf2[2] = { 0x52, 0xad };
  mySerial.write(buf2, 2);
 
  // base will respond, but we don't care about the response
  delay(10);
}

int cycleCount = 0;
int cmd_forward = 0; 
int cmd_turn = 0;

void sendJoystickPacket(int forward, int turn, int speed)
{
  unsigned char buffer[6] = { 0x4a, 0x01, 0x03, 0x00, 0x00, 0x00 };

  buffer[2] = speed;
  buffer[3] = forward;
  buffer[4] = turn;
  buffer[5] = 0xff - (buffer[0] + buffer[1] + buffer[2] + buffer[3] + buffer[4]);

  mySerial.write(buffer, 6);

//  Serial1.print("f=");
//  Serial1.print(forward);
//  Serial1.print(" t=");
//  Serial1.println(turn);
}

void loop()
{
  int tmp_cmd_forward = 0;
  int tmp_cmd_turn = 0;

  if (stringComplete) {
    int inputlen = inputString.length();
//    Serial1.print("inputString=");
//    Serial1.println(inputString);
//    Serial1.print("inputlen=");
//    Serial1.println(inputlen);
//    Serial1.print("inputString[0]=");
//    Serial1.println(inputString[0]);

    char c1 = inputString[0];
    char c2;
    char c3;

//    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

    if ( c1 == 'C' && inputlen > 2 )
    {
      c2 = inputString[2];

//      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  
      if ( c2 == '1' )
        force_factor = 1;
      if ( c2 == '2' )
        force_factor = 2;
      if ( c2 == '3' )
        force_factor = 3;
      if ( c2 == '4' )
        force_factor = 4;

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
      c2 = inputString[2];
      c3 = inputString[3];

      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

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
        cmd_forward = (tmp_cmd_forward-8) * 2; 
        cmd_turn    = (tmp_cmd_turn-8) * 2; 

//    Serial1.println(cmd_forward);
//    Serial1.print(" ");
//    Serial1.println(cmd_turn);
      }
    }
    
//    Serial1.print(inputlen);
//    Serial1.print(" ");
//    Serial1.println(inputString);

    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  unsigned long currentMillis = millis();

  // Continuously sending soft serial commands to jazzy every 10 ms
  if ((currentMillis-last25Millis) >= 10)
  {
    last25Millis = currentMillis;
    CntJazzySerial++;
    
    if (cycleCount <= 1)
    {  // don't drive for first second, otherwise it will not work
       sendJoystickPacket(0, 0, 7);
    }
    else
    {
       sendJoystickPacket(cmd_forward*force_factor, cmd_turn*force_factor, 7);

//       cmd_forward++;
//       if ( cmd_forward > 63 )
//         cmd_forward = -63;
/*
     if (cycleCount % 10 < 5) // every other second go forward and back
          sendJoystickPacket(32, 0, 31); // drive straight forward
       else
          sendJoystickPacket(-32, 0, 31);  // drive straight backwards                 
*/
    }

    //  blinking LED every 10 Jazzy command send
    if ( ledFlashCounter == 0 )
    {
      ledFlashCounter = 10;
    }
    else
    {
      ledFlashCounter = ledFlashCounter-1;
    }
    lastCalculationTimeInMs = currentMillis;
    currentMillis = millis();
    JazzySerialTimeInMs += (currentMillis-lastCalculationTimeInMs);
  }
  // chair will respond, but we don't care

  // Continuously sending odom pose via serial (115200) commands to ROS every 100 ms
  if ((currentMillis-last100Millis) >= 100)
  {
    last100Millis = currentMillis;
    cal_odometry_pose(currentMillis);
  }

  if ((currentMillis-last1second) >= 1000)
  {
    last1second = currentMillis;
    cycleCount++;
    CntSecond++;
//    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

//    digitalWrite(PIN_OUT_UVLIGHT, !digitalRead(PIN_OUT_UVLIGHT));
//    digitalWrite(PIN_OUT_FOGGER, !digitalRead(PIN_OUT_FOGGER));
//    digitalWrite(38, !digitalRead(38));
//    digitalWrite(39, !digitalRead(39));
//    digitalWrite(40, !digitalRead(40));
//    digitalWrite(41, !digitalRead(41));

//    Serial1.print(CntSecond);
//    Serial1.print(" ");
//    Serial1.print(CntJazzySerial);
//    Serial1.print(" ");
//    Serial1.println(JazzySerialTimeInMs);
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
//    Serial1.print(inChar, HEX);
//    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // turn the LED on (HIGH is the voltage level)  

    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n' || inChar == 0x0A) {
      stringComplete = true;
      Serial1.print("A:");
      Serial1.print(inputString);
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
        if (Serial1.available() > 0)//串口接收到数据
        {
            char const_header = '$';
            char incomedate = Serial1.read();
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
                      //Serial1.println(strg);
                      split_count = split(odom_data,strg,&sim);
                    }
                    
                    //获取的控制数据在odom_data中
                    for(int i =0;i<split_count;++i)
                    {
                        Serial1.println(odom_data[i]);
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
              //Serial1.println(strg);
              split_count = split(odom_data,strg,&sim);
            }
            
            for(int i =0;i<split_count;++i)
            {
                Serial1.println(odom_data[i]);
            }            
          }  
          break;
        }
    }
}
