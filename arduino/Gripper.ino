#include <Dynamixel2Arduino.h>
#include <EEPROM.h>

//For Arduin Mega board
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL Serial1
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN

int currentOpen = -20; //RAW VALUE: -20;
int currentClose = 60; //RAW VALUE: 60;
int currentRelease = 0;//RAW VALUE: 0;
String bt = "";
String Angle;
String GripperState;
char data;
float pos;
int tmpMapStartDegree = -90;
int tmpMapStartPWM = -2550;
int tmpMapEndDegree = 90;
int tmpMapEndPWM = 3250;
int mapStartDegree = -90;
int mapStartPWM = -2550;
int mapEndDegree = 90;
int mapEndPWM = 3250;

const uint8_t DXL_ID_L = 2; //gripper motor
const uint8_t DXL_ID_R = 1; //turning motor

const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT);
  digitalWrite(13, LOW);
  
  //declare Serial2 for bluetooth
  //Plug HC-05 bluetooth module into Serial2 TX2/RX2 pin16&17
  Serial2.begin(57600);
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600/115200bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_L);
  dxl.ping(DXL_ID_R);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_L);
  dxl.torqueOff(DXL_ID_R);
  dxl.setOperatingMode(DXL_ID_L, OP_CURRENT);
  dxl.setOperatingMode(DXL_ID_R, OP_EXTENDED_POSITION);
  dxl.torqueOn(DXL_ID_L);
  dxl.torqueOn(DXL_ID_R);
  if(EEPROM.read(1) - EEPROM.read(3) > 1000 || EEPROM.read(1) - EEPROM.read(3) < -1000)
  mapStartDegree = (int8_t)EEPROM.read(0);
  mapStartPWM = (int8_t)EEPROM.read(1);
  mapEndDegree = (int8_t)EEPROM.read(2);
  mapEndPWM = (int8_t)EEPROM.read(3);
 }

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial2.available())
  {     
      //**Bluetooth
      // reading the data received from the bluetooth module
      //Please input command in the format: (x,000)*
      //where x is o(open), c(close), r(release), d(turning) or f(finish)
      //000 is the desired turning degree from -90 to 90deg
      //REMEMBER to send 'f' command before powering off!!    
      char s;
      while(Serial2.available()>0)
      {
        delay(10);  //small delay to allow input buffer to fill
        s = Serial2.read();  //gets one byte from serial buffer
        if (s == '*'){
          break;
        } //breaks out of capture loop
        bt = bt+s;
      }
      
      int posCom = bt.indexOf(',');
      int posEnd = bt.indexOf(')');       
      GripperState = bt.substring(1,posCom);
      Angle = bt.substring((posCom+1),posEnd); 

      data = GripperState.charAt(0);
      pos = Angle.toFloat();
      
      switch(data)
      {
        //open gripper
        case 'o':   // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
        // Set Goal Current using percentage (-100.0 [%] ~ 100.0[%])
          dxl.setGoalCurrent(DXL_ID_L, currentOpen);
          delay(10);
          Serial2.print("Present Current(ratio) : ");
          Serial2.println(dxl.getPresentCurrent(DXL_ID_L));
          break;

        //close gripper          
        case 'c': 
          dxl.setGoalCurrent(DXL_ID_L, currentClose);
          delay(10);
          Serial2.print("Present Current(ratio) : ");
          Serial2.println(dxl.getPresentCurrent(DXL_ID_L));
          break; 

        //release gripper a bit
        case 'r':
          dxl.setGoalCurrent(DXL_ID_L, currentRelease);
          delay(10);
          Serial2.print("Present Current(ratio) : ");
          Serial2.println(dxl.getPresentCurrent(DXL_ID_L));
          break; 

        //turn at certain degree
        case 'd':
          if (pos >=-360 && pos <= 360)
          {
          pos = map(pos, mapStartDegree, mapEndDegree, mapStartPWM, mapEndPWM);
          // Set Goal Position in raw value
          dxl.setGoalPosition(DXL_ID_R, pos);
          delay(10);
          // Print present position in degree value
          Serial2.print("Present Position: ");
          Serial2.print(dxl.getPresentPosition(DXL_ID_R, UNIT_DEGREE));
          Serial2.print("\t");
          Serial2.println(dxl.getPresentPosition(DXL_ID_R));
          }
          else
          {
            Serial2.println("Turning angle must be within 360 and -360");
          }
          
          break; 

        //turn back to ZERO before finish
        case 'f':
          pos = 30;
          // Set Goal Position in degree
          dxl.setGoalPosition(DXL_ID_R, pos, UNIT_DEGREE);
          delay(10000);
          dxl.torqueOff(DXL_ID_L);
          dxl.torqueOff(DXL_ID_R);
          // Tell user the motors are off
          Serial2.println("You may power off now");
          break;           
        case 'x':
          tmpMapStartDegree = pos;
          tmpMapStartPWM = dxl.getPresentPosition(DXL_ID_R);
          break;
        case 'y':
          tmpMapEndDegree = pos;
          tmpMapEndPWM = dxl.getPresentPosition(DXL_ID_R);
          break;
        case 'w':
          Serial2.println("Old Values:");
          Serial2.println((int8_t)EEPROM.read(0));
          Serial2.println((int8_t)EEPROM.read(1));
          Serial2.println((int8_t)EEPROM.read(2));
          Serial2.println((int8_t)EEPROM.read(3));
          mapStartDegree = tmpMapStartDegree;
          mapStartPWM = tmpMapStartPWM;
          mapEndDegree = tmpMapEndDegree;
          mapEndPWM = tmpMapEndPWM;
          EEPROM.write(0, mapStartDegree);
          EEPROM.write(1, mapStartPWM);
          EEPROM.write(2, mapEndDegree);
          EEPROM.write(3, mapEndPWM);
          Serial2.println("New Values:");
          Serial2.println((int8_t)EEPROM.read(0));
          Serial2.println((int8_t)EEPROM.read(1));
          Serial2.println((int8_t)EEPROM.read(2));
          Serial2.println((int8_t)EEPROM.read(3));
          break;
        case 'p':
          Serial2.println(dxl.getPresentPosition(DXL_ID_R));
        default : 
          break;
      }
   }
   while(!Serial2.available());
   bt = "";   //clear data for next loop
   Angle = "";
   GripperState = "";
   delay(50);
}