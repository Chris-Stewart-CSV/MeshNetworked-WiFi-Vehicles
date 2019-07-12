
#include <Servo333Hz.h>
#include <SPI.h>
#include <Console.h>

#include <Bridge.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include <HttpClient.h>

//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_U.h>
//#include <Adafruit_BMP085_U.h>
//#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_10DOF.h>

/************************** Pin/Car Info *******************************/
int servoPinSpeed = 3;          // D3 ORANGE steering actuator motor driver speed pin, moved from pin 9
int servoPinDir = 8;            // D8 steering actuator motor driver direction pin
int servoPinFF1 = 12;           // D6 steering actuator motor driver fault flag 1 pin ////////og 6 blank 5
int servoPinFF2 = 7;            // D7 steering actuator motor driver fault flag 2 pin
int servoPinPos = 0;            // A0 potentiometer in the steering actuator
int motorPinSpeed = 9;          // D9 WHITE rear wheel drive Talon motor driver PWM pin
int sonarPinDistance = 1;       // A1  sonar mounted over front bumper measurement pin
int YUNPinBridge = 13;          // D13 allocated to bridge between Arduino and Linux processors
Servo talon;

int ethernetdelay = 15;         // ms to wait for replies
int zeroToFullSpeed = 1000;     // fully accelerate or decelerate in one second max rate

/* Navigation Sensor:  Assign a unique ID to the sensors */
//Adafruit_10DOF                dof   = Adafruit_10DOF();
//Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
//Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
//Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/************************** Global Variables****************************/

unsigned long timeMillisec;          // elapsed time, millisec
int controllerValuesArray[16];  // parsed values from CSV string from remote server
int targetDirection = 0;            // target direction for vehicle, -100 to +100, 0 is straight ahead, negative is left
int presentDirection;               // reading from the steering position sensor
int sonarReading = 0;               // counts are inches to crashing and I don't mean software
int targetSpeed = 0;                // target speed for vehicle, -100 to +100, 0 is stopped, negative is reverse & =controllerValuesArray [2]
int motorSpeed = 0;                 // is the value mapped from targetSpeed, 0 to 180, to drive motors
int steeringDrive = 0;              // is mapped to targetDirection to control position of actuator and steering direction
int steeringFault;              // combined steering fault flags
int tooClose = 20;              // vehicle will stop or slow down to a predetermined speed if sonar reads obstacle within 10 inches
int turnSpeed = 115;            // speed that will be used when taking sharp turns
String controllerValues;        // webpage read from remote server
unsigned long looptime = 170;    // time in millis for loop to execute
int looptimemargin=0;
int connected2PI = LOW;
//HI or LOW speed GearState variables
#define HI 1;  //HI speed mode is number 1
int GearState = LOW;  // LOW or HI rate of speed, for indoor or outdoor modes
int oldSwitchStates[8] = {0,0,0,0,0,0,0,0}; //state of pushbutton switches on the Android control panel
int maxReverseSpeed[2]= {-35,-75};  //LOW and HIGH scale/limits Rev
int maxForwardSpeed[2]= {+20,+100}; //LOW and HIGH scale/limits Fwd
/* Update this with the correct SLP for accurate altitude measurements */
//float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float roll =0;      //readings from the accelerometer

int raspberryClientCommandsCounter = 0;

HttpClient raspberryClientCommands;
HttpClient raspberryClientTelemetry;  

/************************* Webserver Info ******************************/

/*
 * In WifiBotControl App: Data Processing / Joystick URL is: " http://192.168.0.36/updateStateNew.php? "
 * Raspberry Pi IP: 192.168.0.36
 * Arduino IP: 192.168.0.35
 * Camera IP: 192.168.0.37
 */

//byte mac[] = {0x90, 0xA2, 0xDA, 0x0E, 0x70, 0x5D};
//byte ip[] = {192, 168, 0, 35};
//byte subnet(255, 255, 255, 0);

byte raspberryIP[] = {192, 168, 0, 36};
BridgeServer server(80);

/***************************** Setup **********************************/

void setup() {

  pinMode(servoPinSpeed, OUTPUT);
  pinMode(servoPinDir, OUTPUT);
  pinMode(servoPinFF1, INPUT);
  pinMode(servoPinFF2, INPUT);
  pinMode(motorPinSpeed, OUTPUT);
  pinMode(YUNPinBridge, OUTPUT);   // this is not in the proof of concept that works...         
  
  digitalWrite(YUNPinBridge, LOW);          
  Bridge.begin(); //starts Yun Birdge
  digitalWrite(YUNPinBridge, HIGH);
  
  server.listenOnLocalhost();
  server.begin();

  targetDirection = readServoPosition();
  talon.attach(motorPinSpeed);
  talon.write(90);  //stop wheels till startup
  /**** Used to change PWM output of Arduino to 333Hz ****/
  //  Change PWM to 333 Hz (working) -Divide 62,500 (Timer 2) by 188 and you get 332.5 Hz
  //  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  //  TCCR2B = _BV(WGM22) | _BV(CS22);
  //  OCR2A = 188; //Divisor

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  /*** Danger! Will Robinson Danger! - Do no uncomment line below, the sketch will just hang and not print anything to Serial***/
  //while (!Serial){}
  /*** Danger! Will Robinson Danger! - Do no uncomment line above, the sketch will just hang and not print anything to Serial***/

  // purely for debugging the Serial communications
  //Serial.println("Hello Serial!");
  //raspberryClientCommands.getAsynchronously("http://192.168.0.36/server.php");
  
//  initSensors();
}

/****************************** Loop ***********************************/

void loop() {
  timeMillisec = millis();
  unsigned long nexttime = timeMillisec + looptime;
  ReadFromWebServerYUN();
  returnControllerValues();
  readSonar();
  calcCommandedDirectionSpeed();
  servoSteeringPosition();
  motorDriveSpeed();
  //serviceWebClientsYUN();
  serviceWebClientsPush();
  
  looptimemargin=(nexttime - millis()); // Verify time to execute loop
  while(millis() < nexttime) {} // Waits for millis() = nexttime; continues if missed

}

/*************************** Subroutine ******************************/

void ReadFromWebServerYUN(){
  String oldcontrollerValues = controllerValues;
  controllerValues = "";
  if (raspberryClientCommands.ready()){
    while(raspberryClientCommands.available()) {
      char c = raspberryClientCommands.read();
      controllerValues += c;      
    }
    connected2PI = HIGH;
    raspberryClientCommandsCounter = 0; 
  } else {
    if(connected2PI = HIGH){
      controllerValues = oldcontrollerValues;
    }
    raspberryClientCommandsCounter--; 
    if(raspberryClientCommandsCounter == 0){
      connected2PI = LOW;
    }
  }
  
  //Perform "GET" at end of loop, then website has looptime to respond...
  if(raspberryClientCommandsCounter <=0){
    raspberryClientCommands.getAsynchronously("http://192.168.0.36/server.php");
    raspberryClientCommandsCounter = 3;
  }
}

/**************************************** Subroutine ****************************************/

//void serviceWebClientsYUN(){
//  BridgeClient client = server.accept();
//  
//  if(client){
//    Serial.println("new client");
//    // an http request ends with a blank line
//    boolean currentLineIsBlank = true;
//    while (client.connected()) {
//      if (client.available()) {
//        char c = client.read();
//        Serial.print(c);
//        // if you've gotten to the end of the line (received a newline
//        // character) and the line is blank, the http request has ended,
//        // so you can send a reply
//        if (c == '\n' && currentLineIsBlank) {
//          // send a standard http response header
//          client.println("HTTP/1.1 200 OK");
//          client.println("Content-Type: text/html");
//          client.println("Connection: close");  // the connection will be closed after completion of the response
//          
//          client.println("Refresh: .25");  // refresh the page automatically every 5 sec
//          client.println();
//          client.println("<!DOCTYPE HTML>");
//          client.println("<html>");
//          // output the value of each analog input pin
//          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
//            int sensorReading = analogRead(analogChannel);
//            client.print("analog input ");
//            client.print(analogChannel);
//            client.print(" is ");
//            client.print(sensorReading);
//            client.println("<br />");
//          } // end for loop reading analog pins
//          
//          client.println("</html>");
//          break;
//        }
//        if (c == '\n') {
//          // you're starting a new line
//          currentLineIsBlank = true;
//        } else if (c != '\r') {
//          // you've gotten a character on the current line
//          currentLineIsBlank = false;
//        }
//      }
//    }
//    // give the web browser time to receive the data
//    delay(ethernetdelay);
//    // close the connection:
//    client.stop();
//    Serial.println("client disconnected");
//   
//  }
//  
//}

/**************************************** Subroutine ****************************************/

void serviceWebClientsPush(){
//   if(raspberryClientTelemetry.ready()){   //Flush the buffer if any...
//   while(raspberryClientTelemetry.available()) {
//     char c = raspberryClientTelemetry.read();
//   }}
  String WEBSITE = String((byte) raspberryIP[0])+"."+String((byte)raspberryIP[1])+"."+String((byte)raspberryIP[2])+"."+String((byte)raspberryIP[3]);
  String strTelemetry = 
    String("Elapsed") + "=" + String((unsigned long)timeMillisec)+ "&"+
    "targetDirection"+"="+String((int) targetDirection)+"&"+
    "presentDirection"+"="+String((int) presentDirection)+"&"+
    "sonarReading"+"="+String((int) sonarReading)+"&"+
    "motorSpeed"+"="+String((int) motorSpeed)+"&"+
    "steeringFault"+"="+String((int) steeringFault)+"&"+
    "looptimemargin"+"="+String((int) looptimemargin);

  //String strDataToSend = "GET /var/www/html/arduinoPush.php?"+ strTelemetry + " HTTP/1.1\r\nHost:" + WEBSITE + "\r\n";
  //String strDataToSend.getAsynchronously = "http://" + WEBSITE + "/arduinoPush.php?"+ strTelemetry;
  //strDataToSend.ready();


  raspberryClientTelemetry.getAsynchronously("http://" + WEBSITE + "/arduinoPush.php?"+ strTelemetry);
  //raspberryClientTelemetry.ready();

  //Serial.println("http://" + WEBSITE + "/arduinoPush.php?"+ strTelemetry);
  
    
  //BridgeClient clientPush = server.accept();
  //clientPush.println(strDataToSend);
  //clientPush.println();  //clientPush.fastrprint(F("\r\n"));
  //clientPush.println();

  //raspberryClientTelemetry.close();

}

/**************************************** Subroutine ****************************************/

void servoSteeringPosition()
{  
  presentDirection = readServoPosition();         // steering position sensor reading, in joystick coordinates ~ -100 to +100
  int directionchange = targetDirection - presentDirection;  // amount position sensor needs to change, in joystick counts

  if (directionchange >=0) {
  digitalWrite(servoPinDir, HIGH);
  } else{
      digitalWrite(servoPinDir, LOW);
  }

  steeringDrive = abs(directionchange)*900.0/looptime;  //based on calibration=1534.7, positioner moved 10 counts in 50ms at analogWrite(250)
  if (steeringDrive<50) {steeringDrive=0;}  //turn off motor for drive levels too low to move the motor
  steeringDrive= min(steeringDrive, 255);
  //steeringDrive = map(controllerValuesArray[1], -100, 100, 0, 255);
  analogWrite(servoPinSpeed, steeringDrive);
  steeringFault = digitalRead(servoPinFF1)+2*digitalRead(servoPinFF2);
  Serial.print("Steering Cmd, ");  
  Serial.print(directionchange); Serial.print(", "); //
  Serial.print(steeringDrive); Serial.print(", "); // this is the value tht is returned after mapping the joystick to the actuator
  Serial.print(steeringFault); Serial.print(", "); 
  Serial.print(presentDirection); Serial.print(", "); // 
  Serial.print(targetDirection); Serial.print(", "); // the value given by our joystick
  Serial.println(); // returns 10
}

/**************************************** Subroutine ****************************************/

int readServoPosition()
{
// Original steering code
//  int servoVal = analogRead(servoPinPos); // will read value from potentiometer (between 0 and 1023)
//  return map(servoVal, 54, 876, -100,100); // map the ADC reading to joystick coordinates (supposed to be 0 to 1023, is actually 54 to 876)

// Steering code to stop steering wheel twitching (takes median of three read values to eliminate noisy outliers)
  const int numReadings = 3;
  int servoVal[numReadings];
    for(int thisReading = 0; thisReading < numReadings; thisReading++){
    servoVal[thisReading] = analogRead(servoPinPos);
    delay(1);
  }

  int medianReading = integerMedian(servoVal, numReadings);  //based on QuickStats float library
  return map(medianReading, 54, 876, -100, 100);
}


/**************************************** Subroutine ****************************************/
int integerMedian(int samples[],int m) //calculate the median
{
  //First bubble sort the values: https://en.wikipedia.org/wiki/Bubble_sort
  int sorted[m];   //Define and initialize sorted array.
  int temp=0;      //Temporary float for swapping elements
  /*Serial.println("Before:");
  for(int j=0;j<m;j++){
    Serial.println(samples[j]);
  }*/
  for(int i=0;i<m;i++){
    sorted[i]=samples[i];
  }
  intbubbleSort(sorted,m);  // Sort the values
  /*Serial.println("After:");
  for(int i=0;i<m;i++){
    Serial.println(sorted[i]);
  }*/
  if (bitRead(m,0)==1) {  //If the last bit of a number is 1, it's odd. This is equivalent to "TRUE". Also use if m%2!=0.
    return sorted[m/2]; //If the number of data points is odd, return middle number.
  } else {    
    return (sorted[(m/2)-1]+sorted[m/2])/2; //If the number of data points is even, return avg of the middle two numbers.
  }
}

void intbubbleSort(int A[],int len) {
  unsigned long newn;
  unsigned long n=len;
  int temp=0;
  do {
    newn=1;
    for(int p=1;p<len;p++){
      if(A[p-1]>A[p]){
        temp=A[p];           //swap places in array
        A[p]=A[p-1];
        A[p-1]=temp;
        newn=p;
      } //end if
    } //end for
    n=newn;
  } while(n>1);
}

/**************************************** Subroutine ****************************************/

void motorDriveSpeed()
{
  //motorSpeed = map(controllerValuesArray[2], -100, 100, 0, 180); //motorSpeed is global variable and is called in steering to limit speed on turns
  motorSpeed = map(targetSpeed, -100, 100, 0, 180); // +- 100 is for tilt controller +- 57 is for thumb control
  talon.write(motorSpeed);
  //Serial.println(motorSpeed);
}

/**************************************** Subroutine ****************************************/

void readSonar()
{
    sonarReading = analogRead(sonarPinDistance);
    //Serial.println(sonarReading);  //closer than 15 counts, count increases to 30 or so!!!
    //Serial.println(sonarReading);
}

/**************************************** Subroutine ****************************************/

void returnControllerValues()
{  // parse comma-separated-values string to integers
  String remains = controllerValues;
  int last = sizeof(controllerValuesArray)/sizeof(controllerValuesArray[0]);
  for (int i = 0; i < last; i++){
     int commaIndex = remains.indexOf(','); 
     controllerValuesArray[i]=remains.toInt();  //returns zero if blank
     remains = remains.substring(commaIndex+1);
  }
}

/**************************************** Subroutine ****************************************/

int sign(int number)
{
  if(number>0)
  {
    return 1;
  } else if(number<0){
   return -1;
  } else{
   return 0;
  }
}

/**************************************** Subroutine ****************************************/

void calcCommandedDirectionSpeed()
{
  //determine gearing HI or LOW
  //controllerValuesArray[4] = number of the button pushed (1,2,3,4...) or null (0) if not held pushed
  //button 1 will be the GearState switch.  This logic converts pushbutton to toggle on push (not release)
  int switchPressed = controllerValuesArray[4];
  if ((switchPressed==1) && (oldSwitchStates[0]==0)){GearState = !GearState;}  //detect pushbutton 1 pushed
  if (switchPressed) {oldSwitchStates[switchPressed-1]=1;} else {for( int i=0; i<8; i++){oldSwitchStates[i] = 0;}}
  //Serial.print(GearState);
  
  int oldtargetSpeed = targetSpeed;
  targetSpeed = controllerValuesArray[2];
  int oldtargetDirection = targetDirection;
  if (controllerValues > "") {targetDirection = controllerValuesArray[1];} 

  //direction, scale for more sensitivity; joystick was only using 50% of steering range
  targetDirection = targetDirection *2;
  targetDirection = constrain(targetDirection, -100, +100);

  // speed, scale joystick depeding on GearState HI or LOW
  if(targetSpeed > 0){
    targetSpeed = map(targetSpeed, 0, 100, 0, maxForwardSpeed[GearState]);
  }else{
    targetSpeed = map(targetSpeed, -100, 0, maxReverseSpeed[GearState], 0);
  }
  
  // speed, if sonarReading from subroutine <= 10 inches away then it slows or stops vehicle if too close to obstacle
  targetSpeed = min(1.2 * sonarReading, targetSpeed);
  if(sonarReading <= tooClose && sign(targetSpeed) > 0)
  {
    targetSpeed = 0;  //stop 
  }
 
  //speed, limit reverse speed because no sonar in reverse
  targetSpeed = max(maxReverseSpeed[GearState], targetSpeed);

  //speed, limit speed for indoors with push button 1
  targetSpeed = min(maxForwardSpeed[GearState], targetSpeed);
  
  //speed, constrain rate of speed change
  int maxspeedchange = 100/(zeroToFullSpeed/looptime);  //100 joystick counts (100 to zero) in (zeroToFullSpeed time, per looptime cycle) 
  int speedchange = targetSpeed - oldtargetSpeed;
  targetSpeed = oldtargetSpeed + sign(speedchange)*min(abs(speedchange),maxspeedchange);

  ///Serial.print(targetSpeed); Serial.println("");
}

/**************************************** Subroutine ****************************************/

//void navReadings(){
//
//  // reserve memory space
//  StaticJsonBuffer<200>jsonBuffer;
//  // build object tree in memory
//  JsonObject& root  
//}

//void updateNavigationReadings() {
//
//  sensors_event_t accel_event;
//  sensors_event_t mag_event;
//  sensors_event_t bmp_event;
//  sensors_vec_t   orientation;
//
//  /* Calculate pitch and roll from the raw accelerometer data */
//  accel.getEvent(&accel_event);
//  if (dof.accelGetOrientation(&accel_event, &orientation))
//  {
//    /* 'orientation' should have valid .roll and .pitch fields */
//    Serial.print(F("Roll: "));
//    Serial.print(orientation.roll);
//    Serial.print(F("; "));
//    Serial.print(F("Pitch: "));
//    Serial.print(orientation.pitch);
//    Serial.print(F("; "));
//  }
//  
//  /* Calculate the heading using the magnetometer */
//  mag.getEvent(&mag_event);
//  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
//  {
//    /* 'orientation' should have valid .heading data now */
//    Serial.print(F("Heading: "));
//    Serial.print(orientation.heading);
//    Serial.print(F("; "));
//  }
//
//  /* Calculate the altitude using the barometric pressure sensor */
//  bmp.getEvent(&bmp_event);
//  if (bmp_event.pressure)
//  {
//    /* Get ambient temperature in C */
//    float temperature;
//    bmp.getTemperature(&temperature);
//    /* Convert atmospheric pressure, SLP and temp to altitude    */
//    Serial.print(F("Alt: "));
//    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
//                                        bmp_event.pressure,
//                                        temperature)); 
//    Serial.print(F(" m; "));
//    /* Display the temperature */
//    Serial.print(F("Temp: "));
//    Serial.print(temperature);
//    Serial.print(F(" C"));
//  }
//  
//  Serial.println(F(""));
//  delay(1000);
//}
//
//void initSensors()
//{
//  if(!accel.begin())
//  {
//    /* There was a problem detecting the LSM303 ... check your connections */
//    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
//    while(1);
//  }
//  if(!mag.begin())
//  {
//    /* There was a problem detecting the LSM303 ... check your connections */
//    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
//    while(1);
//  }
//  if(!bmp.begin())
//  {
//    /* There was a problem detecting the BMP180 ... check your connections */
//    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
//    while(1);
//  }
//}


