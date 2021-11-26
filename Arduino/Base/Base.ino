// Assign your channel in pins
#define CHANNEL_A_PIN 2
#define CHANNEL_B_PIN 3

#include "TinyGPS++.h"
#include <SPI.h>
//Radio Head Library:
#include <RH_RF95.h> 


//control variables for debugging/printing
bool doPrintCalculations = false;
bool doprintRadioTrans = false;
bool doprintDataCollection =false;
bool doprintGPSInfo = false;
bool doprintPacket = false;
bool printclosedloop = true;
int packetsSent=0;

bool firsttime = true;

//rf95 setup
RH_RF95 rf95(12,6);
float frequency = 915.0;//921.2;

//gps variable setup
boolean newgps_base = false;
boolean newgps_rover = false;
TinyGPSPlus basegps;
TinyGPSPlus rovergps;
double baseLL[2];
double roverLL[2];
float roverspeed;
float rovercourse;

//keep track of whether we got a new packet THIS LOOP (avoids multiples...)
bool newPacketThisLoop = false;
unsigned long lastPacketTime = 0;
unsigned long newPacketTimeThresh = 3000;//if no new packet for 3 sec, stop panning.

int ledfreq = 0;
int ledpin = 4;

//lead for preview
float lead = 1.0;//seconds, adjustable preview



unsigned long packets_received;

byte payloadBytes[20];

struct payload_t      // 32 bytes max
{
  unsigned long counter;  // 4 bytes
  float lat;       // 4 bytes
  float lng;       // 4 bytes
  float speed;        // 4 bytes
  float course;     // 4 bytes
};

payload_t payload;

//function prototypes
void getRadioData();
void getGPS();
void calculate();



//controller setup
boolean closedloop = true;

float kp = 10;//25.0;
float ki = 0.0;
float kd = 0;//5.0;
float newcommand = 0;
float command = 0;
float oldcommand = 0;
float oldoldcommand = 0;
float battery_voltage = 4.0;

float ethresh = 0.005;//radians, so we don't keep buzzing the pwm

unsigned long microsnow = 0;
unsigned long oldmicros = 0;
int dtmicros = 1000;
float dt = .01;

float e = 0;
float dedt = 0;
float inte = 0;
float olde = 0;

//filter
//SURFING
//float a0 = 0.9391;
//float a1 = -1.9381;
//float b1 = 0.00048327;
//float b0 = 0.00047325;
//2Hz for .0033 dt
//float a0=.9202;
//float a1=-1.9185;
//float b0=.00081843;
//float b1=.00084144;
//.5Hz for dt=.0033
float a0=.9794;
float a1=-1.9793;
float b0=.000053322;
float b1=.00005369;
//0.25Hz for .0033 dt
//float a0=.9897;
//float a1 = -1.9896;
//float b0=.000013423;
//float b1=.00001347;
float oldnewcommand = 0;

volatile long unCountShared = 0;


int cpr = 12 * 250;

float posrad = 0;
float oldposrad = 0;
float velrads = 0;

int in1pin = 4;
int in2pin = 5;
int enpin = 9;

int potval;
float fV;
int V;

//things for counting, etc.
double distancem;  // distance in meters
double courseTo;  // heading
unsigned long currentMillis;
unsigned long globalMillis;
unsigned long startMillis;
int minutes;
int hours;
unsigned long millisSinceLastPacket;
unsigned long packetMillis;
double position = 0;

float omega = 0;

void setup() {
  
  // put your setup code here, to run once:
  SerialUSB.begin(115200);
  delay(3000);
  SerialUSB.println("Starting up! Firmware 12/03/2019 version");
  SerialUSB.println("initializing radio");
  if(!rf95.init()){
    SerialUSB.println("RADIO INIT FAILED!!");
  }
  else{
    SerialUSB.println("RF95 initialization complete");
  }
  rf95.setFrequency(frequency);
  rf95.setTxPower(20,false);

  Serial1.begin(9600);

  //now begin reading from the GPS and establish our current location
  startMillis = millis();
  delay(500);
  Serial.println("Initializing Base Station GPS...");
  // wait until GPS is working
//  baseLL[0] = 0.0;
//  while (baseLL[0] == 0.0) {
//    while ((Serial1.available() > 0) && baseLL[0] == 0) {
//      if (basegps.encode(Serial1.read())) {
//        SerialUSB.println("  read GPS");
//        getGPS();     // get location
////        calculate();    // calculate distance and direction
//
//        // calculate position by compass heading and courseTo (GPS)
//        // position = getPostion(getHeading() + courseTo );
//        //servopos = 90+int(position);
//        //displayDirection(); // display direction and distance
//        SerialUSB.print(baseLL[0]); SerialUSB.print(","); SerialUSB.println(baseLL[1]);
//      }
//    }
//  }
  delay(1000);
  //sGPS.end();
  globalMillis = millis();

  
  
//  while (!SerialUSB) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
  pinMode(enpin, OUTPUT);
  pinMode(in1pin, OUTPUT);
  pinMode(in2pin, OUTPUT);
  //attach the interrupts
  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  attachInterrupt(2, channelA, CHANGE);
  attachInterrupt(3, channelB, CHANGE);
  pinMode(ledpin,OUTPUT);

}

void loop() {

  //encoder read
  static long unCount;
  noInterrupts();
  unCount = unCountShared;
  interrupts();

  if((millis()-lastPacketTime)>newPacketTimeThresh){
    ledfreq = 0;
  }

  if(((millis()%ledfreq)>(ledfreq/2))||(ledfreq==0)){
  digitalWrite(ledpin,HIGH);
 }
 else{
  digitalWrite(ledpin,LOW);
 }

  //timing
  microsnow = micros();
  dtmicros = microsnow - oldmicros;
  dt = dtmicros / (1000000.0);
  oldmicros = microsnow;

  posrad = -(unCount * 2.0 * PI) / (cpr * 1.0);
  velrads = (posrad - oldposrad) / dt;

  globalMillis = millis();
  newPacketThisLoop = false;

  //packet timer
  millisSinceLastPacket = globalMillis - packetMillis;

  //do the calculations
//  calculate();

  //if enough time has passed, read a packet from the radio.
  


  //read potentiometer
  potval = analogRead(0);

  //now get and print radio data
  
  if(closedloop){
  doClosedLoop();
  }
doBaseGPS();
  getRadioData();
//  SerialUSB.print("command ");
//  SerialUSB.println(newcommand);
  newPacketThisLoop = false;
  
  delayMicroseconds(1);
}

void doBaseGPS(){
  while (Serial1.available()&&!newgps_base){
    newgps_base = (basegps.encode(Serial1.read()));
  }
  if(newgps_base){
//  SerialUSB.print("Base: \t");
//  SerialUSB.print(basegps.location.lat());
//  SerialUSB.print("\t");
//  SerialUSB.print(basegps.location.lng());
//  SerialUSB.println();
  newgps_base=false;
  baseLL[0] = basegps.location.lat();
  baseLL[1] = basegps.location.lng();
  calculate();
  calculateOmega();
  }
  
}



void doClosedLoop(){
  
  //now compute the voltage command
  if (closedloop) {
    //figure out where we want motor to go
    
    if(newPacketThisLoop||((millis()-lastPacketTime)>newPacketTimeThresh)){
      newcommand = (potval - 512.0) * 1.0 * PI / 512.0 + (-courseTo*PI/180.0) + lead*omega; //one full revolution
      
    }
    else if(((millis()-lastPacketTime)<=newPacketTimeThresh)){
      if(1){
      newcommand = (potval - 512.0) * 1.0 * PI / 512.0 + (-courseTo*PI/180.0) + lead*omega+omega*(millis()-lastPacketTime)/1000.0;
      //SerialUSB.println(millis()-lastPacketTime);
      }
    else{
      newcommand = command;
    }
    }
    //filter the command position using a second order filter
    command = -a1*oldcommand - a0*oldoldcommand + (b0*oldnewcommand)+(b1)*newcommand;
    oldoldcommand = oldcommand;
    oldcommand = command;
    oldnewcommand = newcommand;
    
    //now compute the error
    e = command - posrad;
 
    //error derivative
    dedt = (e - olde) / dt;
    //integral of error
    inte = 0;//inte + e * dt;
    //store old error value
    olde = e;
    //compute the voltage signal
    fV = kp * e + kd * dedt + ki * inte;
    //convert to counts (signed) -255 to 255 for analogWrite
    V = fV * 255.0/battery_voltage;
    if(V<-255){
      V=-255;
    }
    else if(V>255){
      V=255;
    }
  }


  else {
    //this is open loop then... so just map potentiometer position to V directly
    fV = (potval - 512) * 255.0 / 512.0;
    V = int(fV);

  }



   if(abs(V)<25 ){
      V=0;
    }
    
  if (V >= 0) {
//    digitalWrite(in1pin, HIGH);
    digitalWrite(in2pin, LOW);
    analogWrite(enpin, abs(V));
  }
  else {
//    digitalWrite(in1pin, LOW);
    digitalWrite(in2pin, HIGH);
    analogWrite(enpin, abs(V));
  }

if(printclosedloop){
  SerialUSB.print(dt, 5);
  SerialUSB.print("\t");
  SerialUSB.print(fV);
  SerialUSB.print("\t");
  SerialUSB.print(command);
  SerialUSB.print("\t");
  SerialUSB.print(posrad, 5);
  SerialUSB.print("\t");
  SerialUSB.print(e, 3);
  SerialUSB.print("\t");
  SerialUSB.print(dedt, 3);
  SerialUSB.print("\t");
  SerialUSB.print(inte, 3);
  SerialUSB.println();
  }

  
}


// simple interrupt service routine
void channelA()
{
  //Serial.println("A");
  if (digitalRead(CHANNEL_A_PIN) == HIGH)
  {
    if (digitalRead(CHANNEL_B_PIN) == LOW)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
  else
  {
    if (digitalRead(CHANNEL_B_PIN) == HIGH)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
}

void channelB()
{
  //Serial.println("B");
  if (digitalRead(CHANNEL_B_PIN) == HIGH)
  {
    if (digitalRead(CHANNEL_A_PIN) == HIGH)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
  else
  {
    if (digitalRead(CHANNEL_A_PIN) == LOW)
    {
      unCountShared++;
    }
    else
    {
      unCountShared--;
    }
  }
}



void getRadioData() {

  // Now wait for a reply
  uint8_t buf[20];
  uint8_t len = sizeof(buf);

//  if (rf95.waitAvailableTimeout(1000))
if (rf95.available())
  { newPacketThisLoop = true;
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      
      lastPacketTime = millis();
      memcpy(&payload, buf, sizeof(payload));
      roverLL[0]=payload.lat;
      roverLL[1]=payload.lng;
      roverspeed = payload.speed;
      rovercourse = payload.course;
      SerialUSB.print(millis());
      SerialUSB.print("\t");
      SerialUSB.print("got gps: ");
      SerialUSB.print(payload.lat,5);
      SerialUSB.print("\t");
      SerialUSB.print(payload.lng,5);
      SerialUSB.print("\t");
      SerialUSB.print(roverspeed,5);
      SerialUSB.print("\t");
      SerialUSB.println(rovercourse,5);
      //SerialUSB.println((char*)buf);
//      Serial.print("RSSI: ");
//      Serial.println(rf95.lastRssi(), DEC);
      
      if((payload.lng!=0.00)&&(baseLL[1]!=0.0)){  
      uint8_t heartbeat[] = "1";
      ledfreq = 2000;
      rf95.send(heartbeat,1);
      rf95.waitPacketSent();
      }
      else{
        ledfreq = 500;
        uint8_t heartbeat[]="0";
        rf95.send(heartbeat,1);
      rf95.waitPacketSent();
      }
      
      
    }
    else
    {
      SerialUSB.println("recv failed");
    }
  }

  
//  payload_t payload;
//  bool done = false;
//
//  while (!done) {
//    done =  network.read(header, &payload, sizeof(payload));
//
//    Counter = payload.counter;
//    setLL[0] = payload.lat;
//    setLL[1] = payload.lng;
//    roverspeed = payload.speed;
//    rovercourse = payload.course;
//
//    if (doprintPacket) {
//
//      SerialUSB.print("Packet #");
//      SerialUSB.print(Counter);
//      SerialUSB.print(" Lat: ");
//      SerialUSB.print(roverLL[0], 6);
//      SerialUSB.print(" Lng: ");
//      SerialUSB.print(roverLL[1], 6);
//      SerialUSB.print(" course: ");
//      SerialUSB.print(rovercourse);
//      SerialUSB.print(" speed: ");
//      SerialUSB.print(roverspeed);
//      SerialUSB.print(" Time(hr:min): ");
//      currentMillis = millis() - startMillis;
//      minutes = (currentMillis / (1000.0 * 60.0));
//      hours =  (minutes / 60.0);
//      SerialUSB.print(hours);
//      SerialUSB.print(":");
//      SerialUSB.println((int)minutes % 60);
//    }

//  }
}

void calculateOmega() {
  if (roverspeed > 0.25) { //if we are actually moving
    omega = 1 / distancem * (roverspeed * cos(((90 - courseTo) + 90 - (90 - rovercourse)) * PI / 180.0));
  }
  else {
    omega = 0;
  }
}

//get GPS data
void getGPS() {
  if (basegps.location.isValid()) {
    if ((abs(basegps.location.lat() - roverLL[0]) < .1) || (roverLL[0] == 0)) {
      if ((abs(basegps.location.lng() - baseLL[1]) < .1) || (roverLL[1] == 0)) {
        roverLL[0] = basegps.location.lat();
        roverLL[1] = basegps.location.lng();
      }
    }
  }
  else if ((baseLL[0] == 0) || (baseLL[1] == 0)) {
    baseLL[0] = basegps.location.lat();
    baseLL[1] = basegps.location.lng();
    
  }
}

//calculate distance and heading
void calculate() {
  if ((roverLL[0] != 0.0) && (roverLL[1] != 0.0)) {
    distancem = TinyGPSPlus::distanceBetween(baseLL[0], baseLL[1], roverLL[0], roverLL[1]);
    courseTo =  TinyGPSPlus::courseTo(baseLL[0], baseLL[1], roverLL[0], roverLL[1]);
    if(doPrintCalculations){
      SerialUSB.print("calculations: ");
      SerialUSB.print("distance ");
      SerialUSB.print(distancem);
      SerialUSB.print("\t");
      SerialUSB.print("Angle: ");
      SerialUSB.print(courseTo);
      SerialUSB.println();
    }
  }
}
//
//void printDataCollection() {
//  SerialUSB.print(baseLL[0], 6);
//  SerialUSB.print("\t");
//  SerialUSB.print(baseLL[1], 6);
//  SerialUSB.print("\t");
//  SerialUSB.print(roverLL[0], 6);
//  SerialUSB.print("\t");
//  SerialUSB.print(roverLL[1], 6);
//  SerialUSB.print("\t");
//  SerialUSB.print(distancem);
//  SerialUSB.print("\t");
//  SerialUSB.print(courseTo);
//  SerialUSB.print("\t");
//  SerialUSB.print(base_heading);
//  SerialUSB.print("\t");
//  SerialUSB.print(offsetval);
//  SerialUSB.print("\t");
//  SerialUSB.print(servopos);
//  SerialUSB.print("\t");
//  SerialUSB.print(servovel);
//  SerialUSB.print("\t");
//  SerialUSB.print(millisSinceLastPacket);
//  SerialUSB.print("\t");
//  SerialUSB.print(dT);
//  SerialUSB.print("\t");
//  SerialUSB.print(globalMillis);
//  SerialUSB.print("\t");
//  SerialUSB.print(omega);
//  SerialUSB.print("\t");
//  SerialUSB.print(rovercourse);
//  SerialUSB.print("\t");
//  SerialUSB.print(roverspeed);
//  SerialUSB.print("\t");
//  SerialUSB.print(courseTo);
//  SerialUSB.println();
//}
