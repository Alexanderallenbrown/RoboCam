#include "TinyGPS++.h"
#include <SPI.h>
//Radio Head Library:
#include <RH_RF95.h> 

// We need to provide the RFM95 module's chip select and interrupt pins to the
// rf95 instance below.On the SparkFun ProRF those pins are 12 and 6 respectively.
RH_RF95 rf95(12, 6);
// The broadcast frequency is set to 921.2, but the SADM21 ProRf operates
// anywhere in the range of 902-928MHz in the Americas.
// Europe operates in the frequencies 863-870, center frequency at 868MHz.
// This works but it is unknown how well the radio configures to this frequency:
//float frequency = 864.1; 
float frequency = 915.0;//921.2; //Broadcast frequency

int ledfreq = 0;//start with a solid light
int ledpin =4;
int lastReceiveTime = 0;
int lastHBTime = 0;

bool sendthistime = true;

long beforesend = 0;
long aftersend = 0;
unsigned long lastsendtime = 0;

boolean newgps = false;

// The TinyGPS++ object
TinyGPSPlus gps;

// GPS Latitude[0]/Longitude[1]
float myLL[2];		//current GPS location
float myspeed;
float mycourse;

//commands for GPS

//disable, GPGSA
byte GPGSA[]={0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x53, 0x41, 0x2a, 0x33, 0x33, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x02, 0x00, 0xfc, 0x13};
//disable, GPGSV
byte GPGSV[]={0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x53, 0x56, 0x2a, 0x32, 0x34, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x03, 0x00, 0xfd, 0x15};
//disable, GPGLL
byte GPGLL[]= {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x4c, 0x4c, 0x2a, 0x32, 0x31, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x01, 0x00, 0xfb, 0x11};
//disable, GPGGA
byte GPGGA[]={0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x47, 0x47, 0x41, 0x2a, 0x32, 0x37, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x00, 0x00, 0xfa, 0x0f};
//disable, GPGTV
byte GPGTV[]={0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2c, 0x56, 0x54, 0x47, 0x2a, 0x32, 0x33, 0x0d, 0x0a, 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x05, 0x00, 0xff, 0x19};
//update, 5Hz
byte UPDATE5Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, 0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
//baud, rate, 38400
byte BAUD38400[]={0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0xa8};
//baud rate 115200
byte BAUD115200[]={ 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc4, 0x96, 0xb5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22};
//save, configuration
byte SAVECONFIG[]={0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};
//reset to factory
byte FACTORYRESET[]={ 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x07, 0x1F, 0x9E};
byte UPDATE1HZ[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};



// How many packets have we sent already
unsigned long packets_sent=0;

// Structure of our payload, limited to 32 bytes
struct payload_t			// 32 bytes max
{
  unsigned long counter;	// 4 bytes
  float lat;				// 4 bytes
  float lng;				// 4 bytes
  float speed;				// 4 bytes
  float course;     // 4 bytes
};

byte payloadBytes[20];

// Analog pins
#define VccPin 2
// Digital pins
#define ActivePin 9

float Vcc; 	//Supplied Voltage

// Prototypes
void getGPS();							// get GPS data
static void smartDelay(unsigned long);	// ensures that the gps object is being "fed".
//void getVoltage();						// getVoltage
void sendPayload();						// check if time to send payload

void setup(void)
{
  pinMode(ledpin,OUTPUT);
  delay(1000);
  Serial1.begin(9600);
  //now write commands to disable all but GPRMC
  Serial1.write(GPGSA, sizeof(GPGSA));
  delay(100);
  Serial1.write(GPGSV, sizeof(GPGSV));
  delay(100);
  Serial1.write(GPGGA, sizeof(GPGGA));
  delay(100);
  Serial1.write(GPGSA, sizeof(GPGSA));
  delay(100);
  Serial1.write(GPGTV,sizeof(GPGTV));
  delay(100);
  Serial1.write(UPDATE5Hz,sizeof(UPDATE5Hz));
  //Serial.write(UPDATE1HZ,sizeof(UPDATE1HZ));
  delay(100);
  Serial1.write(SAVECONFIG,sizeof(SAVECONFIG));
  delay(100);
  Serial1.write(BAUD38400,sizeof(BAUD38400));
  //sGPS.write(BAUD115200,sizeof(BAUD115200));
  delay(100);
  Serial1.end();
  delay(100);
  Serial1.begin(38400);
  delay(100);

//  Serial1.begin(9600);
  SerialUSB.begin(9600); 
	SPI.begin();
  SerialUSB.println("Starting UP....");
  delay(2000);
  if(!rf95.init()){
    SerialUSB.println("RADIO INIT FAILED!!");
  }
  else{
    SerialUSB.println("RF95 initialization complete");
  }
  // Set frequency
  rf95.setFrequency(frequency);

   // Transmitter power can range from 14-20dbm.
   rf95.setTxPower(20, false);
}

void loop(void){
//  SerialUSB.print("time since last HB: ");
//  SerialUSB.println(millis()-lastHBTime);
  if((millis()-lastHBTime)>3000){
  ledfreq = 0;
 }

 if(((millis()%ledfreq)>(ledfreq/2))||(ledfreq==0)){
  digitalWrite(ledpin,HIGH);
 }
 else{
  digitalWrite(ledpin,LOW);
 }

  if(!Serial1.available()){
    //SerialUSB.println("Nothing in the queue");
  }
  
	while (Serial1.available()&&(newgps!=1)){
  //gps.encode(sGPS.read());
//  SerialUSB.println("I see bytes but no GPS yet");
//  SerialUSB.write(Serial1.read());
  newgps = (gps.encode(Serial1.read()));
//SerialUSB.println(gps.charsProcessed());
//Serial.println(sGPS.overflow());
//Serial.println(gps.failedChecksum());
	}
 if(newgps){
  SerialUSB.println("trying to send:");
  getGPS();
  beforesend = millis();
  if((millis()-lastsendtime)>=1000){
  sendPayload();
  lastReceiveTime = millis();
  }
  aftersend = millis();
  SerialUSB.print("sending ms: ");
  SerialUSB.println(aftersend-beforesend);
  newgps=false;
 }
 if (rf95.available()){
  uint8_t buf[20];
  uint8_t len = sizeof(buf);
  if(rf95.recv(buf,&len)){
  
  SerialUSB.print("at time: ");
  SerialUSB.print(lastsendtime);
  SerialUSB.print("got heartbeat: ");
  SerialUSB.println(int(buf[0])); 
  lastHBTime = millis();
  if(int(buf[0])==49){
    ledfreq = 2000;
  }
  else{
    ledfreq = 500;
  }
 }
 }
 
   
 //set newgps to 0 so we go back into while loop after success
 newgps=0;
}

//////////////////////////////////////////////////////////////////
// getGPS
//////////////////////////////////////////////////////////////////
void getGPS(){
	if (1){//gps.location.isValid()){
		myLL[0] = float(gps.location.lat());
		myLL[1] = float(gps.location.lng());
   myspeed = float(gps.speed.mps());
   mycourse = float(gps.course.deg());
   if((millis()-lastsendtime)>500){
SerialUSB.print(packets_sent);
SerialUSB.print("\t lat: ");
SerialUSB.print(myLL[0],7);
SerialUSB.print("\t lng: ");
SerialUSB.print(myLL[1],7);
SerialUSB.print("\t speed: ");
SerialUSB.print(myspeed,7);
SerialUSB.print("\t course: ");
SerialUSB.println(mycourse,7);
   }

	}
}


//////////////////////////////////////////////////////////////////////////////////
// sendPayload();					// send payload
//////////////////////////////////////////////////////////////////////////////////
void sendPayload(){
  float latt = myLL[0];
  float lonn = myLL[1];
	payload_t payload = { packets_sent++, latt, lonn, myspeed,mycourse };
 //*((payload_t *)payloadBytes) = payload;

 SerialUSB.print("trying to send bytes: ");
 SerialUSB.println(sizeof(payload));
//	rf95.send(payloadBytes,sizeof(payloadBytes));
 rf95.send((uint8_t*)&payload, sizeof(payload));
  rf95.waitPacketSent();//can put timeout in milliseconds here.
}
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
