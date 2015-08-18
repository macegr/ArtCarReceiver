// Art Car 2011
// Receiver Module
// Garrett Mace


#define debugMode 0

// Included libraries
#include <Servo.h>


// Steering output range
#define steerMax 2000
#define steerMin 1000
#define steerDefault 1500

// Braking output range
#define brakeMax 2000
#define brakeMin 1000
#define brakeDefault 1000

// Throttle output range
#define throttleMax 255
#define throttleMin 0
#define thottleDefault 0

// I/O pin definitions
#define steerPin 7
#define brakePin 8
#define throttlePin 5
#define enableRS485 2
#define statusLED 10


// Object declaration
Servo steerServo;
Servo brakeServo;

// Global variables
int steerValue = 0;
int brakeValue = 0;
int throttleValue = 0;
String ack = char(0x02) + String("OK") + char(0x03);
long activityLEDTimer = 0;
int activityFlag = 0;

unsigned long watchdogTimer = 0;
int errorLEDCount = 0;
byte watchdog = 1;


// Initialize sequence
void setup()
{
  steerServo.attach(steerPin); // Activate steering servo
  brakeServo.attach(brakePin); // Activate brake servo
 
  Serial.begin(57600); // Activate serial port

  // Communication status LED
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED,HIGH);

  // RS485 flow control pin
  pinMode(enableRS485, OUTPUT);
  digitalWrite(enableRS485, LOW);

  // Throttle output pin
  analogWrite(throttlePin, 0);
  
}

// Receive buffer size
#define recvMax 14
int recvIndex = 0;
byte recvBuffer[recvMax] = {0};

// Communication state definitions
  #define IDLE 0
  #define RECEIVE 1
  #define TRANSMIT 2
  #define RXDONE 3
  #define TXDONE 4
  #define ERROR 5

int packetState = IDLE;


// Run CRC on the received data for integrity check
byte doChecksum() {
 
 byte crc = 0, i;
 for (i = 0; i < 12; i++) {
   crc = crc + recvBuffer[i];
 }
 return crc;
}


// Convert receive hexadecimal values to decimal values
byte hex2dec(byte hex) {

  if (hex >= 48 && hex <= 57) {
    return (hex-48);
  } else if (hex >= 97 && hex <= 102) {
    return (hex-87);
  }

}


// Check receive buffer for valid packet and store the data
void checkRS485()
{
  byte tempByte;
  byte serialFloodDetect = 0;
  
  while (Serial.available() > 0 && serialFloodDetect < 50)
  {
    
    tempByte = Serial.read();
    serialFloodDetect++;
    
    if (packetState == RECEIVE) {
      // end of packet
      if (tempByte == 0x03) {
        byte checksum = (hex2dec(recvBuffer[recvIndex-2]))*16 + hex2dec(recvBuffer[recvIndex-1]);
        recvBuffer[12] = checksum;
        
        if (doChecksum() == checksum) {
          packetState = RXDONE;                 
        } else {
          packetState = ERROR;
        }
        // receive data
      } else if (recvIndex < recvMax) {
        recvBuffer[recvIndex] = tempByte;
        recvIndex++;
      } else {
        packetState = ERROR;
      }
     
    }
    
    // start of new packet
    if (tempByte == 0x02) {
      recvIndex = 0;
      packetState = RECEIVE;
    }
    
  }
  
  serialFloodDetect = 0;
  
}


// Convert packet value into an integer
int getPacketValue(byte index) {
 
  int value = (recvBuffer[index*4]-48)*1000;
  value += (recvBuffer[index*4+1]-48)*100;
  value += (recvBuffer[index*4+2]-48)*10;
  value += (recvBuffer[index*4+3]-48); 
  return value;
  
}

// Send received commands to the control systems
void writeOutputs() {
 
  if (packetState != ERROR && watchdog == 0) {

    // Write brake value to the brake servo controller
    brakeValue = getPacketValue(1);
    
    if (brakeValue > brakeMax) {
      brakeValue = brakeMax;
    } else if (brakeValue < brakeMin) {
      brakeValue = brakeMin;
    }
    
    brakeServo.write(brakeValue);

    // Write steering value to the steering servo controller
    steerValue = getPacketValue(0);
    steerServo.write(steerValue);

    // Write throttle value to the throttle PWM output
    throttleValue = getPacketValue(2);
    if (throttleValue >= throttleMin && throttleValue <= throttleMax) {
      analogWrite(throttlePin, throttleValue);
    }

  }
  
}






void checkWatchdog() {

  // Watch for communication disruption
  if (millis() - watchdogTimer > 200) { // Trigger watchdog if no valid data in 200ms
      watchdog = 1;
      // steerServo.detach();  // Shut down steering
      // brakeServo.detach();  // Shut down brakes
      analogWrite(throttlePin, 0); // Shut down throttle
  }
  
}


void processPacket() {



  //if (packetState == ERROR) Serial.println("ERROR");

  if (packetState == RXDONE) {
        
    // Reset watchdogtimer
    watchdogTimer = millis();
    watchdog = 0;
    
    // Reset intial values
    watchdog = 0;
    packetState = IDLE;
    writeOutputs(); 
    delay(1);
    

    // Send packet acknowledgement
    digitalWrite(enableRS485, HIGH);  // enable RS485 transmit
    Serial.print(ack);                // sendt serial packet
    Serial.flush();                   // wait for transmission to complete 
    digitalWrite(enableRS485,LOW);    // disable RS485 transmit
    
    digitalWrite(statusLED, LOW);
    activityFlag = 1;
    activityLEDTimer = millis()+10;

  }

   // Turn off activityLED after a delay
   if (activityFlag == 1 && millis() > activityLEDTimer) {
     activityFlag = 0;
     digitalWrite(statusLED, HIGH);
   }
    
}


// Main control loop
void loop()
{

  checkWatchdog();
  checkRS485();
  processPacket();

}