#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#define CPU_MHZ 80
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1000  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 5 //set PPM signal output pin on the arduino
#define DEBUGPIN 4
volatile unsigned long next;
volatile unsigned int ppm_running=1;
int ppm[8];
const byte captive_portal=0;
#define SC1 D1 // ena1 pin of motor driver/speed control of motor 1
#define SC2 D2 // ena2 pin of motor driver/speed control of motor 2
#define M1 D5 // motor1 wire 1
#define M2 D6 // motor1 wire 2
#define M3 D7 // motor2 wire 1
#define M4 D8 // motor2 wire 2

int spd = 500;

unsigned int alivecount=0;

unsigned long time_now = 0;
const char* ssid     = "OnePlus 8T";
const char* password = "chandupurna";


WebSocketsServer webSocket = WebSocketsServer(81);

//....................forward()....................
void forward()
{
  analogWrite(SC1, spd); // writing the speed to left motor
  analogWrite(SC2, spd); // writing the speed to right motor
  // left motor forward direction
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  //right motor forward direction
  digitalWrite(M3, HIGH);
  digitalWrite(M4, LOW);
  delay(50);
  Serial.println("FORWARD");
}
//....................backward()....................
void backward()
{
  analogWrite(SC1, spd); // writing the speed to left motor
  analogWrite(SC2, spd); // writing the speed to right motor
  // left motor backward direction
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  //right motor backward direction
  digitalWrite(M3, LOW);
  digitalWrite(M4, HIGH);
  delay(50);
  Serial.println("BACKWARD");
}
//....................left()....................
void left()
{
  analogWrite(SC1, spd); // writing the speed to left motor
  analogWrite(SC2, spd); // writing the speed to right motor
  // left motor backward direction
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  //right motor forward direction
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);
  delay(50);
  Serial.println("LEFT");
}
//....................right()....................
void right()
{
  analogWrite(SC1, spd); // writing the speed to left motor
  analogWrite(SC2, spd); // writing the speed to right motor
  // left motor forward direction
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  //right motor backward direction
  digitalWrite(M3, LOW);
  digitalWrite(M4, HIGH);
  delay(50);
  Serial.println("RIGHT");
}
//....................STOP()....................
void STOP()
{
 
  analogWrite(SC1, spd); // writing the speed to left motor
  analogWrite(SC2, spd); // writing the speed to right motor
  // left motor stop
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  //right motor stop
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);
  delay(50);
  Serial.println("STOP");
}

void inline ppmISR(void){
  static boolean state = true;

  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    next = next + (PULSE_LENGTH * CPU_MHZ);
    state = false;
    alivecount++;
  } 
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      next = next + ((FRAME_LENGTH - calc_rest) * CPU_MHZ);
      calc_rest = 0;
      digitalWrite(DEBUGPIN, !digitalRead(DEBUGPIN));
    }
    else{
      next = next + ((ppm[cur_chan_numb] - PULSE_LENGTH) * CPU_MHZ);
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
  timer0_write(next);
}

void handleRoot() {
   if(ppm_running==0)
  {
    noInterrupts();
    timer0_isr_init();
    timer0_attachInterrupt(ppmISR);
    next=ESP.getCycleCount()+1000;
    timer0_write(next);
    for(int i=0; i<CHANNEL_NUMBER; i++){
      ppm[i]= CHANNEL_DEFAULT_VALUE;
    }
    ppm_running=1;
    interrupts();
  }
 }
 



void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {
    
    switch(type) {
        case WStype_DISCONNECTED:      
            break;
        case WStype_CONNECTED: 
            {
              IPAddress ip = webSocket.remoteIP(num);
             // Serial.println("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);    
            }
            break;
        
        case WStype_TEXT:
            {
           
              String _payload = String((char *) &payload[0]);
              //Serial.println(_payload);
              if (_payload == "FORWARD")
              {
                forward();
                //delay(2000);
              }
              else if (_payload == "BACKWARD")
              {
                backward();
                //delay(2000);
              }
              else if (_payload == "LEFT")
              {
                left();
                //delay(2000);
              }
              else if (_payload == "RIGHT")
              {
                right();
                //delay(2000);
              }
              else if (_payload == "STOP")
              {
                STOP();
                //delay(2000);
              }
              else
              {
                STOP();
                //delay(2000);
              }
              String ch1 = (_payload.substring(_payload.indexOf(":")+1,_payload.indexOf("a")));
              
              String ch2 = (_payload.substring(_payload.indexOf("a")+1,_payload.indexOf("b")));
              
              String ch3 = (_payload.substring(_payload.indexOf("b")+1,_payload.indexOf("c")));
              
              String ch4 = (_payload.substring(_payload.indexOf("c")+1,_payload.indexOf("d")));
              
              String ch5 = (_payload.substring(_payload.indexOf("d")+1,_payload.indexOf("e")));
              
              String ch6 = (_payload.substring(_payload.indexOf("e")+1,_payload.indexOf("f")));
              
              String ch7 = (_payload.substring(_payload.indexOf("f")+1,_payload.indexOf("g")));
              
              String ch8 = (_payload.substring(_payload.indexOf("g")+1,_payload.indexOf("h")));
              ppm[0]=ch1.toInt();ppm[1]=ch2.toInt();
              ppm[2]=ch3.toInt();ppm[3]=ch4.toInt();
              ppm[4]=ch5.toInt();ppm[5]=ch6.toInt();
              ppm[6]=ch7.toInt();ppm[7]=ch8.toInt();
            /*  Serial.print(ppm[0]);Serial.print(" ");Serial.print(ppm[1]);Serial.print(" ");
              Serial.print(ppm[2]);Serial.print(" ");Serial.print(ppm[3]);Serial.print(" ");
              Serial.print(ppm[4]);Serial.print(" ");Serial.print(ppm[5]);Serial.print(" ");
              Serial.print(ppm[6]);Serial.print(" ");Serial.print(ppm[7]);Serial.println();*/
            }   
            break;     
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(SC1,OUTPUT);
  pinMode(SC2,OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  // initially stop the motor
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(M3, LOW);
  digitalWrite(M4, LOW);

  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
     Serial.print(".");
     delay(200);
  }
    
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);  
 digitalWrite(2,LOW);
  Serial.println("Start Websocket Server");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  pinMode(sigPin,OUTPUT);
  digitalWrite(sigPin, !onState); //set the PPM signal pin to the default state (off)
  pinMode(DEBUGPIN,OUTPUT);
  digitalWrite(DEBUGPIN, !onState); //set the PPM signal pin to the default state (off)
  noInterrupts();
  timer0_detachInterrupt();
  ppm_running=0;
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(ppmISR);
  next=ESP.getCycleCount()+1000;
  timer0_write(next);
  ppm[0]=1500;//yaw
  ppm[1]=1000;//throttle
  ppm[2]=1500;//roll
  ppm[3]=1500;//pitch
  ppm[4]=1000;//aux//flightmode
  ppm[5]=1000;//aux
  ppm[6]=1000;//aux
  ppm[7]=1000;//aux//arm disarm
  interrupts();

  
}

void loop() {
  webSocket.loop();
}
