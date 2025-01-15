#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
const int a11=D1;
const int a12=D3;
const int b11=D2;
const int b12=D4;
const int shoot=D5;
//motor A D1,D3 left motor
//motor B D2,D4 right motor
//Solenoid actuator
//H hello
//F forward
//B backward
//R forward right
//L forward left
//S shoot
//E stop
//D back right
//G back left
//K right turn
//M left turn

const char *ssid = "shiva's tesla";
const char *password = "shivakalyan@2003";

ESP8266WebServer server(80);

void handleRequest() {
  String comand = server.arg("command");
  String responseMessage;
  //Serial.println("HI FRIENDS");
  char command=comand[0];
  switch(command)
  {
    case 'H':
      responseMessage = "HELLO!";
      Serial.println(responseMessage);
      break;
    case 'F':
      // Handle forward command
      digitalWrite(a11,HIGH);
      digitalWrite(a12,LOW);
      digitalWrite(b11,HIGH);
      digitalWrite(b12,LOW);
      responseMessage = "Moving forward!";
      Serial.println(responseMessage);
      break;
    case 'B':
      // Handle backward command
      digitalWrite(a11,LOW);
      digitalWrite(a12,HIGH);
      digitalWrite(b11,LOW);
      digitalWrite(b12,HIGH);
      responseMessage = "Moving backward!";
      Serial.println(responseMessage);
      break;
    case 'L':
      // Handle left command
      digitalWrite(a11,LOW);
      digitalWrite(a12,LOW);
      digitalWrite(b11,HIGH);
      digitalWrite(b12,LOW);
      responseMessage = "Moving F Left!"; 
      Serial.println(responseMessage);
      break;
    case 'M':
      // Handle left command
      digitalWrite(a11,LOW);
      digitalWrite(a12,HIGH);
      digitalWrite(b11,HIGH);
      digitalWrite(b12,LOW);
      responseMessage = "Moving Left!"; 
      Serial.println(responseMessage);
      break;   
    case 'R':
        // Handle Right command
      digitalWrite(a11,HIGH);
      digitalWrite(a12,LOW);
      digitalWrite(b11,LOW);
      digitalWrite(b12,LOW);
      responseMessage = "Moving F Right!";
      Serial.println(responseMessage);
      break;
    case 'K':
        // Handle Right command
      digitalWrite(a11,HIGH);
      digitalWrite(a12,LOW);
      digitalWrite(b11,LOW);
      digitalWrite(b12,HIGH);
      responseMessage = "Moving Right!";
      Serial.println(responseMessage);
      break;   
    case 'S':
      digitalWrite(shoot,LOW);
      delay(50);
      digitalWrite(shoot,HIGH);
      responseMessage = "Shooting The Ball!";
      Serial.println(responseMessage);
      break;
    case 'E':
      digitalWrite(a11,LOW);
      digitalWrite(a12,LOW);
      digitalWrite(b11,LOW);
      digitalWrite(b12,LOW);
      responseMessage = "Stopped!"; 
      Serial.println(responseMessage);
      break;
    case 'D':
      // Handle left command
      digitalWrite(a11,LOW);
      digitalWrite(a12,HIGH);
      digitalWrite(b11,LOW);
      digitalWrite(b12,LOW);
      responseMessage = "Moving B Right!"; 
      Serial.println(responseMessage);
      break;
    case 'G':
      // Handle left command
      digitalWrite(a11,LOW);
      digitalWrite(a12,LOW);
      digitalWrite(b11,LOW);
      digitalWrite(b12,HIGH);
      responseMessage = "Moving B Left!"; 
      Serial.println(responseMessage);
      break;      
    default:
      digitalWrite(a11,LOW);
      digitalWrite(a12,LOW);
      digitalWrite(b11,LOW);
      digitalWrite(b12,LOW);
      break;
  }
  server.send(200, "text/plain", responseMessage);
}

void setup() {
  Serial.begin(9600);
  // Create WiFi access point
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  pinMode(a11,OUTPUT);
  pinMode(b11,OUTPUT);
  pinMode(a12,OUTPUT);
  pinMode(b12,OUTPUT);
  pinMode(shoot,OUTPUT);
  digitalWrite(shoot,HIGH);
  // Define route for the GET request
  server.on("/control", HTTP_GET, handleRequest);

  // Start server
  server.begin();
}

void loop() {
  // Handle client requests
  server.handleClient();
}
