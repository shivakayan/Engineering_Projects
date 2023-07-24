#include <ESP8266WiFi.h>
#include <Servo.h>
#include <ESP8266WebServer.h>
char* tag;
const char* ssid = "Shiva Kalyan";
const char* password = "shivakalyan@2003";
int serverPort = 80;  // Port for the server
Servo tl,fan,cl1,cl2,bl;
int TL,FAN,CL1,CL2,BL;
ESP8266WebServer server(serverPort);
String receivedRequest; // Variable to store the received request

void handleRoot() {
  server.send(200, "wait");
}

void handleRequest() {
  receivedRequest = server.uri(); // Store the received request in a variable
  if(receivedRequest!="/")
    server.send(404, "text/plain", "Not Found"); // Send a 404 Not Found response
  else
    server.send(200, "text/plain", "OK"); // Send a 200 OK response
}

void setup() {
  Serial.begin(9600);
  TL=FAN=CL1=CL2=BL=0;
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Print the IP address
  Serial.print("NodeMCU IP address: ");
  Serial.println(WiFi.localIP());
  tl.attach(D3);
  fan.attach(D4);
  cl1.attach(D5);
  cl2.attach(D6);
  bl.attach(D7);
  // Start the server
  tl.write(0);
  fan.write(0);
  bl.write(0);
  cl1.write(0);
  cl2.write(0);
  server.on("/", handleRoot);
  server.onNotFound(handleRequest); // Handle all requests with the same function
  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient();
  // Perform actions based on the received request
  if (!receivedRequest.isEmpty()) {
    // Handle the request using a switch-case statement
    Serial.println(receivedRequest);
    if(receivedRequest.indexOf("TL")>0)
    {
      if(TL==1){
        tl.write(0);
        TL=0;
      }
      else{
        tl.write(90);
        TL=1;
      }
    }
    else if(receivedRequest.indexOf("FAN")>0)
    {
      if(FAN==1){
        fan.write(0);
        FAN=0;
      }
      else{
        fan.write(90);
        FAN=1;
      }
    }
    else if(receivedRequest.indexOf("CL1")>0)
    {
      if(CL1==1){
        cl1.write(0);
        CL1=0;
      }
      else{
        cl1.write(90);
        CL1=1;
      }
    }
    else if(receivedRequest.indexOf("CL2")>0)
    {
      if(CL2==1){
        cl2.write(0);
        CL2=0;
      }
      else{
        cl2.write(90);
        CL2=1;
      }
    }
    else if(receivedRequest.indexOf("BL")>0)
    {
      if(BL==1){
        bl.write(0);
        BL=0;
      }
      else{
        bl.write(90);
        BL=1;
      }
    }
    else{
      Serial.println("It is an invalid request!!!");
    }
    receivedRequest="";
  }
}
