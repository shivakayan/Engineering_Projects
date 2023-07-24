#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// Replace with your access point credentials
const char* ssid = "MyAccessPoint";
const char* password = "MyPassword";

// Set up push button
const int buttonPin = D4; // GPIO2
const int alcoholpin = D6;

// Server IP and port
const String serverIP = "192.168.4.2";
const int serverPort = 8080;

void setup() {
  Serial.begin(9600);

  // Configure access point
  WiFi.softAP(ssid, password);
  
  // Set up push button
  pinMode(buttonPin, INPUT);

  Serial.println("Access point started");
}

void sendRequest(int k) {
  Serial.println("Sending request...");
  
  WiFiClient client;
  HTTPClient http;
  if(k==1)
  {
    Serial.println("Stop the vehicle");
    if (http.begin(client, "http://" + serverIP + ":" + String(serverPort))) 
    {
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST("STOP");
    
      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("HTTP request sent successfully");
        Serial.println("Response: " + response);
      } else {
        Serial.println("HTTP request failed");
      }
      
      http.end();
    } 
    else {
      Serial.println("Unable to connect to the server");
    }
  }
  else
  {
    Serial.println("START THE VEHICLE");
    if (http.begin(client, "http://" + serverIP + ":" + String(serverPort))) 
    {
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.POST("START");
    
      if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("HTTP request sent successfully");
        Serial.println("Response: " + response);
      } else {
        Serial.println("HTTP request failed");
      }
      
      http.end();
    } 
    else 
    {
      Serial.println("Unable to connect to the server");
    }
  }
}

void loop() {
  Serial.println("The alcohol sensor value is:");
  Serial.println(digitalRead(alcoholpin));
  if ((digitalRead(buttonPin) == HIGH)||(digitalRead(alcoholpin)==LOW))
    sendRequest(1);
  else
    sendRequest(0);
  delay(10);    
}
