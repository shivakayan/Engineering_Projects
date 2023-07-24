#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Replace with your network credentials
const char* ssid = "MyAccessPoint";
const char* password = "MyPassword";
String request;

// Server port
const int serverPort = 8080;

WiFiServer server(8080);

int handleRequest() {
  int ans;
  WiFiClient client = server.available();
  if (!client)  {
    return 1;
  }
  else
  request = client.readString();
  //Serial.println(request);
  if(request.indexOf("START") > 0)
    ans=0;
  if(request.indexOf("STOP")>0)
    ans=1;  
  client.println("HTTP/1.1 200 OK\r\n");
  client.println( "");
  client.flush();
  return ans;
}

void setup() {
  Serial.begin(9600);

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
  Serial.print("Local IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");
  pinMode(D5,OUTPUT);
}

void loop() {
  int ans;
  ans=handleRequest();
  Serial.println(ans);
  if(ans==1)
  {
      digitalWrite(D5,LOW);
      Serial.println("The vehicle must be stopped!");
  }
  else
  {
    digitalWrite(D5,HIGH);
    Serial.println("The vehicle can move withot hurdles");
  }
  delay(20);  
}
