#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SPI.h>
#include <MFRC522.h>

constexpr uint8_t RST_PIN = D3;     // Configurable, see typical pin layout above
constexpr uint8_t SS_PIN = D4;     // Configurable, see typical pin layout above

MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key;

String tag;

const char* ssid = "Shiva Kalyan";//Keep your ssid
const char* password = "shivakalyan@2003";//keep your pswd
const char* scriptURL = "https://script.google.com/macros/s/AKfycbwQzrSAu1WHKjJODoisEVes1qqs65Ez2xvkGA3XZf2TEv-A8zhBJIYoUit6K7dsjakTGg/exec";//This is the web app id

void setup() {
  Serial.begin(9600);
    SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

void loop() {
  // Your code here
  if ( ! rfid.PICC_IsNewCardPresent())
    return;
  if (rfid.PICC_ReadCardSerial()) {
      for (byte i = 0; i < 4; i++) {
      tag += rfid.uid.uidByte[i];
      }
  HTTPClient http;
  WiFiClientSecure client;
  client.setInsecure();
  String url = scriptURL;
  http.begin(client, url);

  // Set the content type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  // Create the payload string with the parameters
  String payload = String("rfid=") + tag;
  Serial.println(payload);
  int httpResponseCode = http.POST(payload);

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  tag = "";
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  http.end();

  }
    // Make the HTTP GET request
}
