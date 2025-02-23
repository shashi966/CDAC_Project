#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>

// WiFi configuration
const char* ssid = "SIGNAL";
const char* password = "Shik96@@";

// ThingSpeak configuration
const char* server = "https://api.thingspeak.com"; //ThinkSpeak server
const String apiKey = "1VAG85OYP5WFGIQX"; // ThinkSpeak API key

//Serial monitor usart Rx, Tx
#define RX_PIN D1  // Connect this to the TX pin of STM32
#define TX_PIN D2  // Connect this to the RX pin of STM32
SoftwareSerial mySerial(RX_PIN, TX_PIN);


void setup() {
  // Start Serial Monitor
  Serial.begin(115200);
  Serial.println();

  
  mySerial.begin(9600); //baudrate equal to stm32
  digitalWrite(D0, LOW);
  Serial.println("Setup completed.");
   //Connect to Wi-Fi
   Serial.print("Connecting to WiFi...");
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(1000);
     Serial.print(".");
   }
   Serial.println("\nWiFi connected!");
   Serial.print("IP Address: ");
   Serial.println(WiFi.localIP());
 }

void loop() {

   if (mySerial.available() > 0) {                                        //check data on serail monitor from stm32
    String receivedData = mySerial.readStringUntil('\r');               //read until "/r"
    receivedData.trim(); // Remove any extra whitespace 
    Serial.print("Message from STM32: ");
    Serial.println(receivedData); // Print the received message to Serial Monitor
   int values[5] = {0 , 0 , 0 , 0 , 0};
   int index = 0;
    String numericValue = "";
    for(int i = 0; i< receivedData.length(); i++){
      if(isDigit(receivedData[i])){
          numericValue += receivedData[i]; 
       } 
       else if(numericValue.length() > 0)
       {
        values[index] = numericValue.toInt();
        numericValue = "";
        index++;
        if (index >= 5) break;
        }
       }
     if(numericValue.length() > 0 && index < 5)
     {
      values[index] = numericValue.toInt();
      }

      Serial.println("Extracted Values : ");
      for( int i = 0 ; i < 5 ; i++)
      {
        Serial.print("Field");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(values[i]);
        }
    Serial.print("Length of the received string: ");
    int length = receivedData.length();
    Serial.println(length);
    delay(1000);
    //THingspeak code (WiFi)

    if (WiFi.status() == WL_CONNECTED && numericValue.length()>0) {
       String url = "http://api.thingspeak.com/update?api_key=" + apiKey +
                         "&field1=" + String(values[0]) +
                         "&field2=" + String(values[1]) +
                         "&field3=" + String(values[2]) +
                         "&field4=" + String(values[3]) +
                         "&field5=" + String(values[4]);
      WiFiClient client; // Create a WiFi client
     HTTPClient http;   // Create an HTTP client
     http.begin(client, url);
     http.addHeader("User-Agent", "ESP8266");
    int httpResponseCode = http.GET();
//  //   // Generate the URL to send data
     Serial.print("Uploading data to ThingSpeak: ");
    Serial.println(url);
//
//  //   // Make HTTP request
   //  http.begin(client, url); // Specify the client and URL
 //  int httpResponseCode = http.GET();
//
   if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
     Serial.println(httpResponseCode);
      if (httpResponseCode == 200) {
        Serial.println("Data uploaded successfully!");
     } else {
       Serial.println("Failed to upload data. Check API key and network.");
      }
    } else {
    Serial.print("Error on HTTP request: ");
       Serial.println(http.errorToString(httpResponseCode).c_str());
    }
//
     http.end(); // Free resources
   } else {
    Serial.println("WiFi not connected. Reconnecting...");
    WiFi.begin(ssid, password);
   }
//
delay(15000); // Send data every 15 seconds
   }


}
