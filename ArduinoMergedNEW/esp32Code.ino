#include "WiFi.h"
 
char* ssid = "AndroidAP";
char* password =  "enwf7546";

char server[] = "o1.prota.space";
WiFiClient client;

void buttonPress(int button);

void setup() {
 
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  
  pinMode(12, INPUT); //Pin 44 on Mega, 1st bit
  pinMode(13, INPUT); //Pin 42 on Mega, 2nd bit
  pinMode(14, INPUT); //Pin 40 on Mega, 3rd bit
}

void buttonPress(int button){
  if (WiFi.status() == WL_CONNECTED){
     if (client.connect(server, 80)) {
        Serial.println("connected to server");
      
        // Make a HTTP request:
        switch (button){
          case 1: //blue button
            client.println("GET /mib/do/press?_id=bbabb42a48b8bc5e924d9b339f173a84 HTTP/1.1");
          break;

          case 2: //red button
            client.println("GET /mib/do/press?_id=f76118b5e04f232db1f972234bdb184c HTTP/1.1");
          break;

          case 3: // yellow button
            client.println("GET /mib/do/press?_id=5a342aa2e37ffc129b6dbf0edf512d52 HTTP/1.1");
          break; 
          
           case 4: //green button
            client.println("GET /mib/do/press?_id=eb071473bcb60b44925b67dc495d884d HTTP/1.1");
          break;
                           
        }
         
        }        
        client.println("Host: o1.prota.space");
        client.println("Connection: close");  
        client.println();
    }
  }


 
void loop() {
int counter = 0;

  if (digitalRead(12) == HIGH){
   counter += 1;
    }
    
  if (digitalRead(13) == HIGH){
  counter += 2;
  }

  if (digitalRead(14) == HIGH){
  counter += 4;
  }

        switch (counter){
          case 1: //press button 1
            buttonPress(1);
            delay(100);
          break;

          case 2: //press button 2
            buttonPress(2);
            delay(100);
          break;

          case 3: //press button 2
            buttonPress(3);
            delay(100);
          break;

          case 4: //press button 2
            buttonPress(4);
            delay(100);
          break;
        }
        
  delay(10);
  }
