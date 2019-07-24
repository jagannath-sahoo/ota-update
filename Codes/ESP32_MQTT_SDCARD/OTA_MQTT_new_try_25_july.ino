/*
  SimpleMQTTClient.ino
  The purpose of this exemple is to illustrate a simple handling of MQTT and Wifi connection.
  Once it connects successfully to a Wifi network and a MQTT broker, it subscribe to a topic and send a message to it.
  It will also send a message delayed 5 seconds later.
*/

#include "EspMQTTClient.h"


const String updateAvailable = "update_available";
const String endOfFile     = "E_O_F";
//const String endOfFile         = "eof";

String packet= "";
String codeBytes = "";


volatile char updateAvailableFlag      = 'F'; 
volatile char updateCompleted          = 'F'; 

String dummyCodeFile = "";

void notify();


EspMQTTClient client(
  "Vrinda_33",
  "littleindia",
  "192.168.0.134",  // MQTT Broker server ip
 // "MQTTUsername",   // Can be omitted if not needed
 // "MQTTPassword",   // Can be omitted if not needed
  "TestClient",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);

void setup()
{
  Serial.begin(115200);

  // Optionnal functionnalities of EspMQTTClient : 
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true
}

// This function is called once everything is connected (Wifi and MQTT)
// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient


void onConnectionEstablished()
{
 // static char updateAvailableFlag    = 'F'; 
                    client.subscribe("mytopic/test/request", [](const String & payload) 
                                              {
                                                  packet = payload;
                                                  if (payload != endOfFile && (payload == updateAvailable || updateAvailableFlag    == 'T'))
                                                  {
                                                       Serial.println("Inside the first if");

                                                      
                                                     client.publish("mytopic/test/response", "UpdateAck");                                                    

                                                     // filtering the code bytes
                                                     // raising the flag high till the update code bytes are available
                                                     updateAvailableFlag    = 'T';
                                                     if(packet != "update_available")
                                                     {
                                                       Serial.println(payload);
                                                        dummyCodeFile += payload;  
                                                        client.publish("mytopic/test/file", dummyCodeFile);                                                                                                     
                                                     }
                                                  }

                                                  else if(payload == endOfFile &&  updateAvailableFlag  == 'T')
                                                  {
                                                       Serial.println("Inside the second if");
                                                    
                                                     client.publish("mytopic/test/response", "EOF Received");

                                                     updateAvailableFlag    = 'F';
                                                     updateCompleted        = 'T';
                                                      
                                                  }
                                                  else
                                                  {
                                                    
                                                       Serial.println("Inside the else state of flag is ");
                                                       Serial.println(updateAvailableFlag);
                                                    client.publish("mytopic/test/response", "No Action Available for this request");
                                                    Serial.println("No Action Available for this request");
                                                  }
                                              
                                              
                                              });

      
  /*
  // Execute delayed instructions
  client.executeDelayed(5 * 1000, []() {
    client.publish("mytopic/test", "This is a message sent 5 seconds later");
  });
  */
}

void notify()
{
      //  Means update has been done sucessfully
    Serial.println("Update has been done successfully. Content of file are presented below");
    client.publish("mytopic/test/response", "File Has been received sucessfully");

    Serial.println(dummyCodeFile);

    // Ready for next iteration
    updateCompleted     = 'F';

  }

void loop()
{
  if(updateCompleted     == 'T')
  {
    notify();
  }
  client.loop();
}
