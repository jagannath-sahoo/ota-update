/********************

    == SD card is not implemented yet ==

serial data from ESP32 to the serial device (STM)
is working fine

MQTT publisher (reading sensor data from the STM and uploading the same data to MQTT is working)

Publisher and subscriber is also working fine now in this system


previously:
              There is a bug wherein while receiving "check_update" from the STM, ESP32 is directly replying to the STM32
              with the old string, we need to update the same

Update:  
              bug has been removed now from the givens system, whering the bug was encountered as mentined above

*************************/


//-------------------- Includes -----------------------
#include "FS.h" 
//#include "SD.h" 
#include "SPI.h"
#include "EspMQTTClient.h"
//-------------------- Includes End Here---------------

#define RXD2 16
#define TXD2 17
#define SEND 1
#define RECIEVE 2
//--------------------Declarations -------------------
int initSD();
void initMQTT();
void publishRecieve(unsigned char , const String);         // this will initiate client.loop()
void readAndPublishSensorData();  // for publishing the sensor data
void sendStringToUart();
String recieveStringFromUart();
//void publishRecieve(unsigned char request, const String topic)  // this will initiate client.loop()

//--------------------Declarations End Here-----------

//-------------------Global Variables ----------------
  EspMQTTClient client(
  "Vrinda_33",
  "littleindia",
  "192.168.0.112",  // MQTT Broker server ip
  //"dyyphfkl",   // Can be omitted if not needed
  //"UCwekHlkR7Fp",   // Can be omitted if not needed
  "TestClient",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);


/********************************************************
 *  In my code, I have used only two callbacks for the 
 *  MQTT, previously I have used multiple call backs of 
 *  same type in my code, for the given pub or sub operation
 * 
 *  Calling functions will update the given string "mqttTopic" 
 *  with suitable MQTT topic, hence for a given function
 * 
 *  Our callbacks will work dynamically
 ********************************************************/

String payloadReceived;
String payloadToBeSent;

const String receiveRequest    = "mytopic/test/request"; // sub
const String publishResponse   = "mytopic/test/response";// pub
const String publishSensor     = "mytopic/test/sensor";  // pub

String mqttPublishTopic  = "";

volatile char receptionFlag = 'F';
volatile char checkUpdateFlag = 'F';
//----------------------------------------------------


void setup() {
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200); 
initMQTT();
}

void loop() {
    
    //while(!(client.isConnected())){client.loop();}
    client.loop();
    
    switch(checkUpdateFlag)
    {
        case 'F':
        {

            // here we just need to update the data on the web
            readAndPublishSensorData();
            break;  
        }

        case 'T':
        {
            // here we need to stop uploading the data to the web
            // and we need to check for the given update and
            // send the received String to the STM32
            // action topic
            checkForUpdateFromInternet();
            receptionFlag = 'F';
            break;  
        }

        case 'D':   // update downloadMode and Flash
        {   
          
            Serial.println("Download the Update and Flash mode");
            // here we need to stop uploading the data to the web
            // and we need to check for the given update and
            // send the received String to the STM32

            //              action     topic

            // after performing the Update, changing the flag to False
            checkUpdateFlag = 'F'; //for normal functionality
            break;  
        }

        default:
        {
          break;  
        }
    }
}


void sendStringToUart(String dummyString)
{
  char bufferarr[64];
  memset(bufferarr,'*',(sizeof(bufferarr)-1));

  dummyString.toCharArray(bufferarr,(sizeof(bufferarr)-1));
  bufferarr[dummyString.length()] = '*';

  for(  int arrayIndex = 0; arrayIndex<strlen(bufferarr); arrayIndex++)
  {
      Serial2.print(bufferarr[arrayIndex]);  
      Serial.print(bufferarr[arrayIndex]);  
  }
}


void checkForUpdateFromInternet()
{
            // requesting the update from the server
            // sending request to the Server
    
      // here we have sent the request to the server,         
            publishRecieve(SEND,publishResponse);
            
      // now server will respond with a payload, which we need to send as it is to the Serial2 
      // to the STM board and start doing old work
      // here we need a blocking call and wait till the reception of the data from Server

            publishRecieve(RECIEVE,receiveRequest); 

            if(receptionFlag == 'T')    // means response has been received from server
            {
              sendStringToUart(payloadReceived);
              checkUpdateFlag = 'F';    // continue receiving the sensor data
              return;
            }            
}

void publishRecieve(unsigned char requestType, const String topic)  // this will initiate client.loop()
{
      switch(requestType)
      {
        case SEND:
        {
        client.publish(topic,payloadToBeSent); // You can activate the retain flag by setting the third parameter to true  
        break;
        }
        case RECIEVE:
        {
          // this is the blocking call to recieve a message from the Subscriber
          while(receptionFlag != 'T'){client.loop();}
      
          Serial.print("Message is received from MQTT: ");
          Serial.println(payloadReceived);
          break;
        }
      }
}

String recieveStringFromUart()
{
        String messageFromSTM ="";

        Serial.print("Reading string from STM, \n Message is ");
        
        if(Serial2.available()>0)
        {
          messageFromSTM = Serial2.readString();
          messageFromSTM.trim(); 
          Serial.println(messageFromSTM);        
          return(messageFromSTM); 
        }
}


//-------------- Part1 is accomplished here -----------
void readAndPublishSensorData()
{
  if(Serial2.available() > 0)
   {
        String uartStringRecieved = recieveStringFromUart();   

  //-------------no modification required-------------------------------------         
        if(uartStringRecieved!="check_update" && uartStringRecieved !="ready")
        {
            Serial.println("Received sensor data");
           // Copying the sensor data to the Server
           payloadToBeSent  = uartStringRecieved; 

           //publising the data
           publishRecieve(SEND,publishSensor);
           return;
        }
  //---------------------------------------------------------------------------         

  //-------------no modification required-------------------------------------         
        else if(uartStringRecieved =="check_update")
        {
           Serial.println("STM requested update");

            // requesting the data from the server for the given string
            // here we are updating the "payloadToBeSent" with the with received string
           payloadToBeSent  = uartStringRecieved;

            // enabling checkUpdateFlag state to True
            // now in loop() code will have different functionality
           checkUpdateFlag  = 'T';
           return;
        }
  //---------------------------------------------------------------------------         


        else if(uartStringRecieved =="ready")  // stm wishes to get updated
        {
            Serial.println("STM wants firmware");
           // enabling the update download and flashing functionality
           checkUpdateFlag  ='D';           // download mode
        }
   }
}
//----------------------------------------------------------

//---------MQTT callbacks for connection establishment------
void onConnectionEstablished()
{
            client.subscribe(receiveRequest, [](const String & payload) {
            payloadReceived = payload;
            Serial.println(payload);
            receptionFlag = 'T';
            return;
            });        
                
            //client.publish(mqttPublishTopic,payloadToBeSent);
}


void initMQTT()
{
  // Optionnal functionnalities of EspMQTTClient : 
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overrited with enableHTTPWebUpdater("user", "password").
  client.enableLastWillMessage("TestClient/lastwill", "I am going offline");  // You can activate the retain flag by setting the third parameter to true
}
