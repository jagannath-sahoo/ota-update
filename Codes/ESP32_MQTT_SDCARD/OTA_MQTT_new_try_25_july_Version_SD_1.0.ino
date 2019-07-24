/**************************************************************************************

  Author: Aman Kanwar
  
  The purpose of this exemple is to illustrat// we need to take this value as the last char will be null character which will end the file
char buf[65];
e a simple handling of MQTT and Wifi connection.
  Once it connects successfully to a Wifi network and a MQTT broker, it subscribe to a topic and send a message to it.

  This code is fully functional and is performing the below mentioned tasks

    - Subscribes to "mytopic/test/request"

      Listens to the commands and data published on the above mentioned topic and 
      responds accordingly in addition to storing the data published on the given
      topic in a buffer (later on a file in upcoming update)

    - Publishes to  "mytopic/test/response"

      Once the data is command for an OTA is received then the server is notified
      about the reception of the signal/update notification. After this this send
      the acknowledgement of each message received from the server.

Furture Goals:
      In the upcoming code, I will be adding the functionality of SD card in this
      code, wehere the received data file will be stored in the text file which
      will be created in the SD card (using an SD card module).
 
      Once the file download/ is complete, then the HOST (STM32) will be notified
      regarding the same using any of the GPIO present on the ESP32, this will
      interrupt the STM32 informing that an update is available.

**************************************************************************************/

/***********************************************************************************
 * Previously:
 * 
 * This code is working and implements the writing of the data from a given Arrray to
 * the SD card in 64 Byte format, further the data from the SD card is read and displayed
 * on the serial terminal Byte-by-Byte till the end of line.
 * 
 * Further Implementation:
 * 
 * This code is futher implementation of my previous code of SD card read and write
 * 
 * 
 ***********************************************************************************/



//-----------------------------Includes---------------------------
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include "EspMQTTClient.h"
//-----------------------------------------------------------------

//-------------------------- Global Variables ---------------------
const String updateAvailable = "update_available";
const String endOfFile       = "E_O_F";
String packet                = "";

volatile char updateAvailableFlag      = 'F'; 
volatile char updateCompleted          = 'F'; 

String dummyCodeFile = "";

// we need to take this value as the last char will be null character which will end the file
char codeBuffer[65];

//-----------------------------------------------------------------

//--------------- Declarations----------
int initSD();
void notify();
//--------------------------------------

//==========================================================
//-------------- Class Template Start-----------------------
//==========================================================
class SDoperations{

  public:

    //----------- Available Methods in this class-----------
    // ----------- Some are for testing purpose ------------
    
    // /*  Uncomment this line comment to comment this method
    
    void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
      Serial.printf("Listing directory: %s\n", dirname);
  
      File root = fs.open(dirname);
      if(!root){
          Serial.println("Failed to open directory");
          return;
      }
      if(!root.isDirectory()){
          Serial.println("Not a directory");
          return;
      }
  
      File file = root.openNextFile();
      while(file){
          if(file.isDirectory()){
              Serial.print("  DIR : ");
              Serial.println(file.name());
              if(levels){
                  listDir(fs, file.name(), levels -1);
              }
          } else {
              Serial.print("  FILE: ");
              Serial.print(file.name());
              Serial.print("  SIZE: ");
              Serial.println(file.size());
          }
          file = root.openNextFile();
      }
  }    //  */ 



    // /*  Uncomment this line comment to comment this method
       void readFile(fs::FS &fs, const char * path){
        Serial.printf("Reading file: %s\n", path);
    
        File file = fs.open(path);
        if(!file){
            Serial.println("Failed to open file for reading");
            return;
        }
    

        char codeByte;
        
        Serial.print("Read from file: ");
                            
                            
        Serial.println();

        while(file.available()){
           
             delay(100);
             codeByte = file.read();

            if(codeByte ==  ':')
            {
            Serial.println();
            Serial.print(codeByte);  
            }
            else 
            {
              Serial.print(codeByte);
            }
            
        }

        file.close();
    }    //  */ 

    // /*  Uncomment this line comment to comment this method
    void writeFile(fs::FS &fs, const char * path, const char * message){
        Serial.printf("Writing file: %s\n", path);
    
        File file = fs.open(path, FILE_WRITE);
        if(!file){
            Serial.println("Failed to open file for writing");
            return;
        }
        if(file.print(message)){
            Serial.println("File written");
        } else {
            Serial.println("Write failed");
        }
        file.close();
    }// */

    // /*  Uncomment this line comment to comment this method
    void appendFile(fs::FS &fs, const char * path, const char * message){
        Serial.printf("Appending to file: %s\n", path);
    
        File file = fs.open(path, FILE_APPEND);
        if(!file){
            Serial.println("Failed to open file for appending");
            return;
        }
        if(file.print(message)){
            Serial.println("Message appended");
        } else {
            Serial.println("Append failed");
        }
        file.close();
    }// */

    // /*  Uncomment this line comment to comment this method
    void renameFile(fs::FS &fs, const char * path1, const char * path2){
        Serial.printf("Renaming file %s to %s\n", path1, path2);
        if (fs.rename(path1, path2)) {
            Serial.println("File renamed");
        } else {
            Serial.println("Rename failed");
        }
    } // */

    // /*  Uncomment this line comment to comment this method
    void deleteFile(fs::FS &fs, const char * path){
        Serial.printf("Deleting file: %s\n", path);
        if(fs.remove(path)){
            Serial.println("File deleted");
        } else {
            Serial.println("Delete failed");
        }
    } // */
    
  

    /*
    void createDir(fs::FS &fs, const char * path){
        Serial.printf("Creating Dir: %s\n", path);
        if(fs.mkdir(path)){
            Serial.println("Dir created");
        } else {
            Serial.println("mkdir failed");
        }
    }
    */
    /*
    void removeDir(fs::FS &fs, const char * path){
        Serial.printf("Removing Dir: %s\n", path);
        if(fs.rmdir(path)){
            Serial.println("Dir removed");
        } else {
            Serial.println("rmdir failed");
        }
    }
    */
    
      /*
      void testFileIO(fs::FS &fs, const char * path){
          File file = fs.open(path);
          static uint8_t buf[512];
          size_t len = 0;
          uint32_t start = millis();
          uint32_t end = start;
          if(file){
              len = file.size();
              size_t flen = len;
              start = millis();
              while(len){
                  size_t toRead = len;
                  if(toRead > 512){
                      toRead = 512;
                  }
                  file.read(buf, toRead);
                  len -= toRead;
              }
              end = millis() - start;
              Serial.printf("%u bytes read for %u ms\n", flen, end);
              file.close();
          } else {
              Serial.println("Failed to open file for reading");
          }
      
      
          file = fs.open(path, FILE_WRITE);
          if(!file){
              Serial.println("Failed to open file for writing");
              return;
          }
      
          size_t i;
          start = millis();
          for(i=0; i<2048; i++){
              file.write(buf, 512);
          }
          end = millis() - start;
          Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
          file.close();
      }
      */
   friend void setup();
   friend void loop();
   friend void onConnectionEstablished();
   friend  int initSD();
  };


//==========================================================
//-------------- Class Template Ends-----------------------
//==========================================================



SDoperations mySD;          // object of the class SDoperations 



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

  if(initSD())
  {
    Serial.println("SD card initialization failed");
    while(1);  
  }
    Serial.println("SD card initialization done");

    

  

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

                                                       // copying the data of payload into buffer
                                                       payload.toCharArray(codeBuffer,sizeof(codeBuffer));

                                                       // making the last element of received string zero
                                                       // basically we are removing the terminating null of the string
                                                       // since our buffer is of 65 bytes the null is at the end of buffer itself
                                                       // hence we need to remove the NULL character at the end of received string
                                                       codeBuffer[payload.length()] = '0';

                                                       // writing the data to the file

                                                       mySD.appendFile(SD, "/myFirmware.txt", codeBuffer);

                                                       
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


    Serial.println("\n\n\nFile data is provided below");

    
    mySD.readFile(SD, "/myFirmware.txt");

    // Ready for next iteration
    updateCompleted     = 'F';

  }

int initSD()
{
  if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return 1;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return 1;
    }

  // created the class instance of the SD class

/*  if(SD.exists("/myFirmware.txt"))
  {
    // if the file already exists in the SD card, delete the file
    mySD.deleteFile(SD, "/myFirmware.txt");
  }
  */
  // creating a file named myFirmware.txt
  mySD.writeFile(SD, "/myFirmware.txt", "");
  mySD.listDir(SD,"/",0);


  // filling the code buffer with zeroes 64 positions
  memset(codeBuffer,'0',(sizeof(codeBuffer)-1));

  Serial.println("In initSD() method, content of buffer is");
  Serial.println(codeBuffer);
  
  return 0;
}

void loop()
{
  if(updateCompleted     == 'T')
  {
    notify();
  }
  client.loop();
}
