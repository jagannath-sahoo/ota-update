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

/**************************************************************
In this Version V2.0

I have implemented the functionality of Dual UARTS, wherein
one Uart is implemented as a Serial port for debugging purpose
and another Uart (Uart 2) is used to connect with the Host Controller
in this case, I have implemented the code with ESP32 as host,
later on this code will be tested for STM as well 


There some challenges in this as the Uart 2 of ESP32 is present
oN GPIO pins 17 and 16, till the time the GPIOs are not configured
as Uart TX and RX, these were sending garbage value to the 
host controller (ESP32 in this case).

However, this problem is solved using primary initialization of
Uart 2

Now, further implementation is still under consideration for the
given code, wherein the host machine will interrupt/signal my
ESP32 to flash the updated firmware in the host controller.

For this, I am looking to use interrupts in the given case
wherein the host will interrupt my controller requesting for
update.

However, there are some scenarios in this, wherein if the file
is not currently downloaded by ESP32 then what?

  Case1 :           File is not downloaded by ESP32

         In this case, my ESP will send a message to the STM 
         as "UFND" means Updated Firmware not downloaded

         After doing so, ESP32 will look for the updated Firmware
         on the given subscribed topic

         Once downloaded, STM will be interrupted

  Case2:            ESP32 is downloading the file

         In this case, STM will be sent a message (using Uart2)
         "UFCD" means Updated Firmware is downloading

         After receiving this, the STM will again ask for update
         from ESP32 after 15 seconds (enough time for ESP32)

         Till the complition of 15th Second, ESP32 would have
         downloaded the file

/************ important *******************************
 *  
 *  
 *  Corrected a bug, wherein the given array was not fully cleared
 *  by the memset,
         

**************************************************************/

//-----------------------------Includes---------------------------
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include "EspMQTTClient.h"


#define RXD2 16
#define TXD2 17

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
           
             delay(5);
             codeByte = file.read();

             

            if(codeByte ==  ':')
            {

            // for testing
            //--------- here we have added the functionality of second serial connection----
            Serial2.println();
            Serial2.print(codeByte);              
            //------------------------------------------------------------------------------            
            Serial.println();
            Serial.print(codeByte);  
            }
            else 
            {
              Serial.print(codeByte);

              // for testing
              Serial2.print(codeByte);            
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
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
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
    
                                                       /*-----------bug update --------------------
                                                         Previously here we were using the code as it is
                                                         and    
                                                         payload.toCharArray(codeBuffer,sizeof(codeBuffer));
                                                         was used with codeBuffer, which is an array of
                                                         64 bytes was filled with 0s once, and was used here
                                                         but if the data coming is smaller than the previous 
                                                         data then the old used bytes were retained by the
                                                         array, which would have caused serious errors, hence
                                                         i have corrected this bug by doing again a memset here
                                                       --------------------------------------------*/
                                                         // filling the code buffer with * 64 positions
                                                       memset(codeBuffer,'*',(sizeof(codeBuffer)-1));


                                                       //after this array has again been updated with default value
                                                       // we will copy the payload in the given array :) 
                                                       
                                                       payload.toCharArray(codeBuffer,sizeof(codeBuffer));

                                                       // making the last element of received string zero
                                                       // basically we are removing the terminating null of the string
                                                       // since our buffer is of 65 bytes the null is at the end of buffer itself
                                                       // hence we need to remove the NULL character at the end of received string

                                                       /*------------------------------------------------------------
                                                         Caught another bug
                                                         wherein if the payload is of 64 bytes then the below
                                                         mentioned code (used earlier) will write * on the 65th location
                                                         of the array (since the array is actually of 65 bytes)
                                                         wherein last location 65th was left for the NUll char

                                                         This bug is fixed by changing the code line from

                                                         codeBuffer[payload.length()] = '*';

                                                         with
                                                          if(payload.length() <65)
                                                          {
                                                           codeBuffer[payload.length()] = '*';
                                                          }
                                                         
                                                        ------------------------------------------------------------*/

                                                       // removing null from the end
                                                       codeBuffer[payload.length()] = '*';

                                                       // if the data is of 64 bytes then 0-63 locations (64) will have
                                                       // data and "codeBuffer[payload.length()] = '*';" code line
                                                       // will make the location codebuffer[64] = '*', which will
                                                       // cause an issue wherein the SD card will get 65 bytes instead
                                                       // of 64
                                                       
                                                       if(payload.length()==64)
                                                       {
                                                          codeBuffer[64] = '\0';
                                                       }

                                                       int len = strlen(codeBuffer);
                                                       Serial.print("Length of codeBuffer");
                                                       Serial.println(len);
                                                       
                                                       // writing the data to the file

                                                       mySD.appendFile(SD, "/myFirmware.txt", codeBuffer);

                                                       /*------------------------------------------------------------
                                                         Another bug is fixed here, which was causing the Heap to 
                                                         overflow (seriously!), which was causing a system hard reset
                                                         NMI (kind of),
                                                         however, caught this bug on time and replaced this code line
                                                        dummyCodeFile += payload;    with
                                                        dummyCodeFile  = payload;

                                                         since, our firmware (data from the payload) will be stored
                                                         on the SD card hence there is no need for the given String
                                                         buffer, 

                                                         Furthermore, this code line will be commented as it was only
                                                         for testing purose, or can be used for sending an ack on the
                                                         topic "mytopic/test/response" for the reception of payload.

                                                         Hence, for now this BUG is also fixed
                                                        -------------------------------------------------------------*/
                                                            
                                                                                                         
                                                        //dummyCodeFile = payload;  
                                                        client.publish("mytopic/test/file", codeBuffer);                                                                                                     
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


  // filling the code buffer with * 64 positions
  memset(codeBuffer,'*',(sizeof(codeBuffer)-1));

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
