/*********************** This is final Working code as of now and all the functionalities are working fine *******************

    == SD card has been implemented, just checksum is added ==

serial data from ESP32 to the serial device (STM)
is working fine

MQTT publisher (reading sensor data from the STM and uploading the same data to MQTT is working)

Publisher and subscriber is also working fine now in this system


previously:
              There is a bug wherein while receiving "check_update" from the STM, ESP32 is directly replying to the STM32
              with the old string, we need to update the same

Update:  
              bug has been removed now from the givens system, whering the bug was encountered as mentined above


    Minor Bug is there while requesting the update from the server,
    where the ESP32 should accept only "file_confirmed", then it should start requesting the file
    however the ESP32 is requesitng the file for any string 
*************************/


//-------------------- Includes -----------------------
#include "FS.h" 
#include "SD.h" 
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
void flashSTMBorad();
//void publishRecieve(unsigned char request, const String topic)  // this will initiate client.loop()

//--------------------Declarations End Here-----------







//-------------------Global Variables ----------------
  EspMQTTClient client(
  "Vrinda_33",
  "littleindia",
  "192.168.0.122",  // MQTT Broker server ip
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
String codeBytePayload;

const String updateAvailable = "update_available";
const String endOfFile       = "E_O_F";


const String receiveRequest    = "mytopic/test/request"; // sub
const String publishResponse   = "mytopic/test/response";// pub
const String publishSensor     = "mytopic/test/sensor";  // pub

String mqttPublishTopic  = "";

volatile char receptionFlag = 'F';
volatile char checkUpdateFlag = 'F';

// we need to take this value as the last char will be null character which will end the file
char codeBuffer[65];
//----------------------------------------------------

//#############New Added Update#########################
//######################################################
String ack             = "ACK$0$";
String ackAppend;
long int lineCount     =  0;
volatile char fileDownloadCompleted  = 'F';     
volatile char updateCompleted        = 'F';
volatile char serverConfirmation     = 'F'; 
//######################################################
//######################################################




//----------------------------------------------


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
    // /*  Uncomment this line comment to comment this method
       void readFile(fs::FS &fs, const char * path)
       {
              Serial.printf("Reading file: %s\n", path);
    
              File file = fs.open(path);
              if(!file){
                  Serial.println("Failed to open file for reading");
                  return;
              }
            
              Serial.print("Read from file: ");                            
              Serial.println();

                char codeByte;
                unsigned char arr[64];
                String checkSum;
                String recCheckSum;
                int myIndex=0;
                unsigned char *ptr;
                char authflag = 'F';
                char ch;  
                while(file.available())
                {
                   
                     delay(5);

                     // reading the file data into an array 
                     for(int i = 0; i<sizeof(arr); i++)
                     {
                        arr[i] = file.read();
                     }
      
                     // array created, now checksum needs to be calculated
                     
                      myIndex=0;  
                      ptr = arr;

                      // locating the * in the array
                      while(*ptr++ != '*')myIndex++;
                          
                      //  /*  for testing
                      Serial.print("* Found at index ");
                      Serial.println(myIndex);
                      // */

                      
                      Serial.print("Check sum is:");


                      // creating the checksum
                      ch = arr[myIndex-2];
                      checkSum  = String(ch);
                      //Serial.print(ch);
                      ch = arr[myIndex-1];                      
                      //Serial.println(ch);
                      checkSum  += String(ch);
                                 
                      Serial.print("Checksum is: ");
                      Serial.println(checkSum);

                     

                     for(int j = 0; j<sizeof(arr); j++)
                     {
                      ch = arr[j];
                     Serial2.print(ch);
                     }

                     //============Checksum check========================
                     while(authflag != 'T')
                       {
                            // here we need to wait for the data from the Serial2  (Uart2)
                            // or else our code will misbehave
                              if(Serial2.available() > 0)
                               {       
                                      recCheckSum = recieveStringFromUart();
                                   
                                      if(recCheckSum == checkSum)
                                      {
                                        authflag = 'T';
                                        Serial.println("Checksum matched!");   
                                      }
                                      else
                                      {
                                          Serial.println("Checksum mismatch! Sending again");                          
                                          // Code byte received by the STM is incorrect, 
                                          //hence send the stiring again to the STM board
                                           for(int j = 0; j<sizeof(arr); j++)
                                           {
                                                ch = arr[j];
                                                Serial2.print(ch);
                                           }
                                      }
                               }
    
                               // if no data is available on the Serial2 then wait for the data
                        } 
                          // enabling the while loop for second iteration
                          authflag = 'F';
                     //====================================================
                }

        file.close();
    }    //  */     //  */ 

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

//######################################################
//######################################################





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
              if(initSD())
              {
                  Serial.println("SD card initialization failed");
                  while(1);  
               }
            Serial.println("SD card initialization done");
            
            // this is the blocking call to get the file downloaded from the internet
            Serial.println("Downloading the file");

            client.publish("mytopic/test/response", "ESP32Ready");

            
            while(fileDownloadCompleted != 'T'){client.loop();}

            Serial.println("File downloaded sucessfully");


            // once the file is downloaded the given code will come here
            // now we need to burn the STM

            while(updateCompleted != 'T')
            {
              flashSTMBorad();
            }  

            // after performing the Update, changing the flag to False
            checkUpdateFlag = 'F';            //for normal functionality
            fileDownloadCompleted = 'F';      //for normal functionality
            updateCompleted = 'F';
            serverConfirmation = 'F';         //for normal functionality
            break;  
        }

        default:
        {
          break;  
        }
    }
}


void flashSTMBorad()
{
        //  Means update has been done sucessfully
    Serial.println("Flashing the file to the STM board");
    
    client.publish("mytopic/test/response", "File Has been received sucessfully");

    
    mySD.readFile(SD, "/myFirmware.txt");
    updateCompleted = 'T';
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
              
            if(checkUpdateFlag == 'D')
            {
                           codeBytePayload = payload;

                          if(codeBytePayload == "file_confirmed")
                            {
                                  // requesting update from the server adn making the confirmation flag true
                                  serverConfirmation = 'T';
                                  // this will now send the request for the line 1
                                  ++lineCount;
                                  ackAppend =  ack + String(lineCount);
                                  client.publish("mytopic/test/response", ackAppend);
                            }

                           
            
                          //############# new added ########
                          if (payload != endOfFile && checkUpdateFlag    == 'D' && serverConfirmation == 'T')
                          {
                                                       Serial.println("Inside the first if");

                                            
                                                       //++lineCount;
                                        
                                                       //ackAppend =  ack + String(lineCount);

                                                    /*
                                                       if(codeBytePayload == "file_confirmed")
                                                       {
                                                          // this will now send the request for the line 1
                                                          client.publish("mytopic/test/response", ackApp                                        end);
                                                       }
                                                     */
            
                                                        // filtering the code bytes
                                                        // raising the flag high till the update code bytes are available
                                                        // updateAvailableFlag    = 'T';
                            

                                                        if(codeBytePayload != "file_confirmed")
                                                        {
                                                          ++lineCount;
                                                          ackAppend =  ack + String(lineCount);
  
                                                                
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
      
                                                             // after the data is written to the file, requesting more data from server
                                                             client.publish("mytopic/test/response", ackAppend);
                                                             Serial.println(ackAppend);
      
      
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

                                                  else if(payload == endOfFile &&  checkUpdateFlag  == 'D')
                                                  {

                                                     // once everything worked fine, we need to make the line count zero
                                                     lineCount = 0;
                                                     // this will make sure that next time data will be requested from the
                                                     // server from line 1
                                                     
                                                     Serial.println("Inside the second if");
                                                     client.publish("mytopic/test/response", "EOF Received");

                                                     checkUpdateFlag    = 'F';
                                                     fileDownloadCompleted  = 'T';                                                      
                                                  }
                                                  else
                                                  {
                                                    
                                                    Serial.println("Inside the else state of flag is ");
                                                    client.publish("mytopic/test/response", "Invalid !!! Send again!!!");
                                                    Serial.println("No Action Available for this request");
                                                  }
            }  

            else
            {
                payloadReceived = payload;
                Serial.println(payload);
                receptionFlag = 'T';
                
            }

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
