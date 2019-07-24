Author: Aman Kanwar

  Current progress:
  
    1: SD card is functional and I am able to create, update, delete a file inside the SD card 
      (interfaced with the ESP32 module using SD card module).
      
    2: MQTT is impelented on ESP32, using which our server (MQTT publisher) will publish the data
       on specific topic (which our ESP32 will be listening), and based on the received payloads
       our ESP32 will react accordingly.
       
    3: Paloads from the MQTT will be written to the file in the SD card.
