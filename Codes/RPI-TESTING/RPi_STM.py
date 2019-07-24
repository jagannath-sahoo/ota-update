#######################################################################################################################
#
#	@name		:	RPi_STM.py
#	@date		:	23/07/2019
#	@SoC		:	RaspberryPi 3B
#	@author		:	Abdul Ahad
#
# 	@Description: 	This Script is used for TESTING of a project OTA update. Here RaspberryPi
#					is communicating with STM32F407VGT6 to update its firmware "OVER THE AIR"
#					This Script is reading the INTEL HEX file from the local storage and sending
# 					the machine data from that file line by line to STM Board via UART.
#					Various HandShaking steps are implemented in this Script to establish
#					synchronization between RPi and STM while data exchange.
#
#					In the real implementation, ESP32 is used instead of RPi for the challenges 	
#					it provide.	
#
#	@Flow 		:	1. 	RPi will interrupt the STM board by setting the pin 3 HIGH which is  
#						connected to the EXTERNAL INTERRUPT PIN PB1 of the STM Board
#
#					2.	Rpi will wait or poll till '$' is received from STM side
#
#					3.	When '$' is received, RPi will send the AUTHORIZATION FRAME to STM.
#						Description of AUTHORIZATION FRAME is as follows:
#
#						$<BoardName>$<FirmwareVersion>$<SecureID>$<FlashSectorNo>$<FlashSectorAddress>$<0000..000>
#						Here,
#								BoardName			:	STM32F407VGT6
#								FirmwareVersion		:	1.0
#								SecureID			:	A1B2C3D4	(Uniqued ID for the controller)
#								FlashSectorNo		:	2			(Sector No of STM FLASh where code is to be written)
#								FlashSectorAddress	:	08008000	(Address Corresponding to above sectorNo)
#								<0000..000>			:	These are the extra Zeros/Null which are added to pad the packet
#
#						NOTE: 	Length of a packet decided to tranmit from RPi to STM is of 64 bytes. Thus a byte array is
#								created in which the above AUTHORIZATION DATA is filled. To give padding to this array, 
#								extra characters (Zeros/Null) are filled in this array at the end.	
#
#					4.	STM Application Code will check the AUTHORIZATION FRAME. If the AUTHORIZATION DATA is valid, STM
#						Application Code will send the ACK PACKET, which is nothing but just the SecureID in Reverse order, 
#						to the RPi. Application code will also notify the user that a valid update is available by turning 
#						ON any LED. Now, if the LED is ON user can press the PUSHBUTTON anytime to start updating the firmware.
#						If STM Application finds update to be invalid, it will send secureID(AS IT IS) as a NACK packet. 
#						This infers that the update is invalid. Apart from ACK/NACK, if any packet is received, it will be 
#						considered as "Error In Connection"
#						NOTE: All ACK/NACK should be terminated by a '$' token.
#   
#						.  
#					5.	If ACK PACKET is received (Update is valid), RPi will wait/poll till '$' is sent from STM side.
#						Whenever user presses the PUSHBUTTON (in STM), Application Code will jump to the BOOTLOADER Code 
#						and the BOOTLOADER code will sent '$' token to RPi notifying that it is ready to receive the 
#						updated firmware
#											
#
#					6.	If '$' is received in the previous step, RPi can send further packets i.e. HEX FILE.
#						RPi will read the HEX file line by line and send the respective line of HEX to the STM.
#						It must be noted that the Hex file read from the local storage should be stripped off of any 
#						white spaces (like CR/LF). Also, as decided packet size is of 64 bytes, extra padding of Zeros/Null
#						must be provided with this HEX line. Thus, making the standard packet size of 64 bytes.
#					
#					7.	Last 2 bytes of the HEX line (without padding) is Checksome. So, after sending the HEX line,
#						RPi will wait/poll for the checksum of the line sent as ACK. If checksum received does not matches
#						with the checksum of the HEX Line sent (NACK), RPi will resend the packet(HEX Line) till correct checksum
#						(ACK) is not received.
#
#					8.	If ACK is received, RPi will read and send the next HEX Line. This process continues till EOF(End Of File)
#						is encountered. Here also, every ACK/NACK should be terminated with '$' token and size of packet to be sent
#						must be of 64 bytes.
#					
####################################################################################################################################


# importing libraries
'''--------------------------------------------------------------------------------------------------------------'''
import RPi.GPIO as gpio
import time
import serial
'''--------------------------------------------------------------------------------------------------------------'''


# defining functions
'''--------------------------------------------------------------------------------------------------------------'''
''' NOT USED
def list_to_hex(check):
	str =""
	check = str.join(check)
	return int(a, 16)	#hex_int
'''

'''--------------------------------------------------------------------------------------------------------------'''

# setting up things
'''--------------------------------------------------------------------------------------------------------------'''
gpio.setmode(gpio.BOARD)				# Setting the mode for RPi GPIO
gpio.setwarnings(False)	
gpio.setup(3, gpio.OUT)					# Making pin 3 as output
boardName   =   ['S','T','M','3','2','F','4','0','7','V','G','T','6']
version     =   ['1','.','0']
secureID    =   ['A','1','B','2','C','3','D','4'];
flashSectorNo=  '2'
sectorAddr  =   ['0','8','0','0','8','0','0','0']

buffer = list(bytearray(64))  			# Creating a buffer of 64 bytes to send over UART

'''Adding Token'''
buffer[0]   =   '$'
buffer[1:len(boardName)]    =   boardName
buffer[len(boardName)+1]   = '$'
buffer[len(boardName)+2:len(boardName)+len(version)+2] = version
buffer[len(boardName)+len(version)+2] = '$'
buffer[len(boardName)+len(version)+3:len(boardName)+len(version)+len(secureID)+3] = secureID
buffer[len(boardName)+len(version)+len(secureID)+3] = '$'
buffer[len(boardName)+len(version)+len(secureID)+4:len(boardName)+len(version)+len(secureID)+len(flashSectorNo)+4] = flashSectorNo
buffer[len(boardName)+len(version)+len(secureID)+len(flashSectorNo)+4] = '$'
buffer[len(boardName)+len(version)+len(secureID)+len(flashSectorNo)+5:len(boardName)+len(version)+len(secureID)+len(flashSectorNo)+len(sectorAddr)+5] = sectorAddr
buffer[len(boardName)+len(version)+len(secureID)+len(flashSectorNo)+len(sectorAddr)+5] = '$'
'''--------------------------------------------------------------------------------------------------------------'''



# main program
'''--------------------------------------------------------------------------------------------------------------'''
ser = serial.Serial("/dev/ttyS0", 115200)    	#Open port with baud rate
#ser.write("Hello from RPi\n\r")

####### STEP 1 ########
gpio.output(3, 1)   							# Interrupting the STM for update
												# Setting the interrupt line HIGH
########################


''' Not reqired Now
while gpio.input(3) != 0:     # Waiting for the Acknowledgement from STM
    pass
'''

############ STEP 2 ################
print("Waiting for $")
time.sleep(1)
received_data = ser.read_until('$')   			# Receiving data (ACK) till '$' is received
print("$ Received")
#####################################



############ STEP 3 ################
ser.write(buffer)  			 					# Sending 1st block for authentication
print(buffer)
####################################



########### STEP 4 #################
'''Checking for ACK --------'''
#Waiting for the
#1. Acknowledgement		: 	Success (Reverse of secureID)
#2. NonAcknowledgement	:	Failure (secureID)
received_data = ser.read_until('$')
received_data = received_data[0:len(received_data)-1] # Removing token($) from the received string
print("Received data= ",received_data)
print(type(received_data))
####################################


''' FOR TESTING'''
test = str(secureID[::-1])
print("secureID = ",test)
print(type(test))


str1 = ""
if received_data == str1.join(secureID[::-1]):
	# Received reverse of secureID (SUCCESS)
	# ACK PACKET RECEIVED
        print("ACK received")
		
		######## STEP 5 ###########
        print("Waiting for $")
        time.sleep(1)
        received_data = ser.read_until('$')   		# Receiving data (ACK) till '$' is received
        print("$ Received")
		###########################
		
	count = 0;
	with open('my.hex', 'r') as reader:
		line = reader.readline()					# Reading HEX File line by line
		
		######## STEP 6 ##########
		while line != '':  							# The EOF char is an empty string
			line = line.strip()     				# Removing Whitespaces(HERE, CRLF) from the line
			checksum = line[len(line)-2:len(line)]	# Extracting CHECKSUM from the HEX line
            print("Checksum = ",checksum)
			buffer = list(bytearray(64))  			# Creating a buffer of 64 bytes to send over UART
			buffer[0:len(line)] = line     			# adding the line of the hex file in the buffer
			print(str(buffer))						# Printing String to be sent
			#print(len(buffer))
			count = count+1;
			ser.write(buffer)  						# Sending a line from HEX file
			
			
			######## STEP 7 ###########
			'''Checking for ACK --------'''
			#Waiting for the
			#1. Acknowledgement		: 	Success (CHECKSUM of the line sent)
			#2. NonAcknowledgement	:	Failure (anything else)
			# if Failure : line will be resend again and again till checksum is received
				#received_data = ser.read_until('$')   					# Receiving data (ACK/NACK) till '$' is received
				#received_data = received_data[0:len(received_data)-1] 	# Removing token($) from the received string
			# Receiving ACK/NACK and checking for CHECKSUM
			while(1):
				received_data = ser.read_until('$')
                received_data = received_data[0:len(received_data)-1]
                print(received_data)
                print("Received ACK/NACK")
			    #time.sleep(1)
                if(received_data != checksum):
				    # Invalid Checksome
				    print("Received NACK: Sending Packet Again")
				    ser.write(buffer) # Sending HEX line again
                else:
                    break
			
			print("Line",count,"Succesfully sent")
			
			######## STEP 8 ###########
			line = reader.readline()
			
			''' FOR TESTING
			if count == 294:
				#break
				pass
				'''
elif received_data == secureID:
	# Received same secureID (FAILURE)
	# NACK PACKET RECEIVED
	print("Invalid Update")

else:
	print("Problem in communication with the TARGET")


'''--------------------------------------------------------------------------------------------------------------'''
