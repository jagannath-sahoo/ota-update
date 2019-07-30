import paho.mqtt.client as mqtt

# OTA-Server is subcribe to 'mytopic/test/response' for receiving respose
# while it can send firmware packets on 'mytopic/test/request' topic
# ESP need to send the line number of starting and ending sequence in the following format
#       ACK$<starting line number>$<ending line number>
#       Ex: ACK$0$50
# An End Of File string is added to notify ESP for completion of fimware data packets if User don't want
# then just put null instead of "E_O_F"

############################################################
#           GOGGLE COLAB Confiuration for Server
# !pip install paho-mqtt
# Load the Drive helper and mount
# from google.colab import drive

# This will prompt for authorization.
# drive.mount('/content/drive')
# !ls "/content/drive/My Drive/OTA-Server"
# File Location
# '/content/drive/My Drive/OTA-Server/my.hex'
############################################################


update_topic_publish = 'mytopic/test/request'
response_topic_subscribe = 'mytopic/test/response'
end_of_file = 'E_O_F'
auth_data = '$STM32F407VGT6$02$A1B2C3D4$2$08008000$'

##########################################################################################
# CLOUD MQTT AUTH.
##########################################################################################
#username = 'xxxx'
#password = 'xxxx'
ipAddress = '192.168.0.126'
portNo = 1883

# OTA firmware data is parsed into a buffer and maintained in string array
ota_data = []
curr_count = 0
with open('my.hex', 'r') as reader:
    line = reader.readline()  # read a line from hex file
    while line != '':  # Looking for null
        line = line.strip()  # Remove CR-LF
        ota_data.append(line)  # Add to the buffer
        line = reader.readline()
        curr_count = curr_count + 1
    ota_data.append(end_of_file)  # Adding end of file string if don't want then just put null
"""
Get the firmware from the string buffer array if it found "ACK"
then it will publish desired sequence of packets to the publish topic
param:  
        msg:    Received from subscribe topic in string
        start:  starting sequence no 
        end:    ending sequence no
return: void
"""


def getFirmware(msg, start, end):
    if msg == "ACK":
        data = ""
        if end > len(ota_data):
            data = ota_data[len(ota_data) - 1]
            mqttc.publish(update_topic_publish, data)
            return
        while start != end:
            data += ota_data[start]
            start += 1
    print(data)  # debug
    mqttc.publish(update_topic_publish, data)


"""
Parse the received message from subscribed topic
param:
        msg: Received msg from the server in byte array
return: void
"""


def parseMsg(msg):
    str_rec = msg.decode(encoding='utf-8', errors='strict')
    if str_rec == "CHECK":
        mqttc.publish(update_topic_publish, "update_available")
    else:
        rec = str_rec.split('$')
        print(rec)
        # getFirmware(rec[0], int(rec[1]), int(rec[2]))
        getFirmware(rec[0], int(rec[2]) - 1, int(rec[2]))


##########################################################################
#       MQTT Callbacks
##########################################################################
def onMessage(client, obj, msg):
    parseMsg(msg.payload)
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


def onPublish(client, obj, mid):
    print("pub")
    print("mid: " + str(mid))


def onSubscribe(client, obj, mid, granted_qos):
    print("sub")
    print("Subscribed: " + str(mid) + " " + str(granted_qos))


def onConnect(client, userdata, flags, rc):
    print("rc: " + str(rc))


# Create mqtt client
mqttc = mqtt.Client()
mqttc.on_message = onMessage
mqttc.on_connect = onConnect
mqttc.on_subscribe = onSubscribe
mqttc.on_publish = onPublish

# Connect
# mqttc.username_pw_set(username, password)
mqttc.connect(ipAddress, portNo)

# mqttc.publish(update_topic_publish, "update_available")
# Start subscribe, with QoS level 0
mqttc.subscribe(response_topic_subscribe, 0)

mqttc.loop_forever()
# Forever loop
# rc = 0
# while rc == 0:
#  mqttc.loop()
# print("rc: " + str(rc))
