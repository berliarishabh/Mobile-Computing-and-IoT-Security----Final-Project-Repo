#
# Code Modified for ECEN 5023 by Omkar Reddy Seelam(2) and Rishabh Berlia(1)
#
# Description: The Code flows like this
# 1.It Detects the Bluetooth adapter interfaced with the Rapsberry Pi ( In our Case IO Gear Bluetooth 4.0)
# 2.It Connects to the AdafruitBluefruit LE UART device on the Gecko
# 3.It Dicovers the UART service defined
# 4.It Starts Readind the data from the Device (It will read until one transmission has completed.)
# 5.We Parse the Values and store it in Appropriate variables.
# 6.We convert the status of the values into states (0 'OFF' and 1'ON')
# 7.We transmitted the converted Value to ThingSpeak
#
# Requirements to run this code:
#   1.Bluez 5.37 and above
#   2.Adafruit_Python Library for Bluefruit
#   3.Configure the bluetooth Deamon
#   4.Make sure the bluetooth is up and running by the code sudo hciconfig hci0 up

from twilio.rest import TwilioRestClient
 
# put your own credentials here 
ACCOUNT_SID = "AC5494daeb03b2d70d515fbb3368686579" 
AUTH_TOKEN = "4e22ad671980b60b21b425be4b23e9ee" 
 
client = TwilioRestClient(ACCOUNT_SID, AUTH_TOKEN) 

message1 = "Test message 1 from RPi"
message2 = 3456
 
client.messages.create(
	to="+7609107488", 
	from_="+15105551234", 
	body=message2,  
)
