#!/usr/bin/env python
# Example of interaction with a BLE UART device using a UART service
# implementation.
# Author: Tony DiCola
#
# Code Modified for ECEN 5023 by Omkar Reddy Seelam and Rishabh Berlia
#
# Description: The Code flows like this
# 1.It Detects the Bluetooth adapter interfaced with the Rapsberry Pi ( In our Case IO Gear Bluetooth 4.0)
# 2.It Connects to the AdafruitBluefruit LE UART device on the Gecko
# 3.It Dicovers the UART service defined
# 4.It Starts Readind the data from the Device (It will read until one transmission has completed.)
# 5.We Parse the Values and store it in Appropriate variables.
# 6.We convert the status of the values into states (0 'OFF' and 1'ON')
# 7.We transmitted the converted Value to ThingSpeak
# 8.It actuates the actuators (Buzzer, Servo etc) if the conditions are met.
#
# Requirements to run this code:
# 1.Bluez 5.37 and above
# 2.Adafruit_Python Library for Bluefruit
# 3.Configure the bluetooth Deamon
# 4.Make sure the bluetooth is up and running by the code sudo hciconfig hci0 up
import time
import Adafruit_BluefruitLE
from Adafruit_BluefruitLE.services import UART

#############################################
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

GPIO.setup(8,GPIO.OUT)                         # for RGB
#GPIO.setup(13,GPIO.OUT)                         # for RGB
#GPIO.setup(15,GPIO.OUT)                         # for RGB

#GPIO.output(13,0)                                       # for RGB
#GPIO.output(15,0)                                       # for RGB

pwm = GPIO.PWM(8,50)

GPIO.setup(7,GPIO.OUT)                          # for relay

GPIO.output(7,0)                                        # For the Relay (for the coffee machine)

def alarm():
    print "Intrusion Detected"
    GPIO.output(7,0)
    time.sleep(5)
    GPIO.output(7,1)

#def led():                                                                              # For turning on/off Bulbs
#    print 'LED'
#    GPIO.output(11,1)
#    time.sleep(0.01)
#    GPIO.output(13,1)
#    time.sleep(0.01)
#    GPIO.output(15,1)
#    time.sleep(0.01)
####################################################################


# Get the BLE provider for the current platform.
ble = Adafruit_BluefruitLE.get_provider()

#Convert Status Function
def ConvStatus(state):
    if state == 'Y':
	return 1
    else:
	return 0
# Main function implements the program logic so it can run in a background
# thread.  Most platforms require the main thread to handle GUI events and other
# asyncronous events like BLE actions.  All of the threading logic is taken care
# of automatically though and you just need to provide a main function that uses
# the BLE provider.
def main():
    while True:
        # Clear any cached data because both bluez and CoreBluetooth have issues with
        # caching data and it going stale.
        ble.clear_cached_data()
        # Get the first available BLE network adapter and make sure it's powered on.
        adapter = ble.get_default_adapter()
        adapter.power_on()
        print('Using adapter: {0}'.format(adapter.name))
        # Disconnect any currently connected UART devices.  Good for cleaning up and
        # starting from a fresh state.
        print('Disconnecting any connected UART devices...')
        UART.disconnect_devices()

        # Scan for UART devices.
        print('Searching for UART device...')
        try:
            adapter.start_scan()
            # Search for the first UART device found (will time out after 60 seconds but you 
            # can specify an optional timeout_sec parameter to change it).
            device = UART.find_device()
            if device is None:
                raise RuntimeError('Failed to find UART device!')
        finally:
            # Make sure scanning is stopped before exiting.
            adapter.stop_scan()

        print('Connecting to device...')
        device.connect()  # Will time out after 60 seconds, specify timeout_sec parameter
                          # to change the timeout.

        # Once connected do everything else in a try/finally to make sure the device
        # is disconnected when done.
        try:
            # Wait for service discovery to complete for the UART service.  Will
            # time out after 60 seconds (specify timeout_sec parameter to override).
            print('Discovering services...')
            UART.discover(device)

            # Once service discovery is complete create an instance of the service
            # and start interacting with it.
            uart = UART(device)

            # Write a string to the TX characteristic.
            # uart.write('RetTemp!')
            # print("Sent 'Command' to the device.")

            # Now wait up to one minute to receive data from the device.
            received = uart.read(timeout_sec=5)
	    ProximityStatus = received[0]
	    LCStatus = received[1]
	    LightStatus = received[2]
	    SecurityStatus = received[3]
	    Temperature = received[4] + received[5] + received[6] + received[7] + received[8]
       	    if received is not None:
       	        # Received data, print it out.
       	        print('Received: {0}'.format(received))
	        print('Proximity status:{0} '.format(ProximityStatus))
	        print('LC status:{0} '.format(LCStatus))
	        print('Security  status:{0} '.format(SecurityStatus))
	        print('Temperature:{0} '.format(Temperature))
	        ProximityStatus = ConvStatus(ProximityStatus)
	        LCStatus = ConvStatus(LCStatus)
	        SecurityStatus = ConvStatus(SecurityStatus)
		    
		##################### #TEXT ALERTS USING TWILIO #######################
		from twilio.rest import TwilioRestClient
		# put your own credentials here
		ACCOUNT_SID = "AC5494daeb03b2d70d515fbb3368686579"
		AUTH_TOKEN = "4e22ad671980b60b21b425be4b23e9ee"
		client = TwilioRestClient(ACCOUNT_SID, AUTH_TOKEN)
		if Temperature >= +30:
			pwm.start(2)
			print "Temperature is above set max"
			client.messages.create(
			        to="+17609107488",
			        from_="+17608002048",
			        body="Temperature is above set max",
			)
			pwm.stop()
                if Temperature <= +15:
			print "Temperature is below set min"
                        client.messages.create(
                                to="+17609107488",
                                from_="+17608002048",
                                body="Temperature is below set min",
                        )
		if ProximityStatus == 1:
			print "Intrusion detected"
			GPIO.output(7,1)
                        client.messages.create(
                                to="+17609107488",
                                from_="+17608002048",
                                body="Intrusion Detected",
                        )
			GPIO.output(7,0)
		#########################################################################

     	        import httplib, urllib
	        params = urllib.urlencode({'field1': Temperature,'field2':SecurityStatus,'field3':ProximityStatus,'field4':LCStatus,'key':'4I4Q3U288NEQ5TDV'})     # use your API key generated in the thingspeak channels for the value of 'key'
                # temp is the data you will be sending to the thingspeak channel for plotting the graph. You can add more than one channel and plot more graphs
                headers = {"Content-typZZe": "application/x-www-form-urlencoded","Accept": "text/plain"}
     	        conn = httplib.HTTPConnection("api.thingspeak.com:80")                
                try:
               	    conn.request("POST", "/update", params, headers)
               	    response = conn.getresponse()
               	    print Temperature
               	    print response.status, response.reason
               	    data = response.read()
               	    conn.close()
                except:
               	    print "connection failed"	
	    else:
             # Timeout waiting for data, None is returned.
                 print('Received no data!')
        finally:
            # Make sure device is disconnected on exit.
            device.disconnect()
	    time.sleep(10)


# Initialize the BLE system.  MUST be called before other BLE calls!
ble.initialize()

# Start the mainloop to process BLE events, and run the provided function in
# a background thread.  When the provided main function stops running, returns
# an integer status code, or throws an error the program will exit.
ble.run_mainloop_with(main)

