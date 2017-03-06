# Mobile-Computing-and-IoT-Security----Final-Project-Repo
# Mobile-Computing-and-IoT-Security----Final-Project

#### Motivation:

With the advent of mass-deployed broadband Internet connections and the development of Wireless technologies (BLE, Zigbee etc.), smart connected devices in the home are a very real and practical possibility. 

A Smart Home is one that is designed to deliver or distribute a number of services within and outside the home through a range of networked devices. [Juniper Research]

#### Objective:

To develop a complete Internet of Thing system for a Smart Home such as Samsung SmartThings, Notion etc. This would require us to build individual BLE based Sensor Tags. Every Tag would have a specific sensor to sense parameters such as Temperature, Humidity, Ambient Light, Proximity, Pressure, Touch etc. 
These Tags will be built on the BLE Nano (based on the Nordic nRF51822 SoC). Each individual tag will have the sensor circuit interfaced with the BLE Nano and would run on a single coin-cell battery (CR-2032). We would use the Leopard Gecko EFM32 Board as one of the Tags as well to use the LESENSE sensor. 

The Sensor Tags would connect and transmit data to a small Microcomputer (Raspberry Pi 3) based hub. Since, the Raspberry Pi has Wi-Fi connectivity it enables us to take our data to the cloud, where the data processing would take place. 

We plan to use one of the following platforms to collect, visualize & analyze data and trigger responses accordingly:
•	Sci-kit Learn
•	ThingSpeak
•	Phant
This would allow us to host a web server on Rasp Pi for the User Interface where the User can monitor all the sensor data on Graphs and Issue Override Commands such as GetTemp, GetPressure etc.
If we are able to accomplish this well within our time bounds, we would try to implement an App to give notifications on the Phone.

Also, since we have the sensor data and trigger points we could use to actuate physical things inside your home e.g. Using the Ambient light and Temperature sensor data, a servo motor could be used to actuate your Window Blinds depending on the Light and temperature settings etc. 

There would be 2 set of Commands, Trigger Commands and Override Commands. 
Trigger Commands are sent through the ThingSpeak or Sci-Kit platform depending on the set points. If the sensor data goes above or below a notification would be given. These commands are even used to Actuate something if required.

Override Commands are sent by the user through the UI, these commands are to get the most recent Sensor Data and post it on the Screen.


#### Block Diagram of a System
![block-diag](https://github.com/berliarishabh/Mobile-Computing-and-IoT-Security----Final-Project/blob/master/Images/BlockDiagram.png?raw=true)

#### Block Diagram of the System
![System-d](https://github.com/berliarishabh/Mobile-Computing-and-IoT-Security----Final-Project/blob/master/Images/SystemOverview.png?raw=true)


#### Test Methodology: 

1.	Testing functionality of individual Sensor Tags
  a.	Validate sensor data 
  b.	Testing if sensor tag posts data according to Set Points
  c.	Testing if sensor tag sends data when requested with a command.
2.	Testing the Server
  a.	Test individual sensor connections to the server
  b.	Test the Data Logging and setup Visualization through Graphs.
  c.	Testing the Trigger commands and Override Commands
3.	Integration and Testing
  Integration of the system and testing individual points
