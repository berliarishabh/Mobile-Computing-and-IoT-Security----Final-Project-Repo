{
  "intents": [
    {
      "intent": "sensorReadIntent",
      "slots": [
        {
          "name": "Sensor",
          "type": "LIST_OF_SENSORS"
        }
      ]
    },
    {
      "intent": "temperatureReadIntent",
      "slots": [
        {
          "name": "Room",
          "type": "ROOM_LIST"
        }
      ]
    },
    {
      "intent": "pressureReadIntent",
      "slots": [
        {
          "name": "Pressure",
          "type": "PRESSURE_LIST"
        }
      ]
    },
    {
      "intent": "actuateIntent",
      "slots": [
        {
          "name": "Actuate",
          "type": "LIST_OF_ACTUATORS"
        }
      ]
    },
 	{
      "intent": "humidityReadIntent",
      "slots": [
        {
          "name": "Humidity",
          "type": "HUMIDITY_LIST"
        }
      ]
    },
    {
      "intent": "thankyouIntent",
      "slots": [
        {
          "name": "Thankyou",
          "type": "THANKYOU_LIST"
        }
      ]
    },
    {
      "intent": "yesIntent"
    },
    {
      "intent": "noIntent"
    },
    {
      "intent": "repeatIntent"
    },
    {
      "intent": "errorIntent"
    },
    {
      "intent": "AMAZON.HelpIntent"
    }
  ]
}