/*********************************************************************************
* 	 REVERB for Amazon ALEXA : HOME AUTOMATION MADE	EASY			 *	*						 				 *
* A Home automation assistant, which not just helps us Monitor and Secure, but 	 *
* also helps us control our house efficiently and without any effort 		 *
*********************************************************************************/

// Global variables
var PREV_STATE = "";			// Initializing variable to store previous state

// Main Alexa Handler
exports.handler = function (event, context) {
    try {
        console.log("event.session.application.applicationId=" + event.session.application.applicationId);
		
        if (event.session.new) {
            onSessionStarted({requestId: event.request.requestId}, event.session);
        }
	// LaunchRequest
        if (event.request.type === "LaunchRequest") {
            onLaunch(event.request,
                event.session,
                function callback(sessionAttributes, speechletResponse) {
                    context.succeed(buildResponse(sessionAttributes, speechletResponse));
                });
        }
	// IntentRequest
	else if (event.request.type === "IntentRequest") {
            onIntent(event.request,
                event.session,
                function callback(sessionAttributes, speechletResponse) {
                    context.succeed(buildResponse(sessionAttributes, speechletResponse));
                });
        }
	// SessionEndRequest
	else if (event.request.type === "SessionEndedRequest") {
            onSessionEnded(event.request, event.session);
            context.succeed();
        }
/*    else {
        invalidIntent(callback);
    }*/
    } catch (e) {
        context.fail("Exception: " + e);
        //invalidIntent(callback);
    }
};

// Function : onLaunchSession
function onSessionStarted(sessionStartedRequest, session) {
    console.log("onSessionStarted requestId=" + sessionStartedRequest.requestId +
        ", sessionId=" + session.sessionId);
}

// Function : onLaunchSession
function onLaunch(launchRequest, session, callback) {
    console.log("onLaunch requestId=" + launchRequest.requestId +
        ", sessionId=" + session.sessionId);

    // Dispatch to your skill's launch; Welcome note
    getWelcomeResponse(callback);
}

// Function : onIntentRequest
function onIntent(intentRequest, session, callback) {
    console.log("onIntent requestId=" + intentRequest.requestId +
        ", sessionId=" + session.sessionId);

    var intent = intentRequest.intent,			// Get the intent
        intentName = intentRequest.intent.name;		// Get intent name

    // Dispatch to your skill's intent handlers
    if ("sensorReadIntent" === intentName) {			// To read sensor data
        sensorReadSession(intent, session, callback);
    } else if ("humidityReadIntent" === intentName) {		// When asked about Humidity
        humidityReadSession(intent, session, callback);
    } else if ("pressureReadIntent" === intentName) {		// When asked about Pressure
        pressureReadSession(intent, session, callback);
    } else if ("temperatureReadIntent" === intentName) {	// When asked about Temperature
        temperatureReadSession(intent, session, callback);
    } else if ("actuateIntent" === intentName) {		// To Brew Coffee (Actuation)
        actuateSession(intent, session, callback);
    } else if ("thankyouIntent" === intentName) {		// During exit; Say Thank you
        thankyouSession(callback);
    } else if ("yesIntent" == intentName){			// Yes intent; Before exiting
        if ("THANKYOU_INTENT" == PREV_STATE){
            yesSession(callback);
        }
    } else if ("errorIntent" == intentName){			// No intent; Before exiting
        //errorSession(callback);
    }
    else if ("AMAZON.HelpIntent" === intentName) {		// Default Intent; For help
        getWelcomeResponse(callback);					// Repeat Welcome Message; If help requested
    } else {
        throw "Invalid intent";
        //invalidIntent(callback);
        console.log("ERROR : Invalid request");
    }
}

// Function : While exiting Session
function onSessionEnded(sessionEndedRequest, session) {
    console.log("onSessionEnded requestId=" + sessionEndedRequest.requestId +
        ", sessionId=" + session.sessionId);
    // Cleanup logic
	// No Logic
}

// Function : Welcome Help
function getWelcomeResponse(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    var sessionAttributes = {};
    var cardTitle = "Welcome";
    var speechOutput = "Welcome to Reverb for Echo, " +
         "Making Home Automation easy, " + ", What can I do for you today?";
    // If the user either does not reply to the welcome message or says something that is not
    // understood, they will be prompted again with this text.
    var repromptText = "Sorry, I didn't hear you, " + "What can I do for you today?";
    var shouldEndSession = false;	// Don't end Alexa session immediately after this

    // Callback for Alexa
    callback(sessionAttributes,
        buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

/*********************************************************************/

// Sensor Read Session : To read sensor data
function sensorReadSession(intent, session, callback) {
    var cardTitle = intent.name;
    var sensorReadSlot = intent.slots.Sensor;
    var repromptText = null;
    var sessionAttributes = {};
    var shouldEndSession = false;
    var speechOutput = "";

    if (sensorReadSlot) {
        var sensorName = sensorReadSlot.value;
        sessionAttributes = createSensorNameAttributes(sensorName);
        
        //console.log("here")
        // HTTP Get
        var jsonResult = getJsonDataThingSpeak(function(jsonResult){
        //console.log("1")
        //console.log(jsonResult);
        var sensorData = jsonResult.feeds[0].field4;
            
        speechOutput = "Requesting the current " + sensorName + " readings from the cloud, " + 
                        "The sensor reading is " + sensorData;
        
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));                                                         
        }); 
    } else {
        //console.log('stuff happened, it\'s bad');
        speechOutput = "I'm not sure what your sensor reading you requested. Please try again";
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
    }
}

// When asked about Temperature
function temperatureReadSession(intent, session, callback) {
    var cardTitle = intent.name;
    var temperatureReadSlot = intent.slots.Room;
    var repromptText = null;
    var sessionAttributes = {};
    var shouldEndSession = false;
    var speechOutput = "";

    if (temperatureReadSlot) {
        var temperatureName = temperatureReadSlot.value;
        sessionAttributes = createSensorNameAttributes(temperatureName);
        if (temperatureName == "living room"){
		//console.log("here")
		// HTTP Get
		var jsonResult = getJsonDataPhoton(function(jsonResult){
		//console.log("1")
		//console.log(jsonResult);
		var temperatureData = jsonResult.feeds[0].field1;
		    
		speechOutput = "Requesting the current temperature readings from the cloud, " + 
		                "It is " + temperatureData + " degrees Celcius";
		
		callback(sessionAttributes,
		    buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));                     
		                            
		});
	    }

    	if (temperatureName == "bedroom"){
    	//console.log("here")
            // HTTP Get
            var jsonResult = getJsonDataEdison(function(jsonResult){
            //console.log("1")
            //console.log(jsonResult);
            var temperatureData = jsonResult.feeds[0].field2;
                
            speechOutput = "Requesting the current temperature readings from the cloud, " + 
                            "It is " + temperatureData + " degrees Celcius";
            
            callback(sessionAttributes,
                buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));                     
                                        
            });
    	}

    } else {
        //console.log('stuff happened, it\'s bad');
        speechOutput = "I'm not sure what your sensor reading you requested. Please try again";
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
    }
}

// Humidity read session
function humidityReadSession(intent, session, callback) {
    var cardTitle = intent.name;
    var humidityReadSlot = intent.slots.Humidity;
    var repromptText = null;
    var sessionAttributes = {};
    var shouldEndSession = false;
    var speechOutput = "";

    if (humidityReadSlot) {
        var humidityName = humidityReadSlot.value;
        sessionAttributes = createSensorNameAttributes(humidityName);
        
        //console.log("here")
        // HTTP Get command
        var jsonResult = getJsonDataPhoton(function(jsonResult){
        //console.log("1")
        //console.log(jsonResult);
        var humidityData = jsonResult.feeds[0].field2;
            
        speechOutput = "Requesting the current humidity readings from the cloud, " + 
                        "The humidity is " + humidityData + "percent";
        
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));                     
                                    
        }); 
    } else {
        //console.log('stuff happened, it\'s bad');
        speechOutput = "I'm not sure what your sensor reading you requested. Please try again";
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
    }
}

// Pressure Read session
function pressureReadSession(intent, session, callback) {
    var cardTitle = intent.name;
    var pressureReadSlot = intent.slots.Pressure;
    var repromptText = null;
    var sessionAttributes = {};
    var shouldEndSession = false;
    var speechOutput = "";

    if (pressureReadSlot) {
        var pressureName = pressureReadSlot.value;
        sessionAttributes = createSensorNameAttributes(pressureName);
        
        //console.log("here")
        // HTTP Get command
        var jsonResult = getJsonDataEdison(function(jsonResult){
        //console.log("1")
        //console.log(jsonResult);
        var pressureData = jsonResult.feeds[0].field1;
            
        speechOutput = "The pressure is " + pressureData + " Kilo Pascals";
        
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));                     
                                    
        }); 
    } else {
        //console.log('stuff happened, it\'s bad');
        speechOutput = "I'm not sure what your sensor reading you requested. Please try again";
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
    }
}

// Thankyou Session
function thankyouSession(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    var sessionAttributes = {};
    var repromptText = null;
    var cardTitle = "Thankyou";
    var speechOutput = "You are welcome, " +
         "Do you want to hear a joke before saying goodbye?";
    var shouldEndSession = false;
    PREV_STATE = "THANKYOU_INTENT";
    callback(sessionAttributes,
        buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

// Yes Session
function yesSession(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    var sessionAttributes = {};
    var repromptText = null;
    var cardTitle = "Yes";
    var speechOutput = "Knock, Knock, , , , ,";
    var shouldEndSession = true;
    callback(sessionAttributes,
        buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

// No Session
function noSession(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    var sessionAttributes = {};
    var repromptText = null;
    var cardTitle = "No";
    var speechOutput = "Huh!, Oh Bummer! I had a good one. Good bye!";
    var shouldEndSession = true;
    callback(sessionAttributes,
        buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

// No Session
function invalidSession(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    var sessionAttributes = {};
    var repromptText = null;
    var cardTitle = "No";
    var speechOutput = "Sorry, I didn't hear you! Can you please repeat that";
    var shouldEndSession = true;
    callback(sessionAttributes,
        buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

/**********************************************************************************/

// Function for HTTP Get (Read Data) from Photon's ThingSpeak Channels
function getJsonDataPhoton(cb) {
    console.log("Function called");
    var url = "https://api.thingspeak.com/channels/102827/feeds.json?results=1";
    var jsonResult = ""; 
    
    
    var http = require('https');

    http.get(url, function(response) {
        console.log('test')
        // Continuously update stream with data
        var body = '';
        response.on('data', function(d) {
            body += d;
        });
        response.on('end', function() {

            // Data reception is done, do whatever with it!
            var parsed = JSON.parse(body);
            jsonResult = parsed;
            
            console.log("2");
            console.log(parsed);
            console.log(jsonResult);
            cb(jsonResult);
        });
    });
}

// Function for HTTP Get (Read Data) from Edison's ThingSpeak channels
function getJsonDataEdison(cb) {
    console.log("getJsonDataEdison called");
    var url = "https://api.thingspeak.com/channels/105448/feeds.json?results=1";
    var jsonResult = ""; 
    
    
    var http = require('https');

    http.get(url, function(response) {
        console.log('test')
        // Continuously update stream with data
        var body = '';
        response.on('data', function(d) {
            body += d;
        });
        response.on('end', function() {

            // Data reception is done, do whatever with it!
            var parsed = JSON.parse(body);
            jsonResult = parsed;
            
            console.log("2");
            console.log(parsed);
            console.log(jsonResult);
            cb(jsonResult);
        });
    });
}

function getJsonDataActuator(cb) {
    console.log("Actuator called");
    var url = "http://192.168.0.102/?coffee=1";
    var jsonResult = ""; 
    
    
    var http = require('http');

    http.get(url, function(response) {
        console.log('test')
        // Continuously update stream with data
//        var body = '';
/*        response.on('data', function(d) {
            body += d;
        });*/
        response.on('end', function() {

        /*    // Data reception is done, do whatever with it!
            var parsed = JSON.parse(body);
            jsonResult = parsed;
            
            jsonResult = 1;
            
            console.log("2");
            console.log(parsed);
            console.log(jsonResult);*/
            cb(jsonResult);
        });
    });
}

function createSensorNameAttributes(sensorName) {
    return {
        sensorName: sensorName
    };
}

// To brew coffee
function actuateSession(intent, session, callback) {
    var cardTitle = intent.name;
    var actuateSlot = intent.slots.Actuate;
    var repromptText = null;
    var sessionAttributes = {};
    var shouldEndSession = false;
    var speechOutput = "";

    if (actuateSlot) {
        
/*		//console.log("actuateCoffee")
		// HTTP Get
		var jsonResult = getJsonDataActuator(function(jsonResult){
		//console.log("1")
		//console.log(jsonResult);
		var data_data = jsonResult.feeds[0];
		    
		speechOutput = "Requesting to brew coffee. It might take a while. Please be patient.";
		
		callback(sessionAttributes,
		    buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));                     
		                            
		});
        */
        
       var actuatePeripheral = actuateSlot.value;
        sessionAttributes = createSensorNameAttributes(actuatePeripheral);
        
        // HTTP Post command
        //var jsonResult = brewCoffee(function(jsonResult){
        console.log("1")
        //console.log(jsonResult);
        
        var http = require('http');
        var options = {
          host : '54.243.17.14',
          path : '/?coffee=1',
          port : '8080'
        };
        
        callback_func = function(response) {
          var str = '';
          console.log('str');
        };

        http.request(options, callback_func).end();
        console.log('111');
        
        speechOutput = "Requesting to brew coffee. It might take a while. Please be patient.";
        
         console.log('2222');
         
        callback(sessionAttributes,
            buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));               
                                    
    } else {
        //speechOutput = "I'm not sure what you said. Please try again";
    }
}

/*******************************************************************************/

function buildSpeechletResponse(title, output, repromptText, shouldEndSession) {
    return {
        outputSpeech: {
            type: "PlainText",
            text: output
        },
        card: {
            type: "Simple",
            title: "SessionSpeechlet - " + title,
            content: "SessionSpeechlet - " + output
        },
        reprompt: {
            outputSpeech: {
                type: "PlainText",
                text: repromptText
            }
        },
        shouldEndSession: shouldEndSession
    };
}

function buildResponse(sessionAttributes, speechletResponse) {
    return {
        version: "1.0",
        sessionAttributes: sessionAttributes,
        response: speechletResponse
    };
}