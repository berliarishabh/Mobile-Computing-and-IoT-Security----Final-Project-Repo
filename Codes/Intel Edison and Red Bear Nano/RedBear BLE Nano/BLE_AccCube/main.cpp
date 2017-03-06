/* Source Code for Hear Rate Monitor obtainfrom the mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited. Licensed under the Apache License, Version 2.0 (the "License");
 * This is the modified version of the said code. 
 *
 * Modified code written by: Omkar Reddy and Rishabh Berlia
 * 
 * Description: This code uses the BLE_HeartRateMonitor example from mbed to transmit Temperature and Pressure values obtained from the BMP180
 * The MMA8452 communicates to the BLE Nano via I2C. We have used the MMA8452 library written by Ashley Mills, Nicholas Herriot (See MMA8452.cpp) and edited it according to our application.
 * 
 */


#include "mbed.h"
#include "MMA8452.h"
#include "ble/BLE.h"
#include "ble/services/HeartRateService.h"
#include "ble/services/HealthThermometerService.h"
#include "ble/services/DeviceInformationService.h"


MMA8452 acc(P0_7, P0_15, 100000); //Declare the SCL(P0_15) and SDA(P0_7) ports and make an instance called i2c. Also pass the sample frequency for the accelerometer in Hz.
Serial pc(USBTX,USBRX);

int x, y, z;       //Variables to get the X axis, Y axis and Z axis
int Px,Py,Pz = 0;  //Previous accelerometer values
int Cx,Cy,Cz;      //Current value of acceleometer 

DigitalOut led1(LED1); //Device status LED

const static char     DEVICE_NAME[]        = "ACC1";    //Name of the Device in advertising mode
static const uint16_t uuid16_list[]        = {GattService::UUID_HEART_RATE_SERVICE,
                                              GattService::UUID_DEVICE_INFORMATION_SERVICE};
static volatile bool  triggerSensorPolling = false;

uint8_t hrmCounter = 0; // init HRM to 100bps

HeartRateService         *hrService;        //The Hear Rate Service instance
DeviceInformationService *deviceInfo;
 
/*
Name: checkMove function.
Description: It reads the accelerometer values and compares the current value to the Previous values. If there is a change in acceleration, it detects the movement and returns 1 for movement and 0 for no change.
*/ 
   
int checkMove(int x, int y, int z)
{
    int val = 0;
    acc.readXYZGravity(&x,&y,&z);
    Cx=x;
    Cy=y;
    Cz=z;
        
    if(Cx!=Px || Cy!=Py || Cz!=Pz)
    {  
    Px=Cx;
    Py=Cy;
    Pz=Cz;
    val=1;
    }
    else
    {val=0;
    }
    return val;
    
}

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance(BLE::DEFAULT_INSTANCE).gap().startAdvertising(); // restart advertising
}

void periodicCallback(void)
{
    led1 = !led1; /* Do blinky on LED1 while we're waiting for BLE events */

    /* Note that the periodicCallback() executes in interrupt context, so it is safer to do
     * heavy-weight sensor polling from the main thread. */
    triggerSensorPolling = true;
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE &ble          = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service. */
    hrService = new HeartRateService(ble, hrmCounter, HeartRateService::LOCATION_FINGER);

    /* Setup auxiliary service. */
    deviceInfo = new DeviceInformationService(ble, "ARM", "Model1", "SN1", "hw-rev1", "fw-rev1", "soft-rev1");

    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_HEART_RATE_SENSOR);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();
}


int main() {
  
    led1 = 1;
    Ticker ticker;
    ticker.attach(periodicCallback, 1); //Blink LED every second and transmit data evry second

    BLE& ble = BLE::Instance(BLE::DEFAULT_INSTANCE);
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }
  
  while(1) {

    
    if (triggerSensorPolling && ble.getGapState().connected) {
        triggerSensorPolling = false;
        
        if(checkMove(x,y,z)!=0)   //Check the return value from checkMoe function and update the variable to transmit.
        hrmCounter=1;
        else
        hrmCounter=0;
        
    hrService->updateHeartRate(hrmCounter); ////Update the Movement value to the Service
    } else {
            ble.waitForEvent(); // low power wait for event
        }

      wait(.50);
   }
}