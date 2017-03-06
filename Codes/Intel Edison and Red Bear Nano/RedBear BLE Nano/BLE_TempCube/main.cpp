/* Source Code for Hear Rate Monitor obtainfrom the mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited. Licensed under the Apache License, Version 2.0 (the "License");
 * This is the modified version of the said code. 
 * 
 * Modified code written by:  Omkar Reddy and Rishabh Berlia
 * 
 * Description: This code uses the BLE_HeartRateMonitor example from mbed to transmit Temperature and Pressure values obtained from the BMP180
 * The BMP180 communicates to the BLE Nano via I2C. We have used the BMP180 library written by Maxim (See BMP180.cpp) and edited it according to our application.
 * 
 */
#include "BMP180.h"                         
#include "mbed.h"
#include "ble/BLE.h"
#include "ble/services/HeartRateService.h"
#include "ble/services/HealthThermometerService.h"
#include "ble/services/DeviceInformationService.h"

I2C i2c(P0_7, P0_15);    //Declare the SCL(P0_15) and SDA(P0_7) ports and make an instance called i2c.
BMP180 bmp180(&i2c);    //Pass the i2c instance to the bmp180 instance. It takes the I2C pins.

DigitalOut led1(LED1);   //Device status LED

const static char     DEVICE_NAME[]        = "HRM1";        //This is the Adverstised Name of our Temp. Cube
static const uint16_t uuid16_list[]        = {GattService::UUID_HEART_RATE_SERVICE,
                                              GattService::UUID_DEVICE_INFORMATION_SERVICE};
static volatile bool  triggerSensorPolling = false;

uint16_t hrmCounter = 100;              //Variable for Temperature 
float hrm = 100;                        //Intermediate Variable for Temperature, we get the alue in float and convert into uint16_t to work with the format specified in the BLE GAP library
uint16_t hrmCounter1 = 0;               //Varuable for pressure

HeartRateService         *hrService,*hrService1;    //Name of the Service
DeviceInformationService *deviceInfo;               

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
    hrService = new HeartRateService(ble, hrmCounter, (uint8_t)hrmCounter1);
    //hrService1 = new HeartRateService(ble, hrmCounter1, HeartRateService::LOCATION_CHEST);
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

int main(void)
{
    led1 = 1;
    Ticker ticker;
    ticker.attach(periodicCallback, 1); // Blink LED every second and transmit data evry second

    BLE& ble = BLE::Instance(BLE::DEFAULT_INSTANCE);
    ble.init(bleInitComplete);

    /* SpinWait for initialization to complete. This is necessary because the
     * BLE object is used in the main loop below. */
    while (ble.hasInitialized()  == false) { /* spin loop */ }
    
    while(1) 
    {
        if (bmp180.init() != 0) {               //Initialize the BMP180. Set i2c communication and veryify if communication is up.
            printf("Error communicating with BMP180\n");
        } else {
            printf("Initialized BMP180\n");
            break;
        }
        wait(1);
    }
    
    // infinite loop
    while (1) {
        // check for trigger from periodicCallback()
        if (triggerSensorPolling && ble.getGapState().connected) {
            triggerSensorPolling = false;

        bmp180.startTemperature();  //Start Temp. measurement
        wait_ms(5);     // Wait for conversion to complete
        if(uint8_t(bmp180.getTemperature(&hrm)) != 0) {
            printf("Error getting temperature\n");
            continue;
        }
        hrmCounter1 = (uint16_t)hrm;            //pass the float value to hrmCounter1 to convert it into uint16_t
        hrService->updateHeartRate(hrmCounter1);    //Update the Temperature value to the service
        bmp180.startPressure(BMP180::ULTRA_LOW_POWER);  //Start pressure measurement in ultra low power.
        wait_ms(10);    // Wait for conversion to complete
        //int pressure;

        if((bmp180.getPressure(&hrmCounter)!= 0)) {
            printf("Error getting pressure\n");
            continue;
        }

        printf("Pressure = %d KPa Temperature = %f C\n", hrmCounter, hrm);
        wait(1);

            hrService->updateHeartRate(hrmCounter);     //Update the Pressure value to the Service

        } else {
            ble.waitForEvent(); // low power wait for event
        }
    }
}
