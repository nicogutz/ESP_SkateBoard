/****************************************************************
 * Example7_DMP_Quat6_EulerAngles.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Paul Clark, April 25th, 2021
 * Based on original code by:
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * ** This example is based on InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".
 * ** We are grateful to InvenSense for sharing this with us.
 *
 * ** Important note: by default the DMP functionality is disabled in the library
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 * ** If you are using Windows, you can find ICM_20948_C.h in:
 * ** Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/

// #define QUAT_ANIMATION // Uncomment this line to output data in the correct format for ZaneL's Node.js Quaternion animation tool: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

#include <WiFi.h>
#include <WiFiAP.h>

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#ifndef LED_BUILTIN
#define LED_BUILTIN 2  // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
#endif

WiFiServer server(80);

// Set these to your desired credentials.
const char *ssid = "NicoAP";
const char *password = "password";

#define USE_SPI  // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI  // Your desired SPI port.       Used only when "USE_SPI" is defined
#define ICM_CS 15     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define ICM_SCK 14
#define ICM_MISO 27
#define ICM_MOSI 13

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif

void setup() {
    SERIAL_PORT.begin(115200);  // Start the serial console
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    Serial.println();
    Serial.println("Configuring access point...");

    if (!WiFi.softAP(ssid, password)) {
        log_e("Soft AP creation failed.");
        while (1);
    }
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    server.begin();

    Serial.println("Server started");

    delay(100);

    SPI_PORT.begin(ICM_SCK, ICM_MISO, ICM_MOSI, ICM_CS);

    bool initialized = false;
    while (!initialized) {
        // Initialize the ICM-20948
        // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
        myICM.begin(ICM_CS, SPI_PORT);

        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok) {
            SERIAL_PORT.println(F("Trying again..."));
            delay(500);
        } else {
            initialized = true;
        }
    }

    SERIAL_PORT.println(F("Device connected!"));

    bool success = true;  // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);  // Set to the maximum
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success) {
        SERIAL_PORT.println(F("DMP enabled!"));
    } else {
        SERIAL_PORT.println(F("Enable DMP failed!"));
        SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1);  // Do nothing more
    }
}

void loop() {
    WiFiClient client = server.accept();  // listen for incoming clients
    if (client) {
        while (client.connected()) {
            icm_20948_DMP_data_t data;
            myICM.readDMPdataFromFIFO(&data);

            if (((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)))  // Was valid data available?
            {
                if ((data.header & DMP_header_bitmap_Quat6) > 0)  // We have asked for GRV data so we should receive Quat6
                {
                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                    double q2sqr = q2 * q2;

                    double t0 = +2.0 * (q0 * q1 + q2 * q3);
                    double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
                    double roll = atan2(t0, t1) * 180.0 / PI;

                    double t2 = +2.0 * (q0 * q2 - q3 * q1);
                    t2 = t2 > 1.0 ? 1.0 : t2;
                    t2 = t2 < -1.0 ? -1.0 : t2;
                    double pitch = asin(t2) * 180.0 / PI;

                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (q0 * q3 + q1 * q2);
                    double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
                    double yaw = atan2(t3, t4) * 180.0 / PI;

                    client.println((String)roll + "," + (String)pitch + "," + (String)yaw);
                }
            }

            if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)  // If more data is available then we should read it right away - and not delay
            {
                delay(10);
            }
        }
    }
    /* code */
}
