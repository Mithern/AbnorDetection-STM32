/* =============
Copyright (c) 2024, STMicroelectronics

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
  products derived from this software without specific prior written permission.

*THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER / OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*
*/

/* If you want to use NEAI functions please, include NEAI library
 * in your Arduino libraries then, uncomment NEAI parts in the following code
 */

/* Libraries part */
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include "NanoEdgeAI.h"
//#include "knowledge.h"

/* Macros definitions */
#define SERIAL_BAUD_RATE  115200

/* Default address is 0x18 but, if SDO is powered at 3v3,
 *  address is set to 0x19, so you need to change it
 *  depending on your current hardware configuration.
 */
#define SENSOR_I2C_ADDR 0x19

/* Sensor data rates.
 * You can choose from:
 * LIS3DH_DATARATE_1_HZ
 * LIS3DH_DATARATE_10_HZ
 * LIS3DH_DATARATE_25_HZ
 * LIS3DH_DATARATE_50_HZ
 * LIS3DH_DATARATE_100_HZ
 * LIS3DH_DATARATE_200_HZ
 * LIS3DH_DATARATE_400_HZ
 * LIS3DH_DATARATE_LOWPOWER_1K6HZ
 * LIS3DH_DATARATE_LOWPOWER_5KHZ
 */
#define SENSOR_DATA_RATE	LIS3DH_DATARATE_100_HZ

/* Sensor ranges.
 * You can choose from:
 * LIS3DH_RANGE_16_G
 * LIS3DH_RANGE_8_G
 * LIS3DH_RANGE_4_G
 * LIS3DH_RANGE_2_G
 */
#define SENSOR_RANGE	LIS3DH_RANGE_4_G

/* NanoEdgeAI defines part
 * NEAI_MODE = 1: NanoEdgeAI functions = AI Mode.
 * NEAI_MODE = 0: Datalogging mode.
 */
#define NEAI_MODE 1
#define SENSOR_SAMPLES	16
#define AXIS  3

bool inversevalue = true;

/* In this example, we use I2C connection */
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

/* Global variables definitions */
static uint16_t neai_ptr = 0;
static float neai_buffer[SENSOR_SAMPLES * AXIS] = {0.0};

/* NEAI library variables */
 static uint8_t neai_code = 0, similarity = 0;
 static uint16_t neai_cnt = 0;

/* Initialization function: In this function,
 *  code runs only once at boot / reset.
 */
void setup() {
  /* Init serial at baud rate 115200 */
  Serial.begin(SERIAL_BAUD_RATE);

  /* I2C workaround: Sometimes, on some boards,
   * I2C get stuck after software reboot, reset so,
   * to avoid this, we toggle I2C clock pin at boot.
   */
  pinMode(SCL, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < 20; i++) {
    digitalWrite(SCL, !digitalRead(SCL));
    delay(1);
  }
  delay(100);

  /* Init I2C connection between board & sensor */
  if(!lis.begin(SENSOR_I2C_ADDR)) {
    Serial.print("Can't initialize I2C comm with LIS3DH sensor...\n");
    while(1);
  }

  /* Init LIS3DH with desired settings: odr & range */
  lis.setRange(SENSOR_RANGE);
  lis.setDataRate(SENSOR_DATA_RATE);

  /* Initialize NanoEdgeAI AI */
  // neai_code = neai_anomalydetection_init();
  // if(neai_code != NEAI_OK) {
  //   Serial.print("Not supported board.\n");
  // }
}

/* Main function: Code run indefinitely */
void loop() {
  /* Get data in the neai buffer */
  while(neai_ptr < SENSOR_SAMPLES) {
     /* Check if new data if available */
    if(lis.haveNewData()) {
      /* If new data is available we read it ! */
      lis.read();
      /* Fill neai buffer with new accel data */
      neai_buffer[AXIS * neai_ptr] = (float) lis.x;
      neai_buffer[(AXIS * neai_ptr) + 1] = (float) lis.y;
      neai_buffer[(AXIS * neai_ptr) + 2] = (float) lis.z;
      /* Increment neai pointer */
      neai_ptr++;
    }
  }
  /* Reset pointer */
  neai_ptr = 0;

  /* Depending on NEAI_MODE value, run NanoEdge AI functions
   * or print accelerometer data to the serial (datalogging)
   */
   //int A =  MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING+70;
   if(NEAI_MODE) {
     if(neai_cnt <MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING)  {
       neai_anomalydetection_learn(neai_buffer);
       //Serial.print((String)"Learn: " + neai_cnt + "/" + MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING + ".\n");
       digitalWrite(LED_BUILTIN, inversevalue);
       inversevalue=!inversevalue;
       neai_cnt++;
       if(neai_cnt==MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING) {
          Serial.print((String)"OK, AI ready.\n");
       }
       delay(150);
     }
     else {
       neai_anomalydetection_detect(neai_buffer, &similarity);
       //Serial.print((String)"Detect: " + similarity + "/100.\n");
       Serial.print((String)similarity + "\n");
     }
   }
   else {
    /* Print the whole buffer to the serial */
    for(uint16_t i = 0; i < AXIS * SENSOR_SAMPLES; i++) {
      Serial.print((String)neai_buffer[i] + " ");
    }
    Serial.print("\n");
   }

  /* Clean neai buffer */
  memset(neai_buffer, 0.0, AXIS * SENSOR_SAMPLES * sizeof(float));
}
