// -----------------------------------------

// Water Tank Monitor
//
// Version: 1.2
// Author: Markus Haack (http://github.com/mhaack)
// -----------------------------------------

#include "SparkFunMAX17043.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// power control and sensor GPIO
const unsigned int powerControl  = A0;
const unsigned int sensorTrigger = D2;
const unsigned int sensorEcho    = D6;
boolean powerState               = LOW;

// distance variables
const unsigned int MEASUREMENTS = 10;
unsigned int distance[MEASUREMENTS];               // distance in cm
retained unsigned int lastDistances[MEASUREMENTS]; // last n distance values stored in backup RAM

// publish buffers
struct Measurement {
  unsigned int distance;
  bool         regular;
  time_t       timestamp;
};
const unsigned int PUBLISH_BUFFER_SIZE     = 48;
retained unsigned int publish_buffer_count = 0;
retained Measurement  publish_buffer[PUBLISH_BUFFER_SIZE];

// measurement and publish interval
const unsigned int MEASUREMENT_INTERVAL       = 60 * 60;      // 1 hour for normal readings
const unsigned int MEASUREMENT_INTERVAL_SHORT = 60 * 5;       // 5 minutes for readings if values changing
const unsigned int PUBLISH_INTERVAL           = 4 * 60 * 60;  // 4 hour as regular publish interval

SerialLogHandler logHandler;

// particle.io publush helper
char publishString[128];

// shift publish buffer array and add new value
void shiftPublishBuffer(struct Measurement *arrayPtr,
                        unsigned int size,
                        unsigned int value, bool regular) {
  for (int i = size - 1; i > 0; i--) {
    arrayPtr[i] = arrayPtr[i - 1];
  }
  arrayPtr[0].distance  = value;
  arrayPtr[0].regular   = regular;
  arrayPtr[0].timestamp = Time.local();
}

// find the highest value in the measurement array
unsigned int arrayMax(unsigned int *arrayPtr, unsigned int size) {
  unsigned int max = arrayPtr[0];

  for (unsigned int i = 0; i < size; i++) {
    if (max < arrayPtr[i]) {
      max = arrayPtr[i];
    }
  }
  return max;
}

// find the lowest value in the measurement array
unsigned int arrayMin(unsigned int *arrayPtr, unsigned int size) {
  unsigned int min = arrayPtr[0];

  for (unsigned int i = 0; i < size; i++) {
    if (min > arrayPtr[i]) {
      min = arrayPtr[i];
    }
  }
  return min;
}

// shift array of historic distance values and add new one
void shiftLastDistances(unsigned int *arrayPtr,
                        unsigned int  size,
                        unsigned int  newValue) {
  for (int i = size - 1; i > 0; i--) {
    arrayPtr[i] = arrayPtr[i - 1];
  }
  arrayPtr[0] = newValue;
}

// calc if a force publish is needed to send updates on defined intervals
bool calcRegularMeasurement() {
  time_t now          = Time.local();
  time_t lastInterval = now - (now % 60);

  boolean regularMeasurement = false;
  if (lastInterval % (MEASUREMENT_INTERVAL) == 0) {
    regularMeasurement = true;
  }
  Log.info("[Sensor] Check for regular measurement: %s", regularMeasurement ? "Yes" : "No");
  return regularMeasurement;
}

// calc if a force publish is needed to send updates on defined intervals
bool calcRegularPublish() {
  time_t now          = Time.local();
  time_t lastInterval = now - (now % 60);

  boolean regularPublish = false;
  if (lastInterval % (PUBLISH_INTERVAL) == 0) {
    regularPublish = true;
  }
  Log.info("[Cloud] Check for regular publish: %s", regularPublish ? "Yes" : "No");
  return regularPublish;
}

// calc sleep time till next measurement
int sleepTime(bool regular) {
  time_t now            = Time.local();
  unsigned int interval = regular ? MEASUREMENT_INTERVAL : MEASUREMENT_INTERVAL_SHORT;
  time_t nextMeasurement = now - (now % interval) + interval;
  time_t nextSave        = now - (now % MEASUREMENT_INTERVAL) + MEASUREMENT_INTERVAL;
  time_t nextPublish     = now - (now % PUBLISH_INTERVAL) + PUBLISH_INTERVAL;

  Log.info("[System] Next measurement %s", Time.format(nextMeasurement, TIME_FORMAT_ISO8601_FULL).c_str());
  Log.info("[System] Next regular save %s", Time.format(nextSave, TIME_FORMAT_ISO8601_FULL).c_str());
  Log.info("[System] Next publish %s", Time.format(nextPublish, TIME_FORMAT_ISO8601_FULL).c_str());
  delay(10); // needed to give serial time to print before going into sleep
  return nextMeasurement - now;
}

// the setup
void setup() {
  Serial.begin(115200);
  delay(5000);
  Log.info("Water Tank Monitor");
  Log.info("[System] Version: %s", (const char*)System.version());

  // Set up power for HC-SR04 sensor
  pinMode(powerControl, OUTPUT);
  digitalWrite(powerControl, powerState);

  // Set up the MAX17043 LiPo fuel gauge
  lipo.begin();
  lipo.wake();
  lipo.quickStart();
  lipo.setThreshold(10);
  Log.info("[System] Battery: %.2f V, %.1f %%", lipo.getVoltage(), lipo.getSOC());

  // inital time sync if real time is not initalized
  Time.zone(1);
  if (!Time.isValid()) {
    Log.info("[System] Time sync ...");
    waitUntil(Particle.connected);
    waitFor(Time.isValid, 60000);
    Log.info("[System] Time sync done");
  } else {
    Log.info("[System] Time is valid");
  }
  Log.info("[System] Current time is: %s", Time.format(Time.local(), TIME_FORMAT_ISO8601_FULL).c_str());

}

// the main loop
void loop() {
  // step 1: measurement
  powerState = !powerState;
  Log.info("[Sensor] Start mesuarement, senor power: %s", powerState ? "on" : "off");
  digitalWrite(powerControl, powerState);
  delay(250);

  for (unsigned int i = 0; i < MEASUREMENTS; i++) {
    ping(sensorTrigger, sensorEcho, i);
    delay(100);
    Log.info("[Sensor] Measurement %d: %lu", i, distance[i]);
  }
  powerState = !powerState;
  digitalWrite(powerControl, powerState);
  Log.info("[Sensor] Mesuarement done, sensor power: %s", powerState ? "on" : "off");

  // step 2: compare current measurement with historic values
  unsigned int current = arrayMax(distance, MEASUREMENTS);
  unsigned int lastMax = arrayMax(lastDistances, MEASUREMENTS);
  unsigned int lastMin = arrayMin(lastDistances, MEASUREMENTS);
  bool inRange         = current >= lastMin && current <= lastMax;
  Log.info("[Sensor] current: %lu, LMin: %lu, LMax: %lu, inRange: %s",
                  current, lastMin, lastMax, inRange ? "Yes" : "No");

  // put current value into distantce history store
  shiftLastDistances(lastDistances, MEASUREMENTS, current);

  // step 3: update publish buffer
  bool regularMeasurement = calcRegularMeasurement();

  if (regularMeasurement || !inRange) {
    Log.info("[Memory] Store measurement - regular: %s, inRange: %s",
        regularMeasurement ? "Yes" : "No", inRange ? "Yes" : "No");
    shiftPublishBuffer(publish_buffer, PUBLISH_BUFFER_SIZE,
                       current, regularMeasurement);
    publish_buffer_count++;
  }

  // step 4: publish data to particle.io cloud
  bool regularPublish = calcRegularPublish();
  Log.info("[Cloud] Check for publish buffer size %lu of %lu",
                  publish_buffer_count, PUBLISH_BUFFER_SIZE);

  if (regularPublish || (publish_buffer_count >= PUBLISH_BUFFER_SIZE)) {
    Particle.connect();

    if (waitFor(Particle.connected, 30000)) {
      Log.info("[Cloud] Send measurement buffer - regular: %s, buffer size: %lu",
        regularPublish ? "Yes" : "No", publish_buffer_count);

      // publish buffer data
      for (unsigned int i = 0; i < publish_buffer_count; i++) {
        sprintf(publishString,
                "{\"cm\": %u, \"regular\": %d, \"createdAt\": %lu000}",
                publish_buffer[i].distance,
                publish_buffer[i].regular ? 1 : 0,
                publish_buffer[i].timestamp);
        Log.info("[Cloud] Send measurement %d: %s", i, publishString);
        Particle.publish("water-sensor", publishString, PRIVATE);
        delay(1000);
      }

      // empty publish buffer
      memset(publish_buffer, 0, sizeof(publish_buffer));
      publish_buffer_count = 0;

      // publish some system condition data
      sprintf(publishString,
              "{\"wifi\": %d, \"v\": %.2f, \"soc\": %.2f, \"alert\": %d}",
              WiFi.RSSI(), lipo.getVoltage(), lipo.getSOC(), lipo.getAlert());
      boolean publishSuccess = Particle.publish("tech-water-sensor", publishString, PRIVATE);
      Log.info("[Cloud] Successfull: %s", publishSuccess ? "Yes" : "No");
    } else {
        // if the buffer still has some space left we can go ahead and try later
        Log.error("[Cloud] Failed, connect timeout.");
        if (publish_buffer_count >= PUBLISH_BUFFER_SIZE) {
            Log.info("[Cloud] buffer is full retry. Will lose old measurements now");
        }
    }
  }

  // step 5: repare sleep
  lipo.sleep();
  System.sleep(SLEEP_MODE_DEEP, sleepTime(inRange));
}

void ping(pin_t trig_pin, pin_t echo_pin, int i) {
  unsigned int duration, cm;
  static bool  init = false;

  if (!init) {
    pinMode(trig_pin, OUTPUT);
    digitalWriteFast(trig_pin, LOW);
    pinMode(echo_pin, INPUT);
    delay(50);
    init = true;
  }

  // Trigger the sensor by sending a HIGH pulse of 10 or more microseconds
  digitalWriteFast(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWriteFast(trig_pin, LOW);

  duration = pulseIn(echo_pin, HIGH);

  // Convert the time into a distance
  // Sound travels at 1130 ft/s (73.746 us/inch)
  // or 340 m/s (29 us/cm), out and back so divide by 2
  // Ref: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  cm          = duration / 29 / 2;
  distance[i] = cm;

  delay(100);
}
