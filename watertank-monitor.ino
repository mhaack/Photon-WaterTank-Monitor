#include "application.h"
#include "SparkFunMAX17043.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

const byte mode = 1;

// power control and sensor GPIO
const unsigned int powerControl = A0;
const unsigned int sensorTrigger = D2;
const unsigned int sensorEcho = D6;
boolean powerState = LOW;

// distance variables
const unsigned int MEASUREMENTS = 10;
unsigned int distance[MEASUREMENTS]; // distance in cm
retained unsigned int lastDistances[MEASUREMENTS]; // last n distance values stored in backup RAM

// measurement and publish interval
const unsigned int MEASUREMENT_INTERVAL = 60 * 15; // 15 minutes for normal readings
const unsigned int PUBLISH_INTERVAL = 60 * 60; // 1 hour as regular publish interval
const unsigned int TIMESYNC_INTERVAL = 24 * 60 * 60; // 1 day to update cloud time

char publishString[128];

void setup() {
        Serial.begin(115200);
        Serial.println("Water Tank Monitor");

        // Set up power for HC-SR04 sensor
        pinMode(powerControl, OUTPUT);
        digitalWrite(powerControl, powerState);

        // Set up the MAX17043 LiPo fuel gauge
        lipo.begin();
        lipo.wake();
        lipo.quickStart();
        lipo.setThreshold(10);
}

void loop() {
        Serial.println("Measurement ...");
        powerState = !powerState;
        Serial.printlnf("Sensor power: %s", powerState ? "on" : "off");
        digitalWrite(powerControl, powerState);
        delay(250);

        for(int i = 0; i < MEASUREMENTS; i++) {
                ping(sensorTrigger, sensorEcho, 100, i);
                Serial.printlnf("Measurement %d: %lu", i, distance[i]);
        }
        powerState = !powerState;
        Serial.printlnf("Sensor power: %s", powerState ? "on" : "off");
        digitalWrite(powerControl, powerState);
        Serial.println("Measurement done");

        // compare current measurement with historic values
        unsigned int current = arrayMax(distance, MEASUREMENTS);
        unsigned int lastMax = arrayMax(lastDistances, MEASUREMENTS);
        unsigned int lastMin = arrayMin(lastDistances, MEASUREMENTS);
        bool inRange = current >= lastMin && current <= lastMax;
        Serial.printlnf("Current: %lu, LMin: %lu, LMax: %lu, inRange: %s", current, lastMin, lastMax, inRange ? "y" : "n");

        // put current value into distantce history store
        shiftLastDistances(current);

        // publish results
        bool force = forcePublish();
        if (force || !inRange) {
                Serial.println("Publish results to cloud");
                Particle.connect();
                waitUntil(Particle.connected);

                sprintf(publishString,"{\"cm\": %lu, \"wifi\": %d, \"v\": %.2f, \"soc\": %.2f, \"alert\": %d, \"fp\": %d}",
                        current, WiFi.RSSI(), lipo.getVoltage(), lipo.getSOC(), lipo.getAlert(), force ? 1 : 0);
                Serial.println(publishString);
                Particle.publish("water-sensor", publishString, PRIVATE);
                Particle.process();
                delay(1000);

                timeSync();
        }

        if (mode == 2) {
                WiFi.off();
                delay(sleepTime() * 1000);
        } else {
                lipo.sleep();
                System.sleep(SLEEP_MODE_DEEP,sleepTime());
        }
}

void ping(pin_t trig_pin, pin_t echo_pin, uint32_t wait, int i) {
        unsigned int duration, cm;
        static bool init = false;
        if (!init) {
                pinMode(trig_pin, OUTPUT);
                digitalWriteFast(trig_pin, LOW);
                pinMode(echo_pin, INPUT);
                delay(50);
                init = true;
        }

        /* Trigger the sensor by sending a HIGH pulse of 10 or more microseconds */
        digitalWriteFast(trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWriteFast(trig_pin, LOW);

        duration = pulseIn(echo_pin, HIGH);

        /* Convert the time into a distance */
        // Sound travels at 1130 ft/s (73.746 us/inch)
        // or 340 m/s (29 us/cm), out and back so divide by 2
        // Ref: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
        cm = duration / 29 / 2;
        distance[i] = cm;

        delay(wait);
}

// find the highest value in the measurement array
unsigned int arrayMax(unsigned int *arrayPtr, unsigned int size) {
        unsigned int max = arrayPtr[0];
        for(int i = 0; i < size; i++) {
                if(max < arrayPtr[i]) {
                        max = arrayPtr[i];
                }
        }
        return max;
}

// find the lowest value in the measurement array
unsigned int arrayMin(unsigned int *arrayPtr, unsigned int size) {
        unsigned int min = arrayPtr[0];
        for(int i = 0; i < size; i++) {
                if(min > arrayPtr[i]) {
                        min = arrayPtr[i];
                }
        }
        return min;
}

// shift array of historic distance values and add new one
void shiftLastDistances(unsigned int newDistance) {
        for(int i = MEASUREMENTS - 1; i > 0; i--) {
                lastDistances[i] = lastDistances[i - 1];
        }
        lastDistances[0] = newDistance;
}

// sync RTC time with cloud
void timeSync() {
        time_t now = Time.now();
        time_t lastInterval = now - (now % TIMESYNC_INTERVAL);
        if (now - lastInterval < 10) {
                Serial.println("force time sync");
                Particle.syncTime();
                for(int i = 0; i < 10; i++) {
                        Particle.process();
                        delay(500);
                }
                Particle.publish("sensor-time", "time sync done");
        }
}

// calc sleep time till next measurement
int sleepTime() {
        time_t now = Time.now();
        time_t nextInterval = now - (now % MEASUREMENT_INTERVAL) + MEASUREMENT_INTERVAL;
        Serial.printf("now %s", asctime(gmtime(&now)));
        Serial.printf("next measurement %s", asctime(gmtime(&nextInterval)));
        delay(10); // needed to give serial time to print before going into sleep
        return nextInterval - now;
}

// calc if a force publish is needed to send updates on defined intervals
bool forcePublish() {
        time_t now = Time.now();
        time_t lastInterval = now - (now % MEASUREMENT_INTERVAL);
        if (lastInterval % (PUBLISH_INTERVAL) == 0) {
                Serial.println("force publish");
                return true;
        }
        return false;
}
