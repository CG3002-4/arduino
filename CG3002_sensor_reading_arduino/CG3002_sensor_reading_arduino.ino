#include "LowPower.h"
#include "Wire.h"


// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h
//files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyroIC1(0x68);
MPU6050 accelgyroIC2(0x69);

#define AD0_PIN_0 4  // Connect this pin to the AD0 pin on IMU #0
#define AD0_PIN_1 5  // Connect this pin to the AD0 pin on IMU #1
#define LED_PIN 13

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;

int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;

int analogPin0 = 0;     // potentiometer wiper (middle terminal) connected to analog pin 0
                       // outside leads to ground and +5V
int analogPin1 = 1;     // potentiometer wiper (middle terminal) connected to analog pin 1
                       // outside leads to ground and +5V
float val = 0;           // variable to store the value read
float cur = 0;           // variable to store the value read
float voltage =0;
float current=0;
float power=0;
float energy=0;
float totalenergy=0;

float currenttime;
float pasttime;

bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz,
    //but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    //accelgyro.initialize();
    accelgyroIC1.initialize();
    accelgyroIC2.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyroIC1.testConnection() ? "MPU6050 #1 connection successful" : "MPU6050 connection failed");
    Serial.println(accelgyroIC2.testConnection() ? "MPU6050 #2 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(13, OUTPUT);
    pinMode(A0,INPUT);
    pinMode(A1,INPUT);
}

void loop() {
    pasttime=millis();

    //LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);

    // read raw accel/gyro measurements from device
    accelgyroIC1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    accelgyroIC2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    currenttime=millis();

    // display tab-separated accel/gyro x/y/z values
    Serial.println("MPU1:\t");
    Serial.print("accel (g):");
    Serial.print(ax1 / 16384.0, 2);Serial.print("\t");
    Serial.print(ay1 / 16384.0, 2);Serial.print("\t");
    Serial.print(az1 / 16384.0, 2);Serial.print("\t");
    Serial.println();
    Serial.print("gyro (deg):");
    Serial.print(gx1 / 131.0, 2); Serial.print("\t");
    Serial.print(gy1 / 131.0, 2); Serial.print("\t");
    Serial.print(gz1 / 131.0, 2); Serial.print("\t");

    Serial.println();
    // display tab-separated accel/gyro x/y/z values
    Serial.println("MPU2:\t");
    Serial.print("accel (g):");
    Serial.print(ax2 / 16384.0, 2); Serial.print("\t");
    Serial.print(ay2 / 16384.0, 2); Serial.print("\t");
    Serial.print(az2 / 16384.0, 2); Serial.print("\t");
    Serial.println();
    Serial.print("gyro (deg):");
    Serial.print(gx2 / 131.0, 2); Serial.print("\t");
    Serial.print(gy2 / 131.0, 2); Serial.print("\t");
    Serial.print(gz2 / 131.0, 2); Serial.println("\t");


    val = analogRead(A0) * 5.0 / 1023.0;     // read the input pin0 for voltage divider
    cur = analogRead(A1) * 5.0 / 1023.0; // read the input pin1 for current sensor

    voltage=(val * 2.0);
    Serial.print(voltage * 1.0, 2);  Serial.println("V");
    //Serial.print(cur * 1.0, 2);  Serial.println("V");
    current=((cur * 1000.0) / (0.09 * 10000.0));
    Serial.print(current * 1000.0, 2); Serial.println("mA");
    power=(voltage * current);
    Serial.print(power * 1000.0, 2);Serial.println("mW");
    energy=(power * (currenttime - pasttime));
    Serial.print(energy * 1.0, 2);Serial.println("J");
    totalenergy=(totalenergy + energy);
    Serial.print(totalenergy * 1.0, 2);Serial.println("J");

    delay(500);
}
