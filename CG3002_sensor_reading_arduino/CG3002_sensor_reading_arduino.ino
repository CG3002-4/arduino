//#include <Arduino_FreeRTOS.h>
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
#define DATA_SIZE 32

int16_t ax1, ay1, az1;
int16_t gx1, gy1, gz1;

int16_t ax2, ay2, az2;
int16_t gx2, gy2, gz2;

float voltage, current, power, energy, totalenergy = 0;

float currenttime, pasttime;

struct dataPacket {
    int16_t ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2;

    int16_t voltage, current, power, energy;
} dataPKT;

char serialized[35] = {0};

void readData();
void updateDataPacket();
char* serializeInt16(int16_t val, char* buf);
char* serializeByte(byte val, char* buf);
void serializeDataPacket();
void getXORChecksum();
void sendDataPacket();

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz,
    //but
    // it's really up to you depending on your project)
    // Serial.begin(38400);
    Serial1.begin(115200);

    // initialize device
    // Serial1.println("Initializing I2C devices...");
    //accelgyro.initialize();
    accelgyroIC1.initialize();
    accelgyroIC2.initialize();

    // verify connection
    // Serial1.println("Testing device connections...");
    // Serial1.println(accelgyroIC1.testConnection() ? "MPU6050 #1 connection successful" : "MPU6050 connection failed");
    // Serial1.println(accelgyroIC2.testConnection() ? "MPU6050 #2 connection successful" : "MPU6050 connection failed");

    // Configure analog pins as input
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
}

void loop() {
    //LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
    readData();
    updateDataPacket();
    sendDataPacket();
    delay(500);
}

void readData() {
    pasttime = currenttime;
    currenttime = millis();

    // read raw accel/gyro measurements from device
    accelgyroIC1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    accelgyroIC2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);

    // All SI units
    voltage = analogRead(A0) * 5.0 / 1023.0; // read the input pin0 for voltage divider
    voltage = 2.0 * voltage; // Account for halving of voltage by voltage divider

    current = analogRead(A1) * 5.0 / 1023.0; // read the input pin1 for current sensor
    current = (current * 10.0) / 9.0;

    power = voltage * current;

    // Might not need the energy variable at all...
    energy = power * (currenttime - pasttime);
    totalenergy = totalenergy + energy;
}

void updateDataPacket() {
    dataPKT.ax1 = (ax1 / 16384.0) * 100;
    dataPKT.ay1 = (ay1 / 16384.0) * 100;
    dataPKT.az1 = (az1 / 16384.0) * 100;
    dataPKT.gx1 = (gx1 / 131.0) * 100;
    dataPKT.gy1 = (gy1 / 131.0) * 100;
    dataPKT.gz1 = (gz1 / 131.0) * 100;

    dataPKT.ax2 = (ax2 / 16384.0) * 100;
    dataPKT.ay2 = (ay2 / 16384.0) * 100;
    dataPKT.az2 = (az2 / 16384.0) * 100;
    dataPKT.gx2 = (gx2 / 131.0) * 100;
    dataPKT.gy2 = (gy2 / 131.0) * 100;
    dataPKT.gz2 = (gz2 / 131.0) * 100;

    dataPKT.voltage = voltage * 100;
    dataPKT.current = current * 100;
    dataPKT.power = power * 100;
    dataPKT.energy = totalenergy * 100;
}

// Returns pointer to next empty position in buf
char* serializeInt16(int16_t val, char* buf) {
    buf[0] = (val >> 8) & 255;
    buf[1] = val & 255;
    return buf + 2;
}

char* serializeByte(byte val, char* buf) {
    buf[0] = val & 255;
    return buf + 1;
}

byte getXORCheckSum() {
    byte checksum = 0;

    for(int i = 0; i < DATA_SIZE; i++){
        checksum = (serialized[i] ^ checksum);
    }

    return checksum;
}

void serializeDataPacket() {
    char* buf = serialized;
    buf = serializeInt16(dataPKT.ax1, buf);
    buf = serializeInt16(dataPKT.ay1, buf);
    buf = serializeInt16(dataPKT.az1, buf);
    buf = serializeInt16(dataPKT.gx1, buf);
    buf = serializeInt16(dataPKT.gy1, buf);
    buf = serializeInt16(dataPKT.gz1, buf);

    buf = serializeInt16(dataPKT.ax2, buf);
    buf = serializeInt16(dataPKT.ay2, buf);
    buf = serializeInt16(dataPKT.az2, buf);
    buf = serializeInt16(dataPKT.gx2, buf);
    buf = serializeInt16(dataPKT.gy2, buf);
    buf = serializeInt16(dataPKT.gz2, buf);

    buf = serializeInt16(dataPKT.voltage, buf);
    buf = serializeInt16(dataPKT.current, buf);
    buf = serializeInt16(dataPKT.power, buf);
    buf = serializeInt16(dataPKT.energy, buf);

    buf = serializeByte(getXORCheckSum(), buf);
}

void sendDataPacket() {
    serializeDataPacket();
    Serial1.write(serialized, DATA_SIZE + 1);   // send the data
}
