#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Servo.h>
#include <analogWrite.h>

#define GPS_RX 4
#define GPS_TX 3
#define MOTOR_PIN1 6
#define MOTOR_PIN2 7
#define STEERING_PIN 9

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
Adafruit_GPS gps(&gpsSerial);
Servo servo;
float currentLat, currentLon, targetLat, targetLon, targetBearing, currentBearing, error ;
int angle;

// Function to convert GPS coordinates to direction angle
float getDirection(float lat1, float lon1, float lat2, float lon2) {
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float y = sin(dLon) * cos(radians(lat2));
  float x = cos(radians(lat1)) * sin(radians(lat2)) - sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
  float bearing = degrees(atan2(y, x));
  if (bearing < 0) {
    bearing += 360;
  }
  return bearing;
}

// Function to control the motor
void driveMotor(float speed) {
  if (speed >= 0) {
    analogWrite(MOTOR_PIN1, speed);
    analogWrite(MOTOR_PIN2, 0);
  } else {
    analogWrite(MOTOR_PIN1, 0);
    analogWrite(MOTOR_PIN2, -speed);
  }
}

// Function to update the target location and bearing
void updateTarget() {
  // Replace with your own algorithm for updating the target location and bearing
  targetLat = 37.7749; // Replace with the latitude of the target location
  targetLon = -122.4194; // Replace with the longitude of the target location
  targetBearing = getDirection(currentLat, currentLon, targetLat, targetLon);
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gps.begin(9600);
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  servo.attach(STEERING_PIN);
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
}

void loop() {
  // Read GPS data
  gps.read();
  if (gps.newNMEAreceived()) {
    if (!gps.parse(gps.lastNMEA())) {
      return;
    }
    if (gps.fix) {
      currentLat = gps.latitudeDegrees;
      currentLon = gps.longitudeDegrees;
      currentBearing = gps.angle;
      // Update the target location and bearing
      updateTarget();
      // Calculate the steering angle
      float error = targetBearing - currentBearing;
      if (error > 180) {
        error -= 360;
      } else if (error < -180) {
        error += 360;
      }
    }
  }
      int angle = map(error, -180, 180, 0, 180);
      servo.write(angle);
      // Drive the motor
      float speed = 100; // Replace with your own algorithm for controlling the speed
      driveMotor(speed);
      Serial.print("Lat: ");
      Serial.print(currentLat);
      Serial.print(" Lon: ");
      Serial.print(currentLon);
      Serial.print(" Bearing: ");
      Serial.print(currentBearing);
}
    
