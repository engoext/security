/* 
hightech_security: Code for an Arduino-based, "high-tech" security system.
The code runs on an Arduino Uno connected to an ultrasonic range sensor, LEDs and a buzzer.
Passing through the security "beam" (ultrasonic range sensor) triggers an alarm!
*/

#define LEDGREENPIN 3
#define LEDREDPIN 4
#define BUZZERPIN 11
#define TRIGGERPIN 10
#define ECHOPIN 9

int alarm_activated = 0; // has the alarm been triggered yet?
int door_width; // the normal distance that the guarding "beam" covers

// Function used to measure distance using ultrasonic range sensor
int measure_distance_ultrasonic() {

  // Send an output pulse to the sonar sensor (1ms wide)
  digitalWrite(TRIGGERPIN, HIGH);
  delay(1); // wait 1ms
  digitalWrite(TRIGGERPIN, LOW);

  // Measure the time taken to receive the pulse back // Will be "0" or "-1" if no pulse is received
  int duration = pulseIn(ECHOPIN, HIGH);

  // Formula for the distance is half the duration, then divide by 2.910
  // to get distance in mm
  int distance = (duration/2) / 2.91;
  return distance;

}

// Function to activate the alarm!
void sound_alarm() {
  digitalWrite(BUZZERPIN, HIGH);
  digitalWrite(LEDREDPIN, HIGH);
  delay(500);
  digitalWrite(BUZZERPIN, LOW);
  digitalWrite(LEDREDPIN, LOW);
  delay(500);
  digitalWrite(BUZZERPIN, HIGH);
  digitalWrite(LEDREDPIN, HIGH);
  delay(500);
  digitalWrite(BUZZERPIN, LOW);
  digitalWrite(LEDREDPIN, LOW);
  delay(500);
  digitalWrite(BUZZERPIN, HIGH);
  digitalWrite(LEDREDPIN, HIGH);
  delay(500);
  digitalWrite(BUZZERPIN, LOW);
}

// This function is called once at the start of the program
void setup() {
  Serial.begin(115200);

  pinMode(LEDGREENPIN, OUTPUT); // Green LED can be switched on/off from this pin
  pinMode(LEDREDPIN, OUTPUT); // Red LED can be switched on/off from this pin
  pinMode(BUZZERPIN, OUTPUT); // Active Buzzer can be switched on/off from this pin
  digitalWrite(BUZZERPIN, LOW); // Turn it off to begin with!

  pinMode(TRIGGERPIN, OUTPUT); // Sonar pulses will be sent out of the Arduino
  pinMode(ECHOPIN, INPUT); // Echo pulses will be received into the Arduino

  delay(50);
  door_width = measure_distance_ultrasonic(); // measure initial doorway width

  Serial.print("door width (mm):\n"); 
  Serial.print(door_width);
  Serial.print("\n"); 

  pinMode(13, OUTPUT); // switch off on-board LED for now
  digitalWrite(13, LOW);
  
  delay(50); // let the sensor stabilise
}

// This function is called repeatedly by the program
void loop() {

  if (alarm_activated == 0) { // not actived yet
    digitalWrite(LEDGREENPIN, HIGH); // green light on
    digitalWrite(LEDREDPIN, LOW); // red light off

    int distance = measure_distance_ultrasonic(); // measure current doorway

    if (distance < 0.8*door_width) { // something is passing the door! (0.8 to avoid false alarms)
      digitalWrite(LEDGREENPIN, LOW);
      sound_alarm(); // Sound the alarm!
      alarm_activated = 1; // we only want to sound the alarm once, so let's switch off future checks
    }

  }

  delay(50); 

}
