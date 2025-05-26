/****
Required Arduino hardware packages:
  https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
Required Libraries:
  Arduino ESP32Servo.h
****/

#include <ESP32Servo.h>

#define NUM_SERVOS 6
#define MIN_SERVO_POS 10
#define MID_SERVO_POS 90
#define MAX_SERVO_POS 170
#define TIMESTAMP_PERIOD 500
#define NUM_PLATFORM_STATES 24

typedef struct  {
  uint8_t pin;
  int8_t trim;
  uint8_t last;
  uint8_t next;
} SERVO_DATA;

bool calibrate = false;
uint32_t timestamp = 0;
uint32_t timenow = 0;
uint8_t platform_state = 0;

// Create parameter Solidworks simulation
float platform_offset = 19.46;

Servo servos[NUM_SERVOS];

SERVO_DATA sdat[NUM_SERVOS] { 
  {12, 5, 90, 90},
  {27, -3, 90, 90},
  {33, -6, 90, 90},
  {15, -2, 90, 90},
  {32, -9, 90, 90},
  {14, 0, 90, 90} };

// Create 24x6 array of DOF platform positions from Solidworks simulation (heave, surge, sway, yaw, pitch, roll)
float platformPositions[NUM_PLATFORM_STATES][NUM_SERVOS] {
  {115.36, 64.64, 115.36, 64.64, 115.36, 64.64},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},
  {137.25,  42.75, 137.25, 42.75, 137.25, 42.75},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},

  {114.57, 65.43, 40.52, 89.79, 90.21, 139.48},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},
  {51.41, 128.59, 134.52, 109.16, 70.84, 45.48},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},

  {108.77, 129.66, 93.24, 52.49, 40.74, 109.11},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},
  {50.57, 72.28, 70.67, 138.81, 125.39, 87.64},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},

  {128.28, 140.74, 128.28, 140.74, 128.28, 140.74},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},
  {39.4, 52.44, 39.4, 52.44, 39.4, 52.44},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},

  {100.02, 79.98, 82.31, 153.6, 26.4, 97.69},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},
  {36.37, 143.63, 53.91, 42.82, 137.18, 126.09},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},

  {102.12, 143.95, 27.08, 101.53, 80.22, 63.03},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46},
  {36.41, 78.18, 116.39, 99.87, 62.67, 152.46},
  {70.54, 109.46, 70.54, 109.46,70.54, 109.46}
};

// Process KVP (KEY=VAL)
String procKVP(String skvp) {
  String sret;
	bool rv = true;

	if (skvp.length()) {
    skvp.toUpperCase();

		int16_t z = skvp.indexOf("=");
    if (z == -1) 
      z = skvp.length();
		String skey = skvp.substring(0, z);
		String sval = skvp.substring(z + 1);
		uint32_t ival = sval.toInt();
		float fval = sval.toFloat();

		//****************************//
		//***** sval length == 0 *****//
		//****************************//
		if (sval.length() == 0) {
      // Set calibration mode
			if (skey == "C") 
        sval = String(calibrate);

      // print trim all servos
      else if (skey == "T") {
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
          sval += " ";
          sval += String(sdat[i].trim);
        }
      }

      // Print trim specific servo
      else if (skey[0] == 'T') {
        int8_t i = skey[1] - 48;
        if ((i  < 0) || (i >= NUM_SERVOS)) 
          rv = false;
        else 
          sval = String(sdat[i].trim);
      }

      // Print offset all servos
      else if (skey == "O") 
        sval  = String(platform_offset);

      // Else error
			else 
        rv = false;
		}

		//*********************************//
		//***** else sval length != 0 *****//
		//*********************************//

    // Set calibration state
    else if (skey == "C") 
      calibrate = ival;

    // Set trim all servos
    else if (skey == "T")
      for (uint8_t i = 0; i < NUM_SERVOS; i++) 
        sdat[i].trim = ival;

    // Set trim specific servo
    else if (skey[0] == 'T') {
      int8_t i = skey[1] - 48;
      if ((i  < 0) || (i >= NUM_SERVOS)) 
        rv = false;
      else 
        sdat[i].trim = ival;
    }

    // Set offset all servos
    else if (skey == "O") 
      platform_offset = ival;

    // Else error
		else 
      rv = false;

    // If command OK
		if (rv == true) {
			if (sval.length()) 
        sret = skey + "=" + sval + " OK";
			else 
        sret = skey + " OK";
    }

    // Else command ER
    else 
      sret = skvp + " ER";

    Serial.println(sret);
	}

	return(sret);
}

// Read KVP
String readKVP(void) {
	bool rv = false;
	static uint8_t x = 0;
	static String sstr = "";

	if (x == 0) 
    sstr = "";

	while (Serial.available()) {
		char c = (char)Serial.read();
		if ((c == '\r') || (c == '\n')) {
			x = 0;
			rv = true;
			break;
		}

		x++;
		sstr += c;
	}

	return(rv ? sstr : String(""));
}

void setup() {
  Serial.begin(115200);

  for (int8_t i = 0; i < NUM_SERVOS; i++) {
      servos[i].attach(sdat[i].pin);
      servos[i].write(MID_SERVO_POS);
  }

  Serial.println("setup() complete");
}

void loop() {
  timenow = millis();
  procKVP(readKVP());

  // If calibrating servo trim
  if (calibrate) {
    for (int8_t i = 0; i < NUM_SERVOS; i++)
      // Set servo to 90 degrees plus trim, use contrain() to limit range
      servos[i].write(constrain(MID_SERVO_POS + sdat[i].trim, MIN_SERVO_POS, MAX_SERVO_POS));
    return;
  }

  // Calculate ratio of timenow to timestamp relative 500ms period
  float ratio = float(timenow - timestamp) / TIMESTAMP_PERIOD;

  for (int8_t i = 0; i < NUM_SERVOS; i++) {
    // Calculate lerped position between last and next using ratio
    float lerpPos = sdat[i].last + (sdat[i].next - sdat[i].last) * ratio;
    servos[i].write(constrain(lerpPos, MIN_SERVO_POS, MAX_SERVO_POS));
  }

	if (timenow - timestamp >= TIMESTAMP_PERIOD) {
    timestamp = timenow;
    
    // Set next platform state (0..23..etc)
    platform_state = (platform_state + 1) % NUM_PLATFORM_STATES;
    
    for (int8_t i = 0; i < NUM_SERVOS; i++) {
      sdat[i].last = sdat[i].next;
      sdat[i].next = constrain(platformPositions[platform_state][i] + sdat[i].trim, MIN_SERVO_POS, MAX_SERVO_POS);
    }
  }
}