#include <WProgram.h>  // This include should go first, otherwise does not compile.
#include <Button.h>
#include <TicksPerSecond.h>
#include <RotaryEncoderAcelleration.h>

static const int buttonPin = 21;	// the number of the pushbutton pin
static const int speakerPin = 8;
static const int rotorPinA = 12;	// One quadrature pin
static const int rotorPinB = 13;	// the other quadrature pin

static Button btn;
static boolean speakerOn = true;
static RotaryEncoderAcelleration rotor;

void UpdateRotor() {
	rotor.update();
}

void setup() {
	pinMode(speakerPin, OUTPUT);
	btn.initialize(buttonPin);
	rotor.initialize(rotorPinA, rotorPinB);
	rotor.setMinMax(0, 50000);
	rotor.setPosition(500);
	attachInterrupt(0, UpdateRotor, CHANGE);
	Serial.begin(9600);
}

long lastRotor = 0;
void loop() {
	btn.update();

	long pos = rotor.getPosition();
	if (btn.isPressed()) {
		speakerOn = !speakerOn;
		if (speakerOn) {
			tone(speakerPin, pos);
		} else {
			noTone(speakerPin);
		}
	}

	if (lastRotor != pos) {
		if (speakerOn) {
			tone(speakerPin, pos);
		}
		float tps = rotor.tps.getTPS();
		Serial.print(pos);
		Serial.print(" ");
		Serial.println(tps);
	}
	lastRotor = pos;
}
