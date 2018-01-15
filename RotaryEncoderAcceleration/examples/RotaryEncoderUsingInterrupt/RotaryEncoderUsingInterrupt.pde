#include <WProgram.h> // This include should go first, otherwise does not compile.
#include <Button.h>
#include <TicksPerSecond.h>
#include <RotaryEncoderAcelleration.h>

const int rotorPinA = 2;
const int rotorPinB = 3;
const int speakerPin = 8;

RotaryEncoderAcelleration rotor;

void UpdateRotor() {
	rotor.update();
}

void setup() {
	pinMode(speakerPin, OUTPUT);
	rotor.initialize(rotorPinA, rotorPinB);
	rotor.setMinMax(50, 5000);
	rotor.setPosition(500);
	attachInterrupt(0, UpdateRotor, CHANGE);
}

void loop() {
	tone(speakerPin, rotor.getPosition());
}

