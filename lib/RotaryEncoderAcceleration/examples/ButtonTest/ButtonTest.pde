#include <WProgram.h>  // This include should go first, otherwise does not compile.
#include <Button.h>

static const int buttonPin = 4; // the number of the pushbutton pin
static const int ledPin = 13; // the number of the LED pin

static Button btn;
static boolean lightOn = false;

void setup() {
	pinMode(ledPin, OUTPUT);
	btn.initialize(buttonPin);
}

void loop() {
	btn.update();
	if (btn.isPressed()) {
		lightOn = !lightOn;
		digitalWrite(ledPin, lightOn ? HIGH : LOW);
	}
}

