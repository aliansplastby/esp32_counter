#include <WProgram.h>  // This include should go first, otherwise does not compile.
#include <Button.h>
#include <TicksPerSecond.h>

static const int buttonPin = 4; // the number of the pushbutton pin
static const int ledPin = 13; // the number of the LED pin

static Button btn;
static boolean lightOn = false;
static TicksPerSecond tps;

void setup() {
	pinMode(ledPin, OUTPUT);
	btn.initialize(buttonPin);
	tps.initialize();
	Serial.begin(9600);
}

void loop() {
	btn.update();
	if (btn.isPressed()) {
		tps.update(true);
		lightOn = !lightOn;
		digitalWrite(ledPin, lightOn ? HIGH : LOW);
	} else {
		tps.update(false);
	}
	Serial.println(tps.getTPS()); // Print button presses presses per second
}

