#include "RotaryEncoderAcelleration.h"

void RotaryEncoderAcelleration::initialize(uint8_t pinNumberA, uint8_t pinNumberB) {
	pinA.initialize(pinNumberA, 1);
	pinB.initialize(pinNumberB, 1);
	tps.initialize();

	position = 0;
	minValue = 0;
	maxValue = 100;
}

void IRAM_ATTR RotaryEncoderAcelleration::update() {
	pinA.update(); // toggle
	pinB.update(); // direction
	if (isTicked()) {
//		tps.update(true);
		int tps1=10;//tps.getIntTPS();
//		int speed = constrain(tps1, MIN_TPS, MAX_TPS) - MIN_TPS;
//		long delta = max(1, (maxValue - minValue) / TICKS_AT_MAX_SPEED_FOR_FULL_SPAN);

		// Linear acceleration (very sensitive - not comfortable)
		// long step = 1 + delta * speed / (MAX_TPS - MIN_TPS);

		// Exponential acceleration - square (OK for [maxValue - minValue] = up to 5000)
		// long step = 1 + delta * speed * speed / ((MAX_TPS - MIN_TPS) * (MAX_TPS - MIN_TPS));

		long step;
		// Exponential acceleration - cubic (most comfortable)
		//step = 1 + delta * speed * speed * speed /
		//		((MAX_TPS - MIN_TPS) * (MAX_TPS - MIN_TPS) * (MAX_TPS - MIN_TPS));
		step=1;
		position = constrain(position + (isIncrementing() ? step : -step),
				minValue, maxValue);
	} else {
//		tps.update(false);
	}

}
