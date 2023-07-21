// mixing the proximity_sensor, color_sensor and gesture_sensor Arduino examples.
// playing a sinewave in the background in the meantime.
// Select the example you want to play with the `example` variable below

#include <Bela.h>
#include <cmath>
#include "Adafruit_APDS9960.h"
#include <Gpio.h>
#include <MiscUtilities.h>

enum ExampleType {
	kProximitySensor,
	kColorSensor,
	kGestureSensor,
};
ExampleType example = kColorSensor;


float gFrequency = 440.0;
float gPhase;
float gInverseSampleRate;

//create the APDS9960 object
Adafruit_APDS9960 apds;
Gpio INT_PIN;

// I2C I/O cannot be performed from the audio thread, so we
// do it in this other thread
void loop(void*) {
	while(!Bela_stopRequested())
	{
		if(kProximitySensor == example)
		{
			//print the proximity reading when the interrupt pin goes low
			if(!INT_PIN.read()){
				printf("%d\n", apds.readProximity());
				//clear the interrupt
				apds.clearInterrupt();
			}
		} else if(kColorSensor == example) {
			//create some variables to store the color data in
			uint16_t r, g, b, c;

			//wait for color data to be ready
			while(!apds.colorDataReady()){
				usleep(5000);
			}

			//get the data and print the different channels
			apds.getColorData(&r, &g, &b, &c);
			printf("red: %d, green: %d, blue: %dm clear: %d\n", r, g, b, c);
			usleep(100000);
		} else if(kGestureSensor == example) {
			//read a gesture from the device
			uint8_t gesture = apds.readGesture();
			if(gesture == APDS9960_DOWN) printf("v\n");
			if(gesture == APDS9960_UP) printf("^\n");
			if(gesture == APDS9960_LEFT) printf("<\n");
			if(gesture == APDS9960_RIGHT) printf(">\n");
		}
		usleep(5000); // how long to sleep between iterations of the loop
	}
}

bool setup(BelaContext *context, void *userData)
{
	// set P2_03 as input for the interrupt pin of the sensor
	// This only works on BelaMini. Find a different pin on Bela
	PinmuxUtils::set("P2_03", "gpio_pd");
	INT_PIN.open(23, Gpio::INPUT); // P2_03

	int i2cBus = 1; // set it according to your wiring
	if(!apds.begin(10, APDS9960_AGAIN_4X, 0x39, i2cBus)){
		fprintf(stderr, "Error initialising the sensor\n");
		return false;
	}
	
	if(kProximitySensor == example)
	{
		// In this mode, this sketch puts the sensor in proximity mode
		// and enables the interrupt to fire when proximity goes over a
		// set value Designed specifically to work with the Adafruit
		// APDS9960 breakout ----> http://www.adafruit.com/products/3595
  		// These sensors use I2C to communicate. The device's I2C address is 0x39
		//enable proximity mode
		apds.enableProximity(true);

		//set the interrupt threshold to fire when proximity reading goes above 175
		apds.setProximityInterruptThreshold(0, 175);

		//enable the proximity interrupt
		apds.enableProximityInterrupt();
	} else if (kColorSensor == example) {
		//enable color sensign mode
		apds.enableColor(true);
	} else if(kGestureSensor == example) {
		//gesture mode will be entered once proximity mode senses something close
		apds.enableProximity(true);
		apds.enableGesture(true);
	}
	// start a separate thread to read the encoder value.
	Bela_runAuxiliaryTask(loop);

	gInverseSampleRate = 1.0 / context->audioSampleRate;
	gPhase = 0.0;

	return true;
}

void render(BelaContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {
		float out = 0.8 * sinf(gPhase);
		gPhase += 2.0 * M_PI * gFrequency * gInverseSampleRate;
		if(gPhase > 2.0 * M_PI)
			gPhase -= 2.0 * M_PI;

		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
			// Two equivalent ways to write this code

			// The long way, using the buffers directly:
			// context->audioOut[n * context->audioOutChannels + channel] = out;

			// Or using the macros:
			audioWrite(context, n, channel, out);
		}
	}
}

void cleanup(BelaContext *context, void *userData)
{

}
