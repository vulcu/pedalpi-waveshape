// (C) 2018, Winry R. Vulcu
//
// NO WARRANTY IS GRANTED. THIS PLUG-IN IS PROVIDED ON AN "AS IS" BASIS,
// WITHOUT WARRANTY OF ANY KIND. NO LIABILITY IS GRANTED, INCLUDING, BUT
// NOT LIMITED TO, ANY DIRECT OR INDIRECT, SPECIAL, INCIDENTAL OR
// CONSEQUENTIAL DAMAGE ARISING OUT OF THE USE OR INABILITY TO USE THIS
// PLUG-IN, COMPUTER FAILTURE OF MALFUNCTION INCLUDED. THE USE OF THE
// SOURCE CODE, EITHER PARTIALLY OR IN TOTAL, IS ONLY GRANTED, IF USED
// IN THE SENSE OF THE AUTHOR'S INTENTION, AND USED WITH ACKNOWLEDGEMENT
// OF THE AUTHOR. FURTHERMORE IS THIS PLUG-IN A THIRD PARTY CONTRIBUTION,
// EVEN IF INCLUDED IN REAPER(TM), COCKOS INCORPORATED OR ITS AFFILIATES
// HAVE NOTHING TO DO WITH IT. LAST BUT NOT LEAST, BY USING THIS PLUG-IN
// YOU RELINQUISH YOUR CLAIM TO SUE IT'S AUTHOR, AS WELL AS THE CLAIM TO
// ENTRUST SOMEBODY ELSE WITH DOING SO.
// 
// Released under GPL:
// <http://www.gnu.org/licenses/>.
//
// ************************************************************************
//
// References:
// Aarts, R.M., Larsen, E., and Schobben, D., 2002, 'Improving Perceived
//   Bass and Reconstruction of High Frequencies for Band Limited Signals'
//   Proc. 1st IEEE Benelux Workshop MPCA-2002, pp. 59-71
// Arora et al., 2006, 'Low Complexity Virtual Bass Enhancement Algorithm
//   for Portable Multimedia Device'
//   AES 29th International Conferance, Seoul, Korea, 2006 September 2-4
// Gerstle, B., 2009, 'Tunable Virtual Bass Enhancement', [ONLINE]
//   <http://rabbit.eng.miami.edu/students/ddickey/pics/
//   Gerstle_Final_Project.pdf>
// Yates, R. and Lyons, R., 2008, 'DC Blocker Algorithms'
//   IEEE Signal Processing Magazine, March 2008, pp. 132-134
//
// ************************************************************************
//
// leaky integrator alg: y[n] = ((1 - A) * x[n]) + (A * y[n - 1]);
// soft knee clip alg:   y[n] = x[n] / (K * abs(x[n]) + 1);
// cubic soft clip alg:  y[n] = (1.5 * threshold * HardClip(x[n])) -
//                                ((0.5 * HardClip(x[n])^3) / threshold);
//
// ************************************************************************
// tags: waveshaping, non-linear distortion
// author: Winry R. Vulcu
// ************************************************************************

#include <bcm2835.h>
#include <math.h>
#include <stdio.h>

// Define User controls as GPIO Input Pins
#define PUSH1 			      RPI_GPIO_P1_08  	//GPIO14
#define PUSH2 			    RPI_V2_GPIO_P1_38  	//GPIO20 
#define TOGGLE_SWITCH 	RPI_V2_GPIO_P1_32 	//GPIO12
#define FOOT_SWITCH     	RPI_GPIO_P1_10 		//GPIO15
#define LED   			    RPI_V2_GPIO_P1_36 	//GPIO16

uint8_t mosi[10] = { 0x01, 0x00, 0x00 }; //12 bit ADC read 0x08 ch0, - 0c for ch1 
uint8_t miso[10] = { 0 };

uint8_t FOOT_SWITCH_val;
uint8_t TOGGLE_SWITCH_val;
uint8_t PUSH1_val;
uint8_t PUSH2_val;

uint32_t read_timer=0;
uint32_t input_signal=0;
uint32_t output_signal=0;
uint32_t fuzz_value=100; //good value to start.

// input gain, -12dB to +12dB, default is 0dB
double InputLevel = 0;

// the type of waveshape distortion applied to the signal
enum WaveshapeType{'leakyIntegrator', 'softKnee', 'cubic'};

// rising and falling time constants for the leaky integrator
// Rise: 0.0001 to 0.5000, default is 0.2300
// Fall: 0.5000 to 0.9999, default is 0.9827
double TcRise = 0.2300;
double TcFall = 0.9827;

// soft clip knee value 'k', 0.1 to 10, default is 0.95
double SoftClipKnee = 0.95;

// threshold of cubic soft clipping, 0 to 1, default is 1
double CubicSoftClipThreshold = 1.0;

// cubic soft clip harmonic balance, 0 to 1, default is 1
double CubicHarmonicBalance;

// 0.9990 to 0.9999, default is 0.9999
const double DC_Cutoff = 0.9999;
double itm = 0;
double otm = 0;

// output gain, -12dB to +12dB, default is 0dB
double OutputLevel = 0;

static void configurePWM(void) {
	//define PWM output configuration
    bcm2835_gpio_fsel(18,BCM2835_GPIO_FSEL_ALT5 ); //PWM0 signal on GPIO18    
    bcm2835_gpio_fsel(13,BCM2835_GPIO_FSEL_ALT0 ); //PWM1 signal on GPIO13    
	  bcm2835_pwm_set_clock(2); // Max clk frequency (19.2MHz/2 = 9.6MHz)
    bcm2835_pwm_set_mode(0,1 , 1); //channel 0, markspace mode, PWM enabled. 
	  bcm2835_pwm_set_range(0,64);   //channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
    bcm2835_pwm_set_mode(1, 1, 1); //channel 1, markspace mode, PWM enabled.
	  bcm2835_pwm_set_range(1,64);   //channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
}

static void configureSPI(void) {
	//define SPI bus configuration
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); 	  // 4MHz clock with _64 
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      // the default
}

static void setupGPIO(void) {
    //Define GPIO pin configuration
    bcm2835_gpio_fsel(PUSH1, BCM2835_GPIO_FSEL_INPT); 			//PUSH1 button as input
  	bcm2835_gpio_fsel(PUSH2, BCM2835_GPIO_FSEL_INPT); 			//PUSH2 button as input
	  bcm2835_gpio_fsel(TOGGLE_SWITCH, BCM2835_GPIO_FSEL_INPT);	//TOGGLE_SWITCH as input
  	bcm2835_gpio_fsel(FOOT_SWITCH, BCM2835_GPIO_FSEL_INPT); 	//FOOT_SWITCH as input
  	bcm2835_gpio_fsel(LED, BCM2835_GPIO_FSEL_OUTP);				//LED as output

    bcm2835_gpio_set_pud(PUSH1, BCM2835_GPIO_PUD_UP);           //PUSH1 pull-up enabled   
  	bcm2835_gpio_set_pud(PUSH2, BCM2835_GPIO_PUD_UP);           //PUSH2 pull-up enabled 
  	bcm2835_gpio_set_pud(TOGGLE_SWITCH, BCM2835_GPIO_PUD_UP);   //TOGGLE_SWITCH pull-up enabled 
  	bcm2835_gpio_set_pud(FOOT_SWITCH, BCM2835_GPIO_PUD_UP);     //FOOT_SWITCH pull-up enabled 
}

// hard clip of input signal
double HardClip(double sample, double thresh) {
  return 0.5 * (abs(sample + thresh) - abs(sample - thresh));
};

// cubic soft clip function
double SoftCubicClip(double sample, double thresh) {
  double threshInv = 1 / thresh;
  return threshInv * ((thresh * 1.5 * HardClip(sample, thresh)) -
    (0.5 * powf(HardClip(sample, thresh), 3.0) * threshInv));
};

// use this to process audio via the SoftCubicClip algorithm
double SoftCubic(double sample) {
  return invsqrt(2) * (SoftCubicClip(sample, CubicSoftClipThreshold) + 
    (CubicHarmonicBalance * SoftCubicClip(abs(sample), CubicSoftClipThreshold)));
};

// soft clip function with adjustable knee
double SKClip(double sample, double knee) {
  return sample / (knee * abs(sample) + 1);
};

// use this to process audio via the SKClip algorithm
double SoftKnee(double sample) {
  SKClip(sample, SoftClipKnee) + ((SoftClipKnee / 2) * SKClip(abs(sample), SoftClipKnee));
};

// use this to process audio via the leaky integrator algorithm
double LeakyInt(double sample, double sampleLast) {
  if sample > sampleLast
    return invsqrt(2) * ((1 - TcRise) * sample) + (TcRise * sampleLast);
  else
    return invsqrt(2) * ((1 - TcFall) * sample) + (TcFall * sampleLast);
  end
};

// apply a DC blocker to processed audio
double BlockDC(double sample) {
  otm = DC_Cutoff * otm + sample - itm;
  itm = sample;
  sample = otm;
  return sample;
};

int main(int argc, char **argv)
{
    // Try to start the BCM2835 Library to access GPIO.
  if (!bcm2835_init()) {
		printf("bcm2835_init failed. Are you running as root??\n");
      	return 1;
	}

	// Try to start the BCM2835 SPI bus.
	if (!bcm2835_spi_begin()) {
		printf("bcm2835_spi_begin failed. Are you running as root??\n");
    	return 1;
	}

	// Main Loop
	while(1) {
		//read 12 bits ADC
		bcm2835_spi_transfernb(mosi, miso, 3);
		input_signal = miso[2] + ((miso[1] & 0x0F) << 8); 

		//Read the PUSH buttons every 50000 times (0.25s) to save resources.
		read_timer++;
		if (read_timer==50000) {
			read_timer=0;
			uint8_t PUSH1_val = bcm2835_gpio_lev(PUSH1);
			uint8_t PUSH2_val = bcm2835_gpio_lev(PUSH2);
			TOGGLE_SWITCH_val = bcm2835_gpio_lev(TOGGLE_SWITCH);
			uint8_t FOOT_SWITCH_val = bcm2835_gpio_lev(FOOT_SWITCH);

			//light the effect when the footswitch is activated.
			bcm2835_gpio_write(LED,!FOOT_SWITCH_val); 
			
			//update booster_value when the PUSH1 or 2 buttons are pushed.
			if (PUSH1_val==0) {
				bcm2835_delay(100); //100ms delay for buttons debouncing
				if (fuzz_value<2047) {
					fuzz_value=fuzz_value+10;
				}
			}
			else if (PUSH2_val==0) {
				bcm2835_delay(100); //100ms delay for buttons debouncing.
					if (fuzz_value>0) {
						fuzz_value=fuzz_value-10;
					}
			}
		}

		//**** FUZZ EFFECT ***///
    // do the thing
		
		//generate output PWM signal 6 bits
		bcm2835_pwm_set_data(1, input_signal & 0x3F);
		bcm2835_pwm_set_data(0, input_signal >> 6);
    }
	
	//close all and exit (does this actually work?)
	bcm2835_spi_end();
    bcm2835_close();
    return 0;
}