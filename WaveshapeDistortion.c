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
// Released under GPL-3.0:
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
#define PUSH1 			    RPI_GPIO_P1_08      //GPIO14
#define PUSH2 			    RPI_V2_GPIO_P1_38  	//GPIO20
#define TOGGLE_SWITCH 	    RPI_V2_GPIO_P1_32 	//GPIO12
#define FOOT_SWITCH         RPI_GPIO_P1_10 	    //GPIO15
#define LED   			    RPI_V2_GPIO_P1_36 	//GPIO16

#define DEBOUNCE_TIMER_DELAY_CYCLES 31250

// ADC read instruction, and 12 bit ADC value (0x08 ch0 - 0c for ch1)
static uint8_t mosi[10] = { 0x01, 0x00, 0x00 };
static uint8_t miso[10] = { 0 };

static uint_fast8_t switch_push_foot;
static uint_fast8_t switch_toggle_0;
static uint_fast8_t switch_push_left;
static uint_fast8_t switch_push_right;

// keep track of cycles and only read user controls a few times per second
static uint_fast16_t read_timer = 0;

// keep track of cycles and wait after a pushbutton event to help with debounce
static uint_fast16_t debounce_timer = 0;

// input (unprocessed) sample, output (processed) sample, and previous output sample
static float_t input_sample = 0.0;
static float_t output_sample = 0.0;
static float_t previous_output_sample = 0.0;

// ADC bias offset, for adding/removing DC offset from incoming signal
static const float_t biasOffsetADC = 2047.0;

// define the sqrt(2) for use in level matching softKnee and softCubic
static const float_t sqrt2 = 1.41421356237;

// define the inverse of sqrt(2) for use in level matching LeakyInt and softCubic
static const float_t invsqrt2 = 0.70710678118;

// the type of waveshape distortion applied to the signal
static enum waveshapers{leakyIntegrator, softKnee, softCubic} waveshapeType;

// rising and falling time constants for the leaky integrator
// Rise: 0.0001 to 0.5000, default is 0.2300
// Fall: 0.5000 to 0.9999, default is 0.9827
static float_t TcRise = 0.2300;
static float_t TcFall = 0.9827;

// soft clip knee value 'k', 0.1 to 10, default is 0.95
static float_t SoftClipKnee = 0.95;

// threshold of cubic soft clipping, 0 to 1, default is 1
static float_t CubicSoftClipThreshold = 1.0;

// cubic soft clip harmonic balance, 0 to 1, default is 0.5
static float_t CubicHarmonicBalance = 0.5;

// rectification state and threshold, 0 to 1, default is 0.35
static uint8_t isRectifierEnabled = 0;
static float_t RectifierThreshold = 0.35;

// additional gain state and value
static uint8_t isAddionalGainEnabled = 0;
static float_t AdditionalGainMultiplier = 2.0;
static float_t absf(float_t x) {
    return (x >= 0.0 ? x : -x);
}

// cube function
static float_t cubef(float_t x) {
    return (x * x * x);
}

// hard clip of input signal
static float_t HardClip(float_t sample, float_t thresh) {
    return 0.5 * (absf(sample + thresh) - absf(sample - thresh));
};

// cubic soft clip function
static float_t SoftCubicClip(float_t sample, float_t thresh) {
    float_t threshInv = 1 / thresh;
    return threshInv * ((thresh * 1.5 * HardClip(sample, thresh)) -
        (0.5 * cubef(HardClip(sample, thresh)) * threshInv));
};

// use this to process audio via the SoftCubicClip algorithm
static float_t SoftCubic(float_t sample) {
    return invsqrt2 * (SoftCubicClip(sample, CubicSoftClipThreshold) +
        (CubicHarmonicBalance * SoftCubicClip(absf(sample), CubicSoftClipThreshold)));
};

// soft clip function with adjustable knee
static float_t SKClip(float_t sample, float_t knee) {
    return sample / (knee * absf(sample) + 1.0);
};

// use this to process audio via the SKClip algorithm
static float_t SoftKnee(float_t sample) {
    return SKClip(sample, SoftClipKnee) + ((SoftClipKnee / 2.0) * SKClip(absf(sample), SoftClipKnee));
};

// use this to process audio via the leaky integrator algorithm
static float_t LeakyInt(float_t sample, float_t previous_sample) {
    if (sample > previous_sample) {
       return invsqrt2 * (((1.0 - TcRise) * sample) + (TcRise * previous_sample));
    }
      else {
       return invsqrt2 * (((1.0 - TcFall) * sample) + (TcFall * previous_sample));
    }
};

static void configurePWM(void) {
    //define PWM output configuration
    bcm2835_gpio_fsel(18,BCM2835_GPIO_FSEL_ALT5 );              //PWM0 signal on GPIO18
    bcm2835_gpio_fsel(13,BCM2835_GPIO_FSEL_ALT0 );              //PWM1 signal on GPIO13
    bcm2835_pwm_set_clock(2); // Max clk frequency (19.2MHz/2 = 9.6MHz)
    bcm2835_pwm_set_mode(0,1 , 1); //channel 0, markspace mode, PWM enabled.
    bcm2835_pwm_set_range(0,64);   //channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
    bcm2835_pwm_set_mode(1, 1, 1); //channel 1, markspace mode, PWM enabled.
    bcm2835_pwm_set_range(1,64);   //channel 0, 64 is max range (6bits): 9.6MHz/64=150KHz switching PWM freq.
}

static void configureSPI(void) {
    //define SPI bus configuration
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);    // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                 // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); 	// 4MHz clock/64 = 62500
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                    // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);    // the default
}

static void configureGPIO(void) {
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

int main(int argc, char **argv) {
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
    
    // Initialize the BCM2835 PWM, SPI, and GPIO configurations
    configurePWM();
    configureSPI();
    configureGPIO();

    // Main Loop
    while(1) {
        // read 12 bits from ADC, subtract ADC DC offset and normalized to between -1 and 1
        bcm2835_spi_transfernb(mosi, miso, 3);
        input_sample = (float_t)((((miso[1] & 0x0F) << 8) + miso[2]) - biasOffsetADC)/(biasOffsetADC);

        // Read the PUSH buttons every 1563 times (0.025s) to save resources.
        read_timer++;
        if ((read_timer >= 1563) & (debounce_timer == 0)) {
            read_timer=0;
            switch_push_left = bcm2835_gpio_lev(PUSH1);
            switch_push_right = bcm2835_gpio_lev(PUSH2);
            switch_toggle_0 = bcm2835_gpio_lev(TOGGLE_SWITCH);
            switch_push_foot = bcm2835_gpio_lev(FOOT_SWITCH);

            //light the effect when the footswitch is activated.
            bcm2835_gpio_write(LED,!switch_push_foot);

            //update waveshapeType when the PUSH1 or 2 buttons are pushed and toggle switch is up
            if (switch_toggle_0 == 0) {
                if (switch_push_left == 0) {
                    //reset delay for button debouncing
                    debounce_timer = DEBOUNCE_TIMER_DELAY_CYCLES;
                    if (waveshapeType > 0) {
                        waveshapeType = (enum waveshapers)((uint8_t)waveshapeType - 1);
                    }
                    else {
                        waveshapeType = (enum waveshapers) 2;
                    }
                }
                else if (switch_push_right == 0) {
                    //100ms delay for button debouncing
                    debounce_timer = DEBOUNCE_TIMER_DELAY_CYCLES;
                    if (waveshapeType < 2 ) {
                        waveshapeType = (enum waveshapers)((uint8_t)waveshapeType + 1);
                    }
                    else {
                        waveshapeType = (enum waveshapers) 0;
                    }
                }
            }
            // change waveshape parameter instead if toggle switch is down
            else {
                if (switch_push_left == 0) {
                    debounce_timer = DEBOUNCE_TIMER_DELAY_CYCLES;
                    //enable additional gain multiplier
                    if (isAddionalGainEnabled != 0){
                        isAddionalGainEnabled = 0;
                    }
                    else {
                        isAddionalGainEnabled = 1;
                    }
                }
                else if (switch_push_right == 0) {
                    debounce_timer = DEBOUNCE_TIMER_DELAY_CYCLES;
                    //enable partial signal rectification
                    if (isRectifierEnabled != 0){
                        isRectifierEnabled = 0;
                    }
                    else {
                        isRectifierEnabled = 1;
                    }
                }
            }
        }
        else if (debounce_timer > 0) {
            debounce_timer -= 1;
        }

        // preprocessing gain and hardclip
        input_sample = HardClip(sqrt2 * input_sample, 1.0);

        //**** WAVESHAPE DISTORTION ***///
        switch (waveshapeType) {
            case leakyIntegrator:
                //bcm2835_gpio_write(LED, 1);  // for verifying pushbuttons cycle through cases
                output_sample = LeakyInt(input_sample, previous_output_sample);
                previous_output_sample = output_sample;
                break;
            case softKnee:
                output_sample =  SoftKnee(input_sample);
                break;
            case softCubic:
                output_sample = SoftCubic(input_sample);
                break;
            default:
                output_sample = input_sample;
        }

        // postprocessing gain and hard clip
        output_sample = HardClip(invsqrt2 * output_sample, 1.0);

        // revert normalized signal to previous scale and add initial DC bias back into signal
        output_sample = (output_sample * biasOffsetADC) + biasOffsetADC;

        //generate two 6-bit PWM outputs to simulate a 12-bit PWM output
        bcm2835_pwm_set_data(1, ((uint16_t)output_sample) & 0x003F);
        bcm2835_pwm_set_data(0, ((uint16_t)output_sample) >> 6);
    }

    //close all and exit (does this actually work?)
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}