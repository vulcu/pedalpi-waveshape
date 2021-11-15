# PedalPi-WaveShape - Audio Distortion for the [Pedal Pi](https://www.electrosmash.com/pedal-pi) #
A waveshaping audio distortion written in C for the [Electrosmash Pedal Pi](https://www.electrosmash.com/pedal-pi), and using the Broadcom BCM2835. It utilizes some of the same waveshaper algorithms as [Waveshape-Synth](https://github.com/vulcu/waveshape-synth) and [WaveDist](https://github.com/vulcu/wavedist).

## Table of Contents ##
* [General Info](#general-info)
* [Features](#features)
* [Installation](#installation)
* [Algorithms](#algorithms)
* [References](#references)

## General Info
This project makes things sound bad, but in a good way. It relies on a handful of waveshaping algorithms to produce differing kinds of overdrive and distortion. The harmonic ratios and the balance between even and odd harmonics varies by algorithm, with some sounding better on certain musical sources than others. There's no hard-and-fast rules here, so just use your ears.

The `RectifierThreshold` parameter can lend itself to some neat octave-doubling effects too!

## Features ##
If bugs count as features, then this has a _lot_ of features. The last time it was tested, in 2018, it was cleanly passing audio but not much else. 

#### To Do ####
* ~~Get the basic UI functioning~~ _done_
* ~~Get audio passing **with** the waveshape processing applied~~ _done_
  * ~~leaky integrator~~ _done_
  * ~~soft-knee~~ _done_
  * ~~cubic~~ _done_
* ~~Create a 'gain boost' on/off control~~ _done_
* ~~Add a partial input rectification on/off control to create additional harmonics~~ _done_
* Experiment with adding a `Wet/Dry` control
* Get this to run on every boot so that you don't have to `ssh` into the Pi on every boot and start it
* Add oversampling + filtering to reduce aliasing artifacts

#### Status: This project is considered complete and not actively maintained. Additional 'to do' items or new features are unlikely, although some bugs may be fixed. ####

## Installation ##
The BCM2835 library is available here:
> https://www.airspayce.com/mikem/bcm2835/

Download the BCM2835 library onto a Pedal Pi and install it with:
```shell
wget https://www.airspayce.com/mikem/bcm2835/bcm2835-1.70.tar.gz;
tar xvfz bcm2835-1.70.tar.gz;
cd bcm2835-1.70;
./configure;
make;
sudo make install
```

Next, download this repo onto a Pedal Pi and compile it with:  
`gcc -o WaveshapeDistortion -l rt WaveshapeDistortion.c -l bcm2835 -lm`
  
then run:  
`sudo ./WaveshapeDistortion`

## Algorithms ##
This project uses the following algorithms for waveshaping and signal limiting:
```
 1) soft clip alg:        y[n] = (1.5*x[n]) - (0.5*x[n]^3);
 2) leaky integrator alg: y[n] = ((1 - A) * x[n]) + (A * y[n - 1]);
 3) soft knee clip alg:   y[n] = x[n] / (K * abs(x[n]) + 1);
 4) cubic soft clip alg:  y[n] = (1.5 * threshold * HardClip(x[n])) -
                                ((0.5 * HardClip(x[n])^3) / threshold);
 5) warp alg: y[n] = (((x[n] * 1.5) - (0.5 * x[n]^3)) * (((2 * K) / (1 - K))
                    + 1)) / ((abs((x[n] * 1.5) - (0.5 * x[n]^3)) 
                    * ((2 * K) / (1 - K))) + 1);
 6) rectify alg: y[n] = ((1 - R) * softclip(x[n])) + (|softclip(x[n])| * R);
```

## References: ##
1)  Aarts, R.M., Larsen, E., and Schobben, D., 2002, 'Improving Perceived Bass and Reconstruction of High Frequencies for Band Limited Signals' Proc. 1st IEEE Benelux Workshop MPCA-2002, pp. 59-71
 2) Arora et al., 2006, 'Low Complexity Virtual Bass Enhancement Algorithm for Portable Multimedia Device' AES 29th International Conferance, Seoul, Korea, 2006 September 2-4
 3) Gerstle, B., 2009, 'Tunable Virtual Bass Enhancement', [ONLINE] <http:rabbit.eng.miami.edu/students/ddickey/pics/Gerstle_Final_Project.pdf>
 4) Yates, R. and Lyons, R., 2008, 'DC Blocker Algorithms' IEEE Signal Processing Magazine, March 2008, pp. 132-134

## ##
(C) 2017-2021, Winry R. Litwa-Vulcu
