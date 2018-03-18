// configure parameter adjustment GUI controls and set their defaults
//slider1:0<-12,12,0.1>Input Level (dB)
//slider2:0<0,1,2{leaky integrator, soft-knee, cubic}>Type:
//slider3:0.2300<0.0001,0.5000,0.0001>-Tc Rise:
//slider4:0.9827<0.5000,0.9999,0.0001>Integrator Tc:
//slider5:0.9500<0.0100,10.000,0.0100>Soft-Clip Knee:
//slider6:1.0<0, 1, 0.0001>-Cubic Soft Clip Threshold:
//slider7:0.5<0, 1, 0.0001>Cubic Harmonic Balance:
//slider8:0.9999<0.9990,0.9999,0.0001>-DC Cutoff:
//slider9:0<-12,12,0.1>Output Level (dB)
// ************************************************************************


// ************************************************************************
// initialize constants
static const uint16_t NLD = 0;
static const uint16_t itm = 0;
static const uint16_t otm = 0;

// -3dB/Octave IIR filter coefficients [B = forward, A = reverse]
// coefficients are for DF2-transposed, and have only been tested at 48kHz
static const float B =
{
    1.0000000000000,
   -3.4673362000000,
    4.4316658211490,
   -2.4427741968430,
    0.4607539307915,
    0.0176931572105 
};
static const float A =
{
    1.0000000000000,
   -4.0469475000000,
    6.3705074734610,
   -4.8224326840640,
    1.7211909155470,
   -0.2223181410020 
};

// hard clip of input signal
function HardClip(s, thresh)(
  0.5 * (abs(s + thresh) - abs(s - thresh));
);

// cubic soft clip function
function SoftCubicClip(s, thresh)(
  threshInv = 1 / thresh;
  threshInv * ((thresh * 1.5 * HardClip(s, thresh)) -
    (0.5 * HardClip(s, thresh)^3 * threshInv));
);
  
// use this to process audio via the SoftCubicClip algorithm
function SoftCubic(sbuf)(
  SoftCubicClip(sbuf, slider6)
    + (slider7 * SoftCubicClip(abs(sbuf), slider6));
);

// soft clip function with adjustable knee
function SKClip(s, knee)(
  s / (knee * abs(s) + 1);
);

// use this to process audio via the SKClip algorithm
function SoftKnee(sbuf)(
  SKClip(sbuf, slider5) + ((slider5 / 2) * SKClip(abs(sbuf), slider5));
);

// use this to process audio via the leaky integrator algorithm
function LeakyInt(sbuf, NLD)(
  sbuf > NLD ? (
    ((1 - slider3) * sbuf) + (slider3 * NLD);
  ) : (
    ((1 - slider4) * sbuf) + (slider4 * NLD);
  );
);

// selects correct NLD algorithm based upon user input selection
function waveshape(sbuf, NLD)(
  slider2 > 0 ? (
    slider2 > 1 ? (
      invsqrt(2) * SoftCubic(sbuf);
    ) : (
      SoftKnee(sbuf);
    );
  ) : (
    NLD = invsqrt(2) * LeakyInt(sbuf, NLD);
  );
);

// applies -3dB/Octave filter [DF2-transposed]
pinkfilt(B, A, x) (
    y = B(1) * x + z;
);