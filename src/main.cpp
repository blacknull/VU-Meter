/*
#ifdef ARDUINO_USB_CDC_ON_BOOT
  #undef ARDUINO_USB_CDC_ON_BOOT
  #define ARDUINO_USB_CDC_ON_BOOT 0
#endif
//*/

#define FASTLED_ALLOW_INTERRUPTS 0
#define FASTLED_INTERRUPT_RETRY_COUNT 1

#include <arduinoFFT.h>
#include <FastLED.h>
#include <Ticker.h>

Ticker tickerAdc;

// Params for width and height
const uint8_t BAR_WIDTH = 16;
const uint8_t BAR_HEIGHT = 8;

#define MAX_DIMENSION ((BAR_WIDTH>BAR_HEIGHT) ? BAR_WIDTH : BAR_HEIGHT)
#define NUM_LEDS (BAR_WIDTH * BAR_HEIGHT)

#ifdef ESP32
//  #define LED_PIN     3
//  #define ADC_PIN     5   // adc1 = 5 on c3 devboard
  #if defined(ARDUINO_USB_CDC_ON_BOOT) && defined(USE_DEV_BOARD)
    #define USB_SERIAL  Serial0
  #else
    #define USB_SERIAL  Serial
  #endif
#else
//  #define LED_PIN     4
//  #define ADC_PIN     A0
  #define USB_SERIAL  Serial
#endif

CRGB leds[NUM_LEDS];
uint8_t brightness = 32;

const uint16_t UNIT_BAR_WIDTH = 8, UNIT_BAR_HEIGHT = 8;
const uint16_t UNIT_BAR_SIZE = UNIT_BAR_WIDTH * UNIT_BAR_HEIGHT;
uint16_t XY( uint8_t x, uint8_t y) {
  x %= BAR_WIDTH, y %= BAR_HEIGHT;

  uint16_t column = x / UNIT_BAR_WIDTH;
  uint16_t row    = y / UNIT_BAR_HEIGHT;
  uint16_t offset = row * UNIT_BAR_SIZE * (BAR_WIDTH / UNIT_BAR_WIDTH) + column * UNIT_BAR_SIZE;

  x %= UNIT_BAR_WIDTH;
  y %= UNIT_BAR_HEIGHT;
  return (x * UNIT_BAR_HEIGHT) + y + offset;
}

struct MUSIC_BAR {
  const int8_t MAX_DELAY_FALL = 2;
  int8_t delayFall[BAR_WIDTH] = { 0 };
  unsigned long times = 0;
  uint8_t maxHeight[BAR_WIDTH] = { 0 };
  uint8_t movingAverage[BAR_WIDTH] = { 0 };

  void showColume(uint8_t x, uint8_t h, CHSV c) {
    x %= BAR_WIDTH, h = h > BAR_HEIGHT ? BAR_HEIGHT : h;

    const float sensitivity = 0.7;
    movingAverage[x] = round(movingAverage[x] * (1 - sensitivity) + h * sensitivity);
    h = movingAverage[x];

    if (h >= maxHeight[x]) {
      maxHeight[x] = h;
      delayFall[x] = MAX_DELAY_FALL;
    }
    else {
      if (maxHeight[x] > 0) {
        delayFall[x]--;
        if (delayFall[x] <= 0) {
          maxHeight[x]--;
          delayFall[x] = MAX_DELAY_FALL;
        }
      }
    }

  
    for (uint8_t i = 0; i < h/*maxHeight[x]*/; i++) {
      leds[XY(x, i)] = c;
    }

    // make the top pixel a little different
    c.h -= 10;  // hue offset
    c.s /= 2;
    c.v = c.v + 10 < 255 ? c.v + 10 : 255;  // brightness inc 10 if passible
    leds[XY(x, maxHeight[x] > 0 ? maxHeight[x] - 1 : 0)] = c;

    times++;
  }
} spectrumMeter;

///////////////////////////////////////////////////////////////
const uint16_t SAMPLES_BATCH_SIZE   = 256;      //This value MUST ALWAYS be a power of 2
const uint16_t MAX_DETECTION_FREQUENCY  = 4000; // Maximum frequency that is going to be procesesd
const uint16_t SAMPLING_FREQ = MAX_DETECTION_FREQUENCY * 2;  // double the max detection frequency
const unsigned long SAMPLEING_PERIOD_US = round(1000000.0 / SAMPLING_FREQ);

double realComponent[SAMPLES_BATCH_SIZE];
double imagComponent[SAMPLES_BATCH_SIZE];
arduinoFFT FFT = arduinoFFT(realComponent, imagComponent, SAMPLES_BATCH_SIZE, SAMPLING_FREQ);

const uint16_t FFT_BUFS = 2;
uint16_t dataAdc[FFT_BUFS][SAMPLES_BATCH_SIZE];
volatile uint8_t idxBuf = 0;

unsigned long mcsSample = 0;
void getAdcData() {
  unsigned long mcsBegin = micros();

  for (uint16_t i = 0; i < SAMPLES_BATCH_SIZE; i++) {
    unsigned long mcsLastTime = micros();
    dataAdc[idxBuf][i] = analogRead(ADC_PIN);
    while (micros() - mcsLastTime < SAMPLEING_PERIOD_US) {
      yield();
    }
  }

  mcsSample = (mcsSample + micros() - mcsBegin) / 2;  // moving average
  
  idxBuf++;
  idxBuf %= FFT_BUFS;
}

struct movingAverage {
  unsigned long mvAverage = 0, usBegin = 0;
  void begin() { usBegin = micros(); }
  void end() { mvAverage = (micros() - usBegin + mvAverage) / 2; }
};

movingAverage timer1, timer2, timer3;

void runFFT() {
  double sensitivity = 1.0;//map(analogRead(A0), 0, 1023, 50, 100);
  uint8_t idx = (idxBuf + FFT_BUFS - 1) % FFT_BUFS; // last sampled buf
  for (int i = 0; i < SAMPLES_BATCH_SIZE; i++) {
    realComponent[i] = dataAdc[idx][i] / sensitivity;
    imagComponent[i] = 0;
  }

  // Compute FFT
  timer1.begin();
  FFT.DCRemoval();
  //FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Windowing(FFT_WIN_TYP_TRIANGLE, FFT_FORWARD);
  timer1.end();

  timer2.begin();
  FFT.Compute(FFT_FORWARD);
  timer2.end();

  timer3.begin();
  FFT.ComplexToMagnitude();
  timer3.end();

  // abandom data in 0 - 1
  //realComponent[0] = realComponent[1] = 0;
}

void setup() {
  USB_SERIAL.begin(115200);
  delay(1000);

  // fast led init
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(brightness);
  FastLED.setDither(brightness < 255);

  // test for timecost of adc/fft
  pinMode(ADC_PIN, INPUT);
#ifdef ESP32
  analogReadResolution(10); // set 10bit adc for compatible
#endif

  unsigned long mbegin;
  for (int i = 0; i < 10; i++) {
    mbegin = micros();
    getAdcData();
    USB_SERIAL.println("adc data cost: " + String(micros() - mbegin) + " micro seconds");
  }

  mbegin = micros();
  runFFT();
  USB_SERIAL.println("run fft cost: " + String(micros() - mbegin) + " micro seconds");
  //*/

  //while(true) {delay(10);}

  // ticker for adc
  tickerAdc.attach_ms(40, getAdcData);   // ms to get adc buf
}

union FREQ_RANGE{
  struct {
    uint8_t low, high;
  };
  uint16_t freq;
};

FREQ_RANGE freqRange[BAR_WIDTH] = { 
{01, 03}, {04, 05}, {06, 07}, {8, 9}, 
{10, 11}, {12, 12}, {13, 13}, {14, 14}, 
{15, 15}, {16, 16}, {17, 17}, 
{18, 19}, {20, 22}, {23, 25}, 
{26, 64}, {65, 127}
};

unsigned long timeLoop = 0;
unsigned long countLoop = 0;
void loop() {
  EVERY_N_MILLISECONDS(35) 
  {
    unsigned long mbegin = micros();
    runFFT();
    timeLoop = (micros() - mbegin + timeLoop) / 2;  // moving average value

    FastLED.clear();
    
    for (uint8_t i = 0; i < BAR_WIDTH; i++) {
      uint16_t data = 0;
      for (uint8_t f = freqRange[i].low; f <= freqRange[i].high; f++) {
        const int CONSTRAIN_MAX = 1024, NOISE_CANCEL = 100;
        realComponent[f] = realComponent[f] > NOISE_CANCEL ? realComponent[f] : 0;
        realComponent[f] = constrain(realComponent[f], 0, CONSTRAIN_MAX);
        realComponent[f] = map(realComponent[f], 0, CONSTRAIN_MAX, 0, BAR_HEIGHT);
        data += realComponent[f];
      }
      // average data
      data /= freqRange[i].high - freqRange[i].low + 1;  

      // decrease bass amplitude
      const int BASS_RANGE = 3;
      if (i < BASS_RANGE) { data *=  1.0f - 0.12f * (BASS_RANGE - i); }  
      
      // show
      spectrumMeter.showColume(i, data, CHSV(countLoop + i * 10, 255, 200));
    }

    FastLED.show();

    //timeLoop = (micros() - mbegin + timeLoop) / 2;  // moving average value
    if (countLoop % 100 == 0) {
      USB_SERIAL.println("runFFT cost: " + String(timeLoop) + " micro seconds");

//* log timecost of fastSqrt
      unsigned long tmCost = 0;
      for (int i = 0; i < 256; i++) {
        uint16_t numTest = random(1024);
        unsigned long tmBegin = micros();
        float root = sqrtf(numTest);
        tmCost += micros() - tmBegin;
      }
      USB_SERIAL.printf("256 times sqrt calc cost us:%u\n", tmCost);
//*/        
      USB_SERIAL.printf("timer1 = %u, timer2 = %u, timer3 = %u\n", 
        timer1.mvAverage, timer2.mvAverage, timer3.mvAverage);
    }

    countLoop++;
  }
}
