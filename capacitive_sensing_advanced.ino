#include <FastLED.h>
#include <CapacitiveSensor.h>
//#include <AnalogSmooth.h>
#include <Average.h>

/**
 * 1M resistor between pins A4 & A2
 * Pin A2 is our sensing pin, that means you can connect your capacitive material to this pin
 * add a wire and or foil if desired.
 */

CapacitiveSensor   cap = CapacitiveSensor(A4,A2);

const int pinPot = A3;
const int NUM_LEDS = 3;

float threshold; // this is the variable where we will store our trigger value
float fadeLevel;
CRGB leds[NUM_LEDS];

float EMA_a = 0.6;      //initialization of EMA alpha
int EMA_S = 0;          //initialization of EMA S

//AnalogSmooth smooth = AnalogSmooth(5);
Average<float> ave(10);
Average<float> aveLum(40);

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()                    
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinPot, INPUT);

  //FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN>(leds, NUM_LEDS);
  FastLED.addLeds<APA102, 4, 3>(leds, NUM_LEDS);
  
   //cap.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   Serial.begin(9600);
}

float read_threshold() {
  int val;
  for(int i = 0; i < 4; i++) {
    val += analogRead(pinPot);
    delay(2);
  }
  val = val >> 2;

  EMA_S = (EMA_a*val) + ((1-EMA_a)*EMA_S);
  
  return (float)EMA_S;
}

void led_proximity(float level) {
  //if(level > 1.0) return;
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(level, 0, level);
    fadeLevel = level;
    //leds[i].fadeToBlackBy( fadeLevel );
  }
}

void led_touch() {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::White;
  }
}

void led_clear() {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
}

void loop()                    
{
    long reading = 0;
    float n, dev, lum, mn;
    
    for(int i = 0; i < 4; i++) {
      reading += cap.capacitiveSensor(30);
      delay(2); // space readings 2ms each
    }
    reading = reading >> 2; // divide by four

    threshold = read_threshold();

    led_clear();
    if(reading > threshold) {
      led_touch();
    } else {
      //n = 1.0f - mapf(1.0*reading, .0, threshold, .0, 1.0 );
      ave.push(reading);
      dev = abs(ave.stddev());
      //lum = mapf(dev, 0, 10, 0, 255);
      //aveLum.push(lum);
      mn = ave.mean();
//      stat.add(n);
//      double dev = stat.pop_stdev();
      if(mn > 160.0) {
        led_proximity(mn);
      }
    }
    FastLED.show(); 

    Serial.print(reading);
    Serial.print(",");
    Serial.print(threshold);
    Serial.print(",");
    Serial.print(dev);
    Serial.print(",");
    Serial.println(mn);

    // limit the frequency rate of the loop
    delay(10); // ms = 1 / delay value
}

