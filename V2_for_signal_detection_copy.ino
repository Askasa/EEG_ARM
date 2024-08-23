#include <Servo.h>

#define THRESHOLD 150

#define EMG_PIN A0
double _mn = 10000;
double _mx = 0;

#define SERVO_PIN 3

Servo SERVO_1;


const int samplingRate = 9600;  
//const float cutoffLow = 20.0;  
//const float cutoffHigh = 500.0; 

void setup() {
  Serial.begin(samplingRate);

  SERVO_1.attach(SERVO_PIN);
}

/*

float bandpassFilter(float input, int samplingRate, float cutoffLow, float cutoffHigh) {
  float dt = 1.0 / samplingRate;
  float RC_low = 1.0 / (2.0 * M_PI * cutoffLow);
  float RC_high = 1.0 / (2.0 * M_PI * cutoffHigh);
  float alpha_low = dt / (dt + RC_low);
  float alpha_high = dt / (dt + RC_high);

  static float prevOutput_low = 0.0;
  static float prevOutput_high = 0.0;

  // Lowpass filter
  float output_low = alpha_low * input + (1 - alpha_low) * prevOutput_low;
  prevOutput_low = output_low;

  // Highpass filter
  float output_high = input - output_low;
  float output = alpha_high * output_high + (1 - alpha_high) * prevOutput_high;
  prevOutput_high = output;

  return output;
}

*/

double get_value(double raw){
  if(raw > _mx) _mx = raw;
  if(raw < _mn) _mn = raw;

  raw =  (raw - _mn)/(_mx - _mn);

  raw = raw * raw * 1000 ;

  return raw;
}
void loop() {
 
  double rawValue = analogRead(EMG_PIN);
  Serial.println(rawValue);

  double output = get_value(rawValue);

  
  if (output > THRESHOLD) {
    SERVO_1.write(60);
    delay(1000);
    bool bend = 1;
    int count = 0;
    while(bend){
      for(int i=0; i<100; i++){
        double arm_state = get_value(analogRead(EMG_PIN));
        if(arm_state > THRESHOLD) count++;
        delay(10);
      }
      count = 0;
      if(count<1){
        bend = 0;
      }
    }

    //if get the value from the first time (wait for )
  }

  // If the filtered sensor is LESS than the THRESHOLD, the servo motor will turn to 10 degrees.
  else {
    SERVO_1.write(180);
  }
  
  // You can use the serial monitor to set THRESHOLD properly, comparing the values shown when you open and close your hand.
  Serial.println(output);

  delay(10);
}
