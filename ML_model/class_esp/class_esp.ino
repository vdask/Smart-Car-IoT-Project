#include "arduinoFFT.h"
#include "model.h"
#include <math.h>

#define MIC 34
#define SOUND_THRESHOLD 20
#define NUM_SAMPLES 512
#define SAMPLING_FREQUENCY 44100

float backgroundSound = 0;

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];
float vRealHALF[NUM_SAMPLES/2];  // classification vector

unsigned int sampling_period;
unsigned long microseconds;

arduinoFFT fft;

// type of classifier we use
Eloquent::ML::Port::SVM classifier;

float mean_bike[256] = {39535.86, 443.87, 362.06, 337.35, 1451.15, 1746.1, 538.6, 271.91, 273.12, 290.0, 260.77, 613.8, 757.64, 727.21, 2186.32, 7762.83, 5575.11, 1003.52, 587.18, 547.08, 523.26, 421.51, 452.09, 566.5, 540.16, 340.7, 284.36, 351.12, 369.51, 347.93, 292.18, 253.67, 246.21, 275.24, 673.74, 512.04, 227.17, 273.68, 321.88, 260.52, 233.64, 306.87, 303.48, 188.46, 92.54, 218.96, 182.64, 150.45, 99.21, 73.86, 93.6, 90.53, 75.06, 64.55, 80.47, 61.21, 120.86, 101.32, 89.19, 62.61, 69.37, 96.72, 76.38, 89.78, 68.53, 64.23, 315.55, 568.61, 249.14, 64.4, 78.16, 93.57, 66.97, 36.49, 49.4, 57.44, 59.4, 51.05, 71.94, 151.13, 179.19, 90.3, 86.85, 60.9, 159.08, 199.77, 336.46, 154.8, 56.62, 53.63, 61.8, 46.58, 55.85, 47.63, 34.15, 43.59, 57.4, 70.44, 40.37, 37.47, 41.76, 43.24, 42.19, 32.72, 34.66, 44.2, 55.15, 48.89, 71.3, 34.85, 38.91, 50.49, 91.38, 138.65, 86.57, 67.48, 42.16, 65.98, 261.1, 284.01, 79.06, 45.01, 65.39, 76.97, 34.13, 39.82, 37.33, 50.54, 50.96, 82.39, 65.44, 50.94, 41.77, 79.75, 81.54, 58.64, 49.75, 190.54, 231.37, 82.1, 82.38, 63.23, 47.59, 51.54, 65.48, 46.27, 67.74, 95.85, 147.87, 98.95, 63.44, 104.68, 136.81, 124.09, 114.37, 122.24, 159.61, 139.65, 183.02, 293.71, 268.45, 187.34, 275.25, 801.84, 2468.47, 2733.4, 707.01, 326.41, 246.66, 857.32, 1934.08, 1377.83, 352.62, 214.46, 193.47, 176.93, 161.64, 132.69, 88.41, 86.03, 123.64, 119.69, 89.35, 102.96, 99.47, 138.49, 113.01, 95.49, 103.7, 155.98, 111.6, 99.39, 98.91, 104.89, 137.11, 208.6, 310.57, 648.64, 1195.37, 496.89, 151.37, 116.95, 101.12, 78.85, 56.24, 73.81, 70.74, 74.25, 98.24, 107.06, 48.62, 95.35, 93.46, 61.06, 101.77, 129.21, 179.18, 144.1, 96.97, 62.38, 51.32, 135.44, 190.67, 86.72, 69.52, 76.27, 77.05, 68.71, 76.92, 79.2, 59.72, 64.42, 89.9, 88.96, 85.01, 78.94, 116.43, 131.54, 48.81, 53.65, 105.32, 172.85, 128.95, 92.39, 64.37, 81.65, 76.39, 68.42, 45.15, 59.72, 50.84, 48.51, 43.74, 52.2, 74.54, 75.07};
float mean_car[256] = {39337.83, 107.14, 84.26, 85.23, 46.28, 43.18, 34.3, 22.47, 28.89, 26.36, 34.71, 31.85, 27.91, 19.23, 21.3, 25.2, 23.11, 29.1, 73.36, 148.28, 70.0, 67.09, 126.42, 214.33, 147.35, 165.86, 512.87, 284.43, 43.04, 156.46, 242.4, 64.59, 54.52, 36.28, 23.07, 24.79, 33.93, 43.04, 57.32, 201.22, 217.19, 154.69, 311.65, 1357.96, 1025.54, 83.86, 34.84, 32.28, 27.8, 111.35, 140.19, 37.31, 56.81, 224.8, 150.36, 32.03, 65.64, 107.39, 53.08, 29.59, 30.37, 25.7, 24.44, 44.33, 60.66, 373.76, 357.48, 56.77, 75.32, 25.35, 25.12, 23.09, 18.65, 28.27, 31.01, 30.52, 26.18, 42.0, 28.08, 131.71, 697.02, 597.5, 70.55, 45.75, 27.0, 34.1, 38.87, 246.65, 314.51, 79.18, 51.85, 27.87, 62.77, 69.57, 23.36, 27.01, 26.76, 29.85, 43.81, 25.93, 17.73, 19.38, 25.47, 59.54, 114.59, 76.5, 81.33, 731.74, 815.59, 453.83, 802.07, 246.84, 140.44, 82.19, 49.64, 42.12, 19.82, 20.09, 23.93, 20.91, 15.89, 15.93, 16.49, 20.07, 34.44, 43.2, 28.31, 36.84, 28.71, 33.02, 41.94, 170.55, 388.91, 161.27, 130.53, 168.5, 54.44, 114.8, 41.64, 26.52, 25.23, 20.52, 25.16, 21.07, 25.66, 19.98, 35.17, 24.89, 27.66, 47.7, 34.93, 47.19, 86.62, 111.62, 329.97, 203.86, 443.19, 351.99, 152.95, 93.95, 29.71, 42.02, 46.94, 23.42, 40.02, 25.88, 20.64, 31.36, 46.68, 42.55, 26.37, 33.3, 35.46, 61.61, 91.73, 148.72, 549.49, 374.29, 551.14, 604.17, 144.05, 162.32, 86.87, 153.99, 132.47, 589.82, 459.81, 67.71, 67.67, 82.32, 48.54, 81.68, 49.14, 38.32, 39.88, 45.19, 56.01, 36.97, 126.23, 129.12, 141.27, 180.42, 96.8, 192.81, 86.17, 62.62, 99.27, 86.64, 143.11, 75.16, 58.83, 53.96, 152.62, 133.48, 81.4, 121.14, 169.08, 95.09, 101.34, 67.18, 198.69, 240.11, 281.52, 507.95, 196.5, 185.69, 117.51, 48.52, 41.22, 48.65, 63.4, 56.66, 44.56, 57.08, 63.1, 92.69, 72.61, 96.79, 99.22, 523.53, 621.57, 205.82, 629.71, 1005.23, 315.56, 400.91, 265.88, 90.03, 79.87, 94.44, 134.64, 67.97, 147.4, 124.02, 67.35, 50.07};

void setup() {
    Serial.begin(57600);
    pinMode(MIC, INPUT);
    sampling_period = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    calibrate();
}

void loop() {
    if (!soundDetected()) {
        delay(10);
        return;
    }

    captureWord();
    classify();
    delay(2000);
}

int16_t readMic() {
    // this translated the analog value to a proper interval
    return  (analogRead(MIC) - NUM_SAMPLES) >> 2;
}

void calibrate() {
    for (int i = 0; i < 10000; i++)
        backgroundSound += readMic();

    backgroundSound /= 10000;
}

bool soundDetected() {
    return abs(readMic() - backgroundSound) >= SOUND_THRESHOLD;
}

void captureWord() {
    for (uint16_t i = 0; i < NUM_SAMPLES; i++) {
        microseconds = micros();    //Overflows after around 70 minutes!
      
        vReal[i] = readMic();
        vImag[i] = 0;
        
        while (micros() < (microseconds + sampling_period)) {
        }
    }
    
    fft.Windowing(vReal, NUM_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    fft.Compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
    fft.ComplexToMagnitude(vReal, vImag, NUM_SAMPLES);

    for (int i = 1; i <= NUM_SAMPLES / 2; i++) //except for the half and the first element
    {
        //Convert feature vectors to float values, since the classification model takes float as an argument
        vRealHALF[i - 1] = static_cast<float>(vReal[i]);
    }
}

void classify() {
    

    float similarity_bike = cosineSimilarity(vRealHALF, mean_bike);
    float similarity_car = cosineSimilarity(vRealHALF, mean_car);

    if((similarity_bike > 0.8) || (similarity_car > 0.80)){
      Serial.print("Predicted class: ");
      Serial.println(classifier.predictLabel(vRealHALF));
      Serial.print("Bike similarity : ");
      Serial.println(similarity_bike);
      Serial.print("Car similarity: ");
      Serial.println(similarity_car);
      Serial1.println("============================================= ");
    }

    else{
      Serial.print("There are no bikes or cars around.");
      Serial.println(classifier.predictLabel(vRealHALF));
      Serial.print("Bike similarity : ");
      Serial.println(similarity_bike);
      Serial.print("Car similarity: ");
      Serial.println(similarity_car);
      Serial1.println("============================================= ");
    }
}

float dotProduct(float v1[], float v2[]) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES / 2; i++) {
    sum += v1[i] * v2[i];
  }
  return sum;
}

float magnitude(float v[]) {
  float sum = 0;
  for (int i = 0; i < NUM_SAMPLES / 2; i++) {
    sum += v[i] * v[i];
  }
  return sqrt(sum);
}

float cosineSimilarity(float v1[], float v2[]) {
  return dotProduct(v1, v2) / (magnitude(v1) * magnitude(v2));
}
