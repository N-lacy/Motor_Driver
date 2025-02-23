#include "RGB_Button.h"

RGB_Button::RGB_Button(int R, int G, int B, int button)
  : redPin(R), greenPin(G), bluePin(B), buttonPin(button) {}

void RGB_Button::init_RGBbutton() {
  
  pinMode(buttonPin, INPUT_PULLUP);
  
 //RGB LED set as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  pinMode(gnd, OUTPUT);
  digitalWrite(gnd, LOW);
  
  pinMode(gnd_pin, OUTPUT);
  digitalWrite(gnd_pin, LOW);

  runColor((const int[]){50,100,50});

}


void RGB_Button::setColor(const int* rgb) {
  color[0] = rgb[0];
  color[1] = rgb[1];
  color[2] = rgb[2];
  
}


void RGB_Button::setColor(int R, int G, int B) {
  color[0] = R;
  color[1] = G;
  color[2] = B;
  
}


void RGB_Button::runColor() {
  analogWrite(redPin, current_color[0] = color[0]);
  analogWrite(greenPin, current_color[1] = color[1]);
  analogWrite(bluePin, current_color[2] = color[2]);
  
}


void RGB_Button::runColor(const int* rgb) {
  color[0] = rgb[0];
  color[1] = rgb[1];
  color[2] = rgb[2];
  analogWrite(redPin, current_color[0] = color[0]);
  analogWrite(greenPin, current_color[1] = color[1]);
  analogWrite(bluePin, current_color[2] = color[2]);
  
}


void RGB_Button::runColor(int R, int G, int B) {
  color[0] = R;
  color[1] = G;
  color[2] = B;
  analogWrite(redPin, current_color[0] = color[0]);
  analogWrite(greenPin, current_color[1] = color[1]);
  analogWrite(bluePin, current_color[2] = color[2]);
  
}


void RGB_Button::getColor(int rgb[3]) {
  for (int i = 0; i < 3; i++)
    rgb[i] = color[i];
    
}


void RGB_Button::getColor(int &R, int &G, int &B) {
  R = color[0];
  G = color[1];
  B = color[2];
  
}


void RGB_Button::getCurrentColor(int rgb[3]) {
  for (int i = 0; i < 3; i++)
    rgb[i] = current_color[i];
    
}


void RGB_Button::getCurrentColor(int &R, int &G, int &B) {
  R = current_color[0];
  G = current_color[1];
  B = current_color[2];
  
}


bool RGB_Button::buttonState() {
  return !digitalRead(buttonPin);
}
