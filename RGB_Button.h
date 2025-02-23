#ifndef RGB_BUTTON_H
#define RGB_BUTTON_H

#include <Arduino.h>


class RGB_Button {
  private:
  const int redPin;
  const int greenPin;
  const int bluePin;
  const int buttonPin;
  bool button_state = false;
  int color[3] = {0, 0, 0};
  int current_color[3] = {0, 0, 0};

  // You could use actual grounds instead of custom grounds for the two pins below.
  int gnd = 42; // pseudo ground needed for my push button
  const int gnd_pin = 47; // RGB ground

  public:
  RGB_Button(int R, int G, int B, int button);
  void init_RGBbutton();
  void setColor(int R, int G, int B);
  void setColor(const int* rgb);
  void getColor(int &R, int &G, int &B);
  void getColor(int rgb[3]);
  void getCurrentColor(int &R, int &G, int &B);
  void getCurrentColor(int rgb[3]);
  void runColor(int R, int G, int B);
  void runColor(const int* rgb);
  void runColor();
  bool buttonState();
  
};





#endif
