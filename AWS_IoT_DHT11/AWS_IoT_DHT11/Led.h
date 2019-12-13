#include <Arduino.h>

#define LED_OFF 0
#define LED_ON 1

class Led {
  private:
    int pin;
    byte state;

  public:
    Led(int pin);
    void init();
    void on();
    void off();
    byte getState();
};
