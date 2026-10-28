#define TAG "Keyboard"
#include "logger.h"

#include <Arduino.h>
#include <Ticker.h>
#include "Keyboard.h"
#include "Sonde.h"
#include "pmu.h"

extern PMU *pmu;
extern uint8_t pmu_irq;
extern void toggleLedMute();

enum KeyPress { KP_NONE = 0, KP_SHORT, KP_DOUBLE, KP_MID, KP_LONG, KP_RINEX, KP_FORMAT };

// "doublepress" is now also used to eliminate key glitch on TTGO T-Beam startup (SENSOR_VN/GPIO39)
struct Button {
  uint8_t pin;
  uint32_t numberKeyPresses;
  KeyPress pressed;
  unsigned long keydownTime;
  int8_t doublepress;
  bool isTouched;
};
static Button button1 = {0, 0, KP_NONE, 0, -1, false};
static Button button2 = {0, 0, KP_NONE, 0, -1, false};

// It is not safe to call millis() in ISR!!!
// millis() does a division int64_t by 1000 for which gcc creates a library call
// on a 32bit system, and the called function has no IRAM_ATTR
// so doing it manually...
// Code adapted for 64 bits from https://www.hackersdelight.org/divcMore.pdf
static int64_t IRAM_ATTR divs10(int64_t n) {
  int64_t q, r;
  n = n + (n >> 63 & 9);
  q = (n >> 1) + (n >> 2);
  q = q + (q >> 4);
  q = q + (q >> 8);
  q = q + (q >> 16);
  q = q + (q >> 32);
  q = q >> 3;
  r = n - q * 10;
  return q + ((r + 6) >> 4);
  // return q + (r > 9);
}

static int64_t IRAM_ATTR divs1000(int64_t n) {
  return divs10(divs10(divs10(n)));
}

static unsigned long IRAM_ATTR my_millis()
{
  return divs1000(esp_timer_get_time());
}

static void checkTouchStatus();
static void touchISR();
static void touchISR2();
static void buttonISR();
static void button2ISR();

static Ticker ticker;

#define IS_TOUCH(x) (((x)!=255)&&((x)!=-1)&&((x)&128))
static void initTouch() {
  // also used for LED
  ticker.attach_ms(300, checkTouchStatus);

  if ( !(IS_TOUCH(sonde.config.button_pin) || IS_TOUCH(sonde.config.button2_pin)) ) return; // no touch buttons configured
  /*
   *  ** no. readTouch is not safe to use in ISR!
      so now using Ticker
    hw_timer_t *timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, checkTouchStatus, true);
    timerAlarmWrite(timer, 300000, true);
    timerAlarmEnable(timer);
  */

  if ( IS_TOUCH(sonde.config.button_pin) ) {
    touchAttachInterrupt(sonde.config.button_pin & 0x7f, touchISR, sonde.config.touch_thresh);
    LOG_I(TAG, "Initializing touch 1 on pin %d\n", sonde.config.button_pin & 0x7f);
  }
  if ( IS_TOUCH(sonde.config.button2_pin) ) {
    touchAttachInterrupt(sonde.config.button2_pin & 0x7f, touchISR2, sonde.config.touch_thresh);
    LOG_I(TAG, "Initializing touch 2 on pin %d\n", sonde.config.button2_pin & 0x7f);
  }
}

static void IRAM_ATTR touchISR() {
  if (!button1.isTouched) {
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) button1.doublepress = 1;
    else button1.doublepress = 0;
    button1.keydownTime = now;
    button1.isTouched = true;
  }
}

static void IRAM_ATTR touchISR2() {
  if (!button2.isTouched) {
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) button2.doublepress = 1;
    else button2.doublepress = 0;
    button2.keydownTime = now;
    button2.isTouched = true;
  }
}

// touchRead in ISR is also a bad idea. Now moved to Ticker task
static void checkTouchButton(Button & button) {
  if (button.isTouched) {
    int tmp = touchRead(button.pin & 0x7f);
    //LOG_D(TAG, "touch read %d: value is %d\n", button.pin & 0x7f, tmp);
    if (tmp > sonde.config.touch_thresh + 5) {
      button.isTouched = false;
      unsigned long elapsed = my_millis() - button.keydownTime;
      if (elapsed > 1500) {
        if (elapsed < 4000) {
          button.pressed = KP_MID;
        }
        else {
          button.pressed = KP_LONG;
        }
      } else if (button.doublepress) {
        button.pressed = KP_DOUBLE;
      } else {
        button.pressed = KP_SHORT;
      }
    }
  }
}

static void checkTouchStatus() {
  checkTouchButton(button1);
  checkTouchButton(button2);
}

static unsigned long bdd1, bdd2;
static bool b1wasdown = false;
static void IRAM_ATTR buttonISR() {
  if (digitalRead(button1.pin) == 0) { // Button down
    b1wasdown = true;
    unsigned long now = my_millis();
    if (now - button1.keydownTime < 500) {
      // Double press
      if (now - button1.keydownTime > 100)
        button1.doublepress = 1;
      bdd1 = now; bdd2 = button1.keydownTime;
    } else {
      button1.doublepress = 0;
    }
    button1.numberKeyPresses += 1;
    button1.keydownTime = now;
  } else { //Button up
    if (!b1wasdown) return;
    b1wasdown = false;
    unsigned long now = my_millis();
    if (button1.doublepress == -1) return;   // key was never pressed before, ignore button up
    unsigned int elapsed = now - button1.keydownTime;
    if (elapsed > 1500) {
      if (elapsed < 4000) {
        button1.pressed = KP_MID;
      }
      else {
        button1.pressed = KP_LONG;
      }
    } else {
      if (button1.doublepress) button1.pressed = KP_DOUBLE;
      else button1.pressed = KP_SHORT;
    }
    button1.numberKeyPresses += 1;
    button1.keydownTime = now;
  }
}

static void IRAM_ATTR button2ISR() {
  if (digitalRead(button2.pin) == 0) { // Button down
    unsigned long now = my_millis();
    if (now - button2.keydownTime < 500) {
      // Double press
      if (now - button2.keydownTime > 100)
        button2.doublepress = 1;
      //bdd1 = now; bdd2 = button1.keydownTime;
    } else {
      button2.doublepress = 0;
    }
    button2.numberKeyPresses += 1;
    button2.keydownTime = now;
  } else { //Button up
    unsigned long now = my_millis();
    if (button2.doublepress == -1) return;   // key was never pressed before, ignore button up
    unsigned int elapsed = now - button2.keydownTime;
    if (elapsed > 1500) {
      if (elapsed < 4000) {
        button2.pressed = KP_MID;
      }
      else {
        button2.pressed = KP_LONG;
      }
    } else {
      if (button2.doublepress) button2.pressed = KP_DOUBLE;
      else button2.pressed = KP_SHORT;
    }
    button2.numberKeyPresses += 1;
    button2.keydownTime = now;
  }
}

static int getKeyPress() {
  KeyPress p = button1.pressed;
  button1.pressed = KP_NONE;
#if 0
  int x = digitalRead(button1.pin);
  LOG_D(TAG, "Debug: bdd1=%ld, bdd2=%ld\n", bdd1, bdd2);
  LOG_D(TAG, "button1 press (dbl:%d) (now:%d): %d at %ld (%d)\n", button1.doublepress, x, p, button1.keydownTime, button1.numberKeyPresses);
#endif
  return p;
}

// called by arduino main loop (from Sonde::waitRXcomplete) as soon as pmu_irq is set
void handlePMUirq() {
  if (sonde.config.button2_axp) {
    // Use AXP power button as second button
    int key = pmu->handleIRQ();
    if (key > 0) {
      button2.pressed = (KeyPress)key;
      button2.keydownTime = my_millis();
    }
  } else {
    // WiFi loop intentionally calls this in order to react to PMU key press
    //Serial.println("handlePMIirq() called. THIS SHOULD NOT HAPPEN w/o button2_axp set");
    pmu_irq = 0;   // prevent main loop blocking
  }
}

static int getKey2Press() {
  // TODO: Should be atomic
  KeyPress p = button2.pressed;
  button2.pressed = KP_NONE;
  //LOG_D(TAG, "button2 press: %d at %ld (%d)\n", p, button2.keydownTime, button2.numberKeyPresses);
  return p;
}

int getKeyPressEvent() {
  int p = getKeyPress();
  if (p == KP_NONE) {
    p = getKey2Press();
    if (p == KP_NONE)
      return EVT_NONE;
    if (p == KP_RINEX)
      return EVT_RINEX;
    if (p == KP_FORMAT)
      return EVT_FORMAT;
    LOG_D(TAG, "Key 2 was pressed [%d]\n", p + 4);
    // maybe not the best place, but easy to do: check for B2 medium keypress to mute LED
    if(p == KP_MID && sonde.config.b2mute > 0) {
       toggleLedMute();
    }
    return p + 4;
  }
  LOG_D(TAG, "Key 1 was pressed [%d]\n", p);
  return p;  /* map KP_x to EVT_KEY1_x / EVT_KEY2_x*/
}

void keyboardInject(int event) {
  switch (event) {
    case EVT_KEY1SHORT:  button1.pressed = KP_SHORT; break;
    case EVT_KEY1DOUBLE: button1.pressed = KP_DOUBLE; break;
    case EVT_KEY1MID:    button1.pressed = KP_MID; break;
    case EVT_KEY1LONG:   button1.pressed = KP_LONG; break;
    case EVT_KEY2SHORT:  button2.pressed = KP_SHORT; break;
    case EVT_KEY2DOUBLE: button2.pressed = KP_DOUBLE; break;
    case EVT_KEY2MID:    button2.pressed = KP_MID; break;
    case EVT_KEY2LONG:   button2.pressed = KP_LONG; break;
    case EVT_RINEX:      button2.pressed = KP_RINEX; break;
    case EVT_FORMAT:     button2.pressed = KP_FORMAT; break;
    default: break;
  }
}

void keyboardInit() {
  button1.pin = sonde.config.button_pin;
  button2.pin = sonde.config.button2_pin;
  if (button1.pin != 0xff) {
    if ( (button1.pin & 0x80) == 0 && button1.pin < 34 ) {
      Serial.println("Button 1 configured as input with pullup");
      pinMode(button1.pin, INPUT_PULLUP);
    } else
      pinMode(button1.pin, INPUT);  // configure as input if not disabled
  }
  if (button2.pin != 0xff) {
    if ( (button2.pin & 0x80) == 0 && button2.pin < 34 ) {
      Serial.println("Button 2 configured as input with pullup");
      pinMode(button2.pin, INPUT_PULLUP);
    } else
      pinMode(button2.pin, INPUT);  // configure as input if not disabled
  }
  // Handle button press
  if ( (button1.pin & 0x80) == 0) {
    attachInterrupt( button1.pin, buttonISR, CHANGE);
    LOG_D(TAG, "button1.pin is %d, attaching interrupt\n", button1.pin);
  }
  // Handle button press
  if ( (button2.pin & 0x80) == 0) {
    attachInterrupt( button2.pin, button2ISR, CHANGE);
    LOG_D(TAG, "button2.pin is %d, attaching interrupt\n", button2.pin);
  }
  initTouch();
}
