#ifndef Keyboard_h
#define Keyboard_h

void keyboardInit();
int getKeyPressEvent();
void keyboardInject(int event);
void handlePMUirq();

#endif
