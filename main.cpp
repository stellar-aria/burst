#include "burst.h"
#include "digital_input.h"

using namespace befaco::burst::pins;
TriggerInput cv_trigger(TRIGGER_STATE, true);

int main() {
  OUT_LED.Init(Pin::Mode::OUTPUT);
  TRIGGER_STATE.Init(Pin::Mode::INPUT_PULLUP);
  //cv_trigger.Init();

  while(true) {
    cv_trigger.Read();
    
    OUT_LED.Write(cv_trigger.high());
  }
}