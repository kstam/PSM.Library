#include "PSM.h"

PSM* _thePSM;

PSM::PSM(uint8_t sensePin, int mode, uint8_t interruptMinTimeDiff) : _sensePin(sensePin), _interruptMinTimeDiff(interruptMinTimeDiff) {
  _thePSM = this;

  pinMode(sensePin, INPUT_PULLUP);

  uint32_t interruptNum = digitalPinToInterrupt(PSM::_sensePin);
  if (interruptNum != NOT_AN_INTERRUPT) {
    attachInterrupt(interruptNum, onZCInterrupt, mode);
  }
}

void PSM::onZCInterrupt(void) {
  if (_thePSM->_interruptMinTimeDiff > 0 && millis() - _thePSM->_interruptMinTimeDiff < _thePSM->_lastMillis) {
    if (millis() >= _thePSM->_lastMillis) {
      return;
    }
  }

  _thePSM->_lastMillis = millis();

  _thePSM->calculateSkipFromZC();

  if (_thePSM->_psmIntervalTimerInitialized) {
    _thePSM->_psmIntervalTimer->setCount(0);
    _thePSM->_psmIntervalTimer->resume();
  }
}

void PSM::onPSMTimerInterrupt(void) {
  _thePSM->_psmIntervalTimer->pause();
  _thePSM->updateControls(true);
}

void PSM::calculateSkipFromZC(void) {
  _thePSM->calculateSkipForControls();
  _thePSM->updateControls(false);
}

void PSM::calculateSkipForControls(void) {
  for (auto* control : _thePSM->_controls) {
    control->calculateSkip();
  }
}

void PSM::updateControls(bool forceDisable) {
  for (auto* control : _thePSM->_controls) {
    control->updateControl(forceDisable);
  }
}

// We just use this to count the ZC frequency
class PsmDummyControl : public PsmControlBase
{
public:
  uint32_t counter = 0;
  PsmDummyControl() {}
  void calculateSkip() override { counter++; }
  void updateControl(bool forceDisable = false) override {}
};

uint16_t PSM::getFrequency(void) {
  std::vector<PsmControlBase*> controlsBackup = _controls;

  auto* dummyControl = new PsmDummyControl();
  _controls = { dummyControl };// TODO don't define output for random pin

  uint32_t stopAt = millis() + 1000;

  while (millis() < stopAt) {
    delay(0);
  }

  uint16_t result = dummyControl->counter;
  _controls = controlsBackup;
  
  delete dummyControl;
  return result;
}

void PSM::initTimer(uint16_t delay, TIM_TypeDef* timerInstance) {
  uint32_t us = delay > 1000u ? delay : delay > 55u ? 5500u : 6600u;

  PSM::_psmIntervalTimer = new HardwareTimer(timerInstance);
  PSM::_psmIntervalTimer->setOverflow(us, MICROSEC_FORMAT);
  PSM::_psmIntervalTimer->setInterruptPriority(0, 0);
  PSM::_psmIntervalTimer->attachInterrupt(onPSMTimerInterrupt);

  PSM::_psmIntervalTimerInitialized = true;
}

// PSMControl
// One per pin where we apply PSM. 
// Example: One for the pump, one for the heater, etc.

PsmPinControl::PsmPinControl(uint8_t controlPin, uint16_t range, uint8_t divider) : _controlPin(controlPin), _range(range), _divider(divider) {
  pinMode(_controlPin, OUTPUT);
  digitalWrite(_controlPin, LOW);
  _a = 0;
}

uint32_t PsmPinControl::getCounter() { return _counter; }
void PsmPinControl::resetCounter() { _counter = 0; }
void PsmPinControl::setValue(uint16_t value) { _value = (value < _range) ? value : _range; }
void PsmPinControl::setDivider(uint8_t divider) { _divider = divider; }

void PsmPinControl::calculateSkip(void) {
  if (_dividerCounter < _divider - 1) {
    _dividerCounter++;
    return;
  }
  _dividerCounter -= _divider - 1;

  _a += _value;

  if (_a >= _range) {
    _a -= _range;
    _skip = false;
  }
  else {
    _skip = true;
  }

  if (_a > _range) {
    _a = 0;
    _skip = false;
  }

  if (!_skip) {
    _counter++;
  }

  if (!_skip
    && _stopAfter > 0
    && _counter > _stopAfter) {
    _skip = true;
  }
}

void PsmPinControl::updateControl(bool forceDisable) {
  if (forceDisable || _skip) {
    digitalWrite(_controlPin, LOW);
  }
  else {
    digitalWrite(_controlPin, HIGH);
  }
}
