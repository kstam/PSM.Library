#ifndef PSM_h
#define PSM_h

#include "cstdint"
#include "vector"
#include "Arduino.h"


class PsmControlBase
{
public:
  virtual void calculateSkip() = 0;
  virtual void updateControl(bool forceDisable = false) = 0;
};

class PsmPinControl : public PsmControlBase
{
public:
  PsmPinControl(uint8_t controlPin, uint16_t range, uint8_t divider = 1);
  uint32_t getCounter();
  void resetCounter();
  void setValue(uint16_t value);
  void setDivider(uint8_t divider);

  void calculateSkip() override;
  void updateControl(bool forceDisable = false) override;

private:
  uint8_t _controlPin;
  uint16_t _value;
  uint16_t _range;
  uint32_t _stopAfter;

  uint8_t _divider;
  uint8_t _dividerCounter = 1;

  uint16_t _a = 0;
  uint32_t _counter = 0;
  bool _skip = true;
};

class PSM
{
public:
  PSM(uint8_t sensePin, int mode = RISING, uint8_t interruptMinTimeDiff = 0);
  void initTimer(uint16_t delay, TIM_TypeDef* timerInstance = TIM1);
  uint16_t getFrequency(void);
  void addControl(PsmControlBase* control) {
    // Check if control pointer is already in the vector and ignore
    for (auto* c : _controls) {
      if (c == control) {
        return;
      }
    }
    _controls.push_back(control);
  }

private:
  std::vector<PsmControlBase*> _controls;

  static inline void onZCInterrupt(void);
  static inline void calculateSkipFromZC(void);
  static inline void onPSMTimerInterrupt(void);
  void calculateSkipForControls(void);
  void updateControls(bool forceDisable = true);

  unsigned char _sensePin;
  unsigned char _interruptMinTimeDiff;
  unsigned long _lastMillis = 0;
  bool _psmIntervalTimerInitialized = false;
  HardwareTimer* _psmIntervalTimer;
};

extern PSM* _thePSM;

#endif
