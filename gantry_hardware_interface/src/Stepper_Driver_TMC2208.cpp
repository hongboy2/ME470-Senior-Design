#include "Stepper_Driver_TMC2208.hpp"
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>
#include <cmath>
#include <thread>
#include <chrono>

#define POSITIVE 1
#define NEGATIVE -1
#define RELATIVE 0
#define ABSOLUTE 1

using namespace std;


// Destructor
StepperMotor::~StepperMotor() {
    setEnabled(false);
    // GPIO cleanup is done automatically in the WiringPi library
}

// Get step pin
int StepperMotor::getPinStep() {
    return _pinStep;
}

// Set step pin
void StepperMotor::setPinStep(int stepPin) {
    _pinStep = stepPin;
}

// Get direction pin
int StepperMotor::getPinDir() {
    return _pinDir;
}

// Set direction pin
void StepperMotor::setPinDir(int pinDir) {
    _pinDir = pinDir;
}

// Get enable pin
int StepperMotor::getPinEn() {
    return _pinEn;
}

// Set enable pin
void StepperMotor::setPinEn(int pinEn) {
    _pinEn = pinEn;
}

// Get direction
int StepperMotor::getDirection() {
    return _direction;
}

// Set direction
void StepperMotor::setDirection(int direction) {
    _direction = direction;
    digitalWrite(_pinDir, direction);
}

// Get enabled status
bool StepperMotor::getEnabled() {
    return _enabled;
}

// Set enabled status
void StepperMotor::setEnabled(bool en) {
    _enabled = en;
}

// Get powered status
bool StepperMotor::getPowered() {
    return _powered;
}

// Set powered status
void StepperMotor::setPowered(bool powered) {
    _powered = powered;
    digitalWrite(_pinEn, !powered);
}

// Get movement mode
bool StepperMotor::getMovementMode() {
    return _movementMode;
}

// Set movement mode
void StepperMotor::setMovementMode(bool movementMode) {
    _movementMode = movementMode;
}

// Get active status
bool StepperMotor::getActive() {
    return _active;
}

// Get seeking status
bool StepperMotor::getSeeking() {
    return _seeking;
}

// Get stopped status
bool StepperMotor::getStopped() {
    return _stopped;
}

// Set stopped status
void StepperMotor::setStopped(bool stop) {
    _stopped = stop;
}

// Get pulses per step
int StepperMotor::getPulsesPerStep() {
    return _pulsesPerStep;
}

// Set pulses per step
void StepperMotor::setPulsesPerStep(int pulses) {
    _pulsesPerStep = pulses;
}

// Get steps per revolution
int StepperMotor::getStepsPerRev() {
    return _stepsPerRev;
}

// Set steps per revolution
void StepperMotor::setStepsPerRev(int steps) {
    _stepsPerRev = steps;
}

// Get pulse count (absolute position)
long StepperMotor::getPulseCountAbs() {
    return _pulseCountAbs;
}

// Set pulse count (absolute position)
void StepperMotor::setPulseCountAbs(long pos) {
    _pulseCountAbs = pos;
}

// Get revolution count (absolute position)
float StepperMotor::getRevCountAbs() {
    return _pulseCountAbs * _pulsesPerStep * _stepsPerRev;
}

// Set revolution count (absolute position)
void StepperMotor::setRevCountAbs(float count) {
    setPulseCountAbs(_pulsesPerStep * _stepsPerRev * count);
}

// Get target pulse speed
float StepperMotor::getPulseSpeedTarget() {
    return _pulseSpeedTarget;
}


StepperDriver::StepperDriver(int pinStep, int pinDir, int pinEn, int pulsesPerStep, int stepsPerRev, double accel0, double accelMax, double accelMin, double speed0, double speedMax, double speedMin)
    : _pinStep(pinStep), _pinDir(pinDir), _pinEn(pinEn), _pulsesPerStep(pulsesPerStep), _stepsPerRev(stepsPerRev),
      _pulseAcceleration(accel0), _pulseAccelerationMax(accelMax), _pulseAccelerationMin(accelMin),
      _pulsePeriod0(1.0 / speed0), _pulsePeriodMin(1.0 / speedMax), _pulsePeriodMax(1.0 / speedMin),
      _pulseCountAbs(0), _pulseCountAbsTarget(0), _pulseTimeLast(0), _pulseAccelCount(0), _pulsePeriod(-1),
      _pulsePeriodTarget(-1), _enabled(false), _active(false), _stopped(false), _movementMode(false), _direction(true),
      _seeking(false) {
    wiringPiSetupGpio();
    pinMode(_pinStep, OUTPUT);
    pinMode(_pinDir, OUTPUT);
    pinMode(_pinEn, OUTPUT);
}

bool StepperMotor::runToPulseCount(long pulses) {
    _active = true;
    _seeking = true;
    if (_movementMode == RELATIVE) {
        _pulseCountAbsTarget = round(_pulseCountAbs + pulses);
    } else {
        _pulseCountAbsTarget = round(pulses);
    }
    _pulsePeriod = 0;
    _pulseSpeed = 0.0;
    _pulseAccelCount = 0;
    computeNewSpeed();
    while (run() && !_stopped && _powered && _enabled) {
        // returns false when target position is reached
    }
    _active = false;
    _seeking = false;
    return !_stopped;
}


bool StepperMotor::runToRevCount(float rev) {
    return runToPulseCount(rev * _pulsesPerStep * _stepsPerRev);
}


bool StepperMotor::run() {
    if (runSpeed()) {
        computeNewSpeed();
    }
    return (_pulseSpeed != 0.0 && pulsesToGo() != 0);
}


long StepperMotor::pulsesToGo() {
    return _pulseCountAbsTarget - _pulseCountAbs;
}


bool StepperMotor::runSpeed() {
    if (!_pulsePeriod) {
        return false;
    }
    auto t = std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count();
    if (t - _pulseTimeLast >= _pulsePeriod) {
        if (_direction == POSITIVE) {
            _pulseCountAbs += 1;
        } else {
            _pulseCountAbs -= 1;
        }
        pulse();
        _pulseTimeLast = t;
        return true;
    } else {
        return false;
    }
}



void StepperMotor::computeNewSpeed() {
    long pulsesToGo = this->pulsesToGo();
    float pulsesToStop = (_pulseSpeed * _pulseSpeed) / (2.0 * _pulseAcceleration); // Equation 16

    if (pulsesToGo == 0 && pulsesToStop <= 1) {
        _pulsePeriod = 0;
        _pulseSpeed = 0.0;
        _pulseAccelCount = 0;
        return;
    }

    if (pulsesToGo > 0) {
        if (_pulseAccelCount > 0) {
            if ((pulsesToStop >= pulsesToGo) || _direction == NEGATIVE) {
                _pulseAccelCount = -pulsesToStop;
            }
        } else if (_pulseAccelCount < 0) {
            if ((pulsesToStop < pulsesToGo) && _direction == POSITIVE) {
                _pulseAccelCount = -_pulseAccelCount;
            }
        }
    } else if (pulsesToGo < 0) {
        if (_pulseAccelCount > 0) {
            if ((pulsesToStop >= -pulsesToGo) || _direction == POSITIVE) {
                _pulseAccelCount = -pulsesToStop;
            }
        } else if (_pulseAccelCount < 0) {
            if ((pulsesToStop < -pulsesToGo) && _direction == NEGATIVE) {
                _pulseAccelCount = -_pulseAccelCount;
            }
        }
    }

    if (_pulseAccelCount == 0) {
        _pulsePeriod = _pulsePeriod0;
        digitalWrite(_pinStep, LOW);
        if (pulsesToGo < 0) {
            setDirection(NEGATIVE);
        } else if (pulsesToGo > 0) {
            setDirection(POSITIVE);
        }
    } else {
        _pulsePeriod = _pulsePeriod - ((2.0 * _pulsePeriod) / ((4.0 * _pulseAccelCount) + 1)); // Equation 13
        _pulsePeriod = std::max(_pulsePeriod, _pulsePeriodTarget);
    }

    _pulseAccelCount += 1;
    _pulsePeriodLast = _pulsePeriod;
    _pulseSpeed = 1000000.0 / _pulsePeriod;

    if (_direction == NEGATIVE) {
        _pulseSpeed = -_pulseSpeed;
    }
}


