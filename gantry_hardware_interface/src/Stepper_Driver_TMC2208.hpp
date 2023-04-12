#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP

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

class StepperMotor {
public:
    StepperMotor(int pinStep, int pinDir, int pinEn, int pulsesPerStep, int stepsPerRev, double accel0, double accelMax, double accelMin, double speed0, double speedMax, double speedMin);
    ~StepperMotor();

    // Getters
    int getPinStep() const;
    int getPinDir() const;
    int getPinEn() const;
    int getDirection() const;
    bool getEnabled() const;
    bool getPowered() const;
    bool getMovementMode() const;
    bool getActive() const;
    bool getSeeking() const;
    bool getStopped() const;
    int getPulsesPerStep() const;
    int getStepsPerRev() const;
    long getPulseCountAbs() const;
    float getRevCountAbs() const;
    float getPulseSpeedTarget() const;

    // Setters
    void setPinStep(int stepPin);
    void setPinDir(int pinDir);
    void setPinEn(int pinEn);
    void setDirection(int direction);
    void setEnabled(bool enabled);
    void setPowered(bool powered);
    void setMovementMode(bool movementMode);
    void setStopped(bool stop);
    void setPulsesPerStep(int pulses);
    void setStepsPerRev(int steps);
    void setPulseCountAbs(long pos);
    void setRevCountAbs(float count);

    // Methods
    bool runToPulseCount(long pulses);
    bool runToRevCount(float rev);
    bool run();
    long pulsesToGo();
    void computeNewSpeed();

private:
    int _pinStep;
    int _pinDir;
    int _pinEn;
    int _pulsesPerStep;
    int _stepsPerRev;
    double _pulseAcceleration;
    double _pulseAccelerationMax;
    double _pulseAccelerationMin;
    double _pulsePeriod0;
    double _pulsePeriodMin;
    double _pulsePeriodMax;
    double _pulseSpeed;
    double _pulseSpeedTarget;
    int _pulseCountAbs;
    int _pulseCountAbsTarget;
    long _pulseTimeLast;
    int _pulseAccelCount;
    double _pulsePeriod;
    double _pulsePeriodTarget;
    bool _enabled;
    bool _powered;
    bool _movementMode;
    bool _active;
    bool _seeking;

};

#endif // STEPPER_MOTOR_HPP
