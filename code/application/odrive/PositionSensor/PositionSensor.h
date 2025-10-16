#ifndef POSITIONSENSOR_H
#define POSITIONSENSOR_H

class PositionSensor {
public:
    virtual void Sample(float dt) =0;

    virtual float GetMechPosition() {
        return 0.0f;
    }

    virtual float GetMechPositionFixed() {
        return 0.0f;
    }

    virtual float GetElecPosition() {
        return 0.0f;
    }

    virtual float GetMechVelocity() {
        return 0.0f;
    }

    virtual float GetElecVelocity() {
        return 0.0f;
    }

    virtual void ZeroPosition(void) =0;

    virtual int GetRawPosition(void) =0;

    virtual void SetElecOffset(float offset) =0;

    virtual float GetElecOffset() =0;;


    virtual int GetCPR(void) =0;

    virtual void WriteLUT(int new_lut[128]) = 0;
};


class PositionSensorAM5047 : public

        PositionSensor {
public:
    PositionSensorAM5047(int CPR, float offset, int ppairs);

    virtual void Sample(float dt);

    virtual float GetMechPosition();

    virtual float GetMechPositionFixed();

    virtual float GetElecPosition();

    virtual float GetMechVelocity();

    virtual float GetElecVelocity();

    virtual int GetRawPosition();

    virtual void ZeroPosition();

    virtual void SetElecOffset(float offset);

    virtual float GetElecOffset() { return ElecOffset; };

    virtual void SetMechOffset(float offset);

    virtual int GetCPR(void);

    virtual void WriteLUT(int new_lut[128]);

private:
    float position, ElecPosition, ElecOffset, MechPosition, MechOffset, modPosition, oldModPosition, oldVel, velVec[40],
            MechVelocity, ElecVelocity, ElecVelocityFilt;
    int raw, _CPR, rotations, old_counts, _ppairs;
    // SPI *spi;
    // DigitalOut *cs;
    int readAngleCmd;
    int offset_lut[128];
};

#endif
