
#ifndef Camera_h
#define Camera_h


#include <Servo.h>


class Camera{
    Servo Pin_D11; // Head servo
    Servo Pin_D10;  // Bottom servo
public:

    void ServoAttach(){
        Pin_D11.attach(11);
        Pin_D10.attach(10);
    }

    void Pitch(int Angle){  // Turn up and Down
        if(Angle > 180) Angle = 180;
        if(Angle < 40) Angle = 40;
        Pin_D11.write(Angle);
    }

    void Yaw(int Angle){  // Turn Left or Right
        if(Angle > 180) Angle = 180;
        if(Angle < 0) Angle = 0;
        Pin_D10.write(Angle);
    }
    
    void Default(){
        Pin_D11.write(83);
        Pin_D10.write(100);
    }
    
    void CameraServoSetUp(){
        ServoAttach();
        Default();
        Catch(0);
    }
};


#endif
