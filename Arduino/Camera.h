
#ifndef Camera_h
#define Camera_h


#include <Servo.h>


class Camera{
    Servo Pin_D11; // Camera Head
    Servo Pin_D10;  // Low Servo
public:

    void ServoAttach(){
        Pin_D11.attach(11);
        Pin_D10.attach(10);
    }

    void Move(int Angle){
        if(Angle > 180) Angle = 180;
        if(Angle < 100) Angle = 100;
        Pin_D9.write(Angle);
    }

    void Catch(int Angle){
        if(Angle > 180) Angle = 180;
        if(Angle < 0) Angle = 0;

        Pin_D8.write(Angle);
    }

    void ServoSetUp(){
        ServoAttach();
        Move(100);
        Catch(0);
    }
};


#endif
