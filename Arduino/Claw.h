
#ifndef Claw_h
#define Claw_h


#include <Servo.h>


class Claw{
    Servo Pin_D9; // Claw mover
    Servo Pin_D8;  // Claw
public:

    void ServoAttach(){
        Pin_D9.attach(9);
        Pin_D8.attach(8);
    }

    void Pitch(int Angle){  // Pitch is TANGAZH(for Artem Kondratev)
        if(Angle > 180) Angle = 180;
        if(Angle < 100) Angle = 100;
        Pin_D9.write(Angle);
    }

    void Catch(int Angle){
        if(Angle > 180) Angle = 180;
        if(Angle < 0) Angle = 0;

        Pin_D8.write(Angle);
    }

    void Default(){
       Pitch(100);
        Catch(0);
    }
    
    void ServoSetUp(){
        ServoAttach();
        Default();
    }
};


#endif
