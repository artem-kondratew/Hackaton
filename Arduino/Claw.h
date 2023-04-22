
#ifndef Claw_h
#define Claw_h


#include <Servo.h>


Servo Pin_D9; // Claw mover
Servo Pin_D8;  // Claw


class Claw {
public:

    static void ServoAttach(){
        Pin_D9.attach(9);
        Pin_D8.attach(8);
    }

    static void moveClaw(int Angle){  // Pitch is TANGAZH(for Artem Kondratev)
        if(Angle > 180) Angle = 180;
        if(Angle < 100) Angle = 100;
        Pin_D9.write(Angle);
    }

    static void push(int Angle){
        if(Angle > 180) Angle = 180;
        if(Angle < 0) Angle = 0;

        Pin_D8.write(Angle);
    }


    static void pop() {
        Pin_D8.write(180);
    }

    static void Default(){
       moveClaw(100);
        push(0);
    }
    
    static void init(){
        ServoAttach();
        Default();
    }

    static void drop() {
        moveClaw(0);
    }


    static void rise() {
        moveClaw(180);
    }
};


#endif
