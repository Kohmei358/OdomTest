#ifndef Chassis_hpp
#define Chassis_hpp
#include "main.h"

using namespace std;

class Singleton {
   static Singleton *instance;
   shared_ptr<OdomChassisController> chassis;

   // Private constructor so that no objects can be created.
   Singleton() {
     chassis =
       ChassisControllerBuilder()
         .withMotors({-20,14,-12},{2,-6,10}) // first set is left, 2nd set is right (- means reversed)
         .withGains(
             {0.000, 0.000, 0.000},
             // {0.0004, 0.000, 0.000025}, // distance controller gains
             {0.001, 0.0002, 0.000037}, // turn controller gains
             {0.000, 0.000, 0.000}
             // {0.000135, 0.000, 0.0000}  // angle controller gains (helps drive straight)
         )
         .withSensors(
             ADIEncoder{'C', 'D', false}, // left encoder in ADI ports A & B
             ADIEncoder{'E', 'F', true},  // right encoder in ADI ports C & D (reversed)
             ADIEncoder{'A', 'B', false}  // middle encoder in ADI ports E & F
         )
         // green gearset, tracking wheel diameter (3.24 in), track (7.32 in), and TPR (256(PPR)*4)
         // 4.5 inch middle encoder distance, and 2.75 inch middle wheel diameter
         // 4.3
         .withDimensions(AbstractMotor::gearset::green, {{3.30_in, 7.20_in, 4.3_in, 2.80_in}, 1024})
         .withOdometry() // use the same scales as the chassis (above)
         .buildOdometry(); // build an odometry chassis
   }

   public:
   static Singleton *getInstance() {
      if (!instance)
      instance = new Singleton;
      return instance;
   }

   shared_ptr<OdomChassisController> getChassis() {
   return this -> chassis;
}
};


#endif
