// Do not remove the include below
#include "PlutoPilot.h"
#include "Utils.h"
#include "Sensor.h"
#include "Control.h"
#include "User.h"
#include "API/Hardware/Led.h"
#include "API/Debug/Print.h"
#include "Peripheral.h"

//Timer test;

//The setup function is called once at Pluto's hardware startup
void plutoInit()
{
// Add your hardware initialization code here
    ADC.init(Pin19);
}



//The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart()
{
//  // do your one time stuffs here
//  LED.flightStatus(DEACTIVATE);
//  LED.set(BLUE, ON);
////ledOp(L_LEFT, ON);
//
//
//    test.reset();



}




// The loop function is called in an endless loop
void plutoLoop()
{

//Add your repeated code here

//LED.set(GREEN, ON);
//  ledOp(L_RIGHT, ON);
//Print.monitor("DScds");
//Print.monitor("\n");
//Print.monitor("X:",3);
//Print.monitor(" Y:",4);
//Print.monitor(" Z:",5);
//Print.monitor("\n");


//
//    if(test.set(200, true)){
//
//    Monitor.println("Gyro: ");
//    Monitor.print("X:",Gyroscope.get(X));
//    Monitor.print(" Y:",Gyroscope.get(Y));
//    Monitor.println(" Z:",Gyroscope.get(Z));
//
//
//    Monitor.println("Acc: ");
//    Monitor.print("X:",Acceleration.get(X));
//    Monitor.print(" Y:",Acceleration.get(Y));
//    Monitor.println(" Z:",Acceleration.get(Z));
//
//
//    Monitor.println("Mag: ");
//    Monitor.print("X:",Magnetometer.get(X));
//    Monitor.print(" Y:",Magnetometer.get(Y));
//    Monitor.println(" Z:",Magnetometer.get(Z));
//
//
//    Monitor.println("Baro: ");
//    Monitor.print("P:",Barometer.get(PRESSURE));
//    Monitor.println(" T:",Barometer.get(TEMPERATURE));
//
//
//
//
//
//
//    }
//    Gyroscope.get(X);
//    Gyroscope.get(Y);
//    Gyroscope.get(Z);
//
//
//
//    Graph.red(Gyroscope.get(X));
//    Graph.green(Gyroscope.get(Y));
//    Graph.blue(Gyroscope.get(Z));


//if(Acceleration.getNetAcc()<2&&(!FlightStatus.check(FS_CRASHED))){
//
//      arm();
//    LED.set(RED, ON);
//    LED.set(GREEN, ON);
//
//}

    Monitor.println("ADC: ",ADC.read(Pin19));

}



//The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish()
{

// do your cleanup stuffs here
//    LED.flightStatus(ACTIVATE);

}




