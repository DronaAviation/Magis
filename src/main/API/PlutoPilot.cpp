// Do not remove the include below
#include "PlutoPilot.h"

void rxconfig(void)
{
  //To setup your Reciever input use this function 
  //RX.InputStatus(PPM_IP);
   //RX.InputStatus(MSP_IP);
}
//The setup function is called once at Pluto's hardware startup
void plutoInit(void)
{
// Add your hardware initialization code here

}



//The function is called once before plutoLoop when you activate Developer Mode
void onLoopStart(void)
{
// do your one time stuffs here



}




// The loop function is called in an endless loop
void plutoLoop(void)
{

//Add your repeated code here


}



//The function is called once after plutoLoop when you deactivate Developer Mode
void onLoopFinish(void)
{

// do your cleanup stuffs here


}




