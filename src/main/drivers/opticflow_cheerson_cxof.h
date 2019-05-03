



extern uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
extern float flowRate[2];          // optical flow angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
extern float bodyRate[2];          // body inertial angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
extern uint32_t last_opticflow_update_ms;

extern float bodyRate1[2];



void initOpticFlow();
void updateOpticFlow();
