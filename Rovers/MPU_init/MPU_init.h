#include "helper_3dmath.h"

class MPU_init 
{    
	private:
	bool dmpReady; // set true if DMP init was successful
	uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
	uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount; // count of all bytes currently in FIFO

	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	float yawOffset, pitchOffset, rollOffset;

	public:
	volatile bool mpuInterrupt;
	//void dmpDataReady();
	bool mpuReadingReady();
	void init();
	void getAngles(float* angles);
	void calibrate();
};
