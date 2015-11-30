//editing on 
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <MPU_init.h>

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_ACCEL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

MPU6050 mpu;

/*void MPU_init::dmpDataReady() {
	mpuInterrupt = true;
}*/

bool MPU_init::mpuReadingReady(){
	return !(!mpuInterrupt && fifoCount < packetSize);
}

void MPU_init::init(){
	mpuInterrupt = false;
	dmpReady = false; // set true if DMP init was successful
	
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	yawOffset = 0; pitchOffset = 0; rollOffset = 0;

	mpu.initialize(); // initialize device

	//PC (should be remove when implementing the code without connecting to the PC)
	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	// wait for ready
	Serial.println(F("\nSend any character to begin DMP programming and demo: "));
	/*while (Serial.available() && Serial.read()); // empty buffer
	while (!Serial.available());                 // wait for data
	while (Serial.available() && Serial.read()); // empty buffer again
    	Serial.println(F("Initializing DMP...")); // load and configure the DMP
	////*/
	
	// load and configure the DMP
	devStatus = mpu.dmpInitialize();
    
	// supply your own gyro offsets here, scaled for min sensitivity
	//mpu.setXGyroOffset(7);
	//mpu.setYGyroOffset(60);
	//mpu.setZGyroOffset(-22);
	//mpu.setXAccelOffset(1550);
	//mpu.setYAccelOffset(1140);
	//mpu.setZAccelOffset(1470); 
    mpu.setXAccelOffset(-1137);
    	mpu.setYAccelOffset(-2570);
    	mpu.setZAccelOffset(922); //16384
	mpu.setXGyroOffset(69);
	mpu.setYGyroOffset(-40);
	mpu.setZGyroOffset(-3);//1688 factory default for my test chip
    
	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		//PC
		Serial.print("Enabling DMP...\n");
		////*/
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		//PC
		Serial.print("Enabling interrupt detection (Arduino external interrupt 0)...\n");
		////*/
		//attachInterrupt(0, dmpDataReady, RISING); declared in the main body of the code
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		//PC
		Serial.print("DMP ready! Waiting for first interrupt...\n");
		////*/
		dmpReady = true;
		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
        } 

	//PC
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
	////*/
}


void MPU_init::getAngles(float* angles)
{
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
    
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //PC
        Serial.println("FIFO overflow!");
        ////

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {

        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

	#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		ypr[0] -= yawOffset; ypr[1] -= pitchOffset; ypr[2] -= rollOffset;
		*(angles) = ypr[0];
		*(angles + 1) = ypr[1];
		*(angles + 2) = ypr[2];
		/*PC
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180/M_PI);
		//*/
	#endif

	/*#ifdef OUTPUT_READABLE_ACCEL
		// display real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);

		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		//PC
		Serial.print("a\t");
		Serial.print(aa.x);
		Serial.print("\t");
		Serial.print(aa.y);
		Serial.print("\t");
		Serial.println(aa.z);
		////
	#endif*/

	/*#ifdef OUTPUT_READABLE_REALACCEL
		// display real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		//PC
		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.print(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);
		////
	#endif*/

	/*#ifdef OUTPUT_READABLE_WORLDACCEL
		// display initial world-frame acceleration, adjusted to remove gravity
		// and rotated based on known orientation from quaternion
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		//PC
		Serial.print("aworld\t");
		Serial.print(aaWorld.x);
		Serial.print("\t");
		Serial.print(aaWorld.y);
		Serial.print("\t");
		Serial.println(aaWorld.z);
		////
	#endif*/

    }
}

void MPU_init::calibrate(){
	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	float old_angles[3], new_angles[3];
	getAngles(&old_angles[0]);
	for (int i = 0; i < 40; i++){
		getAngles(&new_angles[0]);
		if (!((new_angles[0] - old_angles[0]) < 0.004 && (new_angles[1] - old_angles[1]) < 0.004 && (new_angles[2] - old_angles[2]) < 0.004))
			i = 0;
		    
		for (int j = 0; j < 3; j ++) old_angles[j] = new_angles[j];
		delay(100);
	}
	//PC
	Serial.print("calibrated\n\n\n");
	////*/
	yawOffset = new_angles[0]; pitchOffset = new_angles[1]; rollOffset = new_angles[2];
}
