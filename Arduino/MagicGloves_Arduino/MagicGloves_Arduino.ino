#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "TimerOne.h"
#include <math.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
#define PRINT_CALCULATED

Adafruit_BNO055 bno = Adafruit_BNO055();

LSM9DS1 imu2;

int bend_sensors[10];
float filtered_bend[10];
float filtered2_bend[10];
float filtered3_bend[10];
float a = 0.2;
int j = 0;
int k = 0;
int first_yaw = 1;
float output_bend[10];
float bend[10];
float last_bend[10];
float dbend[10];
int sample = 0;
int on = 0;
float roll,pitch,yaw,yaw_ref;
const char *labels[10] = {"L4 ", "L3 ", "L2 ", "L1 ", "L0 ", "R4 ", "R3 ", "R2 ", "R1 ", "R0 "};

byte controlPins[] = {B00000000, 
                      B00000100,
                      B00001000,
                      B00001100,
                      B00010000,
                      B00010100,
                      B00011000,
                      B00011100,
                      B00100000,
                      B00100100 };

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
  Serial.print(imu2.calcGyro(imu2.gx), 2);
  Serial.print(", ");
  Serial.print(imu2.calcGyro(imu2.gy), 2);
  Serial.print(", ");
  Serial.print(imu2.calcGyro(imu2.gz), 2);
  Serial.print(" deg/s  ");
}

void printAccel()
{  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("ax ");
  Serial.println(imu2.calcAccel(imu2.ax), 2);
  Serial.print("ay ");
  Serial.println(imu2.calcAccel(imu2.ay), 2);
  Serial.print("az ");
  Serial.println(imu2.calcAccel(imu2.az), 2);

}

void printMag()
{  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
  Serial.print(imu2.calcMag(imu2.mx), 2);
  Serial.print(", ");
  Serial.print(imu2.calcMag(imu2.my), 2);
  Serial.print(", ");
  Serial.print(imu2.calcMag(imu2.mz), 2);
  Serial.print(" gauss  ");
}



void print_BNO055()
{

	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

	// Display the floating point data 
	float roll,pitch,last_pitch,dpitch;

	roll = 5*(euler.z()/360) + 1;
	if (roll < -1)
	{
		roll = 3.3;
	}
	if (roll < 0 && roll > -0.9)
	{
		roll = 0;
	}

	pitch = (euler.y()/180) + 0.5;
	dpitch = pitch - last_pitch;
	last_pitch = pitch;

	Serial.print("roll ");
	Serial.println(roll);
	Serial.print("pitch ");
	Serial.println(dpitch);
	Serial.print("yaw ");
	Serial.println(euler.x());
}

void print_LSM9DS1()
{
	/*if ( imu2.gyroAvailable() )
    {
    	imu2.readGyro();
    }*/
  	if ( imu2.accelAvailable() )
  	{
    	imu2.readAccel();
  	}
  	/*if ( imu2.magAvailable() )
  	{
    	imu2.readMag();
  	}*/
  	//printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    //printMag();   // Print "M: mx, my, mz"
}

void callback()
{
	sample = 1;
}


void setup() {
  // put your setup code here, to run once:
	pinMode(2,OUTPUT);
	pinMode(3,OUTPUT);
	pinMode(4,OUTPUT);
	pinMode(5,OUTPUT);
	pinMode(8,OUTPUT);
	pinMode(13,OUTPUT);
	Serial.begin(115200);
	digitalWrite(2,LOW);
	digitalWrite(3,LOW);
	digitalWrite(4,LOW);
	digitalWrite(5,LOW);
	Timer1.initialize(20000); // Inicializa o Timer1 e configura para um período de 0,006 segundos
	Timer1.attachInterrupt(callback);

	imu2.settings.device.commInterface = IMU_MODE_I2C;
	imu2.settings.device.mAddress = LSM9DS1_M;
	imu2.settings.device.agAddress = LSM9DS1_AG;

		
  if (!imu2.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  
  


  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);




}

void loop() {
  // put your main code here, to run repeatedly:
	if (sample == 1)
	{	
		//digitalWrite(13,HIGH);
		for (int i = 0; i < 10; ++i)
		{
			PORTD = controlPins[i];
			bend_sensors[i] = analogRead(A0);
			filtered_bend[i] = a*bend_sensors[i] + (1 - a)*filtered_bend[i];
			//filtered2_bend[i] = a*filtered_bend[i] + (1 - a)*filtered2_bend[i];
			//filtered3_bend[i] = a*filtered_bend[i] + (1 - a)*filtered3_bend[i];
			//bend[i] = filtered_bend[i];
			//bend[i] = 0.00127*bend[i] - 0.3;
			Serial.print(labels[i]);
			Serial.println(filtered_bend[i]);

		}
		/*j++;
		if (j == 20)
		{
			for (int i = 0; i < 10; ++i)
			{
				//bend[i] = output_bend[i]/200;
				//output_bend[i] = 0;
				//dbend[i] = bend[i] - last_bend[i];
				//last_bend[i] = bend[i];
				//bend[i] = bend[i] - 2.5*dbend[i];
				//bend[i] = 0.0127*bend[i] - 0.3;
				Serial.print(labels[i]);
				Serial.println(bend[i]);
			}
			j = 0;
		}*/

		sample = 0;
		//digitalWrite(13,LOW);
	}

	/*
	sensors_event_t event;
  	bno.getEvent(&event);

  	imu::Quaternion q = bno.getQuat();
	q.normalize();
	float temp = q.x();  q.x() = -q.y();  q.y() = temp;
	q.z() = -q.z();
	imu::Vector<3> euler = q.toEuler();*/
	//Serial.print(F("Orientation: "));
	//Serial.print(-180/M_PI * euler.x());  // heading, nose-right is positive, z-axis points up
	//Serial.print(F(" "));
	//Serial.print(-180/M_PI * euler.y());  // roll, rightwing-up is positive, y-axis points forward
	//Serial.print(F(" "));
	//Serial.print(-180/M_PI * euler.z());  // pitch, nose-down is positive, x-axis points right
	//Serial.println(F(""));

	//imu::Quaternion q = bno.getQuat();
  	//q.normalize();
  	//float temp = q.x();  q.x() = -q.y();  q.y() = temp;
  	//q.z() = -q.z();

  	//Serial.print("w ");
  	//Serial.print(q.w());
  	//Serial.print("    x ");
  	//Serial.print(q.x());
  	//Serial.print("    y ");
  	//Serial.print(q.y());
  	//Serial.print("    z ");
  	//Serial.print(q.z());

  	//float temp = q.x();  q.x() = -q.y();  q.y() = temp;
  	//q.z() = -q.z();
  	//imu::Vector<3> euler = q.toEuler();

	imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

		// Display the floating point data 

	roll = 5*(euler.z()/360) + 1;
	if (roll < -1)
	{
		roll = 3.3;
	}
	if (roll < 0 && roll > -0.9)
	{
		roll = 0;
	}

	pitch = (euler.y()/180) + 0.5;

	if (first_yaw == 1)
	{
		yaw_ref = euler.x();
		first_yaw = 0;
	}
	yaw = euler.x() - yaw_ref;




	

	//dpitch = pitch - last_pitch;
	//last_pitch = pitch;
  	
	Serial.print("yaw ");
	Serial.println(yaw);
	Serial.print("roll ");
	Serial.println(roll);
	Serial.print("pitch ");
	Serial.println(pitch);
/*
	uint8_t system, gyro, accel, mag = 0;
	bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("     CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
	*/


	

	//Serial.println();
	print_LSM9DS1();
	//print_BNO055();

}
