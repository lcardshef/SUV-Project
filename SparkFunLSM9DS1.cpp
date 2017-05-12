/****************************************************************************** SFE_LSM9DS1.cpp SFE_LSM9DS1 Library Source File Jim Lindblom @ SparkFun Electronics Original Creation Date: February 27, 2015 
https://github.com/sparkfun/LSM9DS1_Breakout

This file implements all functions of the LSM9DS1 class. Functions here range
from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
	IDE: Arduino 1.6
	Hardware Platform: Arduino Uno
	LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM9DS1.h"
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include <wiringPi.h>  // SPI library is used for...SPI.
#include <time.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <errno.h>
#include <pthread.h>
#include <iostream>
#include <math.h>

float ori,angz=0, angy=0, angx=0;
int _fd;
int command = 0;
int finish =0;
LSM9DS1 imu;
typedef unsigned long long timestamp_t;

#define LSM9DS1_M	0x1E
#define	LSM9DS1_AG	0x6B
#define PI 3.14159265

#define PRINT_CALCULATED
#define PRINT_SPEED 250
#define DECLINATION -8.58

// Sensor Sensitivity Constants
// Values set according to the typical specifications provided in
// table 3 of the LSM9DS1 datasheet. (pg 12)
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058

void AddMatrices(float firstMatrix[3][1], float secondMatrix[3][1], float mult[3][1]);
void multiplyMatrices33(float firstMatrix[3][3], float secondMatrix[3][3], float multResult[3][3]);
void multiplyMatrices31(float firstMatrix[3][3], float secondMatrix[3][1], float multResult[3][1]);
void *accel_averages(void*);
void *orientation(void*);
static timestamp_t get_timestamp ();
void *distance(void*);

void print_array(int i,int j,float A[][1])
{
    printf("\n");
    int m,n;
    for (m=0;m<i;m++)
        {
            for (n=0;n<j;n++)
            {
                printf("A[%d,%d]=%f\n",m,n,A[m][n]);
            }
        }
}

float mag_direction(){
    	
    	float mag_direction;
    	while(!imu.magAvailable()) ;
            imu.readMag();
        float x = imu.calcMag(imu.mx);
        float y = imu.calcMag(imu.my);
        float z = imu.calcMag(imu.mz);
    	
    	float M[3][3] = {1.274, 0.025, -0.031, 0.092, 1.284, 0.107, 0.011, -0.142, 1.31};
    	float B[3] = {0.393,0.151, -0.251}; 
    	float part1[3][1]  = { x - B[0], y - B[1], z - B[2]};
    	float result[3][1];
    	multiplyMatrices31(M, part1, result);    
    	
    	if(result[1][0]>0)
    		mag_direction = 90 - (atan(result[0][0]/result[1][0]))*180/PI;
    	else if (result[1][0]<0)
    		mag_direction = 270 - (atan(result[0][0]/result[1][0]))*180/PI;
    	else if ((result[1][0]=0) && (result[0][0]<0))
    		mag_direction = 180;
    	else if ((result[1][0]=0) && (result[0][0]>0))
    		mag_direction = 0;
    
    	return mag_direction;
    }


void *Kalman(void*){
    
    int i;
    double ti=0.01;
    float gx[9], gy[9], gz[9];
	gx[0] = 0;
	gy[0] = 0;
	gz[0] = 0;
	float v =0;
	float pos =0;
	timestamp_t t0 = get_timestamp();
 	for (i = 1; finish < 1 ; i++)
    	{
		if(i ==9 )
		{	gz[0] = gz [8];
			i = 1;
                }
        
        //Obtain values of Angluar Velocity and Position of the Z-axis        
	    while(!imu.gyroAvailable());
            imu.readGyro();
	    gz[i]=imu.calcGyro(imu.gz);
		while(!imu.magAvailable()) ;
            imu.readMag()
        pos = mag_direction(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        
		timestamp_t t1 = get_timestamp();
		ti = (t1 - t0) / 1000000.0L;
		t += ti;
		t0 = 0;
		t1 = 0;
		t0 = get_timestamp();
		
		if(abs(gz[i])>1){
                        v = gz[i]; 
                        //angz += (0.5 * (gz[i-1] + gz[i])*ti)*(PI/180);
                        }
            else v =0;
	
	    float measnoise = 10; // position measurement noise (feet)
    float accelnoise = 0.2; // acceleration noise (feet/sec^2)

    float Fx[2][2] = {1, ti, 0, 1}; // transition matrix
    //B = [dt^2/2; dt]; // input matrix - Relationship between input variable and state
    //c = [1 0]; // measurement matrix
    float x[2] = [pos; v]; // initial state vector
    float xhat [2] = {x[0], x[1]}; // initial state estimate
    
    float Sz = measnoise^2; // measurement error covariance
    float Sw [2][2] = {accelnoise^2 *(ti^3)/3, accelnoise^2 *(ti^2)/2, accelnoise^2 *(ti^2)/2, accelnoise^2 *ti}; // process noise cov
    float P[2][2] = {Sw[0][0], Sw[0][1], Sw[1][0], Sw[1][1]}; // initial estimation covariance

    // Use a constant commanded acceleration of 1 foot/sec^2.
    //u = 1;
    // Simulate the linear system.
    //float ProcessNoise [2] = accelnoise * [(dt^2/2)*randn; dt*randn];
    float temp1 = Fx[0][0] * x[0] + Fx[0][1]* x[0];
    float temp2 = Fx[1][0] * x[1] + Fx[1][1]* x[1];
    x[0] = temp1;
    x[1] = temp2;
     //+ Sw; //b * u + 
    // Simulate the noisy measurement
    float MeasNoise = measnoise * randn;
    y = c * x + MeasNoise;
    // Extrapolate the most recent state estimate to the present time.
    xhat = a * xhat + b * u;
    // Form the Innovation vector.
    Inn = y - c * xhat;
    // Compute the covariance of the Innovation.
    s = c * P * c' + Sz;
    // Form the Kalman Gain matrix.
    K = a * P * c' * inv(s);
    // Update the state estimate.
    xhat = xhat + K * Inn;
    // Compute the covariance of the estimation error.
    P = a * P * a' - a * P * c' * inv(s) * c * P * a' + Sw;
    // Save some parameters for plotting later.
    
        }
    }

void *other(void*){
	FILE *a = fopen("GravityCompandCal90degreeturn.txt", "w");
	if(a ==NULL){
		printf("Error opening file\n");
		exit(1);
		}
		
	float expx, expy;
	float gx[9], gy[9], gz[9];
	gx[0] = 0;
	gy[0] = 0;
	gz[0] = 0;
	float ax[9], ay[9], az[9];
 	float vx[9], vy[9], vz[9];
	float cutoff = 0.05;
 	int i = 0;
	float t =0;
	//clock_t start, end;
 	double ti = 0.01;
	float sx = 0, sy=0, sz=0;
 	ax[0]=0;
 	ay[0]=0;
 	az[0]=0;
 	vx[0]=0;
 	vy[0]=0;
 	vz[0]=0;
	//v = u + at;
	//s = 0.5*(u + v)*t
	//start= clock();    
	timestamp_t t0 = get_timestamp();
 	for (i = 1; finish < 1 ; i++)
    	{
		if(i ==9 )
		{	vx[0] = vx [8];
			vy[0] = vy [8];
			vz[0] = vz [8];
			gx[0] = gx [8];
			gy[0] = gy [8];
			gz[0] = gz [8];
			i = 1;
                }
                while(!imu.accelAvailable());
                	imu.readAccel();
                ax[i]=imu.actualAccel(imu.ax, imu.avex);
                ay[i]=imu.actualAccel(imu.ay, imu.avey);
                az[i]=imu.actualAccel(imu.az, imu.avez);
	       	    while(!imu.gyroAvailable());
                	imu.readGyro();
                gx[i]=imu.calcGyro(imu.gx);
                gy[i]=imu.calcGyro(imu.gy);
	        gz[i]=imu.calcGyro(imu.gz);
		//end = clock();
		//ti = ((double)(end- start))/ CLOCKS_PER_SEC;
		
		timestamp_t t1 = get_timestamp();
		ti = (t1 - t0) / 1000000.0L;
		t += ti;
		t0 = 0;
		t1 = 0;
		t0 = get_timestamp();
		

		if(abs(gx[i])>1){
                       angx += (0.5 * (gx[i-1] + gx[i])*ti)*(PI/180);
                        }
                if(abs(gy[i])>1){
                        angy += (0.5 * (gy[i-1] + gy[i])*ti)*(PI/180);
                        }
		if(abs(gz[i])>1){
                        angz += (0.5 * (gz[i-1] + gz[i])*ti)*(PI/180);
                        }
                        
        float Ta[3][3] = {1.0000, 0, 0, -0.5756, 1.0000, 0, -0.8007, 0.7518, 1.0000};
        float Sa[3][3] = {0.1474, 0, 0, 0, 1.0345, 0, 0, 0, 0.7518};
        float ba[3][1] = { -4.2412, 0.1597, -0.5691};
        float yaba[3][1] = {imu.calcAccel(imu.ax) - ba[0][0], imu.calcAccel(imu.ay) - ba[1][0] , imu.calcAccel(imu.az) - ba[2][0]};
        float TaSa[3][3];
        multiplyMatrices33(Ta, Sa, TaSa);
        float acc [3][1];//=  {imu.calcAccel(imu.ax) , imu.calcAccel(imu.ay) , imu.calcAccel(imu.az)};
        multiplyMatrices31(TaSa, yaba, acc);       
        
        float cbr2 [3][3];
        
        float ave [3][1] = {-1*imu.avex, -1*imu.avey, -1*imu.avez};
        float part1 [3][1];
        float part2 [3][1];
        float result [3][1];
        float Cbr [3][3] = 
	        {cos(angy)*cos(angz), -sin(angz)*cos(angx) + sin(angx)*sin(angy)*cos(angz), sin(angx)*sin(angz) + cos(angx)*sin(angy)*cos(angz),
       		cos(angy)*sin(angz),  cos(angz)*cos(angx) + sin(angx)*sin(angy)*sin(angz),  -sin(angx)*cos(angz) + cos(angx)*sin(angy)*sin(angz),
       		-sin(angy), cos(angy)*sin(angx), cos(angx)*cos(angy)};
        
        multiplyMatrices33(Cbr, Cbr, cbr2);
        multiplyMatrices31(Cbr, acc, part1);
        multiplyMatrices31(cbr2, ave, part2);
        AddMatrices(part1, part2, result);
//	print_array(3,1, part1);
//	print_array(3,1, part2);               

              /*  expy =-sin(angx*PI/180);
                expx = sin(angy*PI/180);*/
                float otherx = result[0][0];//ax[i] - expx;
                float othery = result[1][0]; // ay[i] - expy;
		float otherz = result[2][0];
		
		vx[i]=0;
		if(fabs(otherx)> cutoff){
//			printf("\nI'm in the loop");
			vx[i] = vx[i-1] + ax[i]*ti;
                	sx+= 0.5 * (vx[i-1] + vx[i])*ti;
			}
		vy[i] = 0;
        	if(fabs(othery)> cutoff){
//			printf("\n I'm in the loop");
			vy[i] = vy[i-1] + ay[i]*ti;
                	sy+= 0.5 * (vy[i-1] + vy[i])*ti;
			}
		vz[i] = 0;
		if(fabs(otherz)> cutoff){
//			printf("I'm in the loop");
			vz[i] = vz[i-1] + az[i]*ti;
                	sz+= 0.5 * (vz[i-1] + vz[i])*ti;
			}     
                delay(500);
		
//		printf("Angle x:%f, Angle y: %f, Angle z: %f\n", angx, angy, angz);
                //printf("\nAAA: X:%d, Y:%d, Z:%d", abs(ax[i]), abs(ay[i]), abs(az[i]));
	printf("\nAcceleration X:%f Y:%f  Z:%f", ax[i], ay[i], az[i]);
              printf("\nComped Acceleration X:%f Y:%f  Z:%f", otherx, othery, otherz);
	//	printf("\nExpected Acceleration X:%f  Y:%f", expx, expy);
		 //printf("\nVelocity X:%f  Y:%f  Z:%f", vx[i], vy[i], vz[i]);
	        fprintf(a,"%f\t%f\t%f\t", t, result[0][0], angx);
 	         fprintf(a,"%f\t%f\t%f\n", result[1][0], angy, result[2][0]);
//	printf("\nGravity Values: X:%f Y:%f Z:%f", avex, avey, avez);
//       	printf("\nReal Acceleration X:%f Y:%f  Z:%f", result[0][0], result[1][0], result[2][0]); 	 
//		 fprintf(a ,"%f\t%f\t%f\t%f\t",t, otherx, vx[i], sx);
//		 fprintf(a ,"%f\t%f\t%f\t", othery, vy[i], sy);
//
//		 fprintf(a ,"%f\t%f\t%f\n", otherz, vz[i], sz);
		
	
//		printf("\nDistance X:%f  Y:%f  Z:%f", sx, sy, sz); 
      //        end = time(NULL);
//                ti = difftime(end,begin);
          //      begin = time(NULL);
        }
	
	fclose(a);
   	pthread_exit(NULL);

}		

int main(void){
	char a[20];
	
	

	imu.settings.device.commInterface = IMU_MODE_I2C;
	imu.settings.device.mAddress = LSM9DS1_M;
	imu.settings.device.agAddress = LSM9DS1_AG;
	imu.settings.accel.scale = 2;
	
	if(!imu.begin())
	{
	printf("\nFauled to communicate with LSM9DS1.");
	while(1);
	}
	
	pthread_t threads[1];
	int rc;

	rc = pthread_create(&threads[0], NULL, accel_averages, NULL);

	if(rc){
		printf("Error: Unable to create thread");
		exit(-1);
	}
	
	std::cin >> a;//the poin that way point instructions are entered
//	printf("Accel:%f",imu.avex);
	command =1;
	//printf("Accel:%f",imu.avex);

	//rc = pthread_create(&threads[1], NULL, other , NULL);
	if(rc){
		printf("Error:Unable to create thread");
		exit(-1);
		}
	std::cin >> a;
	
	finish = 1;
	
	
	for (;;) {
        while (!imu.gyroAvailable()) ;
        imu.readGyro();
        while(!imu.accelAvailable()) ;
        imu.readAccel();


	//printf("\nData 1: %f \t Data 2: %f", imu.calcAccel(imu.ax), imu.avex);	
	//printf("\nData 1 - Data 2 = %f", imu.actualAccel(imu.ax, imu.avex));	
        while(!imu.magAvailable()) ;
        imu.readMag();

		
//	printf("Actual Accel: %f, %f, %f, [Gs]\n", imu.actualAccel(imu.ax, imu.avex), imu.actualAccel(imu.ay, imu.avey),imu.actualAccel(imu.az, imu.avez));

  //      printf("Gyro: %f, %f, %f [deg/s]\n", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
       // printf("Accel: %f, %f, %f [Gs]\n", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
     // printf("Mag: %f, %f, %f [gauss]\n", imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
     printf("Mag_Direction: %f",mag_direction(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz))); 

		
	 sleep(1.0);
   }
	
	
}

void *calibration(void*){
	FILE *f = fopen("zaxis1.txt", "w");
	if(f ==NULL){
		printf("Error opening file\n");
		exit(1);
		}
	int a, i;
 	for (i = 1; i < 9 ; i++)
    	{
                printf("Loop %d", i);
		        std::cin >> a;
                while(!imu.accelAvailable());
                imu.readAccel();
	            fprintf(f, "\n%f\t%f\t%f", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
	            
    }

	
    	fclose(f);
       	pthread_exit(NULL);

}		


void *distance(void*){
	FILE *f = fopen("file.txt", "w");
	if(f ==NULL){
		printf("Error opening file\n");
		exit(1);
		}
	float ax[9], ay[9], az[9];
 	float vx[9], vy[9], vz[9];
	float cutoff = 0.05;
 	int i = 0;
	float t =0;
	//clock_t start, end;
 	double ti = 0.01;
	float sx = 0, sy=0, sz=0;
 	ax[0]=0;
 	ay[0]=0;
 	az[0]=0;
 	vx[0]=0;
 	vy[0]=0;
 	vz[0]=0;
	//v = u + at;
	//s = 0.5*(u + v)*t
	//start= clock();    
	timestamp_t t0 = get_timestamp();
 	for (i = 1; finish < 1 ; i++)
    	{
		if(i ==9 )
		{	vx[0] = vx [8];
			vy[0] = vy [8];
			vz[0] = vz [8];
			i = 1;
                }
                while(!imu.accelAvailable());
                imu.readAccel();
                ax[i]=imu.actualAccel(imu.ax, imu.avex);
                ay[i]=imu.actualAccel(imu.ay, imu.avey);
                az[i]=imu.actualAccel(imu.az, imu.avez);
                vx[i] = 0;
		
		//end = clock();
		//ti = ((double)(end- start))/ CLOCKS_PER_SEC;
		
		timestamp_t t1 = get_timestamp();
		ti = (t1 - t0) / 1000000.0L;
		t += ti;
		t0 = 0;
		t1 = 0;
		t0 = get_timestamp();
	
		if(fabs(ax[i])> cutoff){
//			printf("\nI'm in the loop");
			vx[i] = vx[i-1] + ax[i]*ti;
                	sx+= 0.5 * (vx[i-1] + vx[i])*ti;
			}
		vy[i] = 0;
        	if(fabs(ay[i])> cutoff){
//			printf("\n I'm in the loop");
			vy[i] = vy[i-1] + ay[i]*ti;
                	sy+= 0.5 * (vy[i-1] + vy[i])*ti;
			}
		vz[i] = 0;
		if(fabs(az[i])> cutoff){
//			printf("I'm in the loop");
			vz[i] = vz[i-1] + az[i]*ti;
                	sz-= 0.5 * (vz[i-1] + vz[i])*ti;
			}
                delay(100);
  


                //printf("\nAAA: X:%d, Y:%d, Z:%d", abs(ax[i]), abs(ay[i]), abs(az[i]));
	printf("\nActual Acceleration X:%f Y:%f  Z:%f", ax[i], ay[i], az[i]);
                printf("\nVelocity X:%f  Y:%f  Z:%f", vx[i], vy[i], vz[i]);
	         // fprintf(f ,"%f\t%f\t%f\t%f\t", ax[i], vx[i], sx, t);
 	         //fprintf(f ,"%f\t%f\t%f\t", ay[i], vy[i], sy);
        	 //fprintf(f ,"%f\t%f\t%f\n", az[i], vz[i], sz);
		
	
		printf("\nDistance X:%f  Y:%f  Z:%f", sx, sy, sz); 
      //        end = time(NULL);
//                ti = difftime(end,begin);
          //      begin = time(NULL);
        }
	
	fclose(f);
       	pthread_exit(NULL);

}		

static timestamp_t get_timestamp ()
    {
      struct timeval now;
      gettimeofday (&now, NULL);
      return  now.tv_usec + (timestamp_t)now.tv_sec * 1000000;
    }

void *accel_averages(void*){
	int i;
	for(i=0, imu.avex=0, imu.avey=0, imu.avez=0; i<100 ; i++){
		if(command==1)
			pthread_exit(NULL);
		else
		while (!imu.accelAvailable());
		imu.readAccel();
		if(abs(imu.calcAccel(imu.ax))<1);{
		imu.sumx += imu.calcAccel(imu.ax);
		imu.avex = imu.sumx/(i+1);
		imu.sumy += imu.calcAccel(imu.ay);
                imu.avey = imu.sumy/(i+1);
		imu.sumz += imu.calcAccel(imu.az);
                imu.avez = imu.sumz/(i+1);
		}
	}
	return 0;
}

void *orientation(void*){
	float g[9];
	g[0] = 0;
	int i;
	float ti = 0.1;
	for (i = 1; finish < 1 ; i++)
        {
                if(i ==9 )
                {       g[0] = g [8];
                        i = 1;
 		       }
                while(!imu.gyroAvailable());
                imu.readGyro();
                g[i]=imu.calcGyro(imu.gz);
                if(abs(g[i])>1){
			ori += (0.5 * (g[i-1] + g[i])*ti)*(PI/180);
                	}
		sleep(0.1);
                //printf("\nAcceleration X:%f \t\t Y:%f \t\t Z:%f", ax[i], ay[i], az[i]);
                //printf("\nVelocity X:%f \t\t Y:%f \t\t Z:%f", vx[i], vy[i], vz[i]);
                printf("\nOrientation is:%f\t and the time is:%f", ori, ti);
	/*	end = time(NULL);
                ti = difftime(end,begin);
                begin = time(NULL);*/
        }
        pthread_exit(NULL);

}

float LSM9DS1::actualAccel(int16_t accel, float ave)
{       float value;
        value = aRes*accel;
        return (value - ave);     
}

void AddMatrices(float firstMatrix[3][1], float secondMatrix[3][1], float mult[3][1])
{ 
    int i, j;
    
    for(i=0; i<3; ++i){
        for(j=0; j<1; ++j)
        {
            mult[i][j] = firstMatrix[i][j] + secondMatrix[i][j]; 
        }
    }
}


void multiplyMatrices33(float firstMatrix[3][3], float secondMatrix[3][3], float mult[3][3])
{
	int i, j, k;
	
	for(i = 0; i < 3; ++i)
	{
		for(j = 0; j < 3; ++j)
		{
			mult[i][j] = 0;
		}
	}


	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i < 3; ++i)
	{
		for(j = 0; j < 3; ++j)
		{
			for(k=0; k<3; ++k)
			{
				mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}

void multiplyMatrices31(float firstMatrix[3][3], float secondMatrix[3][1], float mult[3][1])
{
	int i, j, k;

	for(i = 0; i < 3; ++i)
	{
		for(j = 0; j < 1; ++j)
		{
			mult[i][j] = 0;
		}
	}



	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 1; j++)
		{
			for(k=0; k<3; k++)
			{
				mult[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
			}
		}
	}
}


LSM9DS1::LSM9DS1()
{
	init(IMU_MODE_I2C, LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1));
}

LSM9DS1::LSM9DS1(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	init(interface, xgAddr, mAddr);
}

void LSM9DS1::init(interface_mode interface, uint8_t xgAddr, uint8_t mAddr)
{
	settings.device.commInterface = interface;
	settings.device.agAddress = xgAddr;
	settings.device.mAddress = mAddr;

	settings.gyro.enabled = true;
	settings.gyro.enableX = true;
	settings.gyro.enableY = true;
	settings.gyro.enableZ = true;
	// gyro scale can be 245, 500, or 2000
	settings.gyro.scale = 245;
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	settings.gyro.sampleRate = 6;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	settings.gyro.bandwidth = 0;
	settings.gyro.lowPowerEnable = false;
	settings.gyro.HPFEnable = false;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	settings.gyro.HPFCutoff = 0;
	settings.gyro.flipX = false;
	settings.gyro.flipY = false;
	settings.gyro.flipZ = false;
	settings.gyro.orientation = 0;
	settings.gyro.latchInterrupt = true;

	settings.accel.enabled = true;
	settings.accel.enableX = true;
	settings.accel.enableY = true;
	settings.accel.enableZ = true;
	// accel scale can be 2, 4, 8, or 16
	settings.accel.scale = 2;
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	settings.accel.sampleRate = 6;
	// Accel cutoff freqeuncy can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	settings.accel.bandwidth = -1;
	settings.accel.highResEnable = false;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	settings.accel.highResBandwidth = 0;

	settings.mag.enabled = true;
	// mag scale can be 4, 8, 12, or 16
	settings.mag.scale = 4;
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	settings.mag.sampleRate = 7;
	settings.mag.tempCompensationEnable = false;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	settings.mag.XYPerformance = 3;
	settings.mag.ZPerformance = 3;
	settings.mag.lowPowerEnable = false;
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	settings.mag.operatingMode = 0;

	settings.temp.enabled = true;
	for (int i=0; i<3; i++)
	{
		gBias[i] = 0;
		aBias[i] = 0;
		mBias[i] = 0;
		gBiasRaw[i] = 0;
		aBiasRaw[i] = 0;
		mBiasRaw[i] = 0;
	}
	_autoCalc = false;
}


uint16_t LSM9DS1::begin()
{
	//! Todo: don't use _xgAddress or _mAddress, duplicating memory
	_xgAddress = settings.device.agAddress;
	_mAddress = settings.device.mAddress;
	
	constrainScales();
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable
	
	// Now, initialize our hardware interface.
	if (settings.device.commInterface == IMU_MODE_I2C)	// If we're using I2C
		initI2C();	// Initialize I2C
	else if (settings.device.commInterface == IMU_MODE_SPI) 	// else, if we're using SPI
		initSPI();	// Initialize SPI
		
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8) | mTest;
	
	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
		return 0;
	
	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	
	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	
	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

void LSM9DS1::initGyro()
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection
	
	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (settings.gyro.enabled)
	{
		tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
	}
	switch (settings.gyro.scale)
	{
		case 500:
			tempRegValue |= (0x1 << 3);
			break;
		case 2000:
			tempRegValue |= (0x3 << 3);
			break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	tempRegValue |= (settings.gyro.bandwidth & 0x3);
	xgWriteByte(CTRL_REG1_G, tempRegValue);
	
	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	xgWriteByte(CTRL_REG2_G, 0x00);	
	
	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (settings.gyro.HPFCutoff & 0x0F);
	}
	xgWriteByte(CTRL_REG3_G, tempRegValue);
	
	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (settings.gyro.enableY) tempRegValue |= (1<<4);
	if (settings.gyro.enableX) tempRegValue |= (1<<3);
	if (settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	xgWriteByte(CTRL_REG4, tempRegValue);
	
	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (settings.gyro.flipX) tempRegValue |= (1<<5);
	if (settings.gyro.flipY) tempRegValue |= (1<<4);
	if (settings.gyro.flipZ) tempRegValue |= (1<<3);
	xgWriteByte(ORIENT_CFG_G, tempRegValue);
}

void LSM9DS1::initAccel()
{
	uint8_t tempRegValue = 0;
	
	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);
	
	xgWriteByte(CTRL_REG5_XL, tempRegValue);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;
	}
	switch (settings.accel.scale)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	if (settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (settings.accel.bandwidth & 0x03);
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void LSM9DS1::calibrate(bool autoCalc)
{  
	uint8_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = {0, 0, 0};
	int32_t gBiasRawTemp[3] = {0, 0, 0};
	
	// Turn on FIFO and set threshold to 32 samples
	enableFIFO(true);
	setFIFO(FIFO_THS, 0x1F);
	while (samples < 0x1F)
	{
		samples = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
	}
	for(ii = 0; ii < samples ; ii++) 
	{	// Read the gyro data stored in the FIFO
		readGyro();
		gBiasRawTemp[0] += gx;
		gBiasRawTemp[1] += gy;
		gBiasRawTemp[2] += gz;
		readAccel();
		aBiasRawTemp[0] += ax;
		aBiasRawTemp[1] += ay;
		aBiasRawTemp[2] += az - (int16_t)(1./aRes); // Assumes sensor facing up!
	}  
	for (ii = 0; ii < 3; ii++)
	{
		gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		gBias[ii] = calcGyro(gBiasRaw[ii]);
		aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		aBias[ii] = calcAccel(aBiasRaw[ii]);
	}
	
	enableFIFO(false);
	setFIFO(FIFO_OFF, 0x00);
	
	if (autoCalc) _autoCalc = true;
}

void LSM9DS1::calibrateMag(bool loadIn)
{
	int i, j;
	int16_t magMin[3] = {0, 0, 0};
	int16_t magMax[3] = {0, 0, 0}; // The road warrior
	
	for (i=0; i<128; i++)
	{
		while (!magAvailable())
			;
		readMag();
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = mx;		
		magTemp[1] = my;
		magTemp[2] = mz;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
	}
	for (j = 0; j < 3; j++)
	{
		mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		mBias[j] = calcMag(mBiasRaw[j]);
		if (loadIn)
			magOffset(j, mBiasRaw[j]);
	}
	
}
void LSM9DS1::magOffset(uint8_t axis, int16_t offset)
{
	if (axis > 2)
		return;
	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;
	mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
	mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
}

void LSM9DS1::initMag()
{
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
	mWriteByte(CTRL_REG1_M, tempRegValue);
	
	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (settings.mag.scale)
	{
	case 8:
		tempRegValue |= (0x1 << 5);
		break;
	case 12:
		tempRegValue |= (0x2 << 5);
		break;
	case 16:
		tempRegValue |= (0x3 << 5);
		break;
	// Otherwise we'll default to 4 gauss (00)
	}
	mWriteByte(CTRL_REG2_M, tempRegValue); // +/-4Gauss
	
	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (settings.mag.operatingMode & 0x3);
	mWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode
	
	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
	mWriteByte(CTRL_REG4_M, tempRegValue);
	
	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	mWriteByte(CTRL_REG5_M, tempRegValue);
}

uint8_t LSM9DS1::accelAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return (status & (1<<0));
}

uint8_t LSM9DS1::gyroAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1<<1)) >> 1);
}

uint8_t LSM9DS1::tempAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1<<2)) >> 2);
}

uint8_t LSM9DS1::magAvailable(lsm9ds1_axis axis)
{
	uint8_t status;
	status = mReadByte(STATUS_REG_M);
	
	return ((status & (1<<axis)) >> axis);
}

void LSM9DS1::readAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
	if ( xgReadBytes(OUT_X_L_XL, temp, 6) == 6 ) // Read 6 bytes, beginning at OUT_X_L_XL
	{
		ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
		ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
		az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
		if (_autoCalc)
		{
			ax -= aBiasRaw[X_AXIS];
			ay -= aBiasRaw[Y_AXIS];
			az -= aBiasRaw[Z_AXIS];
		}
	}
}

int16_t LSM9DS1::readAccel(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;
	if ( xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2) == 2)
	{
		value = (temp[1] << 8) | temp[0];
		
		if (_autoCalc)
			value -= aBiasRaw[axis];
		
		return value;
	}
	return 0;
}

void LSM9DS1::readMag()
{
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	if ( mReadBytes(OUT_X_L_M, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_M
	{
		mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
		my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
		mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
	}
}

int16_t LSM9DS1::readMag(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	if ( mReadBytes(OUT_X_L_M + (2 * axis), temp, 2) == 2)
	{
		return (temp[1] << 8) | temp[0];
	}
	return 0;
}

void LSM9DS1::readTemp()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	if ( xgReadBytes(OUT_TEMP_L, temp, 2) == 2 ) // Read 2 bytes, beginning at OUT_TEMP_L
	{
		temperature = ((int16_t)temp[1] << 8) | temp[0];
	}
}

void LSM9DS1::readGyro()
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp
	if ( xgReadBytes(OUT_X_L_G, temp, 6) == 6) // Read 6 bytes, beginning at OUT_X_L_G
	{
		gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
		gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
		gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
		if (_autoCalc)
		{
			gx -= gBiasRaw[X_AXIS];
			gy -= gBiasRaw[Y_AXIS];
			gz -= gBiasRaw[Z_AXIS];
		}
	}
}

int16_t LSM9DS1::readGyro(lsm9ds1_axis axis)
{
	uint8_t temp[2];
	int16_t value;
	
	if ( xgReadBytes(OUT_X_L_G + (2 * axis), temp, 2) == 2)
	{
		value = (temp[1] << 8) | temp[0];
		
		if (_autoCalc)
			value -= gBiasRaw[axis];
		
		return value;
	}
	return 0;
}

float LSM9DS1::calcGyro(int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro; 
}

float LSM9DS1::calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float LSM9DS1::calcMag(int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void LSM9DS1::setGyroScale(uint16_t gScl)
{
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = xgReadByte(CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
		case 500:
			ctrl1RegValue |= (0x1 << 3);
			settings.gyro.scale = 500;
			break;
		case 2000:
			ctrl1RegValue |= (0x3 << 3);
			settings.gyro.scale = 2000;
			break;
		default: // Otherwise we'll set it to 245 dps (0x0 << 4)
			settings.gyro.scale = 245;
			break;
	}
	xgWriteByte(CTRL_REG1_G, ctrl1RegValue);
	
	calcgRes();	
}

void LSM9DS1::setAccelScale(uint8_t aScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;
	
	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			settings.accel.scale = 2;
			break;
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
}

void LSM9DS1::setMagScale(uint8_t mScl)
{
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	
	switch (mScl)
	{
	case 8:
		temp |= (0x1 << 5);
		settings.mag.scale = 8;
		break;
	case 12:
		temp |= (0x2 << 5);
		settings.mag.scale = 12;
		break;
	case 16:
		temp |= (0x3 << 5);
		settings.mag.scale = 16;
		break;
	default: // Otherwise we'll default to 4 gauss (00)
		settings.mag.scale = 4;
		break;
	}	
	
	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	//mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
}

void LSM9DS1::setGyroODR(uint8_t gRate)
{
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		settings.gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		xgWriteByte(CTRL_REG1_G, temp);
	}
}

void LSM9DS1::setAccelODR(uint8_t aRate)
{
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		xgWriteByte(CTRL_REG6_XL, temp);
	}
}

void LSM9DS1::setMagODR(uint8_t mRate)
{
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	settings.mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
}

void LSM9DS1::calcgRes()
{
	switch (settings.gyro.scale)
	{
	case 245:
		gRes = SENSITIVITY_GYROSCOPE_245;
		break;
	case 500:
		gRes = SENSITIVITY_GYROSCOPE_500;
		break;
	case 2000:
		gRes = SENSITIVITY_GYROSCOPE_2000;
		break;
	default:
		break;
	}
}

void LSM9DS1::calcaRes()
{
	switch (settings.accel.scale)
	{
	case 2:
		aRes = SENSITIVITY_ACCELEROMETER_2;
		break;
	case 4:
		aRes = SENSITIVITY_ACCELEROMETER_4;
		break;
	case 8:
		aRes = SENSITIVITY_ACCELEROMETER_8;
		break;
	case 16:
		aRes = SENSITIVITY_ACCELEROMETER_16;
		break;
	default:
		break;
	}
}

void LSM9DS1::calcmRes()
{
	switch (settings.mag.scale)
	{
	case 4:
		mRes = SENSITIVITY_MAGNETOMETER_4;
		break;
	case 8:
		mRes = SENSITIVITY_MAGNETOMETER_8;
		break;
	case 12:
		mRes = SENSITIVITY_MAGNETOMETER_12;
		break;
	case 16:
		mRes = SENSITIVITY_MAGNETOMETER_16;
		break;
	}	
}

void LSM9DS1::configInt(interrupt_select interrupt, uint8_t generator,
	                     h_lactive activeLow, pp_od pushPull)
{
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	xgWriteByte(interrupt, generator);
	
	// Configure CTRL_REG8
	uint8_t temp;
	temp = xgReadByte(CTRL_REG8);
	
	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);
	
	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);
	
	xgWriteByte(CTRL_REG8, temp);
}

void LSM9DS1::configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn)
{
	uint8_t temp = 0;
	
	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	xgWriteByte(ACT_THS, temp);
	
	xgWriteByte(ACT_DUR, duration);
}

uint8_t LSM9DS1::getInactivity()
{
	uint8_t temp = xgReadByte(STATUS_REG_0);
	temp &= (0x10);
	return temp;
}

void LSM9DS1::configAccelInt(uint8_t generator, bool andInterrupts)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	xgWriteByte(INT_GEN_CFG_XL, temp);
}

void LSM9DS1::configAccelThs(uint8_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_XL, temp);
}

uint8_t LSM9DS1::getAccelIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_XL);
	
	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void LSM9DS1::configGyroInt(uint8_t generator, bool aoi, bool latch)
{
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	xgWriteByte(INT_GEN_CFG_G, temp);
}

void LSM9DS1::configGyroThs(int16_t threshold, lsm9ds1_axis axis, uint8_t duration, bool wait)
{
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	xgWriteByte(INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_G, temp);
}

uint8_t LSM9DS1::getGyroIntSrc()
{
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_G);
	
	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
}

void LSM9DS1::configMagInt(uint8_t generator, h_lactive activeLow, bool latch)
{
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);	
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);
	
	mWriteByte(INT_CFG_M, config);
}

void LSM9DS1::configMagThs(uint16_t threshold)
{
	// Write high eight bits of [threshold] to INT_THS_H_M
	mWriteByte(INT_THS_H_M, uint8_t((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	mWriteByte(INT_THS_L_M, uint8_t(threshold & 0x00FF));
}

uint8_t LSM9DS1::getMagIntSrc()
{
	uint8_t intSrc = mReadByte(INT_SRC_M);
	
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}
	
	return 0;
}

void LSM9DS1::sleepGyro(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::enableFIFO(bool enable)
{
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	xgWriteByte(CTRL_REG9, temp);
}

void LSM9DS1::setFIFO(fifoMode_type fifoMode, uint8_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
	uint8_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
}

uint8_t LSM9DS1::getFIFOSamples()
{
	return (xgReadByte(FIFO_SRC) & 0x3F);
}

void LSM9DS1::constrainScales()
{
	if ((settings.gyro.scale != 245) && (settings.gyro.scale != 500) && 
		(settings.gyro.scale != 2000))
	{
		settings.gyro.scale = 245;
	}
		
	if ((settings.accel.scale != 2) && (settings.accel.scale != 4) &&
		(settings.accel.scale != 8) && (settings.accel.scale != 16))
	{
		settings.accel.scale = 2;
	}
		
	if ((settings.mag.scale != 4) && (settings.mag.scale != 8) &&
		(settings.mag.scale != 12) && (settings.mag.scale != 16))
	{
		settings.mag.scale = 4;
	}
}

void LSM9DS1::xgWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		I2CwriteByte(_xgAddress, subAddress, data);
}

void LSM9DS1::mWriteByte(uint8_t subAddress, uint8_t data)
{
	// Whether we're using I2C or SPI, write a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		 I2CwriteByte(_mAddress, subAddress, data);
}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(_xgAddress, subAddress);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return 0;
	else return 0;
}

uint8_t LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// gyro-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadBytes(_xgAddress, subAddress, dest, count);
	else return 0;
}

uint8_t LSM9DS1::mReadByte(uint8_t subAddress)
{
	// Whether we're using I2C or SPI, read a byte using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadByte(_mAddress, subAddress);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return 0;
	else return 0;
}

uint8_t LSM9DS1::mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	// Whether we're using I2C or SPI, read multiple bytes using the
	// accelerometer-specific I2C address or SPI CS pin.
	if (settings.device.commInterface == IMU_MODE_I2C)
		return I2CreadBytes(_mAddress, subAddress, dest, count);
	else if (settings.device.commInterface == IMU_MODE_SPI)
		return 0;
	else return 0;
}

void LSM9DS1::initSPI()
{
}

void LSM9DS1::SPIwriteByte(uint8_t csPin, uint8_t subAddress, uint8_t data){
}

uint8_t LSM9DS1::SPIreadByte(uint8_t csPin, uint8_t subAddress)
{
	uint8_t temp;
	// Use the multiple read function to read 1 byte. 
	// Value is returned to `temp`.
	SPIreadBytes(csPin, subAddress, &temp, 1);
	return temp;
}

uint8_t LSM9DS1::SPIreadBytes(uint8_t csPin, uint8_t subAddress,
							uint8_t * dest, uint8_t count)
{
	// To indicate a read, set bit 0 (msb) of first byte to 1
	uint8_t rAddress = 0x80 | (subAddress & 0x3F);
	// Mag SPI port is different. If we're reading multiple bytes, 
	// set bit 1 to 1. The remaining six bytes are the address to be read
	if ((csPin == _mAddress) && count > 1)
		rAddress |= 0x40;

	return 0;

}

void LSM9DS1::initI2C()
{
}

// Wire.h read and write protocols
void LSM9DS1::I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    _fd = wiringPiI2CSetup(address);
    if (_fd < 0) {
        fprintf(stderr, "Error: I2CSetup failed\n");
        exit(EXIT_FAILURE);
    }
    wiringPiI2CWriteReg8(_fd, subAddress, data);
    close(_fd);
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
    _fd = wiringPiI2CSetup(address);
    if (_fd < 0) {
        fprintf(stderr, "Error: I2CSetup failed\n");
        exit(EXIT_FAILURE);
    }
    uint8_t data; // `data` will store the register data
    wiringPiI2CWrite(_fd, subAddress);
    data = wiringPiI2CRead(_fd);                // Fill Rx buffer with result
    close(_fd);
    return data;                             // Return data read from slave register
}

uint8_t LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
    _fd = wiringPiI2CSetup(address);
    if (_fd < 0) {
        fprintf(stderr, "Error: I2C Setup\n");
        exit(EXIT_FAILURE);
    }
    wiringPiI2CWrite(_fd, subAddress);
    uint8_t temp_dest[count];
    if ((read(_fd, temp_dest, 6)) < 0) {
        //fprintf(stderr, "Error: read value\n");
        throw 999;
        return 0;
    }
    close(_fd);
    for (int i = 0; i < count; i++) {
        dest[i] = temp_dest[i];
    }
    return count;

}



