
#define ARM_MATH_CM4

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "MadgwickAHRS.h"
#include "arm_math.h"
#include "math_helper.h"

uint8_t snum[100];
uint8_t MPU6050_ADDR = 0x68<<1;
uint8_t MAG_AD = 0xC<<1;
double Ax;
double Ay;
double Az;
double Gx;
double Gy;
double Gz;
float hardx = 9.5, hardy = 37, hardz=-12.5; //hard iron bias (calc in Python)
float softx= 1.02356, softy = 0.95597, softz = 1.02356; //soft iron bias (calc in Python)
int16_t magmax[3] = {-32767, -32767, -32767};
const float32_t A_f32[9] = {0.904593,0.018706, 0.036623,0.018706, 0.875807, -0.040483,0.036623,-0.040483,0.941098};
float32_t AB_f32[9];
float32_t mxyz[3]={1,1,1};
float asax;
float asay;
float asaz;
int16_t gyroxoffset=0;
int16_t gyroyoffset=0;
int16_t gyrozoffset=0;
int16_t samples=0;
float* roll;
float* yaw;
float* pitch;
double x;
double z;
double y;
typedef enum {
 zyx, zyz, zxy, zxz, yxz, yxy, yzx, yzy, xyz, xyx, xzy, xzx
} RotSeq;

double res[3];
double yawdeg;
double pitchdeg;
double rolldeg;
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */


uint8_t ret;
uint8_t pwrmng[2] = {0x6B, 0x00};



  ret = HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, pwrmng, 2, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {
    	   	    serialWrite(ret);
  }

  HAL_Delay(1000);


  uint8_t smpleratediv1[2] = {0x19, 0x00}; 
    ret = HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, smpleratediv1, 2, HAL_MAX_DELAY);
     if ( ret != HAL_OK ) {
       	   	    serialWrite(ret);
     }

     HAL_Delay(1000);


  //ACCEL CONFIG//
  uint8_t accconfig[2] = {0x1C, 0x00};
  ret = HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, accconfig, 2, HAL_MAX_DELAY);
  if (ret != HAL_OK){
	  serialWrite(ret);
  }

  magconfig();
   
/* USING MAGNETO CORRECTION
     arm_matrix_instance_f32 A;
     arm_matrix_instance_f32 B;
     arm_matrix_instance_f32 AB;
     arm_status status;

      arm_mat_init_f32(&A, 3, 3, (float32_t *)A_f32);
      arm_mat_init_f32(&B, 3, 1, (float32_t *)mxyz);
      arm_mat_init_f32(&AB, 3, 1, (float32_t *)AB_f32);

       status = arm_mat_mult_f32(&A, &B, &AB);
       */
	
  while (1)
  {
    read_mag(mxyz);
	  	/* USING MAGNETO CORRECTION
	  status = arm_mat_mult_f32(&A, &B, &AB); */

     read_gyro();

     read_acc();


   //Accounting for Misalignment in Mag axis compared to ACC & Gyro
    MadgwickAHRSupdate(Gy, Gx, -Gz,  Ay, Ax, -Az,  mxyz[0],  mxyz[1],  mxyz[2]);


	z = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1);
	x = -asin(2*q1*q3 + 2*q0*q2);
	y = atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1);

//sends yaw, pitch and roll values through UART to be read in Python (Vpython)
  sprintf((char*)snum, "%.2f,%.2f,%.2f,\r \n ", x, y, z);

	HAL_Delay(25);

	HAL_UART_Transmit(&huart2, snum, strlen((char*)snum), HAL_MAX_DELAY);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void read_acc(void){

	 uint8_t Rec_Data[6];
	 int16_t Accel_X_RAW;
	 int16_t Accel_Y_RAW;
	 int16_t Accel_Z_RAW;

	 uint8_t ACCEL_XOUT_H_REG = 0x3B;

	  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	  Ax = (Accel_X_RAW/16384.0)+0.43;  // get the float g
	  Ay = (Accel_Y_RAW/16384.0)+0.71;
	  Az = (Accel_Z_RAW/16384.0)+0.08;
}

void read_gyro(void){

	uint8_t GYRO_XOUT_H_REG = 0x43;

	uint8_t Rec_Data[6];

	double degtorad = 0.0174532925;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;


	 HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		  Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		  Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		  Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

     if(samples<128){
    	 samples++;
    	 return;
     } else if (samples < 256) {

    	 gyroxoffset += Gyro_X_RAW;
    	 gyroyoffset += Gyro_Y_RAW;
    	 gyrozoffset += Gyro_Z_RAW;
    	 samples++;
    	 return;
     } else if (samples ==256){
    	 gyroxoffset /= 128;
    	 gyroyoffset /= 128;
    	 gyrozoffset /= 128;
     }


		  Gx = (Gyro_X_RAW/131.0)*degtorad;
		  Gy = (Gyro_Y_RAW/131.0)*degtorad;
		  Gz = (Gyro_Z_RAW/131.0)*degtorad;
}



void serialWrite(unsigned int n)
 {
     itoa(n, (char*)snum, 10);
 }

double* read_mag(double magfinal[3])
	  {

	  uint8_t magdata[7];
	  //Reading MAG data
	  uint8_t ST1 = 0x02;
	  uint8_t status1; //might have to change to buffer and remove pointer in READ func below
	  float SCALE = 0.1499;

	  uint8_t DATA_READY = 0x02; //might have to change to 1
	  uint8_t HXL_AD = 0x03; //first measurement byte
	  uint8_t HOFL_MASK = 0x8; //reading HOFL ST2 bit
	  int16_t MAG_X_RAW;
	  int16_t MAG_Y_RAW;
	  int16_t MAG_Z_RAW;


	  HAL_I2C_Mem_Read(&hi2c1, MAG_AD, ST1, 1, &status1, 1, 1000);
	   if((status1 & DATA_READY) == DATA_READY){
	 	  HAL_I2C_Mem_Read(&hi2c1, MAG_AD, HXL_AD, 1, magdata, 7, 1000);

	 	  if(!(magdata[6] & HOFL_MASK)){ //if no overflow store data

	 		  MAG_X_RAW = magdata[0] | (magdata[1]<<8);
	 		  MAG_Y_RAW = magdata[2] | (magdata[3]<<8);
	 		  MAG_Z_RAW = magdata[4] | (magdata[5]<<8);

	 		 /*
	 		 magfinal[0] = MAG_X_RAW*asax*SCALE; //Gauss (Gs)
	 		 magfinal[1] = MAG_Y_RAW*asay*SCALE;
	 		 magfinal[2] = MAG_Z_RAW*asaz*SCALE; */

	 		//withcalibration
			 
			  /* USING MAGNETO CORRECTION
	 		 magfinal[0] = MAG_X_RAW*asax*SCALE-13.578836;
	 		 magfinal[1] = MAG_Y_RAW*asay*SCALE-15.216318;
	 		 magfinal[2] = MAG_Z_RAW*asaz*SCALE-6.231530; */


	 		magfinal[0] = ((MAG_X_RAW*asax*SCALE)-(hardx))*softx; //Gauss (Gs)
	 	    magfinal[1] = ((MAG_Y_RAW*asay*SCALE)-(hardy))*softy;
	 	    magfinal[2] = ((MAG_Z_RAW*asaz*SCALE)-(hardz))*softz;


	 	  }
	 	  else{

	 		  strcpy((char*)snum, "Overflow Tx\r\n");

	 	  }

	 	  }
	   return magfinal;
	  }

void magconfig(void)
{
uint8_t ret;
uint8_t magsenstivity[3];
uint8_t USER_CTRL_AD[2] = {0x6A, 0x00}; //disables I2C master (MPU9250)
ret = HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, USER_CTRL_AD, 2, HAL_MAX_DELAY);
if ( ret != HAL_OK ) {
     	   	    serialWrite(ret);
   }
uint8_t INT_BYPASS_CONFIG_AD[2]= {0x37, 0x02}; //enables bypass multiplexer (MPU9250)
ret = HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR, INT_BYPASS_CONFIG_AD, 2, HAL_MAX_DELAY);
if ( ret != HAL_OK ) {
     	   	    serialWrite(ret);
   }
uint8_t CNTL1_AD[2]= {0x0A, 0x1F}; //FUSE ROM ACCESS MODE
ret = HAL_I2C_Master_Transmit(&hi2c1,MAG_AD, CNTL1_AD, 2, HAL_MAX_DELAY);
if ( ret != HAL_OK ) {
     	   	    serialWrite(ret);
   }
HAL_Delay(100);
uint8_t ASAX_AD = 0x10; //Adjustment Values
HAL_I2C_Mem_Read(&hi2c1, MAG_AD, ASAX_AD, 1, magsenstivity, 3, 1000);
asax = (magsenstivity[0]-128)*0.5/128+1;
asay = (magsenstivity[1]-128)*0.5/128+1;
asaz = (magsenstivity[2]-128)*0.5/128+1;

uint8_t CNTL1_AD1[2]= {0x0A, 0x00}; //sets magnometer power down mode
ret = HAL_I2C_Master_Transmit(&hi2c1,MAG_AD, CNTL1_AD1, 2, HAL_MAX_DELAY);
if ( ret != HAL_OK ) {
     	   	    serialWrite(ret);
   }
HAL_Delay(100);

uint8_t CNTL1_AD2[2]= {0x0A, 0x16}; //sets magnometer continuous mode2 & 16-bit output
ret = HAL_I2C_Master_Transmit(&hi2c1,MAG_AD, CNTL1_AD2, 2, HAL_MAX_DELAY);
if ( ret != HAL_OK ) {
     	   	    serialWrite(ret);
   }
HAL_Delay(100);
}
