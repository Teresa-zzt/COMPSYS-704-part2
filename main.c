/**
  ******************************************************************************
  * @file    main.c

	#CS704

	README:
	1. BLE and USB-CDC Serial Debug is already done for you.
		1.1 BLE - When connected to the STM BLE Sensor App -
			data in 9 global variables :
			ACC_Value.x, ACC_Value.y, ACC_Value.z &
			MAG_Value.x, MAG_Value.y, MAG_Value.z &
			COMP_Value.Steps , COMP_Value.Heading, COMP_Value.Distance
			gets sent to the app and plotted in the graphs Accelerometer, Magnetometer and Gyroscope respectively.
		1.2 Serial Debug - Use XPRINTF("",param1,..) to print debug statements to USB-UART
	2. Intialise all sensors, variables, etc in the Block ***Intialise Here***
	3. Read sensors and process the sensor data in the block ***READ SENSORS & PROCESS**
		3.1 This block gets executed every 100ms, when ReadSensor gets set by TIM4 - you can change TIM4 clock input to change this, in the InitTimers function
	4. The device name seen by the ST BLE Sensor app is set by the NodeName[1] - NodeName[7] variables, change them in ***Intialise here**** block
	5. The SPI Read and Write functions have been written for you. Example usage for these are in the function InitLSM(). Noted there are separate functions
		for magnetometer and accelerometer.
	6. startAcc and startMag are empty functions to initialise acc and mag sensors - use hints in these functions
	7. readAcc and readMag are empty functions to read acc and mag sensors - use hints in these functions
	8. SensorTile has multiple SPI devices on the same bus as LSM303AGR(acc and mag sensor). It is critical to keep the I2C interface disabled for all sensors.
		9.1 For this - Bit 0 in Register 0x23 in accelerometer should always be SET(1)
		9.2 For this - Bit 5 in Register 0x62 in magnetometer should always be SET(1)
		9.3 Examples for these are present in InitLSM function.
		9.4 Failure to do (8), this will prevent acc and mag sensors from responding over SPI
	9. Important parts of the code are marked with #CS704


*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>

#include <limits.h>
#include "main.h"

#include "ble_interface.h"
#include "bluenrg_utils.h"
#include "SensorTile.h"
#include <stdbool.h>

#include "SensorTile_bus.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define STATUS_REG_A 0x27
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D


// for MAG
#define STATUS_REG_M  0x67
#define CFG_REG_A_M   0x60
#define CFG_REG_B_M   0x61
#define CFG_REG_C_M   0x62
#define OUTX_L_REG_M  0x68
#define OUTX_H_REG_M  0x69
#define OUTY_L_REG_M  0x6A
#define OUTY_H_REG_M  0x6B
#define OUTZ_L_REG_M  0x6C
#define OUTZ_H_REG_M  0x6D


#define pi 3.141592
// threshold of step
#define stepStartACC 1150
#define stepStopACC 800


/* Shutdown mode enabled as default for SensorTile */
#define ENABLE_SHUT_DOWN_MODE 0


#define BSP_LSM6DSM_INT2_GPIO_PORT           GPIOA
#define BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define BSP_LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define BSP_LSM6DSM_INT2                 GPIO_PIN_2
#define BSP_LSM6DSM_INT2_EXTI_IRQn           EXTI2_IRQn

#define BSP_LSM6DSM_CS_PORT GPIOB
#define BSP_LSM6DSM_CS_PIN GPIO_PIN_12
#define BSP_LSM6DSM_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_M_CS_PORT GPIOB
#define BSP_LSM303AGR_M_CS_PIN GPIO_PIN_1
#define BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_X_CS_PORT GPIOC
#define BSP_LSM303AGR_X_CS_PIN GPIO_PIN_4
#define BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()

#define BSP_LPS22HB_CS_PORT GPIOA
#define BSP_LPS22HB_CS_PIN GPIO_PIN_3
#define BSP_LPS22HB_CS_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()


#define LSM_MAG_CS_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET);
#define LSM_MAG_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET);

#define LSM_ACC_CS_LOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_RESET);
#define LSM_ACC_CS_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4,GPIO_PIN_SET);

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;


/* BlueNRG SPI */
extern SPI_HandleTypeDef SPI_SD_Handle;

extern volatile float RMS_Ch[];
extern float DBNOISE_Value_Old_Ch[];
extern uint16_t PCM_Buffer[];
extern uint32_t NumSample;

/* Exported Variables -------------------------------------------------------------*/
extern SPI_HandleTypeDef hbusspi2;

extern uint8_t NodeName[8];

//float sensitivity;

/* Acc sensitivity multiply by FROM_MG_TO_G constant */
//float sensitivity_Mul;


uint32_t ConnectionBleStatus  =0;

uint32_t FirstConnectionConfig =0;
uint8_t BufferToWrite[256];
int32_t BytesToWrite;
TIM_HandleTypeDef    TimCCHandle;
TIM_HandleTypeDef    TimEnvHandle;

uint8_t bdaddr[6];
uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;
uint32_t MagCalibrationData[5];
uint32_t AccCalibrationData[7];
uint8_t  NodeName[8];

static int stepCount =0;			// check for static and just int
bool walkingState = false;
bool orientatiaon_state=true;
bool MAG_limitation=true;
static int16_t headingOffest;

UART_HandleTypeDef UartHandle;


/* Private variables ---------------------------------------------------------*/
static volatile int MEMSInterrupt        =0;
static volatile uint32_t ReadSensor        =0;
static volatile uint32_t SendAccGyroMag  =0;
static volatile uint32_t TimeStamp = 0;
volatile uint32_t HCI_ProcessEvent=0;
typedef struct  {
	 uint32_t Steps;
	 uint32_t Heading;
	 uint32_t Distance;
}COMP_Data;
BSP_MOTION_SENSOR_Axes_t ACC_Value;
COMP_Data COMP_Value;
// to convert from float to int
typedef struct {
	int32_t x;
	int32_t y;
    int32_t z;
} MAG_raw;

MAG_raw MAG_raw_data;

typedef struct {
	int32_t x;
	int32_t y;
    int32_t z;
} MAG_convert;

MAG_convert MAG_calibrate;
BSP_MOTION_SENSOR_Axes_t MAG_Value;
// Min/max

static float mag_x_min =  32767.0f;
static float mag_y_min =  32767.0f;
static float mag_x_max = -32768.0f;
static float mag_y_max = -32768.0f;

static float offest_MagX = 0.0f;
static float offest_MagY = 0.0f;
static float scaleFact_MagX  = 1.0f;
static float scaleFact_MagY  = 1.0f;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);

static void DeinitTimers(void);
static void InitTimers(void);
static void SendMotionData(void);
static void InitTargetPlatform(void);
static void InitLSM();


void MX_UART5_UART_Init();

uint8_t Sensor_IO_SPI_CS_Init_All(void);
static int32_t BSP_LSM303AGR_WriteReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);
static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len);
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead);
void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val);
void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val);


static void InitLSM() {
	uint8_t inData[10];
	//setup CS pins on all SPI devices
	Sensor_IO_SPI_CS_Init_All();
	//#CS704 - sample usage of SPI READ and WRITE functions
	//Disable I2C on Acc and Mag - WRITE
	inData[0] = 0x01;
	BSP_LSM303AGR_WriteReg_Acc(0x23,inData,1);
	inData[0] = 0x20;
	BSP_LSM303AGR_WriteReg_Mag(0x62U,inData,1);

	//Read IAM registers for Acc and Mag to verify connection - READ
	BSP_LSM303AGR_ReadReg_Mag(0x4F,inData,1);
	XPRINTF("IAM Mag= %d,%d",inData[0],inData[1]);
	BSP_LSM303AGR_ReadReg_Acc(0x0F,inData,1);
	XPRINTF("IAM Acc= %d,%d",inData[0],inData[1]);
}



static void startAcc() {
  // using 10 bytes array to hold data
	uint8_t inData[10];
  //#CS704 - Write SPI commands to initialize Accelerometer
  // All settings are based on datasheet recommendations - self test procedure
  // LSM303AGR datasheet 4.1.4 accelerometer self-test page 29 (canvas provided)
	// Reset all the settings in control register 2
	inData[0] = 0x00; //00000000
	BSP_LSM303AGR_WriteReg_Acc(CTRL_REG2_A,inData,1);
	// Reset all the settings in control register 3
	inData[0] = 0x00; //00000000
	BSP_LSM303AGR_WriteReg_Acc(CTRL_REG3_A,inData,1);
	// Set full-scale selection and enable high-resolution output mode
  // Reset all the settings in control register 4
	inData[0] = 0x81; //10000001, the bit 1 and 2 set the acc range, (00: ±2g; 01: ±4g; 10: ±8g; 11: ±16g)
	BSP_LSM303AGR_WriteReg_Acc(CTRL_REG4_A,inData,1);
	// Set data rate to 12.5 Hz and enable all axes
  // Reset all the settings in control register 1
	inData[0] = 0x57; //01010111
	BSP_LSM303AGR_WriteReg_Acc(CTRL_REG1_A,inData,1);
}

static void startMag() {
  // using 10 bytes array to hold data
	uint8_t inData[10];
  //#CS704 - Write SPI commands to initialize Magnetometer
  // All settings are based on datasheet recommendations - self test procedure
  // LSM303AGR datasheet 4.1.5 magnetometer self-test page 25 (canvas provided)
	// Follow the self test process
	inData[0] = 0x8C; //10001100
	BSP_LSM303AGR_WriteReg_Mag(CFG_REG_A_M,inData,1);
	// Set the data rate for the Magnetometer
	inData[0] = 0x03; //00000011
	BSP_LSM303AGR_WriteReg_Mag(CFG_REG_B_M,inData,1);
	// Set the device to single conversion mode and enable BDU
	inData[0] = 0x10; //00010000
	BSP_LSM303AGR_WriteReg_Mag(CFG_REG_C_M,inData,1);
}


static void readMag() {
	uint8_t magStatus;
	uint8_t magData[2];
	int16_t magX_raw, magY_raw, magZ_raw;	// variables to hold combined LSB and MSB
	int16_t magX_mg,magY_mg,magZ_mg; //in milli-Gauss
	float mag_Conver_factor = 49.152 / 32768.0;

	// check until ZYXDA bit is ready
  uint8_t dataReadyBit = 0x08;
	do {
		BSP_LSM303AGR_ReadReg_Mag(STATUS_REG_M,&magStatus,1);
	} while (!(magStatus & dataReadyBit));
	// read for X
	BSP_LSM303AGR_ReadReg_Mag(OUTX_L_REG_M,magData,2);
	magX_raw = (int16_t)((magData[1] << 8) | magData[0]);
	magX_mg = (int16_t)(magX_raw * mag_Conver_factor *1000);		// multiply by mag conversion unit in mg

	// read for Y
	BSP_LSM303AGR_ReadReg_Mag(OUTY_L_REG_M,magData,2);
	magY_raw = (int16_t)((magData[1] << 8) | magData[0]);
	magY_mg = (int16_t)(magY_raw * mag_Conver_factor*1000);
	// read for Z
	BSP_LSM303AGR_ReadReg_Mag(OUTZ_L_REG_M,magData,2);
	magZ_raw = (int16_t)((magData[1] << 8) | magData[0]);
	magZ_mg = (int16_t)(magZ_raw * mag_Conver_factor*1000);

  // store the value into global variable
  MAG_Value.x= magX_mg;
	MAG_Value.y= magY_mg;
	MAG_Value.z= magZ_mg;
  // for calibration
	MAG_raw_data.x= magX_raw;
	MAG_raw_data.y= magY_raw;
	MAG_raw_data.z= magZ_raw;

  // for debug, uncomment if needed
//	XPRINTF("ROW MAG=%d,%d,%d\r\n",magX_raw,magY_raw,magZ_raw);
////	 print magnetometer value in milli-Gauss
//	XPRINTF("Converted MAG=%d,%d,%d\r\n",magX_mg, magY_mg, magZ_mg);
}

static void readAcc() {
	uint8_t accStatus;
	uint8_t accData[2];
	int16_t accX_raw, accY_raw, accZ_raw;
	const float accFullScale = 2.0;
	const float acc_Conver_factor = accFullScale / 32768.0;
	int16_t accX_mg, accY_mg, accZ_mg;
	uint8_t dataReadyBit = 0x08;
	do {
		BSP_LSM303AGR_ReadReg_Acc(STATUS_REG_A,&accStatus,1);
	} while (!(accStatus & dataReadyBit));

	// For ACC X
	BSP_LSM303AGR_ReadReg_Acc(OUT_X_L_A,accData,2);
	// shift MSB to the left
	accX_raw = (int16_t)((accData[1] <<8) | accData[0]);
	// multiply by factor to get +-2g.
	accX_mg = (int16_t)(accX_raw * acc_Conver_factor *1000);

	// For ACC Y
	BSP_LSM303AGR_ReadReg_Acc(OUT_Y_L_A,accData,2);
	accY_raw = (int16_t)((accData[1] <<8) | accData[0]);
	accY_mg = (int16_t)(accY_raw * acc_Conver_factor *1000);

	// For ACC Z
	BSP_LSM303AGR_ReadReg_Acc(OUT_Z_L_A,accData,2);
	accZ_raw = (int16_t)((accData[1] <<8) | accData[0]);
	accZ_mg = (int16_t)(accZ_raw * acc_Conver_factor * 1000);

  // store the value into global variable
  ACC_Value.x= accX_mg;
	ACC_Value.y= accY_mg;
	ACC_Value.z= accZ_mg;

  // for debug, uncomment if needed
	//XPRINTF("Row ACC=%d,%d,%d\r\n",accX_raw,accY_raw,accZ_raw);
	// print accelerometer value in milli-g
	//XPRINTF("Converted ACC=%d,%d,%d mg\r\n",accX_mg,accY_mg,accZ_mg);
}



void avoidLimit(void){
  // check if the mag value is within the limitation
	 int32_t int_x = MAG_raw_data.x;
	 int32_t int_y = MAG_raw_data.y;
//	 int32_t int_z = MAG_raw_data.z;

  // if the value is within the limitation range
	 if ((int_x > -300 && int_x < 500) ||
	     (int_y > -300 && int_y < 500) ) {
		 MAG_limitation = true;
	 }else{
		 MAG_limitation = false;
	 }
}

void calibration(void)
{
	avoidLimit();
	if (MAG_limitation){

	   float x = (int32_t)MAG_raw_data.x;
	   float y = (int32_t)MAG_raw_data.y;
//	   float z = (int32_t)MAG_raw_data.z;

	   //  update the max and min value for mag
	   if (x < mag_x_min){
		   mag_x_min = x;
	   }
	   if (x > mag_x_max ) {
		   mag_x_max = x;
	   }

	   if (y < mag_y_min) {
		   mag_y_min = y;
	   }
	   if (y > mag_y_max) {
		   mag_y_max = y;
	   }
   // apply hard-iron
   // calculate the offest value
		offest_MagX = (mag_x_max + mag_x_min) * 0.5f;
		offest_MagY = (mag_y_max + mag_y_min) * 0.5f;

	//apply soft iron
  // calculate the scale factor
		float dx = (mag_x_max - mag_x_min) * 0.5f;
		float dy = (mag_y_max - mag_y_min) * 0.5f;
		float avg = (dx + dy ) * 0.5f;
    
    // check for zero division
		if (dx != 0) {
			scaleFact_MagX = avg / dx;
		}
		if (dy != 0) {
			scaleFact_MagY = avg / dy;
		}


		 int32_t int_x = MAG_raw_data.x;
		 int32_t int_y = MAG_raw_data.y;

		// calculate the calibration value
		MAG_calibrate.x = (int_x - offest_MagX) * scaleFact_MagX;
		MAG_calibrate.y = (int_y - offest_MagY) * scaleFact_MagY;

	}else{
		XPRINTF(" the mag value out of limitation")
	}

}


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	int16_t accZ;
	int32_t heading = 0;
  bool risingAcc, fallingAcc;


  HAL_Init();

  // Configure the System clock
  SystemClock_Config();

  InitTargetPlatform();

//  Initialize the BlueNRG
  Init_BlueNRG_Stack();

// Initialize the BlueNRG Custom services
  Init_BlueNRG_Custom_Services();

//  initialize timers
  InitTimers();
//
  if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
    // Starting Error
    Error_Handler();
  }
  connected = FALSE;


  //***************************************************
  //***************** #CS704 **************************
  //************ Initialise here **********************
  //***************************************************


  //#CS704 - use this to set BLE Device Name
  // GROUP19
  NodeName[1] = 'G';
  NodeName[2] = 'R';
  NodeName[3] = 'O';
  NodeName[4] = 'U';
  NodeName[5] = 'P';
  NodeName[6] = '1';
  NodeName[7] = '9';

  startMag();
  startAcc();
  // orientation initial state for initial heading offset
  orientatiaon_state=true;


  //***************************************************
  //************ Initialise ends **********************
  //***************************************************


  /* Infinite loop */
  while (1)
  {
      if(!connected) {
          if(!(HAL_GetTick()&0x3FF)) {
        	  BSP_LED_Toggle(LED1);
//        	  HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)BufferToWrite, 5);
          }
      }
    if(set_connectable){

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }

    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }

    //***************************************************
    //***************** #CS704 **************************
    //*********** READ SENSORS & PROCESS ****************
    //***************************************************

    //#CS704 - ReadSensor gets set every 100ms by Timer TIM4 (TimEnvHandle)
    if(ReadSensor) {
    	ReadSensor=0;

	//*********get sensor data**********

      // get readings and process
    	readMag();
    	readAcc();
    	calibration();

      // Heading calculation
      // atan2 to get heading in radians
      // convert to degrees
		float heading_float = atan2f(MAG_calibrate.y, MAG_calibrate.x);
    // set heading between 0 to 2pi
		if (heading_float < 0)
    // if the angle is negative, convert to positive angle
			heading_float += 2.0f * pi;
		 heading = heading_float * 180.0f / pi;
//     convert heading between 0 to 360
      if (heading<0){
        heading +=360;
      }

      // Initialise the heading offset
      if (orientatiaon_state){
        headingOffest = heading;
        orientatiaon_state = false;
      }

      // relative heading calculation
      int16_t relativeHeading = heading - headingOffest;
      if (relativeHeading <0){
        relativeHeading += 360;
      }
// debug print heading
//      XPRINTF("Heading  = %d degrees, Relative Heading from offset is= %d degrees\r\n", heading, relativeHeading);


      //Step detection
      accZ = ACC_Value.z;
       risingAcc = (accZ >= stepStartACC);
       fallingAcc = (accZ <= stepStopACC);
       // de
//       XPRINTF("%d \r\n",accZ);

      if (!walkingState && risingAcc){
        walkingState = true;
        // debug print start of one step
//        XPRINTF(" start one step \r\n");
      }
      // Detect the end of the step
      if (walkingState && fallingAcc){
        // debug print end of one step
//        XPRINTF("end one step\r\n");
        walkingState = false;
        stepCount ++;
        // debug print step count
//        XPRINTF("Step count: %d\r\n", stepCount);
      }
      // assign to Gyroscope for bluetooth testing
      // comment one of them when testing independently (for a better view on the app)
      COMP_Value.Steps= stepCount;
      COMP_Value.Heading =relativeHeading;


    }

    //***************************************************
    //***************************************************


    /* Motion Data */
    if(SendAccGyroMag) {
		SendMotionData();
    	SendAccGyroMag=0;
    }

    /* Wait for Event */
    __WFI();
  }
}

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{


  /* Initialize LED */
  BSP_LED_Init( LED1 );
  /* Initialize UART */
  MX_UART5_UART_Init();

  InitLSM(); //N4S


}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART5_UART_Init(void)
{
  UartHandle.Instance = UART5;
  UartHandle.Init.BaudRate = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

//  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
//  {
//    Error_Handler();
//  }
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

}

void MX_USART1_UART_Init(void)
{
  UartHandle.Instance = USART1;
  UartHandle.Init.BaudRate = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();

  }

}

void UART5_Transmit(uint8_t* BufferToWrite, uint16_t BytesToWrite) {

	HAL_UART_Transmit(&UartHandle, (uint8_t*)BufferToWrite, BytesToWrite,1000);
}


/**
  * @brief  Output Compare callback in non blocking mode
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;



  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
  }
}


/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


  if(htim == (&TimEnvHandle)) {
	  ReadSensor=1;


  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{

  AccGyroMag_Update(&ACC_Value,(BSP_MOTION_SENSOR_Axes_t*)&COMP_Value,&MAG_Value);
}



/**
* @brief  Function for initializing timers for sending the information to BLE:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void InitTimers(void)
{

  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM4 counter clock equal to 10 KHz Hz */

 // #CS704 -  change TIM4 configuration here to change frequency of execution of *** READ Sensor and Process Block **
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  /* Set TIM4 instance ( Environmental ) */
  TimEnvHandle.Instance = TIM4;
  /* Initialize TIM4 peripheral */
  TimEnvHandle.Init.Period = 655;
  TimEnvHandle.Init.Prescaler = uwPrescalerValue;
  TimEnvHandle.Init.ClockDivision = 0;
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK) {
    /* Initialization Error */
  }


  /* Compute the prescaler value to have TIM1 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  /* Set TIM1 instance ( Motion ) */
  TimCCHandle.Instance = TIM1;
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

 /* Configure the Output Compare channels */
 /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Code for MotionFX integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = DEFAULT_uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionFX integration - End Section */

  /* Code for MotionCP & MotionGR integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel2 */
  sConfig.Pulse = DEFAULT_uhCCR2_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionCP & MotionGR integration - End Section */

  /* Code for MotionAR & MotionID & MotionPE integration - Start Section */
  /* Output Compare Toggle Mode configuration: Channel3 */
  sConfig.Pulse = DEFAULT_uhCCR3_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
  /* Code for MotionAR & MotionID & MotionPE integration - End Section */

  /* Output Compare Toggle Mode configuration: Channel4 */
  sConfig.Pulse = DEFAULT_uhCCR4_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
}


/**
  * @brief  Set all sensor Chip Select high. To be called before any SPI read/write
  * @param  None
  * @retval HAL_StatusTypeDef HAL Status
  */
uint8_t Sensor_IO_SPI_CS_Init_All(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t inData[2];
  /* Set all the pins before init to avoid glitch */
  BSP_LSM6DSM_CS_GPIO_CLK_ENABLE();
  BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE();
  BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE();
  BSP_LPS22HB_CS_GPIO_CLK_ENABLE();

  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pin = BSP_LSM6DSM_CS_PIN;
  HAL_GPIO_Init(BSP_LSM6DSM_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LSM303AGR_X_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_X_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN,GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LSM303AGR_M_CS_PIN;
  HAL_GPIO_Init(BSP_LSM303AGR_M_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN,GPIO_PIN_SET);

  GPIO_InitStruct.Pin = BSP_LPS22HB_CS_PIN;
  HAL_GPIO_Init(BSP_LPS22HB_CS_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);


  //setup SPI interface
  	 if(BSP_SPI2_Init() == BSP_ERROR_NONE)
  	  {
  		 //XPRINTF("NO SPI Error\r\n");
  	  }

  //disable I2C modes on all SPI devices
  //LSM303 Mag
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x62U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x20;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);

  //LSM303 Acc
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x23U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x01;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM303AGR_X_CS_PORT, BSP_LSM303AGR_X_CS_PIN, GPIO_PIN_SET);

  //LPS22HB
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x10U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x01;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LPS22HB_CS_PORT, BSP_LPS22HB_CS_PIN, GPIO_PIN_SET);

  //LSM6DSM
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_RESET);
  inData[0] = (0x12U);
  BSP_SPI2_Send(inData,1);
  inData[0] = 0x0C;
  BSP_SPI2_Send(inData,1);
  HAL_GPIO_WritePin(BSP_LSM6DSM_CS_PORT, BSP_LSM6DSM_CS_PIN, GPIO_PIN_SET);


  return HAL_OK;
}

/**
 * @brief  Write register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_WriteReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  LSM_MAG_CS_LOW();

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  LSM_MAG_CS_HIGH();

  return ret;
}

/**
* @brief  Read register by SPI bus for LSM303AGR
* @param  Reg the starting register address to be read
* @param  pdata the pointer to the data to be read
* @param  len the length of the data to be read
* @retval BSP status
*/
static int32_t BSP_LSM303AGR_ReadReg_Mag(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_RESET);
//  XPRINTF("Data Read = %d,%d\r\n",(dataReg) | 0x80,pdata[0]);
  LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80);
  __HAL_SPI_DISABLE(&hbusspi2);
  SPI_1LINE_RX(&hbusspi2);

  if (len > 1)
  {
    LSM303AGR_SPI_Read_nBytes(&hbusspi2, (pdata), len);
  }
  else
  {
    LSM303AGR_SPI_Read(&hbusspi2, (pdata));
  }

  /* CS Disable */
  HAL_GPIO_WritePin(BSP_LSM303AGR_M_CS_PORT, BSP_LSM303AGR_M_CS_PIN, GPIO_PIN_SET);
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  return ret;
}

/**
 * @brief  Write register by SPI bus for LSM303AGR
 * @param  Reg the starting register address to be written
 * @param  pdata the pointer to the data to be written
 * @param  len the length of the data to be written
 * @retval BSP status
 */
static int32_t BSP_LSM303AGR_WriteReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  LSM_ACC_CS_LOW();

  if (BSP_SPI2_Send(&dataReg, 1) != 1)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  if (BSP_SPI2_Send(pdata, len) != len)
  {
    ret = BSP_ERROR_UNKNOWN_FAILURE;
  }

  /* CS Disable */
  LSM_ACC_CS_HIGH();

  return ret;
}

/**
* @brief  Read register by SPI bus for LSM303AGR
* @param  Reg the starting register address to be read
* @param  pdata the pointer to the data to be read
* @param  len the length of the data to be read
* @retval BSP status
*/
static int32_t BSP_LSM303AGR_ReadReg_Acc(uint16_t Reg, uint8_t *pdata, uint16_t len)
{
  int32_t ret = BSP_ERROR_NONE;
  uint8_t dataReg = (uint8_t)Reg;

  /* CS Enable */
  LSM_ACC_CS_LOW();
  if (len > 1) {
	  LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80 | 0x40);
  }
  else {
	  LSM303AGR_SPI_Write(&hbusspi2, (dataReg) | 0x80);
  }
  __HAL_SPI_DISABLE(&hbusspi2);
  SPI_1LINE_RX(&hbusspi2);

  if (len > 1)
  {
    LSM303AGR_SPI_Read_nBytes(&hbusspi2, (pdata), len);
  }
  else
  {
    LSM303AGR_SPI_Read(&hbusspi2, (pdata));
  }

  /* CS Disable */
  LSM_ACC_CS_HIGH();
  SPI_1LINE_TX(&hbusspi2);
  __HAL_SPI_ENABLE(&hbusspi2);
  return ret;
}


/**
* @brief  This function reads multiple bytes on SPI 3-wire.
* @param  xSpiHandle: SPI Handler.
* @param  val: value.
* @param  nBytesToRead: number of bytes to read.
* @retval None
*/
void LSM303AGR_SPI_Read_nBytes(SPI_HandleTypeDef* xSpiHandle, uint8_t *val, uint16_t nBytesToRead)
{
  /* Interrupts should be disabled during this operation */
  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);

  /* Transfer loop */
  while (nBytesToRead > 1U)
  {
    /* Check the RXNE flag */
    if (xSpiHandle->Instance->SR & SPI_FLAG_RXNE)
    {
      /* read the received data */
      *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
      val += sizeof(uint8_t);
      nBytesToRead--;
    }
  }
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit of the last Byte received */
  /* __DSB instruction are inserted to garantee that clock is Disabled in the right timeframe */

  __DSB();
  __DSB();
  __HAL_SPI_DISABLE(xSpiHandle);

  __enable_irq();

  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
* @brief  This function send a command through SPI bus.
* @param  command: command id.
* @param  uint8_t val: value.
* @retval None
*/
void LSM303AGR_SPI_Read(SPI_HandleTypeDef* xSpiHandle, uint8_t *val)
{
  /* In master RX mode the clock is automaticaly generated on the SPI enable.
  So to guarantee the clock generation for only one data, the clock must be
  disabled after the first bit and before the latest bit */
  /* Interrupts should be disabled during this operation */

  __disable_irq();
  __HAL_SPI_ENABLE(xSpiHandle);
  __asm("dsb\n");
  __asm("dsb\n");
  __HAL_SPI_DISABLE(xSpiHandle);
  __enable_irq();


  while ((xSpiHandle->Instance->SR & SPI_FLAG_RXNE) != SPI_FLAG_RXNE);
  /* read the received data */
//  XPRINTF("Before READ, %d,%d\r\n",val[0],xSpiHandle->Instance);
  *val = *(__IO uint8_t *) &xSpiHandle->Instance->DR;
//  XPRINTF("READ, %d\r\n",val[0]);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}

/**
* @brief  This function send a command through SPI bus.
* @param  command : command id.
* @param  val : value.
* @retval None
*/
void LSM303AGR_SPI_Write(SPI_HandleTypeDef* xSpiHandle, uint8_t val)
{
  /* check TXE flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_TXE) != SPI_FLAG_TXE);

  /* Write the data */
  *((__IO uint8_t*) &xSpiHandle->Instance->DR) = val;

  /* Wait BSY flag */
  while ((xSpiHandle->Instance->SR & SPI_FLAG_FTLVL) != SPI_FTLVL_EMPTY);
  while ((xSpiHandle->Instance->SR & SPI_FLAG_BSY) == SPI_FLAG_BSY);
}




/**
* @brief  Function for De-initializing timers:
 *  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
 *  - 1 for sending the Environmental info
 * @param  None
 * @retval None
 */
static void DeinitTimers(void)
{

  /* Set TIM4 instance (Environmental) */
  TimEnvHandle.Instance = TIM4;
  if(HAL_TIM_Base_DeInit(&TimEnvHandle) != HAL_OK) {
    /* Deinitialization Error */
    Error_Handler();
  }

  /* Set TIM1 instance (Motion)*/
  TimCCHandle.Instance = TIM1;
  if(HAL_TIM_Base_DeInit(&TimCCHandle) != HAL_OK)
  {
    /* Deinitialization Error */
	  Error_Handler();
  }
}

/** @brief Initialize the BlueNRG Stack
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Stack(void)
{
  char BoardName[8];
  char customName[8] = "CSys704";
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  data_len_out;
  uint8_t  hwVersion;
  uint16_t fwVersion;



//  for(int i=0; i<7; i++)
//    BoardName[i]= NodeName[i+1];

  for(int i=0; i<7; i++)
    BoardName[i]= customName[i];

  BoardName[7]= 0;

  /* Initialize the BlueNRG SPI driver */
  hci_init(HCI_Event_CB, NULL);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, 6, &data_len_out, bdaddr);

  if ((bdaddr[5] & 0xC0) != 0xC0) {
    XPRINTF("\r\nStatic Random address not well formed.\r\n");
    while(1);
  }

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, data_len_out,
                                  bdaddr);

/* Sw reset of the device */
  hci_reset();

  ret = aci_gatt_init();
  if(ret){
     XPRINTF("\r\nGATT_Init failed\r\n");
     goto fail;
  }

  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS){
     XPRINTF("\r\nGAP_Init failed\r\n");
     goto fail;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret){
     XPRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) {
     XPRINTF("\r\nGAP setting Authentication failed\r\n");
     goto fail;
  }

  XPRINTF("SERVER: BLE Stack Initialized \r\n"
         "\tHWver= %d.%d\r\n"
         "\tFWver= %d.%d.%c\r\n"
         "\tBoardName= %s\r\n"
         "\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
         ((hwVersion>>4)&0x0F),(hwVersion&0x0F),
         fwVersion>>8,
         (fwVersion>>4)&0xF,
         ('a'+(fwVersion&0xF)),
         BoardName,
         bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
 * @param None
 * @retval None
 */
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
     XPRINTF("HW & SW Service W2ST added successfully\r\n");
  }
  else
  {
     XPRINTF("\r\nError while adding HW & SW Service W2ST\r\n");
  }

}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 6
  *            PLL_N                          = 40
  *            PLL_R                          = 4
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }

  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();

  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    while(1);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    while(1);
  }

  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();

  /* Select MSI output as USB clock source */
//  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
//  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
//  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK){
    while(1);
  }
}






/**
  * @brief This function provides accurate delay (in milliseconds) based
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1){
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin){
  case HCI_TL_SPI_EXTI_PIN:
      hci_tl_lowlevel_isr();
      HCI_ProcessEvent=1;
    break;

//  case BSP_LSM6DSM_INT2:
//    MEMSInterrupt=1;
//    break;
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: ALLMEMS1_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif



