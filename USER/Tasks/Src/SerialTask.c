//
// Created by YanYuanbin on 22-10-5.
//

#include "cmsis_os.h"

#include "SerialTask.h"
#include "InsTask.h"
#include "ChassisTask.h"

#include "myusart.h"

/* USER CODE BEGIN Header_SerialTask */
/**
* @brief Function implementing the mySerialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SerialTask */
void SerialTask(void const * argument)
{
  /* USER CODE BEGIN SerialTask */
	TickType_t systick = 0;
  
  /* Infinite loop */
  for(;;)
  {
		systick = osKernelSysTick();
		
//		myprintf(imu_pitgyro*100,Imu.pit_gyro*100);

    osDelayUntil(&systick,10);
  }
  /* USER CODE END SerialTask */
}




