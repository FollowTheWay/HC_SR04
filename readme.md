@[TOC](STM32 HAL库+CubeMX 输入捕获 超声波模块)

# 前言
本教程使用的是STM32F103C6T6，利用定时器输入捕获双边沿，实现超声波模块的使用。
创作初衷是在于大多数博客都是照抄前人的形式，且代码上有出入，cubemx开启通道一，代码内却是通道二等情况很多，对于新手来说很不友好。

## 软件及硬件准备
软件：
			keil5
			CubeMX
			XCOM
硬件：
			主控板：STM32F103C6T6（也可以使用STM32其他型号）
			超声波模块：**HC_SR04**
			USB转TTL

## 硬件连接
![在这里插入图片描述](https://img-blog.csdnimg.cn/f338e565c2cc4331af6d5742cafe0911.png#pic_center)
VCC   ——   5v(该模块工作电压需要5v，如果接3.3v，模块可能不工作)
GND   ——  GND
**Trig   ——   连接单片机的io口（任意io口都可，起控制作用）
Echo   ——   连接单片机的定时器输入捕获通道**

# CubeMX配置
## 定时器配置
![在这里插入图片描述](https://img-blog.csdnimg.cn/62fd316c73db418091137735d09ef41f.png#pic_center)
>如图，按照指示进行定时器配置即可
>我选择的是**TIM2CH2**,可根据自己的需求选择定时器和通道

![在这里插入图片描述](https://img-blog.csdnimg.cn/e41c1aef3dea46349f3fdd0c8c716e3f.png#pic_center)
>**这里是最重要的一步，使能定时器中断，如果最后超声波模块的值一直为0，一定要回来检查定时器配置**

![在这里插入图片描述](https://img-blog.csdnimg.cn/7113934ceb2644928b321e268e91727b.png#pic_center)
>选择任意io口，作为Trig脚来使用，配置为output输出模式即可，不必上下拉，初始化为低电平
>此处我使用的是**PA5**

## 串口配置
![在这里插入图片描述](https://img-blog.csdnimg.cn/2a72320a10d54dbdb53f3331df22d5bd.png#pic_center)
>开启串口，选择异步模式，修改波特率为115200，可自行修改为其他波特率

# keil软件编写

```c
/* USER CODE BEGIN 1 */
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
uint8_t TIM2CH2_CAPTURE_STA;  // 输入捕获状态
uint16_t TIM2CH2_CAPTURE_VAL; //输入捕获值
//溢出回调函数和捕获回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((TIM2CH2_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
  {
    if (TIM2CH2_CAPTURE_STA & 0x40) // 捕获到一个下降沿
    {
      if ((TIM2CH2_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
      {
        TIM2CH2_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
        TIM2CH2_CAPTURE_VAL = 0XFFFF;
      }
      else
        TIM2CH2_CAPTURE_STA++; // 否则标记溢出数加1
    }
  }
}

// 捕获中断发生时执行 上升沿复位开始计时，下降沿获取捕获值计算
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((TIM2CH2_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM2CH2_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM2CH2_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM2CH2_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM2CH2_CAPTURE_STA = 0;
      TIM2CH2_CAPTURE_VAL = 0;
      TIM2CH2_CAPTURE_STA |= 0X40; //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim2);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);
      TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);
      __HAL_TIM_ENABLE(&htim2);
    }
  }
}
/* USER CODE END 1 */
```
>代码是原子哥写的，我只是搬运工，在原来注释的基础上，加了一些我自己的注释
>将该段代码复制到tim.c中，在cubemx配置好定时器后，会自动生成tim.c文件。
>在tim.c文件最下方可以看见/* USER CODE BEGIN 1 */,复制到这里即可

```c
/**
 * @file HCSR04.c
 * @author Zhong Zepeng (1935595312@qq.com)
 * @brief
 * @version 0.1
 * @date 2022-11-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "HCSR04.h"
#include "gpio.h"
#include "tim.h"
/**
 * @brief 激活超声波定时器
 * 
 */
void HCSR_04()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 计算超声波检测的距离
 * 
 * @return float 
 */
float getSR04Distance()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM2CH2_CAPTURE_STA & 0X80) //输入捕获 触发
    {
        time = TIM2CH2_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM2CH2_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离

        TIM2CH2_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}

```
>编写SR04.c文件，将代码复制进去

```c
#ifndef __HCSR04_H
#define __HCSR04_H

#include "stdint.h"


void HCSR_04(void);
float getSR04Distance(void);

#endif

```
>SR04.h文件

```c
#include "User_Debug.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "usart.h"

void usart_printf(const char *fmt,...)
{
  static uint8_t tx_buf[256] = {0};
  static va_list ap;
  static uint16_t len;
		
  va_start(ap, fmt);
  len = vsprintf((char *)tx_buf, fmt, ap);
  va_end(ap);
  HAL_UART_Transmit(&huart1,tx_buf,len,100);
}
```
>编写串口打印文件，我使用的是USART1,如果使用其他串口，只需修改最后一行的&huart1
>在头文件里声明usart_printf()函数即可
```c
/* USER CODE BEGIN Includes */
#include "User_Debug.h"
#include "HCSR04.h"
/* USER CODE END Includes */
```

>在main.c里引用SR04头文件和串口打印头文件

```c
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);                 //开启定时器
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); //开启TIM2的捕获通道2，并开启捕获
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE); //使能更新中断
  usart_printf("start ok\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(100); //延时,不做延时的话，会超过采样频率
    HCSR_04();      //激活超声波模块
    float distance = getSR04Distance();
    usart_printf("dis = %.2f\r\n", distance);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```
>在main函数的while(1)及上方添加上述代码

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TRIG_Pin GPIO_PIN_5
#define TRIG_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
extern uint8_t TIM2CH2_CAPTURE_STA;  // 输入捕获状态
extern uint16_t TIM2CH2_CAPTURE_VAL; //输入捕获值
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

```
>在main.h中添加全局变量

# 代码讲解

```c
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数
uint8_t TIM2CH2_CAPTURE_STA;  // 输入捕获状态
uint16_t TIM2CH2_CAPTURE_VAL; //输入捕获值
//溢出回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((TIM2CH2_CAPTURE_STA & 0x80) == 0) // 还未捕获成功
  {
    if (TIM2CH2_CAPTURE_STA & 0x40) // 已捕获到一个上升沿
    {
      if ((TIM2CH2_CAPTURE_STA & 0x3F) == 0x3F) // 高电平的时间太长
      {
        TIM2CH2_CAPTURE_STA |= 0X80; // 标记为成功捕获一次
        TIM2CH2_CAPTURE_VAL = 0XFFFF;
      }
      else
        TIM2CH2_CAPTURE_STA++; // 否则标记溢出数加1
    }
  }
}
```
>原子哥的代码采用的是寄存器的思想，设置一个8位的状态寄存器TIM2CH2_CAPTURE_STA
>该寄存器**最高位**为定时器**是否捕捉到边沿变化**
>**次高位**为定时器是否开始捕捉**下降沿**，用于记录当前定时器状态
   
>所以  if ((TIM2CH2_CAPTURE_STA & 0x80) == 0) 该句的意思为如果最高位是0，即还未捕捉到边沿变化时，会进入if语句内（0x80 = 1000 0000）

>if (TIM2CH2_CAPTURE_STA & 0x40) 该句的意思为 次高位如果是1，则进入if语句（0x40 = 0100 0000）
>所以现在有两种情况
>**1、还未捕捉到上升沿，进入第一个if语句后，不会做任何操作。**
>**2、已经捕捉到了上升沿，等待捕捉下降沿，会进入到第二个if语句中，直到定时器捕捉到下降沿才会跳出if语句**

>第二个if内部有一组if else。其意思是STA会在此处自加，但是如果STA加到0x3f，就会手动置最高位为1，**使程序跳出**，不会死锁在此处。并将VAL写为0xFFFF，将VAL填满，表示为**距离超过最大测量范围。**
>**因为STA的最高位和最低位是状态位，所以STA最高可以计数到0x3f = 0011 1111。**

```c
// 捕获中断发生时执行 上升沿复位开始计时，下降沿获取捕获值计算
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((TIM2CH2_CAPTURE_STA & 0X80) == 0) //还未捕获成功  [7]:0,没有成功的捕获;1,成功捕获到一次.
  {
    if (TIM2CH2_CAPTURE_STA & 0X40) // 成功率捕获到1个下降沿  [6]:0,还没捕获到低电平;1,已经捕获到低电平了.
    {
      // usart_printf("get down\r\n");
      TIM2CH2_CAPTURE_STA |= 0X80;                                            // 标记成功，捕获到1次高电平完成
      TIM2CH2_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2); // 捕获当前设置捕获值
      TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);                       // 清除原来设置
      TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);  // 捕获到下降沿之后，将捕获到复位为上升沿
    }
    else // 捕获到一个上升沿
    {
      // usart_printf("get up\r\n");
      TIM2CH2_CAPTURE_STA = 0;
      TIM2CH2_CAPTURE_VAL = 0;
      TIM2CH2_CAPTURE_STA |= 0X40;                                            //将STA置为0x40 当下一次触发中断时，会进入上面的if语句
      __HAL_TIM_DISABLE(&htim2);                                              //关闭定时器
      __HAL_TIM_SET_COUNTER(&htim2, 0);                                       //将定时器计数值清零
      TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2);						  //清除输入捕获标志位
      TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING); //将输入捕获上升沿改为捕获下降沿
      __HAL_TIM_ENABLE(&htim2);												  //使能定时器，开启定时器
    }
  }
}
```
>中断函数，当定时器捕获到边沿变换时，会执行下列操作，第一个if前面已经讲过，咱直接看里面的if else

>我们先看else，当**我们捕获到上升沿时会进入else语句**中，此时会将**寄存器清零，并将次高位置1**，表示**已经捕获到上升沿，接下来要捕获下降沿**，VAL清零。然后将**定时器复位**。
>**else语句完成后，定时器会复位并被打开，且此时开始捕捉下降沿**

>当定时器再一次捕捉时，捕捉到的是下降沿，此时超声波模块已经完成了一次工作，我们只需把时间记录下来就可以计算距离了。
>首先使**STA最高位置1，使程序跳出上面的溢出函数**，这时STA不再自加，然后将**定时器里的值读取到VAL**中。最后修改复位一下定时器，即**完成了一次超声波模块的完整工作**

```c
void HCSR_04()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}
float getSR04Distance()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM2CH2_CAPTURE_STA & 0X80) //输入捕获 触发
    {
        time = TIM2CH2_CAPTURE_STA & 0X3f;   //获得溢出次数
        time *= 65536;                       //一次溢出为65536 得到溢出的时间
        time += TIM2CH2_CAPTURE_VAL;         //溢出的时间+现在定时器的值 得到总的时间
        len = time * 342.62 * 100 / 2000000; // 计算得到距离

        TIM2CH2_CAPTURE_STA = 0; //清除溢出
    }
    return len;
}
```
>我们在main函数中调用了这两个函数，上面一个函数用来**激活超声波模块**，根据超声波模块的使用要求，先**将Trig引脚置高，等待至少10us，再将Trig置低**，就会激活超声波模块开始工作

>下面这个函数用来计算距离，当超声波模块完成一次工作后，STA的最高位会被置为1
>**TIM2CH2_CAPTURE_STA |= 0X80; 将最高位置为1；**
>此时程序不会再进行输入捕获，所以在我们计算得到距离后，需要将STA清零，使得超声波可以进行下一次工作，**TIM2CH2_CAPTURE_STA = 0;**
>距离是由时间计算出来的，而时间由两部分组成，一部分是定时器溢出次数，一部分是定时器内还未溢出的值，即**STA*65536+VAL**
>得到time后，按照公式即可算出距离**len = time * 342.62 * 100 / 2000000;**



# 注意事项
**当返回值一直为0
1、首先检查配置，特别是cubemx中定时器中断是否开启
2、超声波模块是否激活，Trig引脚是否拉高了有10个us，一般在15到20us**

>[github下载地址：https://github.com/FollowTheWay/HC_SR04.git](https://github.com/FollowTheWay/HC_SR04.git)
[CSND下载地址：https://download.csdn.net/download/qq_51967985/87196492](https://download.csdn.net/download/qq_51967985/87196492)
