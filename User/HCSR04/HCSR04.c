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
#include "usart.h"
#include "User_Debug.h"
/**
 * @brief ���������ʱ��
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
 * @brief ���㳬�������ľ���
 *
 * @return float
 */
uint8_t unit_change;
float getSR04Distance()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM2CH2_CAPTURE_STA & 0X80) // ���벶�� ����
    {
        time = TIM2CH2_CAPTURE_STA & 0X3f; // ����������
        time *= 65536;                     // һ�����Ϊ65536 �õ������ʱ��
        time += TIM2CH2_CAPTURE_VAL;       // �����ʱ��+���ڶ�ʱ����ֵ �õ��ܵ�ʱ��
        if (unit_change == 0)
        {
            len = time * 342.62 * 100 / 2000000; // ����õ����� cm
            usart_printf("cm\r\n");
        }
        else if (unit_change == 1)
        {
            len = time * 342.62 * 100 / 200000000; // ����õ����� m
            usart_printf("m\r\n");
        }

        TIM2CH2_CAPTURE_STA = 0; // ������
    }
    return len;
}
