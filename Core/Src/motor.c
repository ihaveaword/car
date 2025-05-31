#include"motor.h"
extern TIM_HandleTypeDef htim1; // 定义定时器句柄

int abd(int p)
{
    if (p < 0)
        return -p;
    else
        return p;
}
void load(int moto1 ,int moto2)     //-100 100
{
    if (moto1 < 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // A电机正转
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // A电机反转
    }
    else if (moto1 > 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // A电机正转
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // A电机反转
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // A电机停止
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // A电机停止
        
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, abd(moto1)); // 设置A电机PWM占空比
  
    if (moto2 < 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // B电机正转
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // B电机反转
    }
    else if (moto2 > 0)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // B电机正转
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // B电机反转
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // B电机停止
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // B电机停止
    } 
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, abd(moto2)); // 设置B电机PWM占空比
}
