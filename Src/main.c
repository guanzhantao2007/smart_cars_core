/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu.h"
#include "pid_tune.h"
#include "serial_async.h"
#include "task.h"
#include "yaw_loop.h"
#include <stdint.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
all_task_state_t current_task_state=SELECT_TASK;//创建全局变量，表示当前任务状态
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 最近一次故障的触发点与时刻。调试期用；定位完可以整段删掉。 */
static const char * volatile s_fault_tag;
static volatile uint32_t    s_fault_tick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================ 故障上报 ============================
 * 旧版 Error_Handler 是 __disable_irq() + while(1)，后果有两层：
 *   1) 关中断 —— USART2 的收发中断、TIM7 的控制节拍一起停；
 *   2) 死循环 —— 主循环再也走不到 PidTune_Process()，命令永远不处理。
 * 于是只要有一次初始化失败、或 USART1 收到一个坏字节，整块板就静默死亡，
 * 而现象和"USB-TTL 接错口"完全一样，光看是分不出来的。
 *
 * 现在改成四步：记下触发点 → 开中断 → 点亮两颗用户灯 → 寄存器直发打一行 → 返回。
 * 用寄存器直发（Serial_EmergencyPuts）而不是 printf，是因为故障发生时
 * HAL 的 gState 可能还停在 BUSY_TX，printf 会一个字都发不出来。
 * ================================================================ */
void Fault_Report(const char *tag)
{
  s_fault_tag  = tag;
  s_fault_tick = HAL_GetTick();

  __enable_irq();          /* 关键：中断不关，串口和定时器才有机会活过来 */

  /* 两颗用户灯常亮 = "报过错"。这是没有串口时唯一的物理证据：
     灯亮 = 跑过、但某步失败了；灯不亮 = 根本没跑到这里。 */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin, GPIO_PIN_SET);

  Serial_EmergencyPuts("\r\n#FAULT ");
  Serial_EmergencyPuts((tag != 0) ? tag : "?");
  Serial_EmergencyPuts("\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef boot_status;
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  /* 【上电进度打点 · 调试期临时增加】
   * 这一行是判断"死在上半段还是下半段"的分界线：
   *   - 看得到它 —— 说明所有 MX_xxx_Init 都过了，问题在 Task_Init 里，
   *                 后面每一步都有各自的 [BOOT] / #FAULT 标签；
   *   - 看不到它 —— 说明死在上面某个 MX_xxx_Init() 里，那时 USART2 的时钟
   *                 可能还没开，串口打不出字，只能看两颗 LED 是否常亮。
   * 定位完之后这几行 printf 可以整段删掉。 */
  printf("\n[BOOT] all MX_xxx_Init done\n");

  printf("[BOOT] Task_Init start\n");
  boot_status = Task_Init();
  if (boot_status != HAL_OK)
  {
    /* 不再进入死循环：只上报，然后继续把串口调参拉起来。
       外设坏一半也总比整块板哑掉强 —— 至少能读回寄存器和参数。
       注意这里先判 s_fault_tag：Task_Init 内部已经用精确标签
       （"OLED_Init"/"IMU_Init"…）报过一次，不能再被 "Task_Init" 盖掉，
       否则 500 ms 重复提醒里那个最有用的名字就丢了。 */
    if (s_fault_tag == 0)
    {
      Fault_Report("Task_Init");
    }
  }

  /* 【2026-11-22 零偏学习开关】补上 s_learn_en 的置位点。
   *
   * 背景：IMU_SetBiasLearn() 在全工程只有定义和声明，**零处调用**；
   *   而 s_learn_en 在 IMU_Init 里被清 0 —— 于是 IMU_ONLINE_BIAS_ENABLE=1
   *   只是把那段 #if 编译进来了，开关本身从没打开过。
   *   后果：上电 500 次校准留下的残差永远扣不掉。实测该残差为 −0.079 dps，
   *   而 yaw 是 gz 的纯积分，60 s 静止就漂 −7.8°（−7.79 °/min）。
   *
   * 为什么现在才开：重启后残差已经降到 0.08 dps，正是让在线学习继续磨平的
   *   时候。若残差仍是之前那种 −3.4 dps 的量级，**先别开** —— 那个量级说明
   *   上电校准整体没成功，学习只会把错误基线一点点搬进 s_bias。
   *
   * 注意本行只解决"开关没开"这一半。学习真正跑起来还要求
   *   still_cnt > IMU_STILL_TICKS(300)，而静止判据要求 |gx|,|gy|,|gz|
   *   全部 < 0.6 dps 且加速度模长偏差 < 0.05g。当前回显是 ST=1 / SET=0，
   *   说明那道门没过 —— 得看 gx/gy，波形没有这两列，另议。
   *
   * 写法注意：在 USER CODE BEGIN 2 内，CubeMX 重新生成不会覆盖。 */
#if TASK_IMU_ENABLE
  IMU_SetBiasLearn(1U);
  printf("[BOOT] bias learn on\n");
#else
  /* IMU 已关（task.h 的 TASK_IMU_ENABLE=0）：零偏学习没有载体，
   * 这句置位只会让启动日志说谎，所以一起跳过。 */
  printf("[BOOT] bias learn skipped (IMU off)\n");
#endif

  /* 【2026-11-22 T09 补漏】角度环初始化。
   *
   * 为什么必须在这里补：YawLoop_Init() 原本**全工程零调用点**，
   *   于是 s_yaw 落在 BSS 里全是 0 —— 包括 pid.OutMax。
   *   YawLoop_Tick() 算完 ω 后按 ±OutMax 限幅，0 就把结果压成 0，
   *   波形 CH13 恒 0；而"误差本来为 0"时 CH13 也是 0，两者从波形上
   *   完全分不出来，看起来像角度环没接进来。
   *
   * 为什么放在这个位置（Task_Init 之后、PidTune_Init 之前）：
   *   1) 必须在 Task_Init 之后 —— 模块本身不碰硬件，但保持"先外设后逻辑"
   *      的上电顺序，Init 里若将来加了读 IMU 的动作也不会踩空；
   *   2) 必须在 Task_Init 的失败分支**之外** —— 即使外设坏了一半，
   *      也要把角度环置成可用状态，否则串口还能收命令、CH13 却永远是 0，
   *      故障现象会叠加，更难分离；
   *   3) 真正的硬约束是「必须在 PidTune_WaveEnable() 之前」—— 波形一开始
   *      推送，CH13 就会被人解读，那一刻 OutMax 必须是 100。本句在中间，
   *      波形开关在最后一行，天然满足。
   *
   * 【一处容易推错的事实，已核对源码】TIM7 不是在这里启动的，而是在
   *   encoder.c:101 的 Encoder_Init() 里就 HAL_TIM_Base_Start_IT(&htim7)
   *   了 —— 也就是说 TIM7 比本句更早开始跑，YawLoop_Tick() 在本句之前
   *   已经被调用若干次。那几次不会出错：此时 state 是 BSS 里的 0，即
   *   YAW_OFF，Tick 在判 state 那一步就 return 并把 ω 置 0，跟 OutMax
   *   是 0 还是 100 无关。（所以"晚于 TIM7 启动"不是缺陷，不必为此把
   *   Init 提到 Task_Init 之前 —— 那反而会让它跑在时钟配置之前。）
   *
   * 写法注意：这句在 USER CODE BEGIN 2 里，CubeMX 重新生成代码不会覆盖。
   *   （对比：TIM7 回调里的 YawLoop_Tick() 调用在 stm32f4xx_it.c 的
   *     HAL_TIM_PeriodElapsedCallback 中，那段没有 USER CODE 包裹，
   *     重新生成 .c 会丢掉 —— 这是已知遗留风险，改 .ioc 前要先备份。） */
  YawLoop_Init();

  /* 调参模块必须在 Task_Init() 之后：那时 motor/encoder 已初始化完，
   * 且 printf 那句"初始化成功"已经打完，不会和波形文本混在一起。 */
  printf("[BOOT] PidTune_Init start\n");
  PidTune_Init();

  printf("[BOOT] ready. fault=%s\n",
         (s_fault_tag != 0) ? (const char *)s_fault_tag : "none");
  printf("[BOOT] try: ?   /   tgt 30   /   o 20\n");

  /* 启动日志到此打完，才允许 TIM7 把 50 Hz 波形推出去。
   * 顺序反了会怎样：波形（IT 发送）和 printf（阻塞发送）抢同一个 USART2->DR，
   * 日志被截成半行、和波形数字粘在一起，"打到哪一步"就看不出来了。 */
  PidTune_WaveEnable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 【2026-11-22 P0 新增】姿态解算的"运算"必须由主循环驱动。
     *
     * 为什么以前没有这一句也能编译通过、也没有任何报错：
     *   IMU_Tick1ms() 只负责 s_tick++（在 TIM6 里，940 Hz），
     *   而真正读传感器、扣零偏、积分 yaw 的是 IMU_Update()。
     *   全工程从来没人调用它 —— 于是 IMU_GetYawDeg() 恒返回 0，
     *   而且这个"0"看起来完全合法，不会触发任何错误路径。
     *
     * 为什么放在主循环而不是 TIM6/TIM7 中断里：
     *   里面是一次阻塞式 I2C 读（400 kHz 下约 350 µs）。塞进 10 ms 的
     *   控制中断会吃掉 3.5% 并让 PID 节拍抖动；塞进 940 Hz 中断更糟。
     *
     * 它自己会判断该不该干活：dt_ticks==0 直接返回，所以这里每轮调用
     * 等于"约 940 Hz 采样"，不会重复读。
     * 代价：主循环里约 1/3 的 CPU 时间花在这次 I2C 读上。如果以后嫌重，
     *   在 imu.c 里按 tick 分频（比如每 2 拍读一次）即可，dt 是按 tick 算的，
     *   分频不会引入积分误差。 */
#if TASK_IMU_ENABLE
    IMU_Update();
#endif

    if (Task1_Run() == TASK_ERROR)
    {
      Error_Handler();
    }

    /* 收 VOFA 下发的命令（非阻塞，有几条处理几条）。
     * 波形不在这里发 —— 波形由 TIM7 的中断直接发，间隔才稳定。 */
    PidTune_Process();

    /* 报过故障就每 500 ms 再提醒一次，但绝不卡住主循环：
     * 这样"死在哪一步"和"命令还能不能进来"可以同时观察到。
     * 用寄存器直发而不是波形通道，是为了保证即使 HAL 的发送状态机
     * 被卡住，这行提醒也一定能出来。 */
    if (s_fault_tag != 0)
    {
      if ((uint32_t)(HAL_GetTick() - s_fault_tick) >= 500U)
      {
        s_fault_tick = HAL_GetTick();
        Serial_EmergencyPuts("#FAULT ");
        Serial_EmergencyPuts((const char *)s_fault_tag);
        Serial_EmergencyPuts("\r\n");
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 【调试期改动】原来这里是 __disable_irq() + while(1)。
   *
   * 为什么必须改：关中断会把 USART2 的收发中断一起停掉，死循环又让主循环
   * 再也走不到 PidTune_Process()。结果是"串口一个字节都不动"，跟接错口、
   * 波特率不对、RX 没接线的现象完全一样，无法区分。
   *
   * 现在：记一笔、点灯、打一行、返回。返回后调用方会继续往下走 ——
   * 对初始化类代码来说这是安全的（后面都是寄存器配置，失败也只是没配成）；
   * 代价是"错误会被继续执行掩盖"，所以正式版发布前要把这条改回死循环，
   * 只保留 Fault_Report 里那句记录。
   *
   * 这里只知道"HAL/CubeMX 的统一入口被调用了"，不知道具体是谁，
   * 所以统一记 "hal/mx"；要精确到步骤，就在各步骤里自己调
   * Fault_Report("那一步的名字")。 */
  Fault_Report("hal/mx");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
