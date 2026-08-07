#ifndef MENU_H_
#define MENU_H_

/*
 * 初始化题目业务、四个短按键、A11 电池 ADC、固定 IPS200、八路灰度
 * 传感器和 K230 UART1。Q0 显示最新 Ball_X；Q6 显示并采用最新触摸目标。
 * Q0 停车态下，KEY_2 只触发回到驱动器既有机械零点，KEY_3 只循环固定绝对脉冲表；
 * Q6 停车态 KEY_2 将 RAM 运行中心减 20 pulse，KEY_3 将其加 20 pulse，成功
 * 发送绝对位置后才提交新中心。任何按键都不能修改驱动器机械零点或回零配置。
 * KEY_4 启动 Q0 自定义调试模式、Q2 或 Q3..Q6。
 */
void my_menu_init (void);

/*
 * 每次调用都先消费 K230 UART1 完整帧并处理 Q6 Touch_X。非题目态在 CPU0
 * 前台扫描短按事件，每 150 ms 刷新灰度、IMU、编码器、题目状态和电池。
 * Q0 KEY_3 测试成功发送后立即输出首行 PT 记录，随后每 150 ms 输出一次；
 * 非零测试位置禁止 KEY_1/KEY_4，须继续 KEY_3 序列或用 KEY_2 触发回零。
 * 题目态不扫描按键、不刷新屏幕，只保留 K230 收帧、Q6 动态目标和
 * 9 ms 电池 ADC 更新；
 * IMU 与灰度仍由固定 3 ms 调度 ISR 每拍采样。
 */
void my_menu_proc (void);

#endif
