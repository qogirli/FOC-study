# 电流采样学习工程

从 `../260901-openloop-svpwm` 当前工作区复制，保留时钟、GPIO、TIM8、ADC 初始化和原有 SVPWM 算法。没有重新运行 CubeMX。

打开 `MDK-ARM/260818-start-six-foc.uvprojx`。工程内部名称和同名 `.ioc` 暂时保留，避免破坏 CubeMX 与 Keil 的关联；通过所在目录区分学习阶段。

## 当前边界

- `Core/Src/main.c` 中 `CURRENT_SAMPLING_ONLY = 1`。
- 不启动 TIM8 PWM 输出与 CH4 中断，不处理按键启动请求。
- PH9（代码别名 LED1）保持低电平，关闭驱动；回调入口也禁止进入使能逻辑。
- ADC1 保留三通道初始化，尚未启动采集、校准零点或接入电流 PI。
- 没有复制旧编译产物。烧录前必须重新编译，不能使用旧工程的 HEX。

下一步：功率级关闭时，可靠地读取三个 ADC rank 并保存，然后逐路统计零电流偏置。不要在扫描结束后连续读取 DR 三次当成三相数据。

本阶段未做硬件验证。解除 CURRENT_SAMPLING_ONLY 前，先完成采样与启动时序检查。
