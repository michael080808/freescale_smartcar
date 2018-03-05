# NXP（恩智浦）【原Freescale（飞思卡尔）】 
# 东南大学第十二届智能车校赛摄像头组
## 免责声明

该代码库仅用于自身队伍的代码托管，并不保证代码本身的可用性和稳定性。


代码的发布和使用均用于学术研究目的，不利用其进行商业开发，


遵循GPL-3.0开源许可，请务必遵循GPL-3.0许可证协议进行代码的使用和修改。


## 开发环境

- ARM系列：ARM Cortex-M4（不支持硬件浮点运算）


- 核心板系列：NXP(Freescale) Kinetis K Series


- 核心板型号：MK60DN512VLL10


- 开发核心：SEU核心板 v1.4 


- 开发扩展：2018 SEU Demo扩展板


- 摄像头：岱默科技OV7620


- 车模类型：飞思卡尔智能车官方B车（旧B车，不是新2B车）


- 调试器：SEGGER J-Link v8


- 调试器驱动：SEGGER J-Link Driver v6.10


- 开发环境：IAR Embedded Workbench 7.1


- 核心驱动库：超核K60 v2.4


- 操作系统：无


- 文件管理：无


## 代码管理

目录结构如下：

- Debug：由IAR Embedded Workbench自动生成，用于代码的二进制执行文件的生成和调试信息存储

- Libraries：本工程的核心驱动库存放位置，使用超核K60开发库

- Settings：由IAR Embedded Workbench自动生成，用于存储工程信息

- User：用户代码存放区域
	
	- main.c 程序入口

	- camera.h/.c OV7620摄像头初始化、图像处理、OLED显示图像及串口转发图像

	- encoder.h/.c 编码盘初始化、 获取编码盘脉冲数量、 速度转换、OLED显示、串口转发
	
	- isr.h/.c 中断处理函数，行、场（GPIO_PORTC组）外部中断处理函数存放位置。 <br><font color="#FF0000">属于该工程内一个Bug，不能放置其它额外的中断处理函数，该函数也不能移动到其它文件，否则编译报错。</font>

	- oled.h/.c OLED单色显示屏（128*64分辨率）显示驱动

	- pid.h/.c PID控制器实现

	- sccbext.h/.c 摄像头SCCB(Serial Camera Control Bus)协议控制，使用不精准的延时实现

- 根目录下的其它文件为IAR的工程启动入口和工程信息文件

## 实现原理

对获得的图像上近端若干行的中点偏离值进行采集，求加权平均

计算实际偏差作为控制舵机的PD的输入，将获得的空行行数作为电机速度PID的输入

## 代码状态

舵机位置式PD和电机增量式PID已经完成，但是还没最后确定PID的参数值。

电池没电了无法进行测试。停车仍然没有实现，在PID调参完成后实现。