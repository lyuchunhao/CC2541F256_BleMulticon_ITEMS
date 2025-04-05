主机：SimpleBLECentral_Mulconnect              1. 原始版本(东方)   ok
      SimpleBLECentral_Mulconnect_V1.0         1. 封装优化代码     ok
	  SimpleBLECentral_Mulconnect_V1.1         1. 处理变量：connectedPeripheralNum/simpleBLEConnHandle变量,
	                                           2. 2018-05-12 解决断开一个从机，另一个无法通讯的问题，simpleBLECharHdl的完善处理
											   3. BLE部分已经解决OK
	  SimpleBLECentral_Mulconnect_V1.1.0       1. 在版本V1.1的基础上给陈东旭提供一个版本：准备添加LCD和键盘	
                                               2. 讨论决定不再添加LCD和键盘，添加一个CPU通过串口通讯	 

      SimpleBLECentral_Mulconnect_V1.1.1	   1. 测试下GATTServApp_ProcessCharCfg()函数的主接收，因为刺激器中的从机发送采用的是此种方式	
                                               2. AT+NOTIFY使能从机Notify功能(simpleBLEEnableNotify(0, 0x2E))[扫到char4句柄为0x2E,但在函数内部写需要+1=0x2F]	
                                                  可以接收到TI-Demo中的0x03	
                                               3. 整理了扫描数据及扫描回应数据的串口打印	
	  
	  SimpleBLECentral_Mulconnect_V2.0         1. 添加串口通信协议
	                                           2.  未添加主从机LCD协议前的成功版本
											   3. 支持AT指令,Uart接收数据转发给从机
											   4. 扫描、连接、断开过程需要单步骤AT指令控制
											   
	  SimpleBLECentral_Mulconnect_V2.1.0       1. 添加正式LCD及与刺激器通讯的代码
	                                           2. 测试OK
	  
	  
从机：SimpleBLEPeripheral_Mulconnect           1. BLE部分OK
      SimpleBLEPeripheral_Mulconnect_V1.0      2. 添加MCP4728-I2C-DA部分
	  SimpleBLEPeripheral_Mulconnect_V1.1.0    3. 完善MCP4728快速写连续写API+开启两个定时器实现4路PWM
	                                           4. (不准备用定时器的PWM:由于Timer2/3配置的PWM周期0xFF固定,Timer1的1234通道定时器可
											      有CCTL0控制但4路PWM周期会一样)
											  
	  SimpleBLEPeripheral_Mulconnect_V1.1.1    1. TimerClkConfig()函数将定时器标定频率改为8M
	                                           2. 屏蔽掉函数HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT )解决了PWM频率不准问题
											   3. MCP4728-DAC以及PWM高频/低频 OK(1-30不能整除的频率误差稍大)
											   4. 需要后期验证1和2是否会对BLE通讯产生影响
											   5. 需要屏蔽HAL_LED=FALSE，否则会与硬件PWM冲突造成BLE无法连接
											  
	  SimpleBLEPeripheral_Mulconnect_V1.1.2    1. 高频PWM由硬件PWM实现(Timer3/Timer4)
	                                           2. 低频PWM由软件PWM实现(Timer1的20us定时中断+IO翻转)
											   3. 屏蔽掉函数：HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT )
											      [MCU停机的时候系统时钟会不会分频,否则会影响PWM频率]
											   4. 添加新函数：HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE)     
											      [RF工作的时候不停止CPU的运行,否则中断会受影响]
											   5. BLE连接正常,频繁的定时中断并没与RF相互影响
											   6. HAL_LED = FALSE 因为里面定义了P1_0和P1_1和PWM通道相互影响从而导致BLE连接失败
											   7. 可以作为success版本
											   8. simpleGATTprofile.c中的// Characteristic Value 4属性定义为GATT_PERMIT_READ才可以扫到 char4的句柄
											   
	   SimpleBLEPeripheral_Mulconnect_V2.0     1. 添加串口通信协议
	                                           2. 收到串口数据转发给主机
											   
       SimpleBLEPeripheral_Mulconnect_V2.1.0   1. 接收主机BLE数据控制PWM及DA
	   
	   SimpleBLEPeripheral_Mulconnect_V2.1.1   1. 从称重LED-IIC代码中移植过来的代码
	                                           2. 解决DAC-IIC对蓝牙通讯的影响
											   3. 测试OK