������SimpleBLECentral_Mulconnect              1. ԭʼ�汾(����)   ok
      SimpleBLECentral_Mulconnect_V1.0         1. ��װ�Ż�����     ok
	  SimpleBLECentral_Mulconnect_V1.1         1. ���������connectedPeripheralNum/simpleBLEConnHandle����,
	                                           2. 2018-05-12 ����Ͽ�һ���ӻ�����һ���޷�ͨѶ�����⣬simpleBLECharHdl�����ƴ���
											   3. BLE�����Ѿ����OK
	  SimpleBLECentral_Mulconnect_V1.1.0       1. �ڰ汾V1.1�Ļ����ϸ��¶����ṩһ���汾��׼�����LCD�ͼ���	
                                               2. ���۾����������LCD�ͼ��̣����һ��CPUͨ������ͨѶ	 

      SimpleBLECentral_Mulconnect_V1.1.1	   1. ������GATTServApp_ProcessCharCfg()�����������գ���Ϊ�̼����еĴӻ����Ͳ��õ��Ǵ��ַ�ʽ	
                                               2. AT+NOTIFYʹ�ܴӻ�Notify����(simpleBLEEnableNotify(0, 0x2E))[ɨ��char4���Ϊ0x2E,���ں����ڲ�д��Ҫ+1=0x2F]	
                                                  ���Խ��յ�TI-Demo�е�0x03	
                                               3. ������ɨ�����ݼ�ɨ���Ӧ���ݵĴ��ڴ�ӡ	
	  
	  SimpleBLECentral_Mulconnect_V2.0         1. ��Ӵ���ͨ��Э��
	                                           2.  δ������ӻ�LCDЭ��ǰ�ĳɹ��汾
											   3. ֧��ATָ��,Uart��������ת�����ӻ�
											   4. ɨ�衢���ӡ��Ͽ�������Ҫ������ATָ�����
											   
	  SimpleBLECentral_Mulconnect_V2.1.0       1. �����ʽLCD����̼���ͨѶ�Ĵ���
	                                           2. ����OK
	  
	  
�ӻ���SimpleBLEPeripheral_Mulconnect           1. BLE����OK
      SimpleBLEPeripheral_Mulconnect_V1.0      2. ���MCP4728-I2C-DA����
	  SimpleBLEPeripheral_Mulconnect_V1.1.0    3. ����MCP4728����д����дAPI+����������ʱ��ʵ��4·PWM
	                                           4. (��׼���ö�ʱ����PWM:����Timer2/3���õ�PWM����0xFF�̶�,Timer1��1234ͨ����ʱ����
											      ��CCTL0���Ƶ�4·PWM���ڻ�һ��)
											  
	  SimpleBLEPeripheral_Mulconnect_V1.1.1    1. TimerClkConfig()��������ʱ���궨Ƶ�ʸ�Ϊ8M
	                                           2. ���ε�����HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT )�����PWMƵ�ʲ�׼����
											   3. MCP4728-DAC�Լ�PWM��Ƶ/��Ƶ OK(1-30����������Ƶ������Դ�)
											   4. ��Ҫ������֤1��2�Ƿ���BLEͨѶ����Ӱ��
											   5. ��Ҫ����HAL_LED=FALSE���������Ӳ��PWM��ͻ���BLE�޷�����
											  
	  SimpleBLEPeripheral_Mulconnect_V1.1.2    1. ��ƵPWM��Ӳ��PWMʵ��(Timer3/Timer4)
	                                           2. ��ƵPWM�����PWMʵ��(Timer1��20us��ʱ�ж�+IO��ת)
											   3. ���ε�������HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT )
											      [MCUͣ����ʱ��ϵͳʱ�ӻ᲻���Ƶ,�����Ӱ��PWMƵ��]
											   4. ����º�����HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_DISABLE)     
											      [RF������ʱ��ֹͣCPU������,�����жϻ���Ӱ��]
											   5. BLE��������,Ƶ���Ķ�ʱ�жϲ�û��RF�໥Ӱ��
											   6. HAL_LED = FALSE ��Ϊ���涨����P1_0��P1_1��PWMͨ���໥Ӱ��Ӷ�����BLE����ʧ��
											   7. ������Ϊsuccess�汾
											   8. simpleGATTprofile.c�е�// Characteristic Value 4���Զ���ΪGATT_PERMIT_READ�ſ���ɨ�� char4�ľ��
											   
	   SimpleBLEPeripheral_Mulconnect_V2.0     1. ��Ӵ���ͨ��Э��
	                                           2. �յ���������ת��������
											   
       SimpleBLEPeripheral_Mulconnect_V2.1.0   1. ��������BLE���ݿ���PWM��DA
	   
	   SimpleBLEPeripheral_Mulconnect_V2.1.1   1. �ӳ���LED-IIC��������ֲ�����Ĵ���
	                                           2. ���DAC-IIC������ͨѶ��Ӱ��
											   3. ����OK