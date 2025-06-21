# 新文档

正点原子STM32F407VGT6最小系统板  
调试串口：usart1 引脚：PA9(Tx) PA10(Rx) 波特率：921600  
大疆设备通信串口：usart2 引脚：PA2(Tx) PA3(Rx) 
授权信息文件路径：Core/Inc/dji_sdk_app_info.h
# 原文档

Onboard SDK RTOS Sample.   


Recommend Environment:  
Hardware:               STM32F407xG (External Crystal Frequency: 8MHz)
IDE:                    Keil MDK v5.25.2.0  
C Compiler:             Armcc.exe V5.06 update 6 (build 750)  
Assembler:              Armasm.exe V5.06 update 6 (build 750)  
Linker/Locator:         ArmLink.exe V5.06 update 6 (build 750)  
Library Manager:        ArmAr.exe V5.06 update 6 (build 750)  
Hex Converter:          FromElf.exe V5.06 update 6 (build 750)  


Other toolchain:  
Hardware:               STM32F407xG (External Crystal Frequency: 8MHz)  
IDE:                    Clion 2021.1.3  
C Compiler:             gcc version 9.3.1 20200408 (release) (GNU Arm Embedded Toolchain 9-2020-q2-update)  

Pin Definition on Extension Port:  
Console UART:           PA2(TX), PA3(RX)  
Communication UART:     PB10(TX), PB11(RX)  
PPS PIN:                PD2  
LED PIN:                PD12  
USB OTG Connector:      PA11, PA12  

Pin Definition on Payload Port:  
Console UART:           PA2(TX), PA3(RX)  
Communication UART:     PB10(TX), PB11(RX)  
HIGH POWER APPLY PIN:   PD1  
PPS PIN:                PD2  
LED PIN:                PD12 

Console Configuration:
Baud Rate:              921600