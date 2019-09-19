# I_NUCLEO_LRWAN1 MANUAL CONTROL VIA UART

Description: 

UART interface developed in a Nucleo-L476RG Board, it is used to receive AT Commands from a Terminal (USART1/2) and transfer them to a I-NUCLEO-LRWAN1 Shield (USART3) to control it. 

Connection diagram:


+---------------+       +---------------------------+       +-------------------
                |       |                           |       |
                |       |                           |       |
                |       |                           |       |
            TX  |-------|RX                       RX|-------|TX
                |       |                           |       |
    TERMINAL    |       |USART1/2   NUCLEO    USART3|       |   I-NUCLEO-LRWAN1
                |       |           L476RG          |       |
            RX  |-------|TX                       TX|-------|RX
                |       |                           |       |
                |       |                           |       |
                |       |                           |       |
+---------------+       +---------------------------+       +-------------------

Communication parameters (USART 1,2 & 3):

Baudrate: 115200
Data: 8 Bits
Parity: None
Stopbits: 1

Pin Assignment:

USART1

TX: PA9
RX: PA10

USART2

TX: PA2
RX: PA3

USART3

TX: PC10
RX: PC11

References:

Mastering STM32

https://leanpub.com/mastering-stm32

WM-SG-SM-42 AT Command Reference Manual

https://github.com/USIWP1Module/USI_I-NUCLEO-LRWAN1/blob/master/WM-SG-SM-42%20AT%20Command%20Reference%20Manual%20rev.2.3_20180701.pdf

To Do:






