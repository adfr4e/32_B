
; Amount of memory (in bytes) allocated for Stack
Stack_Size      EQU     0x00000100

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; Amount of memory (in bytes) allocated for Heap
Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD    Reset_Handler              ; Reset Handler
                DCD    NMI_Handler                ; NMI Handler
                DCD    HardFault_Handler          ; Hard Fault Handler
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    0
                DCD    SVC_Handler                ; SVCall Handler
                DCD    0                          ; Reserved
                DCD    0                          ; Reserved
                DCD    PendSV_Handler             ; PendSV Handler
                DCD    SysTick_Handler            ; SysTick Handler

                ; External Interrupts
				DCD    UART0_Handler          
				DCD    TIMPLUS0_Handler         
				DCD    PWMBASE0_Handler          
				DCD    PWMPLUS0_Handler          
				DCD    IIC0_Handler           
				DCD    ADC_Handler          
				DCD    SPI0_Handler            
				DCD    IWDT_Handler           
				DCD    GPIOA0_Handler          
				DCD    GPIOA1_Handler          
				DCD    GPIOA2_Handler          
				DCD    GPIOA3_Handler            
				DCD    GPIOA4_Handler         
				DCD    GPIOA5_Handler           
				DCD    GPIOA6_Handler         
				DCD    GPIOA7_Handler          
				DCD    GPIOA8_Handler   
				DCD    GPIOA9_Handler     
				DCD    GPIOA10_Handler      
				DCD    GPIOA11_Handler    
				DCD    GPIOA12_Handler      
				DCD    GPIOA13_Handler       
				DCD    GPIOA14_Handler   
				DCD    GPIOA15_Handler  
				DCD    IRQ24_Handler     
				DCD    IRQ25_Handler    
				DCD    IRQ26_Handler    
				DCD    IRQ27_Handler   
				DCD    IRQ28_Handler     
				DCD    IRQ29_Handler     
				DCD    IRQ30_Handler   
				DCD    IRQ31_Handler  
				
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

Reset_Handler    PROC
                 EXPORT  Reset_Handler          [WEAK]
				 IMPORT  __main
			 
				 LDR     R0, =__main
                 BX      R0
				 
                 ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler             [WEAK]
                B       .
                ENDP

HardFault_Handler PROC
                EXPORT  HardFault_Handler       [WEAK]
                B       .
                ENDP
				
SVC_Handler     PROC
                EXPORT  SVC_Handler             [WEAK]
                B       .
                ENDP

PendSV_Handler  PROC
                EXPORT  PendSV_Handler          [WEAK]
                B       .
                ENDP

SysTick_Handler PROC
                EXPORT  SysTick_Handler         [WEAK]
                B       .
                ENDP

UART0_Handler           PROC
                EXPORT  UART0_Handler           [WEAK]
                B       .
                ENDP

TIMPLUS0_Handler          PROC
                EXPORT  TIMPLUS0_Handler          [WEAK]
                B       .
                ENDP

PWMBASE0_Handler           PROC
                EXPORT  PWMBASE0_Handler           [WEAK]
                B       .
                ENDP

PWMPLUS0_Handler           PROC
                EXPORT  PWMPLUS0_Handler           [WEAK]
                B       .
                ENDP

IIC0_Handler            PROC
                EXPORT  IIC0_Handler            [WEAK]
                B       .
                ENDP

ADC_Handler           PROC
                EXPORT  ADC_Handler           [WEAK]
                B       .
                ENDP

SPI0_Handler             PROC
                EXPORT  SPI0_Handler             [WEAK]
                B       .
                ENDP

IWDT_Handler            PROC
                EXPORT  IWDT_Handler            [WEAK]
                B       .
                ENDP

GPIOA0_Handler          PROC
                EXPORT  GPIOA0_Handler          [WEAK]
                B       .
                ENDP

GPIOA1_Handler          PROC
                EXPORT  GPIOA1_Handler          [WEAK]
                B       .
                ENDP

GPIOA2_Handler           PROC
                EXPORT  GPIOA2_Handler           [WEAK]
                B       .
                ENDP

GPIOA3_Handler             PROC
                EXPORT  GPIOA3_Handler             [WEAK]
                B       .
                ENDP

GPIOA4_Handler          PROC
                EXPORT  GPIOA4_Handler          [WEAK]
                B       .
                ENDP

GPIOA5_Handler            PROC
                EXPORT  GPIOA5_Handler            [WEAK]
                B       .
                ENDP

GPIOA6_Handler          PROC
                EXPORT  GPIOA6_Handler          [WEAK]
                B       .
                ENDP

GPIOA7_Handler          PROC
                EXPORT  GPIOA7_Handler          [WEAK]
                B       .
                ENDP

GPIOA8_Handler    PROC
                EXPORT  GPIOA8_Handler    [WEAK]
                B       .
                ENDP

GPIOA9_Handler     PROC
                EXPORT  GPIOA9_Handler     [WEAK]
                B       .
                ENDP

GPIOA10_Handler       PROC
                EXPORT  GPIOA10_Handler       [WEAK]
                B       .
                ENDP

GPIOA11_Handler     PROC
                EXPORT  GPIOA11_Handler     [WEAK]
                B       .
                ENDP

GPIOA12_Handler       PROC
                EXPORT  GPIOA12_Handler       [WEAK]
                B       .
                ENDP

GPIOA13_Handler       PROC
                EXPORT  GPIOA13_Handler        [WEAK]
                B       .
                ENDP

GPIOA14_Handler    PROC
                EXPORT  GPIOA14_Handler    [WEAK]
                B       .
                ENDP

GPIOA15_Handler   PROC
                EXPORT  GPIOA15_Handler   [WEAK]
                B       .
                ENDP

IRQ24_Handler      PROC
                EXPORT  IRQ24_Handler      [WEAK]
                B       .
                ENDP

IRQ25_Handler     PROC
                EXPORT  IRQ25_Handler     [WEAK]
                B       .
                ENDP

IRQ26_Handler     PROC
                EXPORT  IRQ26_Handler     [WEAK]
                B       .
                ENDP

IRQ27_Handler    PROC
                EXPORT  IRQ27_Handler    [WEAK]
                B       .
                ENDP

IRQ28_Handler        PROC
                EXPORT  IRQ28_Handler      [WEAK]
                B       .
                ENDP

IRQ29_Handler      PROC
                EXPORT  IRQ29_Handler      [WEAK]
                B       .
                ENDP

IRQ30_Handler    PROC
                EXPORT  IRQ30_Handler    [WEAK]
                B       .
                ENDP

IRQ31_Handler   PROC
                EXPORT  IRQ31_Handler   [WEAK]
                B       .
                ENDP

                ALIGN


;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF

                END
