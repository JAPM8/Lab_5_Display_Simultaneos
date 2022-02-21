/*	
    Archivo:		main_preLAB.S
    Dispositivo:	PIC16F887
    Autor:		Javier Alejandro Pérez Marín 20183
    Compilador:		pic-as (v2.30), MPLABX V6.00

    Programa:		Contador de 8 bits con botón de aumento y de decremento
			mediante interrupciones y resistencias Pull up del PIC
    Hardware:		LEDs en el puerto A y 2 pb en puerto b

    Creado:			20/02/22
    Última modificación:	20/02/22	
*/
    
PROCESSOR 16F887
// config statements should precede project file includes.
#include <xc.inc>
 
; CONFIG1
CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
CONFIG  PWRTE = ON            ; Power-up Timer Enable bit (PWRT enabled)
CONFIG  MCLRE = OFF           ; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)

CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
CONFIG  LVP = ON              ; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

; CONFIG2
CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

UP   EQU 0		      ; Definimos nombres a pines 0 y 1
DOWN EQU 1
 
; Status para interrupciones
PSECT udata_shr		      ; Common memory
   W_TEMP:	DS 1	      ; 1 Byte
   STATUS_TEMP: DS 1	      ; 1 Byte
    
; CONFIG Vector RESET    
PSECT resVect, class=CODE, abs, delta=2
ORG 00h                       ; posición 0000h para el reset


; ---------------vector reset--------------
resetVec:
    PAGESEL MAIN
    GOTO    MAIN

ORG 04h			     ; Posición 0004h para las interrupciones
PUSH:			     ; Se guarda el PC en la pila
    MOVWF  W_TEMP	     ; Movemos el registro W a la variable W_TEMP
    SWAPF  STATUS, W	     ; Se hace un swap de Nibbles del status y se guarda en W
    MOVWF  STATUS_TEMP	     ; Se pasa el registro W a la variable STATUS_TEMP
    
ISR:			     ; Rutina de interrupción
    BTFSC   RBIF	     ; Se verifica la bandera de cambio de estado de PORTB
    CALL    INT_IOCB	     ; Pasamos a subrutina INT_IOCB
POP:			     ; Se regresan las instrucciones de la pila al main
    SWAPF   STATUS_TEMP, W   ; Se hace swap de Nibbles de nuevo al STATUS
    MOVWF   STATUS	     ; Se mueve el registro W a STATUS
    SWAPF   W_TEMP, F	     ; Swap de Nibbles del registro W y se pasa a F
    SWAPF   W_TEMP, W	     ; Swap de Nibbles del registro W y se pasa a W
    RETFIE		     ; Se regresa de la interrupción 

;---------------Subrutinas de int------------
INT_IOCB:
    BANKSEL PORTB
    BTFSS   PORTB, UP	     ; Se verifica estado de los botones para inc o dec del contador
    INCF    PORTA
    BTFSS   PORTB, DOWN
    DECF    PORTA
    
    BCF	    RBIF	     ; Se limpia la flag de cambio de estado del PORTB		
    
    RETURN
    
; CONFIG uCS
PSECT code, delta=2, abs
ORG 100h                      ; posición para el código

 ; ---------------CONFIGURACIÓN--------------
 MAIN:
; Configuración Inputs y Outputs
    CALL    CONFIG_PINES
; Configuración deL Oscilador (1 MHz)
    CALL    CONFIG_RELOJ
; Configuración de interrupciones
    CALL    ENABLE_INTS
; Configuración de lectura de cambios en puerto B
    CALL    CONFIG_IOCRB

LOOP:
    GOTO LOOP

CONFIG_PINES:
    BANKSEL ANSEL	      ; Cambiamos de banco
    CLRF    ANSEL	      ; Ra como I/O digital
    CLRF    ANSELH	      ; Rb como I/O digital
    
    BANKSEL TRISA
    CLRF    TRISA	      ; PORTA como output por completo
    
    BANKSEL TRISB	      ; Cambiamos de banco
    BSF	    TRISB, UP	      ; Rb0 y Rb1 como inputs
    BSF	    TRISB, DOWN
    
    BANKSEL OPTION_REG	      ; Cambiamos de banco
    BCF	    OPTION_REG,	7     ; PORTB pull-up habilitadas (RBPU)
    BANKSEL WPUB
    BSF	    WPUB, UP	      ; Se habilita registro de Pull-up para Rb0 y Rb1
    BSF	    WPUB, DOWN	    
    
    BANKSEL PORTA             ; Cambiamos de banco
    CLRF    PORTA	      ; Limpieza de puertos para que inicie en 0
    CLRF    PORTB
    
    RETURN
    
CONFIG_RELOJ:
    BANKSEL OSCCON	      ; Cambiamos de banco
    BSF	    OSCCON, 0	      ; Seteamos para utilizar reloj interno (SCS=1)
    
    ;Se modifican los bits 4 al 6 de OSCCON al valor de 100b para frecuencia de 1 MHz (IRCF=100b)
    BSF	    OSCCON, 6
    BCF	    OSCCON, 5
    BCF	    OSCCON, 4
    
    RETURN
    
ENABLE_INTS:
    BSF	GIE		      ; Se habilitan todas las interrupciones
    BSF RBIE		      ; Se habilita la interrupción de cambio de estado de PORTB	          
    BCF	RBIF		      ; Flag de cambio de estado de PORTB
    RETURN
    
CONFIG_IOCRB:
    BANKSEL TRISA	      ; Cambio de banco
    BSF	    IOCB, UP	      ; Se habilita interrupción de cambio de estado para Rb0 y Rb1
    BSF	    IOCB, DOWN
    
    BANKSEL PORTA	      ; Cambio de banco
    MOVF    PORTB, W	      ; Al leer termina la condición de mismatch
    BCF	    RBIF	      ; Se limpia la flag de cambio de estado de PORTB
    RETURN
    
END



