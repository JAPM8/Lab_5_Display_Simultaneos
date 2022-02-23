/*	
    Archivo:		main_preLAB.S
    Dispositivo:	PIC16F887
    Autor:		Javier Alejandro Pérez Marín 20183
    Compilador:		pic-as (v2.30), MPLABX V6.00

    Programa:		Contador de 8 bits con botón de aumento y de decremento
			mediante interrupciones y resistencias Pull up del PIC
    Hardware:		LEDs en el puerto A, 2 pb en puerto b, 2 display de 7
			segmentos multiplexados en D con su selector en E

    Creado:			22/02/22
    Última modificación:	23/02/22	
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

RESET_TMR0 MACRO
    BANKSEL TMR0	        ; Cambiamos al banco 1
    ;N=256-((5 ms)(1 MHz)/4*256) -> N= 251 aprox
    MOVLW   251                 ; Se mueve N al registro W
    MOVWF   TMR0	        ; Se le dan los 5 ms de delay a TMR0
    BCF	    T0IF	        ; Limpiamos la bandera de interrupción
    
    ENDM
    
PSECT udata_bank0	      ; Variables a utilizar
    VAR:	    DS 1
    BANDERAS:	    DS 1
    NIBBLE:	    DS 2
    DISPLAY_VAR:    DS 2

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
    BTFSC   T0IF	     ; Verficamos bandera de interrupción del TMR0
    CALL    CONT_TMR0	     ; Pasamos a subrutina de interrupción del TMR0
    
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
    CALL    INC_CONT	     ; Subrutina para incremento de contador
    BTFSS   PORTB, DOWN
    CALL    DEC_CONT	     ; Subrutina para incremento de contador
    
    BCF	    RBIF	     ; Se limpia la flag de cambio de estado del PORTB		
    
    RETURN
INC_CONT:
    INCF    PORTA
    INCF    VAR
    
    RETURN
    
DEC_CONT:
    DECF    PORTA
    DECF    VAR
    
    RETURN
    
CONT_TMR0:
    RESET_TMR0		     ; Reinicio TMR0
    CLRF      PORTE          ; Limpiamos selector
    BTFSC     BANDERAS,0     ; Test bandera de display secundario
    GOTO      DISP_SEC	     ; Subrutina para display secundario
    
DISP_PRINC:
    MOVF      DISPLAY_VAR, W ; Movemos valor de disp principal a w
    MOVWF     PORTD	     ; Valor de la tabla a puerto
    BSF	      PORTE, 1	     ; Se enciende display principal (derecho)
    
    BSF	      BANDERAS, 0    ; Se altera para pasar a modificar el secundario en la siguiente repetición.
        
    RETURN

DISP_SEC:
    MOVF    DISPLAY_VAR+1, W ; Movemos valor de disp principal a w
    MOVWF   PORTD	     ; Valor de la tabla a puerto
    BSF	    PORTE, 0	     ; Se enciende display secundario (Izquierdo)
    
    BCF	    BANDERAS, 0	     ; Se altera para pasar a modificar el principal en la siguiente repetición.
        
    RETURN
    
        
; CONFIG uCS
PSECT code, delta=2, abs
ORG 100h                     ; posición para el código
TABLA:
    CLRF    PCLATH	     ; Limpiamos registro PCLATH
    BSF	    PCLATH, 0	     ; Posicionamos el PC en dirección 01xxh
    ANDLW   0x0F	     ; No saltar más del tamaño de la tabla
    ADDWF   PCL		     ; Apuntamos el PC a PCLATH + PCL + W
    retlw 00111111B ;0
    retlw 00000110B ;1
    retlw 01011011B ;2
    retlw 01001111B ;3
    retlw 01100110B ;4
    retlw 01101101B ;5
    retlw 01111101B ;6
    retlw 00000111B ;7
    retlw 01111111B ;8
    retlw 01101111B ;9
    retlw 01110111B ;10 (A)
    retlw 01111100B ;11 (b)
    retlw 00111001B ;12 (C)
    retlw 01011110B ;13 (d)
    retlw 01111001B ;14 (E)
    retlw 01110001B ;15 (F)
    
 ; ---------------CONFIGURACIÓN--------------
 MAIN:
; Configuración Inputs y Outputs
    CALL    CONFIG_PINES
; Configuración deL Oscilador (1 MHz)
    CALL    CONFIG_RELOJ
; Configuración Timer0
    CALL    CONFIG_TIMER0
; Configuración de interrupciones
    CALL    ENABLE_INTS
; Configuración de lectura de cambios en puerto B
    CALL    CONFIG_IOCRB

    BANKSEL PORTA   
    
LOOP:
    CALL SEPARACION_NIBBLES   ; Subrutina para obtención del valor de cada contador
    CALL CONFIG_DISPLAY	      ; Subrutina para traducción de valores a 7 seg
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
    
    CLRF    TRISD	      ; PORTD completo como output
    BCF	    TRISE, 0	      ; RE0 como output
    BCF	    TRISE, 1	      ; RE1 como output
    
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

CONFIG_TIMER0:
    BANKSEL OPTION_REG	      ; Cambiamos de banco
    BCF	    T0CS	      ; Seteamos TMR0 como temporizador(T0CS)
    BCF	    PSA		      ; Se asigna el prescaler a TMR0(PSA)
   ; Se setea el prescaler a 256 BSF <2:0>
    BSF	    PS2		        ; PS2
    BSF	    PS1		        ; PS1
    BSF	    PS0		        ; PS0
    
    RESET_TMR0		      ; Macro
    
    RETURN
    
ENABLE_INTS:
    BSF	GIE		      ; Se habilitan todas las interrupciones
    BSF RBIE		      ; Se habilita la interrupción de cambio de estado de PORTB	          
    BCF	RBIF		      ; Flag de cambio de estado de PORTB
    BSF	    T0IE	      ; Se habilita interrupción del TMR0
    BCF	    T0IF	      ; Flag de interrupción TMR0
    RETURN
    
CONFIG_IOCRB:
    BANKSEL TRISA	      ; Cambio de banco
    BSF	    IOCB, UP	      ; Se habilita interrupción de cambio de estado para Rb0 y Rb1
    BSF	    IOCB, DOWN
    
    BANKSEL PORTA	      ; Cambio de banco
    MOVF    PORTB, W	      ; Al leer termina la condición de mismatch
    BCF	    RBIF	      ; Se limpia la flag de cambio de estado de PORTB
    RETURN
    
SEPARACION_NIBBLES:           ; Subrutina para obtención del valor de cada contador
    ; Nibble bajo
    MOVLW   0X0F	      ; Literal a w
    ANDWF   VAR, W	      ; AND para pasar 4 bits menos significativos
    MOVWF   NIBBLE	      ; Pasa un nibble al primer byte
    
    ;Nibble alto:
    MOVLW   0XF0	      ; Literal a w
    ANDWF   VAR, W	      ; AND para pasar 4 bits más significativos
    MOVWF   NIBBLE+1	      ; Pasa un nibble al segundo byte
    SWAPF   NIBBLE+1, F	      ; Swap del nibble para que los 4 menos significativos los tenga con VAR
    RETURN
    
CONFIG_DISPLAY:               ; Subrutina para traducción de valores a 7 seg
    MOVF    NIBBLE, W	      ; nibble bajo a W
    CALL    TABLA	      ; Se pasa el valor a la tabla de 7 segmentos.
    MOVWF   DISPLAY_VAR       ; La configuración de pines va a la variable display_var
    
    MOVF    NIBBLE+1, W       ; Nibble alto a W
    CALL    TABLA	      ; Se pasa el valor a la tabla de 7 segmentos.
    MOVWF   DISPLAY_VAR+1     ; La configuración de pines va al segundo byte de la variable display_var
    RETURN
    
END






