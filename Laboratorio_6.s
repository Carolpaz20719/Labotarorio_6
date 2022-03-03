; Archivo:	Laboratorio6.s
; Dispositivo:	PIC16F887
; Autor:	Carolina Paz 20719
; Compilador:	pic-as (v2.30), MPLABX V5.40
; 
; Programa:	Temporizador con TRM1 Y TMR2
; Hardware:	Pic16f887, leds
;
; Creado:	01 mar, 2022
; Última modificación: 02 mar, 2022
    
PROCESSOR 16F887
    
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

// config statements should precede project file includes.
#include <xc.inc>

;---------------------MACROS----------------------

Timer_reset MACRO TMR_VAR
    BANKSEL TMR0	    ; cambiamos de banco
    MOVLW   TMR_VAR	    ; Literal a guardar en TMR_VAR
    MOVWF   TMR0	    ; configura tiempo de retardo (10ms de retardo)
    BCF	    T0IF	    ; limpiamos bandera de interrupción
    ENDM 
  
TMR1_reset MACRO TMR1_H, TMR1_L	
    BANKSEL TMR1H	    ; Cambiamos de banco
    MOVLW   TMR1_H	    ; Literal a guardar en TMR1H
    MOVWF   TMR1H	    ; Guardamos literal en TMR1H
    MOVLW   TMR1_L	    ; Literal a guardar en TMR1L
    MOVWF   TMR1L	    ; Guardamos literal en TMR1L
    BCF	    TMR1IF	    ; Limpiamos bandera de int. TMR1
    ENDM
  
; ------- VARIABLES EN MEMORIA -------------------
PSECT udata_shr		    ; Memoria compartida
    W_TEMP:		DS 1
    STATUS_TEMP:	DS 1
    
PSECT udata_bank0               ; Reservar memoria
    cont_seg:		DS 1    ; Contiene el valor de segundos
    valor:		DS 1	; Contiene valor a mostrar en los displays de 7-seg
    banderas:		DS 1	; Indica que display hay que encender
    num:                DS 2    ; Contiene el valor de las decenas y unidades
    display:		DS 2	; Representación de cada número en el display de 7-seg
   
PSECT resVect, class=CODE, abs, delta=2
ORG 00h	    ; posición 0000h para el reset
;------------ VECTOR RESET --------------
resetVec:
    PAGESEL Main	    ; Cambio de pagina
    GOTO    Main

PSECT intVect, class=CODE, abs, delta=2
ORG 04h			    ; Posición 0004h para interrupciones
;------- VECTOR INTERRUPCIONES ----------
    
PUSH:
    MOVWF   W_TEMP	    ; Guardamos W
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP	    ; Guardamos STATUS
    
ISR:
    BTFSC   T0IF	    ; Verificamos bandera del TMR0
    CALL    INT_TMR0	    ; Llamar a su subrutina de interrupción correspondiente
    
    BTFSC   TMR1IF	    ; Verificamos bandera del TMR1
    CALL    INT_TMR1	    ; Llamar a su subrutina de interrupción correspondiente
    
    BTFSC   TMR2IF	    ; Verificamos bandera del TMR2
    CALL    INT_TMR2	    ; Llamar a su subrutina de interrupción correspondiente
         
POP:
    SWAPF   STATUS_TEMP, W  
    MOVWF   STATUS	    ; Recuperamos el valor de reg STATUS
    SWAPF   W_TEMP, F	    
    SWAPF   W_TEMP, W	    ; Recuperamos valor de W
    RETFIE		    ; Regresamos a ciclo principal  
    
PSECT code, delta=2, abs
ORG 100h                    ; posición 100h para el codigo
 
 
;-------------SUBRUTINAS DE INTERRUPCION--------
INT_TMR0:
    Timer_reset	252	    ; Llamar la macro del Timer_reset
    CALL    MOSTRAR_VALOR   ; Mostramos valor en decimal en los displays
    RETURN
    
INT_TMR1:
    TMR1_reset 0xC2, 0xF7   ; Reiniciamos TMR1 para 1000m
    MOVLW   60		    ; Mover la litaral a W
    INCF    PORTA           ; Incrementar 1 y guardaar 
    SUBWF   PORTA, W        ; Restar PORTA - W 
    BTFSS   STATUS, 2       ; Revisar la resta
    RETURN		    ; Si no cumple
    CLRF    PORTA	    ; si, si cumple (limpiar puertoA)
    RETURN

INT_TMR2:
    BCF	    TMR2IF	    ; Limpiamos bandera de interrupcion de TMR2
    INCF    PORTB	    ; Incremento en PORTB
    RETURN
    
;-----------Tablas---------------------------
ORG 200h
Tabla:
    CLRF    PCLATH	    ; Limpiamos registro PCLATH
    BSF	    PCLATH, 1	    ; Posicionamos el PC en dirección 02xxh
    ANDLW   0x0F	    ; No saltar más del tamaño de la tabla
    ADDWF   PCL
    RETLW   00111111B	;0
    RETLW   00000110B	;1
    RETLW   01011011B	;2
    RETLW   01001111B	;3
    RETLW   01100110B	;4
    RETLW   01101101B	;5
    RETLW   01111101B	;6
    RETLW   00000111B	;7
    RETLW   01111111B	;8
    RETLW   01101111B	;9
    RETLW   01110111B	;A
    RETLW   01111100B	;b
    RETLW   00111001B	;C
    RETLW   01011110B	;d
    RETLW   01111001B	;E
    RETLW   01110001B	;F
    
;------------- CONFIGURACION -----------------
Main:
    CALL    IO_config	    ; Configuración de I/O
    CALL    Reloj_config    ; Configuración de Oscilador
    CALL    Timer0_config
    CALL    TMR1_config	    ; Configuración de TMR1
    CALL    TMR2_config	    ; Configuración de TMR2
    CALL    INT_config	    ; Configuración de las interrupciones
    BANKSEL PORTB
       
;----------------loop principal-----------------
Loop:
    MOVF    PORTA, W	    ; Valor del PORTA a W
    MOVWF   valor	    ; Movemos W a variable valor
    CALL    OBTENER_NUM     ; Guardamos decenas y unidades
    CALL    SET_DISPLAY	    ; Guardamos los valores a enviar en PORTC para mostrar valor en decimal
    CLRF    num		    ; Limpiar num
    CLRF    num + 1	    ; Limpiar num+1
    goto    Loop	    ; Loop 
    
;------------- SUBRUTINAS ---------------
IO_config:
    BANKSEL ANSEL	    ; Cambiar de Banco
    CLRF    ANSEL
    CLRF    ANSELH	    ; Poner I/O digitales
    
    BANKSEL TRISA	    ; Cambiar de Banco
    CLRF    TRISA	    ; PORTA como salida
    BCF     TRISB, 0	    ; PORTB como salida
    CLRF    TRISC	    ; Poner como salida
    BCF	    TRISD, 0	    ; RD0 como salida / display unidades
    BCF	    TRISD, 1	    ; RD1 como salida / display decimales
            
    BANKSEL PORTA	    ; Cambiar de Banco
    CLRF    PORTA	    ; Limpiar PORTA
    CLRF    PORTB	    ; Limpiar PORTB
    CLRF    PORTC	    ; Limpiar PORTC
    CLRF    PORTD	    ; Limpiar PORTD
    CLRF    banderas        ; Limpiamos GPR
    RETURN
    
Reloj_config:
   BANKSEL  OSCCON          ; cambiamos a banco 1
   BSF	    SCS	            ; SCS =1, Usamos reloj interno
   BCF	    IRCF2           ; IRCF 0
   BSF	    IRCF1           ; IRCF 1
   BSF	    IRCF0           ; IRCF 1 --> 110 500kHz
   RETURN

Timer0_config:
   BANKSEL  OPTION_REG	    ; cambiamos de banco de OPTION_REG
   BCF	    T0CS	    ; Timer0 como temporizador
   BCF	    PSA   	    ; Prescaler a TIMER0
   BSF	    PS2	            ; PS2
   BCF	    PS1	            ; PS1
   BSF	    PS0	            ; PS0 Prescaler de 1 : 64
    
   BANKSEL  TMR0	    ; cambiamos de banco de TMR0
   MOVLW    252 	    ; 2ms = 4*1/500kHz*(256-x)(64)
   MOVWF    TMR0	    ; 2ms de retardo
   BCF	    T0IF	    ; limpiamos bandera de interrupción
   RETURN 
   
TMR1_config:
    BANKSEL T1CON	    ; Cambiamos a banco 00
    BCF	    TMR1CS	    ; Reloj interno
    BCF	    T1OSCEN	    ; Apagamos LP
    BSF	    T1CKPS1	    ; Prescaler 1:8
    BSF	    T1CKPS0
    BCF	    TMR1GE	    ; TMR1 siempre contando
    BSF	    TMR1ON	    ; Encendemos TMR1
    
    TMR1_reset 0xC2, 0xF7   ; TMR1 a 1000ms
    RETURN

TMR2_config:
    BANKSEL PR2		    ; Cambiamos a banco 01
    MOVLW   244		    ; Valor para interrupciones cada 50ms
    MOVWF   PR2		    ; Cargamos litaral a PR2
    
    BANKSEL T2CON	    ; Cambiamos a banco 00
    BSF	    T2CKPS1	    ; Prescaler 1:16
    BSF	    T2CKPS0
    
    BSF	    TOUTPS3	    ; Postscaler 1:16
    BSF	    TOUTPS2
    BSF	    TOUTPS1
    BSF	    TOUTPS0
    
    BSF	    TMR2ON	    ; Encendemos TMR2
    RETURN
    RETURN
    
INT_config:
    BANKSEL PIE1	    ; Cambiamos a banco 01
    BSF	    TMR1IE	    ; Habilitamos int. TMR1
    BSF	    TMR2IE	    ; Habilitamos int. TMR2
        
    BANKSEL INTCON	    ; Cambiamos a banco 00
    BSF	    PEIE	    ; Habilitamos int. perifericos
    BSF	    GIE		    ; Habilitamos interrupciones
    BSF	    T0IE  	    ; Habilitamos interrupcion TMR0
    BCF	    T0IF	    ; Limpiamos bandera de TMR0
    BCF	    TMR1IF	    ; Limpiamos bandera de TMR1
    BCF	    TMR2IF	    ; Limpiamos bandera de TMR2
    RETURN
    
OBTENER_NUM:
    DECENAS:
	MOVLW  10           ; Mover la literal de 10 a W
	SUBWF  valor, W     ; valor - 10
	BTFSS  STATUS, 0    ; Verificar la resta
        GOTO   UNIDADES	    ; Vamos a unidades
	MOVWF  valor	    ; mover valor a w al registro f
	INCF   num 	    ; incrementamos el contador de decenas
	GOTO   DECENAS	    ; regresar a decenas
    
    UNIDADES:
	MOVLW  1            ; Mover la literal de 1 a W
	SUBWF  valor, W     ; valor - 1
	BTFSS  STATUS, 0    ; Verificar la resta
        RETURN
	MOVWF  valor	    ; mover valor a w al registro f
	INCF   num+1	    ; incrementamos el contador de unidades
	GOTO   UNIDADES     ; regresar a unidades     
    
    
SET_DISPLAY:
    MOVF    num, W		; Movemos nibble bajo a W
    CALL    Tabla		; Buscamos valor a cargar en PORTC
    MOVWF   display		; Guardamos en display
    
    MOVF    num+1, W	        ; Movemos nibble alto a W
    CALL    Tabla		; Buscamos valor a cargar en PORTC
    MOVWF   display+1		; Guardamos en display+1
    RETURN
    
MOSTRAR_VALOR:
    BCF	    PORTD, 0		; Apagamos display de unidades
    BCF	    PORTD, 1		; Apagamos display de decenas
    BTFSC   banderas, 0		; Verificamos bandera
    GOTO    DISPLAY_1		; Ir al display 1
      
    DISPLAY_0:			
	MOVF    display, W	; Movemos display a W
	MOVWF   PORTC		; Movemos Valor de tabla a PORTC
	BSF	PORTD, 0	; Encendemos display de unidades
	BSF	banderas, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN

    DISPLAY_1:
	MOVF    display+1, W	; Movemos display+1 a W
	MOVWF   PORTC		; Movemos Valor de tabla a PORTC
	BSF	PORTD, 1	; Encendemos display de decenas
	BCF	banderas, 0	; Cambiamos bandera para cambiar el otro display en la siguiente interrupción
    RETURN

END





