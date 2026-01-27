;*******************************************************************************************************
;* COE538 Project: Robot Guidance Challenge - Charran Thangeswaran, Redwan Khalifa, Ethan Soosaipillai *
;*******************************************************************************************************
; export symbols
            XDEF Entry, _Startup                ; export 'Entry' symbol
            ABSENTRY Entry                      ; for absolute assembly: mark this as application entry point

; Include derivative-specific definitions
            INCLUDE 'derivative.inc'

;*******************************************************************************************************
;* EQUATES SECTION                                                                                     *
;*******************************************************************************************************

; LCD Equates
; -----------
CLEAR_HOME    EQU   $01                         ; Clear the display and the home cursor
INTERFACE     EQU   $38                         ; 8 bit interface, two line display
CURSOR_OFF    EQU   $0C                         ; Display on, cursor off
SHIFT_OFF     EQU   $06                         ; Address increments, no character shift
LCD_SEC_LINE  EQU   64                          ; Starting address of 2nd line of LCD (decimal value used here)

; LCD Addresses
LCD_DAT       EQU   PORTB                       ; LCD Data Port, bits - PB7,...,PB0
LCD_CNTR      EQU   PTJ                         ; LCD Control Port: bits - PJ6(RS),PJ7(E)
LCD_E         EQU   $80                         ; LCD E-signal pin
LCD_RS        EQU   $40                         ; LCD RS-signal pin

; Others
NULL          EQU   00                          ; 'Null terminator' string
CR            EQU   $0D                         ; 'Carriage Return' character
SPACE         EQU   ' '                         ; 'Space' character

; Timers
T_LEFT        EQU   8
T_RIGHT       EQU   8

; Robot States
START         EQU   0                           ; START state value
FWD           EQU   1                           ; FORWARD state value
ALL_STOP      EQU   2                           ; ALL STOP state value
LEFT_TRN      EQU   3                           ; LEFT TURN state value
RIGHT_TRN     EQU   4                           ; RIGHT TURN state value
REV_TRN       EQU   5                           ; REVERSE state value         
LEFT_ALIGN    EQU   6                           ; LEFT ALIGN state value
RIGHT_ALIGN   EQU   7                           ; RIGHT ALIGN state value

;*******************************************************************************************************
;* VARIABLE SECTION                                                                                    *
;*******************************************************************************************************
              ORG   $3800

; Initial Values (based on the initial readings & variance)
; ---------------------------------------------------------
BASE_LINE     FCB   $9D
BASE_BOW      FCB   $CA
BASE_MID      FCB   $CA
BASE_PORT     FCB   $CC
BASE_STBD     FCB   $CC

LINE_VAR      FCB   $18                         ; Add variance based on testing to establish baseline for sensors
BOW_VAR       FCB   $30                     
PORT_VAR      FCB   $20                     
MID_VAR       FCB   $20
STARBOARD_VAR FCB   $15

TOP_LINE      RMB   20                          ; Top line of display
              FCB   NULL                        ; Null terminated
              
BOT_LINE      RMB   20                          ; Bottom line of display
              FCB   NULL                        ; Null terminated

CLEAR_LINE    FCC   '                  '        ; Clear line of display
              FCB   NULL                        ; Null terminated

TEMP          RMB   1                           ; Temporary location

; Storage Registers (9S12C32 RAM space: $3800 ... $3FFF)
; ------------------------------------------------------
SENSOR_LINE   FCB   $01                         ; Storage for guider sensor readings (initialized to test values)
SENSOR_BOW    FCB   $23                         
SENSOR_PORT   FCB   $45
SENSOR_MID    FCB   $67
SENSOR_STBD   FCB   $89
SENSOR_NUM    RMB   1                           ; Currently selected sensor

; Variables
; ---------
              ORG   $3850                       ; TOF counter register lives here

TOF_COUNTER   DC.B  0                           ; The timer, incremented at 23Hz

CRNT_STATE    DC.B  2                           ; Current state register

T_TURN        DS.B  1                           ; Time to stop turning

TEN_THOUS     DS.B  1                           ; 10,000 digit
THOUSANDS     DS.B  1                           ; 1,000 digit
HUNDREDS      DS.B  1                           ; 100 digit
TENS          DS.B  1                           ; 10 digit
UNITS         DS.B  1                           ; 1 digit
NO_BLANK      DS.B  1                           ; Used in 'leading zero' blanking by BCD2ASC
HEX_TABLE     FCC   '0123456789ABCDEF'          ; Table for converting values
BCD_SPARE     RMB   2

;*******************************************************************************************************
;* CODE SECTION                                                                                        *
;*******************************************************************************************************
              ORG   $4000                       ; Program starts here (Flash memory)
Entry:                                                                       
_Startup: 

              LDS   #$4000                      ; Initialize the stack pointer
              CLI                               ; Enable interrupts
              
              JSR   INIT                        ; Initialize ports
              JSR   openADC                     ; Initialize the ATD
              JSR   initLCD                     ; Initialize the LCD
              JSR   CLR_LCD_BUF                 ; Write 'space' characters to the LCD buffer
              
              BSET  DDRA,%00000011              ; STAR_DIR, PORT_DIR                        
              BSET  DDRT,%00110000              ; STAR_SPEED, PORT_SPEED                    
              JSR   initAD                      ; Initialize ATD converter                  
              JSR   initLCD                     ; Initialize the LCD                        
              JSR   clrLCD                      ; Clear LCD & home cursor                   
              LDX   #msg1                       ; Display msg1                              
              JSR   putsLCD                     ;       "                                   
              LDAA  #$C0                        ; Move LCD cursor to the 2nd row           
              JSR   cmd2LCD                     ;                                           
              LDX   #msg2                       ; Display msg2                              
              JSR   putsLCD                     ;       "      
              JSR   ENABLE_TOF                  ; Jump to TOF initialization
       
; Display Sensors
; ---------------
        MAIN: JSR   G_LEDS_ON                   ; Enable the guider LEDs   
              JSR   READ_SENSORS                ; Read the 5 guider sensors
              JSR   G_LEDS_OFF                  ; Disable the guider LEDs                   
              JSR   UPDT_DISPL                  ; Update LCD
              LDAA  CRNT_STATE                  ; Display the current state
              JSR   DISPATCHER         
              BRA   MAIN                        ; Loop forever

;*******************************************************************************************************
;* DATA SECTION                                                                                        *
;*******************************************************************************************************
msg1:         DC.B  "Voltage",0                 ; Battery voltage label
msg2:         DC.B  "State",0                   ; Current state label

tab:          DC.B  "START",0                   ; START state
              DC.B  "FWD",0                     ; FORWARD state
              DC.B  "ALL STOP",0                ; ALL STOP state
              DC.B  "LEFT TURN",0               ; LEFT TURN state
              DC.B  "RIGHT TURN",0              ; RIGHT TURN state
              DC.B  "REV TURN",0                ; REVERSE state
              DC.B  "LEFT TIMED",0              ; LEFT ALIGN state
              DC.B  "RIGHT TIMED",0             ; RIGHT ALIGN state

;*******************************************************************************************************
;* STATES SECTION                                                                                      *
;*******************************************************************************************************
DISPATCHER          JSR   VERIFY_START          ; Start of the dispatcher
                    RTS

VERIFY_START        CMPA  #START                ; Check if the robot's state is START
                    BNE   VERIFY_FORWARD        ; If not, move to validating FORWARD state
                    JSR   START_ST              ; Validate START state
                    RTS                                         

VERIFY_FORWARD      CMPA  #FWD                  ; Check if the robot's state is FORWARD
                    BNE   VERIFY_STOP           ; If not, move to validating ALL STOP state
                    JSR   FWD_ST                ; Validate FORWARD state
                    RTS
                  
VERIFY_REV_TRN      CMPA  #REV_TRN              ; Check if the robot's state is REVERSE
                    BNE   VERIFY_LEFT_ALIGN     ; If not, move to validating LEFT ALIGN state
                    JSR   REV_TRN_ST            ; Validate REVERSE state
                    RTS                                           

VERIFY_STOP         CMPA  #ALL_STOP             ; Check if the robot's state is ALL STOP
                    BNE   VERIFY_LEFT_TRN       ; If not, move to validating LEFT TURN state
                    JSR   ALL_STOP_ST           ; Validate ALL STOP state
                    RTS                                         

VERIFY_LEFT_TRN     CMPA  #LEFT_TRN             ; Check if the robot's state is LEFT TURN
                    BNE   VERIFY_RIGHT_TRN      ; If not, move to validating RIGHT TURN state
                    JSR   LEFT                  ; Validate LEFT TURN state
                    RTS                                                                                                                      

VERIFY_LEFT_ALIGN   CMPA  #LEFT_ALIGN           ; Check if the robot's state is LEFT ALIGN
                    BNE   VERIFY_RIGHT_ALIGN    ; If not, move to validating RIGHT ALIGN state
                    JSR   LEFT_ALIGN_DONE       ; Validate LEFT ALIGN state
                    RTS

VERIFY_RIGHT_TRN    CMPA  #RIGHT_TRN            ; Check if the robot's state is RIGHT TURN
                    BNE   VERIFY_REV_TRN        ; If not, move to validating REVERSE state
                    JSR   RIGHT                 ; Validate RIGHT TURN state                                      

VERIFY_RIGHT_ALIGN  CMPA  #RIGHT_ALIGN          ; Check if the robot's state is RIGHT ALIGN
                    JSR   RIGHT_ALIGN_DONE      ; Validate RIGHT ALIGN state
                    RTS                         ; INVALID state

; Movement
; --------
START_ST            BRCLR PORTAD0, %00000100, RELEASE                                    
                    JSR   INIT_FWD                                                               
                    MOVB  #FWD, CRNT_STATE

RELEASE             RTS                                                                                                                                 

FWD_ST              BRSET PORTAD0, $04, NO_FWD_BUMP           ; Check if bow bumper is hit                           
                    MOVB  #REV_TRN, CRNT_STATE                ; If true, enter the REVERSE state                               
                                                                                            
                    JSR     UPDT_DISPL                        ; Update the display                                
                    JSR     INIT_REV                                                                
                    LDY     #6000                                                                   
                    JSR     del_50us                                                                
                    JSR     INIT_RIGHT                                                              
                    LDY     #6000                                                                   
                    JSR     del_50us                                                                
                    LBRA    EXIT                                                                    

NO_FWD_BUMP         BRSET   PORTAD0, $04, NO_FWD_REAR_BUMP    ; Checks if stern bumper is hit
                    MOVB    #ALL_STOP, CRNT_STATE             ; If true, enter the ALL STOP state                  
                    JSR     INIT_STOP                           
                    LBRA    EXIT 
                  
NO_FWD_REAR_BUMP    LDAA    SENSOR_BOW                                                              
                    ADDA    BOW_VAR                                                               
                    CMPA    BASE_BOW                                                                
                    BPL     NOT_ALIGNED                                                                
                    LDAA    SENSOR_MID                                                              
                    ADDA    MID_VAR                                                                
                    CMPA    BASE_MID                                                                
                    BPL     NOT_ALIGNED                                                               
                    LDAA    SENSOR_LINE                                                             
                    ADDA    LINE_VAR                                                                
                    CMPA    BASE_LINE                                                               
                    BPL     CHECK_RIGHT_ALIGN                                                          
                    LDAA    SENSOR_LINE                                                             
                    SUBA    LINE_VAR                                                                
                    CMPA    BASE_LINE                                                              
                    BMI     CHECK_LEFT_ALIGN                                                                 

NOT_ALIGNED         LDAA    SENSOR_PORT                                                            
                    ADDA    PORT_VAR                                                               
                    CMPA    BASE_PORT                                                              
                    BPL     PARTIAL_LEFT_TRN                                                        
                    BMI     NO_PORT                                                             

NO_PORT             LDAA    SENSOR_BOW                                                             
                    ADDA    BOW_VAR                                                                 
                    CMPA    BASE_BOW                                                                
                    BPL     EXIT                                                                    
                    BMI     NO_BOW                                                              

NO_BOW              LDAA    SENSOR_STBD                                                             
                    ADDA    STARBOARD_VAR                                                               
                    CMPA    BASE_STBD                                                               
                    BPL     PARTIAL_RIGHT_TRN                                                         
                    BMI     EXIT 

PARTIAL_LEFT_TRN    LDY     #6000                                                                 
                    JSR     del_50us                                                                
                    JSR     INIT_LEFT                                                               
                    MOVB    #LEFT_TRN, CRNT_STATE                                                  
                    LDY     #6000                                                                   
                    JSR     del_50us                                                                
                    BRA     EXIT                                                                    

CHECK_LEFT_ALIGN    JSR     INIT_LEFT                                                               
                    MOVB    #LEFT_ALIGN, CRNT_STATE                                                 
                    BRA     EXIT

PARTIAL_RIGHT_TRN   LDY     #6000                                                                  
                    JSR     del_50us                                                                
                    JSR     INIT_RIGHT                                                              
                    MOVB    #RIGHT_TRN, CRNT_STATE                                                 
                    LDY     #6000                                                                   
                    JSR     del_50us                                                                
                    BRA     EXIT                                                                   

CHECK_RIGHT_ALIGN   JSR     INIT_RIGHT                                                              
                    MOVB    #RIGHT_ALIGN, CRNT_STATE                                                
                    BRA     EXIT                                                                                                                                                         

EXIT                RTS                                                                         

LEFT                LDAA    SENSOR_BOW                                                              
                    ADDA    BOW_VAR                                                                 
                    CMPA    BASE_BOW                                                               
                    BPL     LEFT_ALIGN_DONE                                                        
                    BMI     EXIT

LEFT_ALIGN_DONE     MOVB    #FWD, CRNT_STATE                                                        
                    JSR     INIT_FWD                                                                
                    BRA     EXIT                                                                    

RIGHT               LDAA    SENSOR_BOW                                                              
                    ADDA    BOW_VAR                                                                
                    CMPA    BASE_BOW                                                                
                    BPL     RIGHT_ALIGN_DONE                                                        
                    BMI     EXIT 

RIGHT_ALIGN_DONE    MOVB    #FWD, CRNT_STATE                                                        
                    JSR     INIT_FWD                                                                
                    BRA     EXIT                                                                    

REV_TRN_ST          LDAA    SENSOR_BOW                                                              
                    ADDA    BOW_VAR                                                                 
                    CMPA    BASE_BOW                                                                
                    BMI     EXIT                                                                    
                    JSR     INIT_LEFT                                                               
                    MOVB    #FWD, CRNT_STATE                                                        
                    JSR     INIT_FWD                                                                
                    BRA     EXIT                                                                    

ALL_STOP_ST         BRSET   PORTAD0, %00000100, NO_START_BUMP                                       
                    MOVB    #START, CRNT_STATE                                                      

NO_START_BUMP       RTS                                                                             

;*******************************************************************************************************
;* INITIALIZATION SECTION                                                                              *
;*******************************************************************************************************
INIT_RIGHT          BSET    PORTA,%00000010          
                    BCLR    PORTA,%00000001           
                    LDAA    TOF_COUNTER                        ; Mark FORWARD time 
                    ADDA    #T_RIGHT
                    STAA    T_TURN
                    RTS

INIT_LEFT           BSET    PORTA,%00000001         
                    BCLR    PORTA,%00000010          
                    LDAA    TOF_COUNTER                        ; Mark TOF time
                    ADDA    #T_LEFT                            ; Add LEFT TURN
                    STAA    T_TURN                    
                    RTS

INIT_FWD            BCLR    PORTA, %00000011                   ; Set FORWARD direction for both motors
                    BSET    PTT, %00110000                     ; Turn on the drive motors
                    RTS 

INIT_REV            BSET    PORTA,%00000011                    ; Set REVERSE direction for both motors
                    BSET    PTT,%00110000                      ; Turn on the drive motors
                    RTS

INIT_STOP           BCLR    PTT, %00110000                     ; Turn off the drive motors
                    RTS


; Initialize Sensors
; ------------------
INIT                BCLR   DDRAD,$FF                           ; Set PORTAD as input (DDRAD @ $0272)
                    BSET   DDRA,$FF                            ; Set PORTA as output (DDRA @ $0002)
                    BSET   DDRB,$FF                            ; Set PORTB as output (DDRB @ $0003)
                    BSET   DDRJ,$C0                            ; Set Pins 6, 7 of PTJ as outputs (DDRJ @ $026A)
                    RTS


; Initialize ADC 
; --------------             
openADC             MOVB   #$80,ATDCTL2                        ; Turn on ADC (ATDCTL2 @ $0082)
                    LDY    #1                                  
                    JSR    del_50us                             
                    MOVB   #$20,ATDCTL3                        ; 4 conversions on channel AN1 (ATDCTL3 @ $0083)
                    MOVB   #$97,ATDCTL4                        ; 8-bit resolution, prescaler = 48 (ATDCTL4 @ $0084)
                    RTS

; Clear LCD Buffer
; ----------------
CLR_LCD_BUF         LDX   #CLEAR_LINE
                    LDY   #TOP_LINE
                    JSR   STRCPY

CLB_SECOND          LDX   #CLEAR_LINE
                    LDY   #BOT_LINE
                    JSR   STRCPY

CLB_EXIT            RTS

; String Copy
; -----------
STRCPY              PSHX                                       ; Protect the registers used
                    PSHY
                    PSHA

STRCPY_LOOP         LDAA 0,X                                   ; Get a source character
                    STAA 0,Y                                   ; Copy to the destination
                    BEQ STRCPY_EXIT                            ; Exit if null
                    INX                                        ; Otherwise, increment the pointers
                    INY
                    BRA STRCPY_LOOP                            ; Do it again

STRCPY_EXIT         PULA                                       ; Restore the registers
                    PULY
                    PULX
                    RTS  

; Guider LEDS
; -----------
G_LEDS_ON           BSET PORTA,%00100000                       ; Set bit 5                                                 
                    RTS                                                                                                                                               |
G_LEDS_OFF          BCLR PORTA,%00100000                       ; Clear bit 5                                               
                    RTS                                                                                 

; Read Sensors
; ------------
READ_SENSORS        CLR   SENSOR_NUM                           ; Select sensor # 0
                    LDX   #SENSOR_LINE                         ; Point to the start of the sensor array

RS_MAIN_LOOP        LDAA  SENSOR_NUM                           ; Select the correct sensor input
                    JSR   SELECT_SENSOR  ; on the hardware
                    LDY   #400         ; 20 ms delay to allow the
                    JSR   del_50us       ; sensor to stabilize
                  
                    LDAA  #%10000001                           ; Start A/D conversion on AN1
                    STAA  ATDCTL5
                    BRCLR ATDSTAT0,$80,*                       ; Repeat until A/D signals done
                  
                    LDAA  ATDDR0L                              ; A/D conversion is complete in ATDDR0L
                    STAA  0,X                                  ; Copy to sensor register
                    CPX   #SENSOR_STBD                         ; If this is the last reading...
                    BEQ   RS_EXIT                              ; ...then exit
                  
                    INC   SENSOR_NUM                           ; Otherwise, increment the sensor number...
                    INX                                        ; ...and the pointer into the sensor array
                    BRA   RS_MAIN_LOOP                         ; Do it again

RS_EXIT             RTS

; Select Sensor
; -------------   
SELECT_SENSOR       PSHA                                       ; Save the sensor number for the moment

                    LDAA  PORTA                                ; Clear the sensor selection bits to zeros
                    ANDA  #%11100011
                    STAA  TEMP           
                  
                    PULA                                       ; Get the sensor number
                    ASLA                                       ; Shift the selection number left, twice
                    ASLA 
                    ANDA  #%00011100                           ; Clear irrelevant bit positions
                  
                    ORAA  TEMP           
                    STAA  PORTA                                
                    RTS

; Display Sensor Readings
; -----------------------

DP_FRONT_SENSOR     EQU TOP_LINE+3
DP_PORT_SENSOR      EQU BOT_LINE+0
DP_MID_SENSOR       EQU BOT_LINE+3
DP_STBD_SENSOR      EQU BOT_LINE+6
DP_LINE_SENSOR      EQU BOT_LINE+9

DISPLAY_SENSORS     LDAA  SENSOR_BOW                           ; Get the FRONT sensor value
                    JSR   BIN2ASC                              ; Convert to ASCII string in D
                    LDX   #DP_FRONT_SENSOR                     ; Point to the LCD buffer position
                    STD   0,X                                  ; Write the 2 ASCII digits there
                  
                    LDAA  SENSOR_PORT                          ; Do the same for the PORT sensor value
                    JSR   BIN2ASC
                    LDX   #DP_PORT_SENSOR
                    STD   0,X
                  
                    LDAA  SENSOR_MID                           ; Do the same for the MID sensor value
                    JSR   BIN2ASC
                    LDX   #DP_MID_SENSOR
                    STD   0,X
                  
                    LDAA  SENSOR_STBD                          ; Do the same for the STARBOARD sensor value
                    JSR   BIN2ASC
                    LDX   #DP_STBD_SENSOR
                    STD   0,X
                  
                    LDAA  SENSOR_LINE                          ; Do the same for the LINE sensor value
                    JSR   BIN2ASC
                    LDX   #DP_LINE_SENSOR
                    STD   0,X
                  
                    LDAA  #CLEAR_HOME                          ; Clear the display and the home cursor
                    JSR   cmd2LCD           
                  
                    LDY   #40                                  ; Wait 2 ms until "clear display" command is complete
                    JSR   del_50us
                  
                    LDX   #TOP_LINE                            ; Copy the buffer top line to the LCD
                    JSR   putsLCD
                  
                    LDAA  #LCD_SEC_LINE                        ; Position the LCD cursor on the second line
                    JSR   LCD_POS_CRSR
                  
                    LDX   #BOT_LINE                            ; Copy the buffer bottom line to the LCD
                    JSR   putsLCD
                    RTS

;*******************************************************************************************************
;* UPDATE DISPLAY SECTION                                                                              *
;*******************************************************************************************************
UPDT_DISPL          MOVB    #$90,ATDCTL5                       ; R-just., uns., sing. conv., mult., ch=0, start
                    BRCLR   ATDSTAT0,$80,*                     ; Wait until conversion is complete
                    LDAA    ATDDR0L                            ; Load the ch0 result - battery volt - into A
                    LDAB    #39                                
                    MUL                                        
                    ADDD    #600                             
                    JSR     int2BCD
                    JSR     BCD2ASC
                    LDAA    #$8D                               ; Move LCD cursor to the 1st row, end of msg1
                    JSR     cmd2LCD
                    LDAA    TEN_THOUS                          ; Output the TEN_THOUS ASCII character
                    JSR     putcLCD                            ; Put it into LCD (same for subsequent ones)
                    LDAA    THOUSANDS                          ; Output the THOUSANDS ASCII character
                    JSR     putcLCD
                    LDAA    #'.'                               ; Include the decimal place
                    JSR     putcLCD                            
                    LDAA    HUNDREDS                           ; Output the HUNDREDS ASCII character
                    JSR     putcLCD         
                    LDAA    #$C7                               ; Move LCD cursor to the 2nd row, end of msg2
                    JSR     cmd2LCD         
                    LDAB    CRNT_STATE                         ; Display the current state
                    LSLB                    
                    LSLB                    
                    LSLB
                    LDX     #tab            
                    ABX                     
                    JSR     putsLCD         
                    RTS

ENABLE_TOF          LDAA    #%10000000
                    STAA    TSCR1                              ; Enable TCNT
                    STAA    TFLG2                              ; Clear the TOF
                    LDAA    #%10000100                         ; Enable TOI and select prescale factor = 16
                    STAA    TSCR2
                    RTS

TOF_ISR             INC     TOF_COUNTER
                    LDAA    #%10000000                         
                    STAA    TFLG2                              ; Clear the TOF
                    RTI

;*******************************************************************************************************
;* UTILITY SUBROUTINES SECTION                                                                         *
;*******************************************************************************************************
initLCD:            BSET    DDRB,%11111111                     ; Set pins PS7, PS6, PS5, PS4 as output
                    BSET    DDRJ,%11000000                     ; Set pins PE7, PE4 as output
                    LDY     #2000
                    JSR     del_50us
                    LDAA    #$28
                    JSR     cmd2LCD
                    LDAA    #$0C
                    JSR     cmd2LCD
                    LDAA    #$06
                    JSR     cmd2LCD
                    RTS

clrLCD:             LDAA  #$01
                    JSR   cmd2LCD
                    LDY   #40
                    JSR   del_50us
                    RTS

del_50us            PSHX                                       ; (2 E-clk) Protect the X register

eloop               LDX   #300                               ; (2 E-clk) Initialize the inner loop counter

iloop               NOP                                        ; (1 E-clk) No operation
                    DBNE X,iloop                               ; (3 E-clk) Loop again if the inner counter is not 0
                    DBNE Y,eloop                               ; (3 E-clk) Loop again if the outer counter is not 0
                    PULX                                       ; (3 E-clk) Restore the X register
                    RTS                                        ; (5 E-clk) Otherwise, return

cmd2LCD:            BCLR  LCD_CNTR, LCD_RS                     ; Select LCD instruction
                    JSR   dataMov                              
                    RTS

putsLCD:            LDAA  1,X+                                 ; Get one character from string
                    BEQ   donePS                               ; Get a NULL character
                    JSR   putcLCD
                    BRA   putsLCD

donePS              RTS

putcLCD:            BSET  LCD_CNTR, LCD_RS                     ; Select the LCD Data Register
                    JSR   dataMov                              
                    RTS

dataMov:            BSET  LCD_CNTR, LCD_E                      ; Pull LCD E-signal high
                    STAA  LCD_DAT                              ; Send upper 4 bits of data to LCD
                    BCLR  LCD_CNTR, LCD_E                      ; Pull LCD E-signal low
                    LSLA                                       ; Match lower 4 bits with LCD data pins
                    LSLA                    
                    LSLA                    
                    LSLA                   
                    BSET  LCD_CNTR, LCD_E                      ; Pull LCD E-signal high
                    STAA  LCD_DAT                              ; Send lower 4 bits of data to LCD
                    BCLR  LCD_CNTR, LCD_E                      ; Pull LCD E-signal low
                    LDY   #1                
                    JSR   del_50us          
                    RTS

initAD              MOVB  #$C0,ATDCTL2                         
                    JSR   del_50us                             
                    MOVB  #$00,ATDCTL3                         ; 8 conversions in a sequence
                    MOVB  #$85,ATDCTL4                         
                    BSET  ATDDIEN,$0C                          ; Set pins AN03, AN02 as inputs
                    RTS

int2BCD             XGDX                                       ; Save binary number into .X
                    LDAA #0                                    ; Clear BCD Buffer
                    STAA TEN_THOUS
                    STAA THOUSANDS
                    STAA HUNDREDS
                    STAA TENS
                    STAA UNITS
                    STAA BCD_SPARE
                    STAA BCD_SPARE+1
                    CPX #0                                     ; Check for a zero input
                    BEQ CON_EXIT                               ; Exit if there is
                    XGDX                                       ; If not, get the binary number back to .D as dividend
                    LDX #10                                    ; Setup 10 as the divisor
                    IDIV                                       ; Quotient is in .X and remainder is in .D
                    STAB UNITS                                 ; Store remainder
                    CPX #0                                     
                    BEQ CON_EXIT                               ; Exit if the quotient = 0
                    XGDX                                       ; Otherwise, swap the first quotient back into .D...
                    LDX #10                                    ; ...and setup for another divide by 10
                    IDIV
                    STAB TENS
                    CPX #0
                    BEQ CON_EXIT
                    XGDX                    
                    LDX #10                
                    IDIV
                    STAB HUNDREDS
                    CPX #0
                    BEQ CON_EXIT
                    XGDX                    
                    LDX #10                 
                    IDIV
                    STAB THOUSANDS
                    CPX #0
                    BEQ CON_EXIT
                    XGDX                   
                    LDX #10                 
                    IDIV
                    STAB TEN_THOUS

CON_EXIT            RTS                                       ; Conversion finished!

LCD_POS_CRSR        ORAA #%10000000                           ; Set high bit of the control word
                    JSR cmd2LCD                               ; Set the cursor address
                    RTS

BIN2ASC             PSHA                                      ; Save a copy of the input number
                    TAB            
                    ANDB #%00001111                           ; Strip off the upper nibble
                    CLRA                                      ; D contains 000n (n is the LSnibble)
                    ADDD #HEX_TABLE                           ; Set up for indexed load
                    XGDX                
                    LDAA 0,X                                  ; Get the LSnibble character
                    
                    PULB                                      ; Retrieve number here
                    PSHA                                      ; Push LSnibble character in its place
                    RORB                                      ; Move upper nibble of the input number into lower nibble position
                    RORB                                      
                    RORB
                    RORB 
                    ANDB #%00001111                           ; Strip off the upper nibble
                    CLRA                                      ; D contains 000n (n is the MSnibble)
                    ADDD #HEX_TABLE                           ; Set up for indexed load
                    XGDX                                                               
                    LDAA 0,X                                  ; Get the MSnibble character
                    PULB                                      ; Retrieve number here
                    RTS

; BCD to ASCII Conversion Routine
; -------------------------------
BCD2ASC             LDAA    #0            
                    STAA    NO_BLANK

C_TTHOU             LDAA    TEN_THOUS                         ; Check the Ten Thousands digit for blankness
                    ORAA    NO_BLANK
                    BNE     NOT_BLANK1

ISBLANK1            LDAA    #' '          
                    STAA    TEN_THOUS                         ; Store a space since it's blank
                    BRA     C_THOU                            ; Then check the Thousands digit

NOT_BLANK1          LDAA    TEN_THOUS                         ; Get the Ten Thousands digit
                    ORAA    #$30                              ; Convert to ASCII
                    STAA    TEN_THOUS
                    LDAA    #$1                               ; 'Non-blank' digit has been spotted
                    STAA    NO_BLANK

C_THOU              LDAA    THOUSANDS     
                    ORAA    NO_BLANK      
                    BNE     NOT_BLANK2

ISBLANK2            LDAA    #' '          
                    STAA    THOUSANDS     
                    BRA     C_HUNS        

NOT_BLANK2          LDAA    THOUSANDS     
                    ORAA    #$30
                    STAA    THOUSANDS
                    LDAA    #$1
                    STAA    NO_BLANK

C_HUNS              LDAA    HUNDREDS      
                    ORAA    NO_BLANK      
                    BNE     NOT_BLANK3

ISBLANK3            LDAA    #' '          
                    STAA    HUNDREDS       
                    BRA     C_TENS         

NOT_BLANK3          LDAA    HUNDREDS         
                    ORAA    #$30
                    STAA    HUNDREDS
                    LDAA    #$1
                    STAA    NO_BLANK

C_TENS              LDAA    TENS          
                    ORAA    NO_BLANK      
                    BNE     NOT_BLANK4

ISBLANK4            LDAA    #' '          
                    STAA    TENS          
                    BRA     C_UNITS       

NOT_BLANK4          LDAA    TENS          
                    ORAA    #$30
                    STAA    TENS

C_UNITS             LDAA    UNITS                             ; Convert to ASCII (blank check not necessary here)
                    ORAA    #$30
                    STAA    UNITS
                    RTS                                       ; Done!

; Display Battery Voltage
; -----------------------
                    LDAA    #$C7                              ; Move LCD cursor to the 2nd row, end of msg2
                    JSR     cmd2LCD                             
                    LDAB    CRNT_STATE                        ; Display the current state
                    LSLB                    
                    LSLB                    
                    LSLB
                    LDX     #tab            
                    ABX                     
                    JSR     putsLCD         
                    RTS

;*******************************************************************************************************
;* INTERRUPT VECTORS                                                                                   *
;*******************************************************************************************************
                    ORG     $FFFE
                    DC.W    Entry                             ; Reset Vector

                    ORG     $FFDE
                    DC.W    TOF_ISR                           ; Timer Overflow Interrupt Vector