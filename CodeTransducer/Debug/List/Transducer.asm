
;CodeVisionAVR C Compiler V3.12 Advanced
;(C) Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
;http://www.hpinfotech.com

;Build configuration    : Debug
;Chip type              : ATmega48A
;Program type           : Application
;Clock frequency        : 4.000000 MHz
;Memory model           : Small
;Optimize for           : Size
;(s)printf features     : int, width
;(s)scanf features      : int, width
;External RAM size      : 0
;Data Stack size        : 128 byte(s)
;Heap size              : 0 byte(s)
;Promote 'char' to 'int': Yes
;'char' is unsigned     : Yes
;8 bit enums            : Yes
;Global 'const' stored in FLASH: Yes
;Enhanced function parameter passing: Yes
;Enhanced core instructions: On
;Automatic register allocation for global variables: On
;Smart register allocation: On

	#define _MODEL_SMALL_

	#pragma AVRPART ADMIN PART_NAME ATmega48A
	#pragma AVRPART MEMORY PROG_FLASH 4096
	#pragma AVRPART MEMORY EEPROM 256
	#pragma AVRPART MEMORY INT_SRAM SIZE 512
	#pragma AVRPART MEMORY INT_SRAM START_ADDR 0x100

	.LISTMAC
	.EQU EERE=0x0
	.EQU EEWE=0x1
	.EQU EEMWE=0x2
	.EQU UDRE=0x5
	.EQU RXC=0x7
	.EQU EECR=0x1F
	.EQU EEDR=0x20
	.EQU EEARL=0x21
	.EQU EEARH=0x22
	.EQU SPSR=0x2D
	.EQU SPDR=0x2E
	.EQU SMCR=0x33
	.EQU MCUSR=0x34
	.EQU MCUCR=0x35
	.EQU WDTCSR=0x60
	.EQU UCSR0A=0xC0
	.EQU UDR0=0xC6
	.EQU SPL=0x3D
	.EQU SPH=0x3E
	.EQU SREG=0x3F
	.EQU GPIOR0=0x1E

	.DEF R0X0=R0
	.DEF R0X1=R1
	.DEF R0X2=R2
	.DEF R0X3=R3
	.DEF R0X4=R4
	.DEF R0X5=R5
	.DEF R0X6=R6
	.DEF R0X7=R7
	.DEF R0X8=R8
	.DEF R0X9=R9
	.DEF R0XA=R10
	.DEF R0XB=R11
	.DEF R0XC=R12
	.DEF R0XD=R13
	.DEF R0XE=R14
	.DEF R0XF=R15
	.DEF R0X10=R16
	.DEF R0X11=R17
	.DEF R0X12=R18
	.DEF R0X13=R19
	.DEF R0X14=R20
	.DEF R0X15=R21
	.DEF R0X16=R22
	.DEF R0X17=R23
	.DEF R0X18=R24
	.DEF R0X19=R25
	.DEF R0X1A=R26
	.DEF R0X1B=R27
	.DEF R0X1C=R28
	.DEF R0X1D=R29
	.DEF R0X1E=R30
	.DEF R0X1F=R31

	.EQU __SRAM_START=0x0100
	.EQU __SRAM_END=0x02FF
	.EQU __DSTACK_SIZE=0x0080
	.EQU __HEAP_SIZE=0x0000
	.EQU __CLEAR_SRAM_SIZE=__SRAM_END-__SRAM_START+1

	.MACRO __CPD1N
	CPI  R30,LOW(@0)
	LDI  R26,HIGH(@0)
	CPC  R31,R26
	LDI  R26,BYTE3(@0)
	CPC  R22,R26
	LDI  R26,BYTE4(@0)
	CPC  R23,R26
	.ENDM

	.MACRO __CPD2N
	CPI  R26,LOW(@0)
	LDI  R30,HIGH(@0)
	CPC  R27,R30
	LDI  R30,BYTE3(@0)
	CPC  R24,R30
	LDI  R30,BYTE4(@0)
	CPC  R25,R30
	.ENDM

	.MACRO __CPWRR
	CP   R@0,R@2
	CPC  R@1,R@3
	.ENDM

	.MACRO __CPWRN
	CPI  R@0,LOW(@2)
	LDI  R30,HIGH(@2)
	CPC  R@1,R30
	.ENDM

	.MACRO __ADDB1MN
	SUBI R30,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDB2MN
	SUBI R26,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDW1MN
	SUBI R30,LOW(-@0-(@1))
	SBCI R31,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW2MN
	SUBI R26,LOW(-@0-(@1))
	SBCI R27,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	SBCI R22,BYTE3(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1N
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	SBCI R22,BYTE3(-@0)
	SBCI R23,BYTE4(-@0)
	.ENDM

	.MACRO __ADDD2N
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	SBCI R24,BYTE3(-@0)
	SBCI R25,BYTE4(-@0)
	.ENDM

	.MACRO __SUBD1N
	SUBI R30,LOW(@0)
	SBCI R31,HIGH(@0)
	SBCI R22,BYTE3(@0)
	SBCI R23,BYTE4(@0)
	.ENDM

	.MACRO __SUBD2N
	SUBI R26,LOW(@0)
	SBCI R27,HIGH(@0)
	SBCI R24,BYTE3(@0)
	SBCI R25,BYTE4(@0)
	.ENDM

	.MACRO __ANDBMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ANDWMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ANDI R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ANDD1N
	ANDI R30,LOW(@0)
	ANDI R31,HIGH(@0)
	ANDI R22,BYTE3(@0)
	ANDI R23,BYTE4(@0)
	.ENDM

	.MACRO __ANDD2N
	ANDI R26,LOW(@0)
	ANDI R27,HIGH(@0)
	ANDI R24,BYTE3(@0)
	ANDI R25,BYTE4(@0)
	.ENDM

	.MACRO __ORBMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ORWMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ORI  R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ORD1N
	ORI  R30,LOW(@0)
	ORI  R31,HIGH(@0)
	ORI  R22,BYTE3(@0)
	ORI  R23,BYTE4(@0)
	.ENDM

	.MACRO __ORD2N
	ORI  R26,LOW(@0)
	ORI  R27,HIGH(@0)
	ORI  R24,BYTE3(@0)
	ORI  R25,BYTE4(@0)
	.ENDM

	.MACRO __DELAY_USB
	LDI  R24,LOW(@0)
__DELAY_USB_LOOP:
	DEC  R24
	BRNE __DELAY_USB_LOOP
	.ENDM

	.MACRO __DELAY_USW
	LDI  R24,LOW(@0)
	LDI  R25,HIGH(@0)
__DELAY_USW_LOOP:
	SBIW R24,1
	BRNE __DELAY_USW_LOOP
	.ENDM

	.MACRO __GETD1S
	LDD  R30,Y+@0
	LDD  R31,Y+@0+1
	LDD  R22,Y+@0+2
	LDD  R23,Y+@0+3
	.ENDM

	.MACRO __GETD2S
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	LDD  R24,Y+@0+2
	LDD  R25,Y+@0+3
	.ENDM

	.MACRO __PUTD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R31
	STD  Y+@0+2,R22
	STD  Y+@0+3,R23
	.ENDM

	.MACRO __PUTD2S
	STD  Y+@0,R26
	STD  Y+@0+1,R27
	STD  Y+@0+2,R24
	STD  Y+@0+3,R25
	.ENDM

	.MACRO __PUTDZ2
	STD  Z+@0,R26
	STD  Z+@0+1,R27
	STD  Z+@0+2,R24
	STD  Z+@0+3,R25
	.ENDM

	.MACRO __CLRD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R30
	STD  Y+@0+2,R30
	STD  Y+@0+3,R30
	.ENDM

	.MACRO __POINTB1MN
	LDI  R30,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW1MN
	LDI  R30,LOW(@0+(@1))
	LDI  R31,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTD1M
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __POINTW1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	LDI  R22,BYTE3(2*@0+(@1))
	LDI  R23,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTB2MN
	LDI  R26,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW2MN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTW2FN
	LDI  R26,LOW(2*@0+(@1))
	LDI  R27,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD2FN
	LDI  R26,LOW(2*@0+(@1))
	LDI  R27,HIGH(2*@0+(@1))
	LDI  R24,BYTE3(2*@0+(@1))
	LDI  R25,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTBRM
	LDI  R@0,LOW(@1)
	.ENDM

	.MACRO __POINTWRM
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __POINTBRMN
	LDI  R@0,LOW(@1+(@2))
	.ENDM

	.MACRO __POINTWRMN
	LDI  R@0,LOW(@2+(@3))
	LDI  R@1,HIGH(@2+(@3))
	.ENDM

	.MACRO __POINTWRFN
	LDI  R@0,LOW(@2*2+(@3))
	LDI  R@1,HIGH(@2*2+(@3))
	.ENDM

	.MACRO __GETD1N
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __GETD2N
	LDI  R26,LOW(@0)
	LDI  R27,HIGH(@0)
	LDI  R24,BYTE3(@0)
	LDI  R25,BYTE4(@0)
	.ENDM

	.MACRO __GETB1MN
	LDS  R30,@0+(@1)
	.ENDM

	.MACRO __GETB1HMN
	LDS  R31,@0+(@1)
	.ENDM

	.MACRO __GETW1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	.ENDM

	.MACRO __GETD1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	LDS  R22,@0+(@1)+2
	LDS  R23,@0+(@1)+3
	.ENDM

	.MACRO __GETBRMN
	LDS  R@0,@1+(@2)
	.ENDM

	.MACRO __GETWRMN
	LDS  R@0,@2+(@3)
	LDS  R@1,@2+(@3)+1
	.ENDM

	.MACRO __GETWRZ
	LDD  R@0,Z+@2
	LDD  R@1,Z+@2+1
	.ENDM

	.MACRO __GETD2Z
	LDD  R26,Z+@0
	LDD  R27,Z+@0+1
	LDD  R24,Z+@0+2
	LDD  R25,Z+@0+3
	.ENDM

	.MACRO __GETB2MN
	LDS  R26,@0+(@1)
	.ENDM

	.MACRO __GETW2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	.ENDM

	.MACRO __GETD2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	LDS  R24,@0+(@1)+2
	LDS  R25,@0+(@1)+3
	.ENDM

	.MACRO __PUTB1MN
	STS  @0+(@1),R30
	.ENDM

	.MACRO __PUTW1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	.ENDM

	.MACRO __PUTD1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	STS  @0+(@1)+2,R22
	STS  @0+(@1)+3,R23
	.ENDM

	.MACRO __PUTB1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRB
	.ENDM

	.MACRO __PUTW1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRW
	.ENDM

	.MACRO __PUTD1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRD
	.ENDM

	.MACRO __PUTBR0MN
	STS  @0+(@1),R0
	.ENDM

	.MACRO __PUTBMRN
	STS  @0+(@1),R@2
	.ENDM

	.MACRO __PUTWMRN
	STS  @0+(@1),R@2
	STS  @0+(@1)+1,R@3
	.ENDM

	.MACRO __PUTBZR
	STD  Z+@1,R@0
	.ENDM

	.MACRO __PUTWZR
	STD  Z+@2,R@0
	STD  Z+@2+1,R@1
	.ENDM

	.MACRO __GETW1R
	MOV  R30,R@0
	MOV  R31,R@1
	.ENDM

	.MACRO __GETW2R
	MOV  R26,R@0
	MOV  R27,R@1
	.ENDM

	.MACRO __GETWRN
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __PUTW1R
	MOV  R@0,R30
	MOV  R@1,R31
	.ENDM

	.MACRO __PUTW2R
	MOV  R@0,R26
	MOV  R@1,R27
	.ENDM

	.MACRO __ADDWRN
	SUBI R@0,LOW(-@2)
	SBCI R@1,HIGH(-@2)
	.ENDM

	.MACRO __ADDWRR
	ADD  R@0,R@2
	ADC  R@1,R@3
	.ENDM

	.MACRO __SUBWRN
	SUBI R@0,LOW(@2)
	SBCI R@1,HIGH(@2)
	.ENDM

	.MACRO __SUBWRR
	SUB  R@0,R@2
	SBC  R@1,R@3
	.ENDM

	.MACRO __ANDWRN
	ANDI R@0,LOW(@2)
	ANDI R@1,HIGH(@2)
	.ENDM

	.MACRO __ANDWRR
	AND  R@0,R@2
	AND  R@1,R@3
	.ENDM

	.MACRO __ORWRN
	ORI  R@0,LOW(@2)
	ORI  R@1,HIGH(@2)
	.ENDM

	.MACRO __ORWRR
	OR   R@0,R@2
	OR   R@1,R@3
	.ENDM

	.MACRO __EORWRR
	EOR  R@0,R@2
	EOR  R@1,R@3
	.ENDM

	.MACRO __GETWRS
	LDD  R@0,Y+@2
	LDD  R@1,Y+@2+1
	.ENDM

	.MACRO __PUTBSR
	STD  Y+@1,R@0
	.ENDM

	.MACRO __PUTWSR
	STD  Y+@2,R@0
	STD  Y+@2+1,R@1
	.ENDM

	.MACRO __MOVEWRR
	MOV  R@0,R@2
	MOV  R@1,R@3
	.ENDM

	.MACRO __INWR
	IN   R@0,@2
	IN   R@1,@2+1
	.ENDM

	.MACRO __OUTWR
	OUT  @2+1,R@1
	OUT  @2,R@0
	.ENDM

	.MACRO __CALL1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	ICALL
	.ENDM

	.MACRO __CALL1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	RCALL __GETW1PF
	ICALL
	.ENDM

	.MACRO __CALL2EN
	PUSH R26
	PUSH R27
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMRDW
	POP  R27
	POP  R26
	ICALL
	.ENDM

	.MACRO __CALL2EX
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	RCALL __EEPROMRDD
	ICALL
	.ENDM

	.MACRO __GETW1STACK
	IN   R30,SPL
	IN   R31,SPH
	ADIW R30,@0+1
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1STACK
	IN   R30,SPL
	IN   R31,SPH
	ADIW R30,@0+1
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z
	MOVW R30,R0
	.ENDM

	.MACRO __NBST
	BST  R@0,@1
	IN   R30,SREG
	LDI  R31,0x40
	EOR  R30,R31
	OUT  SREG,R30
	.ENDM


	.MACRO __PUTB1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RNS
	MOVW R26,R@0
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	RCALL __PUTDP1
	.ENDM


	.MACRO __GETB1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R30,Z
	.ENDM

	.MACRO __GETB1HSX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	.ENDM

	.MACRO __GETW1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z+
	LD   R23,Z
	MOVW R30,R0
	.ENDM

	.MACRO __GETB2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R26,X
	.ENDM

	.MACRO __GETW2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	.ENDM

	.MACRO __GETD2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R1,X+
	LD   R24,X+
	LD   R25,X
	MOVW R26,R0
	.ENDM

	.MACRO __GETBRSX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	LD   R@0,Z
	.ENDM

	.MACRO __GETWRSX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	LD   R@0,Z+
	LD   R@1,Z
	.ENDM

	.MACRO __GETBRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	LD   R@0,X
	.ENDM

	.MACRO __GETWRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	LD   R@0,X+
	LD   R@1,X
	.ENDM

	.MACRO __LSLW8SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	CLR  R30
	.ENDM

	.MACRO __PUTB1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __CLRW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __CLRD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R30
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __PUTB2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z,R26
	.ENDM

	.MACRO __PUTW2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z,R27
	.ENDM

	.MACRO __PUTD2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z+,R27
	ST   Z+,R24
	ST   Z,R25
	.ENDM

	.MACRO __PUTBSRX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	ST   Z,R@0
	.ENDM

	.MACRO __PUTWSRX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	ST   Z+,R@0
	ST   Z,R@1
	.ENDM

	.MACRO __PUTB1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __MULBRR
	MULS R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRRU
	MUL  R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRR0
	MULS R@0,R@1
	.ENDM

	.MACRO __MULBRRU0
	MUL  R@0,R@1
	.ENDM

	.MACRO __MULBNWRU
	LDI  R26,@2
	MUL  R26,R@0
	MOVW R30,R0
	MUL  R26,R@1
	ADD  R31,R0
	.ENDM

;NAME DEFINITIONS FOR GLOBAL VARIABLES ALLOCATED TO REGISTERS
	.DEF _Address=R4
	.DEF _status=R3
	.DEF _rx_wr_index0=R6
	.DEF _rx_rd_index0=R5
	.DEF _rx_counter0=R7
	.DEF _rx_counter0_msb=R8

;GPIOR0 INITIALIZATION VALUE
	.EQU __GPIOR0_INIT=0x00

	.CSEG
	.ORG 0x00

;START OF CODE MARKER
__START_OF_CODE:

;INTERRUPT VECTORS
	RJMP __RESET
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP _usart_rx_isr
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00

_tbl10_G101:
	.DB  0x10,0x27,0xE8,0x3,0x64,0x0,0xA,0x0
	.DB  0x1,0x0
_tbl16_G101:
	.DB  0x0,0x10,0x0,0x1,0x10,0x0,0x1,0x0

;GLOBAL REGISTER VARIABLES INITIALIZATION
__REG_VARS:
	.DB  0x0,0x0,0x0,0x0


__GLOBAL_INI_TBL:
	.DW  0x04
	.DW  0x05
	.DW  __REG_VARS*2

_0xFFFFFFFF:
	.DW  0

#define __GLOBAL_INI_TBL_PRESENT 1

__RESET:
	CLI
	CLR  R30
	OUT  EECR,R30
	OUT  MCUCR,R30

;CLEAR R2-R14
	LDI  R24,(14-2)+1
	LDI  R26,2
	CLR  R27
__CLEAR_REG:
	ST   X+,R30
	DEC  R24
	BRNE __CLEAR_REG

;CLEAR SRAM
	LDI  R24,LOW(__CLEAR_SRAM_SIZE)
	LDI  R25,HIGH(__CLEAR_SRAM_SIZE)
	LDI  R26,LOW(__SRAM_START)
	LDI  R27,HIGH(__SRAM_START)
__CLEAR_SRAM:
	ST   X+,R30
	SBIW R24,1
	BRNE __CLEAR_SRAM

;GLOBAL VARIABLES INITIALIZATION
	LDI  R30,LOW(__GLOBAL_INI_TBL*2)
	LDI  R31,HIGH(__GLOBAL_INI_TBL*2)
__GLOBAL_INI_NEXT:
	LPM  R24,Z+
	LPM  R25,Z+
	SBIW R24,0
	BREQ __GLOBAL_INI_END
	LPM  R26,Z+
	LPM  R27,Z+
	LPM  R0,Z+
	LPM  R1,Z+
	MOVW R22,R30
	MOVW R30,R0
__GLOBAL_INI_LOOP:
	LPM  R0,Z+
	ST   X+,R0
	SBIW R24,1
	BRNE __GLOBAL_INI_LOOP
	MOVW R30,R22
	RJMP __GLOBAL_INI_NEXT
__GLOBAL_INI_END:

;GPIOR0 INITIALIZATION
	LDI  R30,__GPIOR0_INIT
	OUT  GPIOR0,R30

;HARDWARE STACK POINTER INITIALIZATION
	LDI  R30,LOW(__SRAM_END-__HEAP_SIZE)
	OUT  SPL,R30
	LDI  R30,HIGH(__SRAM_END-__HEAP_SIZE)
	OUT  SPH,R30

;DATA STACK POINTER INITIALIZATION
	LDI  R28,LOW(__SRAM_START+__DSTACK_SIZE)
	LDI  R29,HIGH(__SRAM_START+__DSTACK_SIZE)

	RJMP _main

	.ESEG
	.ORG 0

	.DSEG
	.ORG 0x180

	.CSEG
;/*******************************************************
;CodeWizardAVR V3.12 Advanced
;Project :
;Version :
;Date    : 9/8/2020
;Author  : M.Mashaghi
;Company : MegaMouj
;
;Chip type               : ATmega48A
;AVR Core Clock frequency: 4.000000 MHz
;*******************************************************/
;#include <mega48a.h>
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x01
	.EQU __sm_mask=0x0E
	.EQU __sm_adc_noise_red=0x02
	.EQU __sm_powerdown=0x04
	.EQU __sm_powersave=0x06
	.EQU __sm_standby=0x0C
	.EQU __sm_ext_standby=0x0E
	.SET power_ctrl_reg=smcr
	#endif
;#include <delay.h>
;
;#define DataDirection PORTC.1
;
;#define _7seg1G PORTD.5
;
;#define _7seg2A PORTB.0
;#define _7seg2B PORTB.1
;#define _7seg2C PORTD.2
;#define _7seg2D PORTD.4
;#define _7seg2E PORTD.3
;#define _7seg2F PORTD.7
;#define _7seg2G PORTD.6
;#define _7seg2dp PORTC.5
;
;#define _7seg3A PORTB.4
;#define _7seg3B PORTB.5
;#define _7seg3C PORTC.2
;#define _7seg3D PORTC.3
;#define _7seg3E PORTC.4
;#define _7seg3F PORTB.3
;#define _7seg3G PORTB.2
;
;void ShowValue(signed char val);
;void ShowFloatValue(float val);
;void Report(void);
;
;unsigned char Address;
;eeprom unsigned char AddressSaved;
;unsigned char rom_codes[1][9];
;float T1, T2;
;unsigned char status;
;
;
;union
;{float f;
; unsigned long int ui32;
; unsigned int ui16[2];
; unsigned char ui8[4];
;}format_converter_32;
;
;// 1 Wire Bus interface functions
;#include <1wire.h>
;
;// DS1820 Temperature Sensor functions
;#include <ds1820.h>
;
;// Declare your global variables here
;
;#define DATA_REGISTER_EMPTY (1<<UDRE0)
;#define RX_COMPLETE (1<<RXC0)
;#define FRAMING_ERROR (1<<FE0)
;#define PARITY_ERROR (1<<UPE0)
;#define DATA_OVERRUN (1<<DOR0)
;
;// USART Receiver buffer
;#define RX_BUFFER_SIZE0 256
;char rx_buffer0[RX_BUFFER_SIZE0];
;
;#if RX_BUFFER_SIZE0 <= 256
;unsigned char rx_wr_index0=0,rx_rd_index0=0;
;#else
;unsigned int rx_wr_index0=0,rx_rd_index0=0;
;#endif
;
;#if RX_BUFFER_SIZE0 < 256
;unsigned char rx_counter0=0;
;#else
;unsigned int rx_counter0=0;
;#endif
;
;// This flag is set on USART Receiver buffer overflow
;bit rx_buffer_overflow0;
;
;// USART Receiver interrupt service routine
;interrupt [USART_RXC] void usart_rx_isr(void)
; 0000 0059 {

	.CSEG
_usart_rx_isr:
; .FSTART _usart_rx_isr
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
; 0000 005A char status,data;
; 0000 005B status=UCSR0A;
	RCALL __SAVELOCR2
;	status -> R17
;	data -> R16
	LDS  R17,192
; 0000 005C data=UDR0;
	LDS  R16,198
; 0000 005D if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
	MOV  R30,R17
	ANDI R30,LOW(0x1C)
	BRNE _0x3
; 0000 005E    {
; 0000 005F    rx_buffer0[rx_wr_index0++]=data;
	MOV  R30,R6
	INC  R6
	LDI  R31,0
	SUBI R30,LOW(-_rx_buffer0)
	SBCI R31,HIGH(-_rx_buffer0)
	ST   Z,R16
; 0000 0060 #if RX_BUFFER_SIZE0 == 256
; 0000 0061    // special case for receiver buffer size=256
; 0000 0062    if (++rx_counter0 == 0) rx_buffer_overflow0=1;
	__GETW1R 7,8
	ADIW R30,1
	__PUTW1R 7,8
	BRNE _0x4
	SBI  0x1E,0
; 0000 0063 #else
; 0000 0064    if (rx_wr_index0 == RX_BUFFER_SIZE0) rx_wr_index0=0;
; 0000 0065    if (++rx_counter0 == RX_BUFFER_SIZE0)
; 0000 0066       {
; 0000 0067       rx_counter0=0;
; 0000 0068       rx_buffer_overflow0=1;
; 0000 0069       }
; 0000 006A #endif
; 0000 006B    }
_0x4:
; 0000 006C }
_0x3:
	LD   R16,Y+
	LD   R17,Y+
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	RETI
; .FEND
;
;#ifndef _DEBUG_TERMINAL_IO_
;// Get a character from the USART Receiver buffer
;#define _ALTERNATE_GETCHAR_
;#pragma used+
;char getchar(void)
; 0000 0073 {
; 0000 0074 char data;
; 0000 0075 while (rx_counter0==0);
;	data -> R17
; 0000 0076 data=rx_buffer0[rx_rd_index0++];
; 0000 0077 #if RX_BUFFER_SIZE0 != 256
; 0000 0078 if (rx_rd_index0 == RX_BUFFER_SIZE0) rx_rd_index0=0;
; 0000 0079 #endif
; 0000 007A #asm("cli")
; 0000 007B --rx_counter0;
; 0000 007C #asm("sei")
; 0000 007D return data;
; 0000 007E }
;#pragma used-
;#endif
;
;// Standard Input/Output functions
;#include <stdio.h>
;
;// Voltage Reference: AVCC pin
;#define ADC_VREF_TYPE ((0<<REFS1) | (1<<REFS0) | (1<<ADLAR))
;
;// Read the 8 most significant bits of the AD conversion result
;unsigned char read_adc(unsigned char adc_input)
; 0000 008A {
_read_adc:
; .FSTART _read_adc
; 0000 008B ADMUX=adc_input | ADC_VREF_TYPE;
	ST   -Y,R26
;	adc_input -> Y+0
	LD   R30,Y
	ORI  R30,LOW(0x60)
	STS  124,R30
; 0000 008C delay_us(10);
	__DELAY_USB 13
; 0000 008D ADCSRA|=(1<<ADSC);
	LDS  R30,122
	ORI  R30,0x40
	STS  122,R30
; 0000 008E while ((ADCSRA & (1<<ADIF))==0);
_0xA:
	LDS  R30,122
	ANDI R30,LOW(0x10)
	BREQ _0xA
; 0000 008F ADCSRA|=(1<<ADIF);
	LDS  R30,122
	ORI  R30,0x10
	STS  122,R30
; 0000 0090 return ADCH;
	LDS  R30,121
	RJMP _0x2080001
; 0000 0091 }
; .FEND
;
;void main(void)
; 0000 0094 {
_main:
; .FSTART _main
; 0000 0095 signed char i, devices;
; 0000 0096 float f;
; 0000 0097 unsigned char analog;
; 0000 0098 
; 0000 0099 // Crystal Oscillator division factor: 1
; 0000 009A #pragma optsize-
; 0000 009B CLKPR=(1<<CLKPCE);
	SBIW R28,4
;	i -> R17
;	devices -> R16
;	f -> Y+0
;	analog -> R19
	LDI  R30,LOW(128)
	STS  97,R30
; 0000 009C CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
	LDI  R30,LOW(0)
	STS  97,R30
; 0000 009D #ifdef _OPTIMIZE_SIZE_
; 0000 009E #pragma optsize+
; 0000 009F #endif
; 0000 00A0 
; 0000 00A1 // Input/Output Ports initialization
; 0000 00A2 DDRB=0x3F;PORTB=0x3F;
	LDI  R30,LOW(63)
	OUT  0x4,R30
	OUT  0x5,R30
; 0000 00A3 DDRC=0x3E;PORTC=0x3C;
	LDI  R30,LOW(62)
	OUT  0x7,R30
	LDI  R30,LOW(60)
	OUT  0x8,R30
; 0000 00A4 DDRD=0xFC;PORTD=0xFC;
	LDI  R30,LOW(252)
	OUT  0xA,R30
	OUT  0xB,R30
; 0000 00A5 
; 0000 00A6 // USART initialization
; 0000 00A7 // Communication Parameters: 8 Data, 1 Stop, No Parity
; 0000 00A8 // USART Receiver: On
; 0000 00A9 // USART Transmitter: On
; 0000 00AA // USART0 Mode: Asynchronous
; 0000 00AB // USART Baud Rate: 9600
; 0000 00AC UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (0<<U2X0) | (0<<MPCM0);
	LDI  R30,LOW(0)
	STS  192,R30
; 0000 00AD UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	LDI  R30,LOW(152)
	STS  193,R30
; 0000 00AE UCSR0C=(0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	LDI  R30,LOW(6)
	STS  194,R30
; 0000 00AF UBRR0H=0x00;
	LDI  R30,LOW(0)
	STS  197,R30
; 0000 00B0 UBRR0L=0x19;
	LDI  R30,LOW(25)
	STS  196,R30
; 0000 00B1 
; 0000 00B2 // ADC initialization
; 0000 00B3 // ADC Clock frequency: 1000.000 kHz
; 0000 00B4 // ADC Voltage Reference: AVCC pin
; 0000 00B5 // Only the 8 most significant bits of the AD conversion result are used
; 0000 00B6 DIDR0=0;
	LDI  R30,LOW(0)
	STS  126,R30
; 0000 00B7 ADMUX=ADC_VREF_TYPE;
	LDI  R30,LOW(96)
	STS  124,R30
; 0000 00B8 ADCSRA=(1<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (1<<ADPS1) | (0<<ADPS0);
	LDI  R30,LOW(130)
	STS  122,R30
; 0000 00B9 ADCSRB=(0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0);
	LDI  R30,LOW(0)
	STS  123,R30
; 0000 00BA 
; 0000 00BB // 1 Wire Bus initialization
; 0000 00BC // 1 Wire Data port: PORTC
; 0000 00BD // 1 Wire Data bit: 0
; 0000 00BE // Note: 1 Wire port settings are specified in the
; 0000 00BF // Project|Configure|C Compiler|Libraries|1 Wire menu.
; 0000 00C0 w1_init();
	RCALL _w1_init
; 0000 00C1 
; 0000 00C2 // Global enable interrupts
; 0000 00C3 #asm("sei")
	sei
; 0000 00C4 
; 0000 00C5 //RetreiveAddress();
; 0000 00C6 
; 0000 00C7 devices=w1_search(0xf0,rom_codes);
	LDI  R30,LOW(240)
	ST   -Y,R30
	LDI  R26,LOW(_rom_codes)
	LDI  R27,HIGH(_rom_codes)
	RCALL _w1_search
	MOV  R16,R30
; 0000 00C8 if (devices) ShowValue(15);
	CPI  R16,0
	BREQ _0xD
	LDI  R26,LOW(15)
	RJMP _0x171
; 0000 00C9 else ShowValue(-3);
_0xD:
	LDI  R26,LOW(253)
_0x171:
	RCALL _ShowValue
; 0000 00CA 
; 0000 00CB delay_ms(2000);
	LDI  R26,LOW(2000)
	LDI  R27,HIGH(2000)
	RCALL _delay_ms
; 0000 00CC //if (ds1820_init(0,-125,125,DS18B20_10BIT_RES)) ShowValue(15);
; 0000 00CD //else ShowValue(-1);
; 0000 00CE 
; 0000 00CF //while(1) {
; 0000 00D0 //ShowValue(read_adc(6)/3);
; 0000 00D1 //putchar(read_adc(6));
; 0000 00D2 //delay_ms(50);
; 0000 00D3 //}
; 0000 00D4 
; 0000 00D5 while (1)
_0xF:
; 0000 00D6     {
; 0000 00D7         T1 = ds1820_temperature_10(0);
	LDI  R26,LOW(0)
	LDI  R27,0
	RCALL _ds1820_temperature_10
	LDI  R26,LOW(_T1)
	LDI  R27,HIGH(_T1)
	RCALL __CWD1
	RCALL __CDF1
	RCALL __PUTDP1
; 0000 00D8         //T1 /= 100;
; 0000 00D9         T1 /= 75; //this is so strange!!! (75 is the result of caliberation with boiling water and freezing water)
	RCALL SUBOPT_0x0
	__GETD1N 0x42960000
	RCALL __DIVF21
	STS  _T1,R30
	STS  _T1+1,R31
	STS  _T1+2,R22
	STS  _T1+3,R23
; 0000 00DA         ShowFloatValue(T1);
	RCALL SUBOPT_0x0
	RCALL _ShowFloatValue
; 0000 00DB 
; 0000 00DC 
; 0000 00DD         T2 = 0;
	LDI  R30,LOW(0)
	STS  _T2,R30
	STS  _T2+1,R30
	STS  _T2+2,R30
	STS  _T2+3,R30
; 0000 00DE //        analog=read_adc(6);
; 0000 00DF //        if (analog > 200) status |= 0b00000001;
; 0000 00E0 //        else if ((analog > 112)&&(analog < 140)) status |= 0b00000001;
; 0000 00E1 //        else status &= 0b11111110; //--> door is open
; 0000 00E2 
; 0000 00E3     if (read_adc(6) < 150) status |= 0b00000010; else status &= 0b11111101; //--> emergency button pressed
	LDI  R26,LOW(6)
	RCALL _read_adc
	CPI  R30,LOW(0x96)
	BRSH _0x12
	LDI  R30,LOW(2)
	OR   R3,R30
	RJMP _0x13
_0x12:
	LDI  R30,LOW(253)
	AND  R3,R30
; 0000 00E4     if (read_adc(7) > 150) status |= 0b00000001; else status &= 0b11111110; //--> door is open
_0x13:
	LDI  R26,LOW(7)
	RCALL _read_adc
	CPI  R30,LOW(0x97)
	BRLO _0x14
	LDI  R30,LOW(1)
	OR   R3,R30
	RJMP _0x15
_0x14:
	LDI  R30,LOW(254)
	AND  R3,R30
; 0000 00E5 
; 0000 00E6 //        if (analog < 138) status |= 0b00000010; else status &= 0b11111101; //--> emergency button pressed
; 0000 00E7 
; 0000 00E8         Report();
_0x15:
	RCALL _Report
; 0000 00E9 
; 0000 00EA         delay_ms(100);
	LDI  R26,LOW(100)
	LDI  R27,0
	RCALL _delay_ms
; 0000 00EB     }
	RJMP _0xF
; 0000 00EC }
_0x16:
	RJMP _0x16
; .FEND
;
;void ShowValue(signed char val) {
; 0000 00EE void ShowValue(signed char val) {
_ShowValue:
; .FSTART _ShowValue
; 0000 00EF     unsigned char dig2, dig3;
; 0000 00F0 
; 0000 00F1     if (val < 0) _7seg1G=0; else _7seg1G=1;
	ST   -Y,R26
	RCALL __SAVELOCR2
;	val -> Y+2
;	dig2 -> R17
;	dig3 -> R16
	LDD  R26,Y+2
	CPI  R26,0
	BRGE _0x17
	CBI  0xB,5
	RJMP _0x1A
_0x17:
	SBI  0xB,5
; 0000 00F2     if (val < 0) val = 0-val;
_0x1A:
	LDD  R26,Y+2
	CPI  R26,0
	BRGE _0x1D
	LDI  R30,LOW(0)
	SUB  R30,R26
	STD  Y+2,R30
; 0000 00F3 
; 0000 00F4     dig2 = val / 10;
_0x1D:
	RCALL SUBOPT_0x1
	RCALL __DIVW21
	MOV  R17,R30
; 0000 00F5     dig3 = val % 10;
	RCALL SUBOPT_0x1
	RCALL __MODW21
	MOV  R16,R30
; 0000 00F6 
; 0000 00F7     switch (dig2) {
	MOV  R30,R17
	LDI  R31,0
; 0000 00F8         case 0: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=0; _7seg2F=0; _7seg2G=1; _7seg2dp=1; break;
	SBIW R30,0
	BRNE _0x21
	RCALL SUBOPT_0x2
	RCALL SUBOPT_0x3
	CBI  0xB,3
	CBI  0xB,7
	SBI  0xB,6
	RJMP _0x172
; 0000 00F9         case 1: _7seg2A=1; _7seg2B=0; _7seg2C=0; _7seg2D=1; _7seg2E=1; _7seg2F=1; _7seg2G=1; _7seg2dp=1; break;
_0x21:
	CPI  R30,LOW(0x1)
	LDI  R26,HIGH(0x1)
	CPC  R31,R26
	BRNE _0x32
	RCALL SUBOPT_0x4
	SBI  0xB,3
	SBI  0xB,7
	SBI  0xB,6
	RJMP _0x172
; 0000 00FA         case 2: _7seg2A=0; _7seg2B=0; _7seg2C=1; _7seg2D=0; _7seg2E=0; _7seg2F=1; _7seg2G=0; _7seg2dp=1; break;
_0x32:
	CPI  R30,LOW(0x2)
	LDI  R26,HIGH(0x2)
	CPC  R31,R26
	BRNE _0x43
	RCALL SUBOPT_0x2
	SBI  0xB,2
	CBI  0xB,4
	CBI  0xB,3
	SBI  0xB,7
	RJMP _0x173
; 0000 00FB         case 3: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=1; _7seg2F=1; _7seg2G=0; _7seg2dp=1; break;
_0x43:
	CPI  R30,LOW(0x3)
	LDI  R26,HIGH(0x3)
	CPC  R31,R26
	BRNE _0x54
	RCALL SUBOPT_0x2
	RCALL SUBOPT_0x3
	SBI  0xB,3
	SBI  0xB,7
	RJMP _0x173
; 0000 00FC         case 4: _7seg2A=1; _7seg2B=0; _7seg2C=0; _7seg2D=1; _7seg2E=1; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
_0x54:
	CPI  R30,LOW(0x4)
	LDI  R26,HIGH(0x4)
	CPC  R31,R26
	BRNE _0x65
	RCALL SUBOPT_0x4
	RJMP _0x174
; 0000 00FD         case 5: _7seg2A=0; _7seg2B=1; _7seg2C=0; _7seg2D=0; _7seg2E=1; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
_0x65:
	CPI  R30,LOW(0x5)
	LDI  R26,HIGH(0x5)
	CPC  R31,R26
	BRNE _0x76
	CBI  0x5,0
	SBI  0x5,1
	RJMP _0x175
; 0000 00FE         case 6: _7seg2A=0; _7seg2B=1; _7seg2C=0; _7seg2D=0; _7seg2E=0; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
_0x76:
	CPI  R30,LOW(0x6)
	LDI  R26,HIGH(0x6)
	CPC  R31,R26
	BRNE _0x87
	CBI  0x5,0
	SBI  0x5,1
	RCALL SUBOPT_0x3
	CBI  0xB,3
	RJMP _0x176
; 0000 00FF         case 7: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=1; _7seg2E=1; _7seg2F=1; _7seg2G=1; _7seg2dp=1; break;
_0x87:
	CPI  R30,LOW(0x7)
	LDI  R26,HIGH(0x7)
	CPC  R31,R26
	BRNE _0x98
	RCALL SUBOPT_0x2
	CBI  0xB,2
	SBI  0xB,4
	SBI  0xB,3
	SBI  0xB,7
	SBI  0xB,6
	RJMP _0x172
; 0000 0100         case 8: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=0; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
_0x98:
	CPI  R30,LOW(0x8)
	LDI  R26,HIGH(0x8)
	CPC  R31,R26
	BRNE _0xA9
	RCALL SUBOPT_0x2
	RCALL SUBOPT_0x3
	CBI  0xB,3
	RJMP _0x176
; 0000 0101         case 9: _7seg2A=0; _7seg2B=0; _7seg2C=0; _7seg2D=0; _7seg2E=1; _7seg2F=0; _7seg2G=0; _7seg2dp=1; break;
_0xA9:
	CPI  R30,LOW(0x9)
	LDI  R26,HIGH(0x9)
	CPC  R31,R26
	BRNE _0x20
	RCALL SUBOPT_0x2
_0x175:
	CBI  0xB,2
	CBI  0xB,4
_0x174:
	SBI  0xB,3
_0x176:
	CBI  0xB,7
_0x173:
	CBI  0xB,6
_0x172:
	SBI  0x8,5
; 0000 0102     }
_0x20:
; 0000 0103 
; 0000 0104     switch (dig3) {
	MOV  R30,R16
	LDI  R31,0
; 0000 0105         case 0: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=0; _7seg3F=0; _7seg3G=1; break;
	SBIW R30,0
	BRNE _0xCE
	RCALL SUBOPT_0x5
	RCALL SUBOPT_0x6
	CBI  0x8,4
	CBI  0x5,3
	SBI  0x5,2
	RJMP _0xCD
; 0000 0106         case 1: _7seg3A=1; _7seg3B=0; _7seg3C=0; _7seg3D=1; _7seg3E=1; _7seg3F=1; _7seg3G=1; break;
_0xCE:
	CPI  R30,LOW(0x1)
	LDI  R26,HIGH(0x1)
	CPC  R31,R26
	BRNE _0xDD
	RCALL SUBOPT_0x7
	SBI  0x8,4
	SBI  0x5,3
	SBI  0x5,2
	RJMP _0xCD
; 0000 0107         case 2: _7seg3A=0; _7seg3B=0; _7seg3C=1; _7seg3D=0; _7seg3E=0; _7seg3F=1; _7seg3G=0; break;
_0xDD:
	CPI  R30,LOW(0x2)
	LDI  R26,HIGH(0x2)
	CPC  R31,R26
	BRNE _0xEC
	RCALL SUBOPT_0x5
	SBI  0x8,2
	CBI  0x8,3
	CBI  0x8,4
	SBI  0x5,3
	RJMP _0x177
; 0000 0108         case 3: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=1; _7seg3F=1; _7seg3G=0; break;
_0xEC:
	CPI  R30,LOW(0x3)
	LDI  R26,HIGH(0x3)
	CPC  R31,R26
	BRNE _0xFB
	RCALL SUBOPT_0x5
	RCALL SUBOPT_0x6
	SBI  0x8,4
	SBI  0x5,3
	RJMP _0x177
; 0000 0109         case 4: _7seg3A=1; _7seg3B=0; _7seg3C=0; _7seg3D=1; _7seg3E=1; _7seg3F=0; _7seg3G=0; break;
_0xFB:
	CPI  R30,LOW(0x4)
	LDI  R26,HIGH(0x4)
	CPC  R31,R26
	BRNE _0x10A
	RCALL SUBOPT_0x7
	RJMP _0x178
; 0000 010A         case 5: _7seg3A=0; _7seg3B=1; _7seg3C=0; _7seg3D=0; _7seg3E=1; _7seg3F=0; _7seg3G=0; break;
_0x10A:
	CPI  R30,LOW(0x5)
	LDI  R26,HIGH(0x5)
	CPC  R31,R26
	BRNE _0x119
	CBI  0x5,4
	SBI  0x5,5
	RJMP _0x179
; 0000 010B         case 6: _7seg3A=0; _7seg3B=1; _7seg3C=0; _7seg3D=0; _7seg3E=0; _7seg3F=0; _7seg3G=0; break;
_0x119:
	CPI  R30,LOW(0x6)
	LDI  R26,HIGH(0x6)
	CPC  R31,R26
	BRNE _0x128
	CBI  0x5,4
	SBI  0x5,5
	RCALL SUBOPT_0x6
	CBI  0x8,4
	RJMP _0x17A
; 0000 010C         case 7: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=1; _7seg3E=1; _7seg3F=1; _7seg3G=1; break;
_0x128:
	CPI  R30,LOW(0x7)
	LDI  R26,HIGH(0x7)
	CPC  R31,R26
	BRNE _0x137
	RCALL SUBOPT_0x5
	CBI  0x8,2
	SBI  0x8,3
	SBI  0x8,4
	SBI  0x5,3
	SBI  0x5,2
	RJMP _0xCD
; 0000 010D         case 8: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=0; _7seg3F=0; _7seg3G=0; break;
_0x137:
	CPI  R30,LOW(0x8)
	LDI  R26,HIGH(0x8)
	CPC  R31,R26
	BRNE _0x146
	RCALL SUBOPT_0x5
	RCALL SUBOPT_0x6
	CBI  0x8,4
	RJMP _0x17A
; 0000 010E         case 9: _7seg3A=0; _7seg3B=0; _7seg3C=0; _7seg3D=0; _7seg3E=1; _7seg3F=0; _7seg3G=0; break;
_0x146:
	CPI  R30,LOW(0x9)
	LDI  R26,HIGH(0x9)
	CPC  R31,R26
	BRNE _0xCD
	RCALL SUBOPT_0x5
_0x179:
	CBI  0x8,2
	CBI  0x8,3
_0x178:
	SBI  0x8,4
_0x17A:
	CBI  0x5,3
_0x177:
	CBI  0x5,2
; 0000 010F     }
_0xCD:
; 0000 0110 
; 0000 0111 }
	RCALL __LOADLOCR2
	RJMP _0x2080006
; .FEND
;
;void ShowFloatValue(float val) {
; 0000 0113 void ShowFloatValue(float val) {
_ShowFloatValue:
; .FSTART _ShowFloatValue
; 0000 0114     signed int sitemp;
; 0000 0115 
; 0000 0116     sitemp = (val * 10);
	RCALL __PUTPARD2
	RCALL __SAVELOCR2
;	val -> Y+2
;	sitemp -> R16,R17
	__GETD2S 2
	__GETD1N 0x41200000
	RCALL __MULF12
	RCALL __CFD1
	MOVW R16,R30
; 0000 0117     if ((sitemp > 99) || (sitemp < -99)) {
	__CPWRN 16,17,100
	BRGE _0x165
	__CPWRN 16,17,-99
	BRGE _0x164
_0x165:
; 0000 0118         ShowValue (sitemp / 10);
	MOVW R26,R16
	LDI  R30,LOW(10)
	LDI  R31,HIGH(10)
	RCALL __DIVW21
	MOV  R26,R30
	RCALL _ShowValue
; 0000 0119         return;
	RCALL __LOADLOCR2
	RJMP _0x2080004
; 0000 011A     }
; 0000 011B 
; 0000 011C     ShowValue (sitemp);
_0x164:
	MOV  R26,R16
	RCALL _ShowValue
; 0000 011D     _7seg2dp=0;
	CBI  0x8,5
; 0000 011E 
; 0000 011F }
	RCALL __LOADLOCR2
	RJMP _0x2080004
; .FEND
;
;void Report(void) {
; 0000 0121 void Report(void) {
_Report:
; .FSTART _Report
; 0000 0122     unsigned char buff[20], crc, i;
; 0000 0123     unsigned char d1, d2, d3;
; 0000 0124     unsigned int uitemp;
; 0000 0125     float ftemp;
; 0000 0126 
; 0000 0127     format_converter_32.f = T1;
	SBIW R28,26
	RCALL __SAVELOCR6
;	buff -> Y+12
;	crc -> R17
;	i -> R16
;	d1 -> R19
;	d2 -> R18
;	d3 -> R21
;	uitemp -> Y+10
;	ftemp -> Y+6
	LDS  R30,_T1
	LDS  R31,_T1+1
	LDS  R22,_T1+2
	LDS  R23,_T1+3
	RCALL SUBOPT_0x8
; 0000 0128     buff[0] = format_converter_32.ui8[0];
	STD  Y+12,R30
; 0000 0129     buff[1] = format_converter_32.ui8[1];
	__GETB1MN _format_converter_32,1
	STD  Y+13,R30
; 0000 012A     buff[2] = format_converter_32.ui8[2];
	__GETB1MN _format_converter_32,2
	STD  Y+14,R30
; 0000 012B     buff[3] = format_converter_32.ui8[3];
	__GETB1MN _format_converter_32,3
	STD  Y+15,R30
; 0000 012C 
; 0000 012D     format_converter_32.f = T2;
	LDS  R30,_T2
	LDS  R31,_T2+1
	LDS  R22,_T2+2
	LDS  R23,_T2+3
	RCALL SUBOPT_0x8
; 0000 012E     buff[4] = format_converter_32.ui8[0];
	STD  Y+16,R30
; 0000 012F     buff[5] = format_converter_32.ui8[1];
	__GETB1MN _format_converter_32,1
	STD  Y+17,R30
; 0000 0130     buff[6] = format_converter_32.ui8[2];
	__GETB1MN _format_converter_32,2
	STD  Y+18,R30
; 0000 0131     buff[7] = format_converter_32.ui8[3];
	__GETB1MN _format_converter_32,3
	STD  Y+19,R30
; 0000 0132     buff[8] = status;
	MOVW R30,R28
	ADIW R30,20
	ST   Z,R3
; 0000 0133 
; 0000 0134     crc = 0;
	LDI  R17,LOW(0)
; 0000 0135     for (i=0; i<9; i++)
	LDI  R16,LOW(0)
_0x16A:
	CPI  R16,9
	BRSH _0x16B
; 0000 0136         crc ^= buff[i];
	RCALL SUBOPT_0x9
	LD   R30,X
	EOR  R17,R30
	SUBI R16,-1
	RJMP _0x16A
_0x16B:
; 0000 0138 buff[9] = crc;
	MOVW R30,R28
	ADIW R30,21
	ST   Z,R17
; 0000 0139 
; 0000 013A     DataDirection = 1;
	SBI  0x8,1
; 0000 013B 
; 0000 013C     putchar(0x84);
	LDI  R26,LOW(132)
	RCALL _putchar
; 0000 013D     putchar(0xB3);
	LDI  R26,LOW(179)
	RCALL _putchar
; 0000 013E     putchar(0xE3);
	LDI  R26,LOW(227)
	RCALL _putchar
; 0000 013F     putchar(0x75);
	LDI  R26,LOW(117)
	RCALL _putchar
; 0000 0140 
; 0000 0141     for (i=0; i<10; i++)
	LDI  R16,LOW(0)
_0x16F:
	CPI  R16,10
	BRSH _0x170
; 0000 0142         putchar(buff[i]);
	RCALL SUBOPT_0x9
	LD   R26,X
	RCALL _putchar
	SUBI R16,-1
	RJMP _0x16F
_0x170:
; 0000 0176 }
	RCALL __LOADLOCR6
	ADIW R28,32
	RET
; .FEND
;

	.CSEG
_ds1820_select:
; .FSTART _ds1820_select
	ST   -Y,R27
	ST   -Y,R26
	ST   -Y,R17
	RCALL _w1_init
	CPI  R30,0
	BRNE _0x2000003
	LDI  R30,LOW(0)
	RJMP _0x2080005
_0x2000003:
	LDD  R30,Y+1
	LDD  R31,Y+1+1
	SBIW R30,0
	BREQ _0x2000004
	LDI  R26,LOW(85)
	RCALL _w1_write
	LDI  R17,LOW(0)
_0x2000006:
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	LD   R30,X+
	STD  Y+1,R26
	STD  Y+1+1,R27
	MOV  R26,R30
	RCALL _w1_write
	SUBI R17,-LOW(1)
	CPI  R17,8
	BRLO _0x2000006
	RJMP _0x2000008
_0x2000004:
	LDI  R26,LOW(204)
	RCALL _w1_write
_0x2000008:
	LDI  R30,LOW(1)
_0x2080005:
	LDD  R17,Y+0
_0x2080006:
	ADIW R28,3
	RET
; .FEND
_ds1820_read_spd:
; .FSTART _ds1820_read_spd
	ST   -Y,R27
	ST   -Y,R26
	RCALL __SAVELOCR4
	LDD  R26,Y+4
	LDD  R27,Y+4+1
	RCALL _ds1820_select
	CPI  R30,0
	BRNE _0x2000009
	LDI  R30,LOW(0)
	RJMP _0x2080003
_0x2000009:
	LDI  R26,LOW(190)
	RCALL _w1_write
	LDI  R17,LOW(0)
	__POINTWRM 18,19,___ds1820_scratch_pad
_0x200000B:
	PUSH R19
	PUSH R18
	__ADDWRN 18,19,1
	RCALL _w1_read
	POP  R26
	POP  R27
	ST   X,R30
	SUBI R17,-LOW(1)
	CPI  R17,9
	BRLO _0x200000B
	LDI  R30,LOW(___ds1820_scratch_pad)
	LDI  R31,HIGH(___ds1820_scratch_pad)
	ST   -Y,R31
	ST   -Y,R30
	LDI  R26,LOW(9)
	RCALL _w1_dow_crc8
	RCALL __LNEGB1
_0x2080003:
	RCALL __LOADLOCR4
_0x2080004:
	ADIW R28,6
	RET
; .FEND
_ds1820_temperature_10:
; .FSTART _ds1820_temperature_10
	ST   -Y,R27
	ST   -Y,R26
	LD   R26,Y
	LDD  R27,Y+1
	RCALL _ds1820_select
	CPI  R30,0
	BRNE _0x200000D
	LDI  R30,LOW(55537)
	LDI  R31,HIGH(55537)
	RJMP _0x2080002
_0x200000D:
	LDI  R26,LOW(68)
	RCALL _w1_write
	LDI  R26,LOW(550)
	LDI  R27,HIGH(550)
	RCALL _delay_ms
	LD   R26,Y
	LDD  R27,Y+1
	RCALL _ds1820_read_spd
	CPI  R30,0
	BRNE _0x200000E
	LDI  R30,LOW(55537)
	LDI  R31,HIGH(55537)
	RJMP _0x2080002
_0x200000E:
	RCALL _w1_init
	__GETB1HMN ___ds1820_scratch_pad,1
	LDI  R30,LOW(0)
	MOVW R26,R30
	LDS  R30,___ds1820_scratch_pad
	LDI  R31,0
	OR   R30,R26
	OR   R31,R27
	LDI  R26,LOW(5)
	LDI  R27,HIGH(5)
	RCALL __MULW12
_0x2080002:
	ADIW R28,2
	RET
; .FEND
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x01
	.EQU __sm_mask=0x0E
	.EQU __sm_adc_noise_red=0x02
	.EQU __sm_powerdown=0x04
	.EQU __sm_powersave=0x06
	.EQU __sm_standby=0x0C
	.EQU __sm_ext_standby=0x0E
	.SET power_ctrl_reg=smcr
	#endif

	.CSEG
_putchar:
; .FSTART _putchar
	ST   -Y,R26
_0x2020003:
	LDS  R30,192
	ANDI R30,LOW(0x20)
	BREQ _0x2020003
	LD   R30,Y
	STS  198,R30
_0x2080001:
	ADIW R28,1
	RET
; .FEND

	.CSEG

	.CSEG

	.DSEG
_rom_codes:
	.BYTE 0x9
_T1:
	.BYTE 0x4
_T2:
	.BYTE 0x4
_format_converter_32:
	.BYTE 0x4
___ds1820_scratch_pad:
	.BYTE 0x9
_rx_buffer0:
	.BYTE 0x100

	.CSEG
;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:5 WORDS
SUBOPT_0x0:
	LDS  R26,_T1
	LDS  R27,_T1+1
	LDS  R24,_T1+2
	LDS  R25,_T1+3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x1:
	LDD  R26,Y+2
	LDI  R27,0
	SBRC R26,7
	SER  R27
	LDI  R30,LOW(10)
	LDI  R31,HIGH(10)
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 6 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x2:
	CBI  0x5,0
	CBI  0x5,1
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x3:
	CBI  0xB,2
	CBI  0xB,4
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x4:
	SBI  0x5,0
	CBI  0x5,1
	CBI  0xB,2
	SBI  0xB,4
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 6 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x5:
	CBI  0x5,4
	CBI  0x5,5
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x6:
	CBI  0x8,2
	CBI  0x8,3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x7:
	SBI  0x5,4
	CBI  0x5,5
	CBI  0x8,2
	SBI  0x8,3
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x8:
	STS  _format_converter_32,R30
	STS  _format_converter_32+1,R31
	STS  _format_converter_32+2,R22
	STS  _format_converter_32+3,R23
	LDS  R30,_format_converter_32
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x9:
	MOV  R30,R16
	LDI  R31,0
	MOVW R26,R28
	ADIW R26,12
	ADD  R26,R30
	ADC  R27,R31
	RET


	.CSEG
_delay_ms:
	adiw r26,0
	breq __delay_ms1
__delay_ms0:
	__DELAY_USW 0x3E8
	wdr
	sbiw r26,1
	brne __delay_ms0
__delay_ms1:
	ret

	.equ __w1_port=0x08
	.equ __w1_bit=0x00

_w1_init:
	clr  r30
	cbi  __w1_port,__w1_bit
	sbi  __w1_port-1,__w1_bit
	__DELAY_USW 0x1E0
	cbi  __w1_port-1,__w1_bit
	__DELAY_USB 0x13
	sbis __w1_port-2,__w1_bit
	ret
	__DELAY_USB 0x65
	sbis __w1_port-2,__w1_bit
	ldi  r30,1
	__DELAY_USW 0x186
	ret

__w1_read_bit:
	sbi  __w1_port-1,__w1_bit
	__DELAY_USB 0x3
	cbi  __w1_port-1,__w1_bit
	__DELAY_USB 0xF
	clc
	sbic __w1_port-2,__w1_bit
	sec
	ror  r30
	__DELAY_USB 0x6B
	ret

__w1_write_bit:
	clt
	sbi  __w1_port-1,__w1_bit
	__DELAY_USB 0x3
	sbrc r23,0
	cbi  __w1_port-1,__w1_bit
	__DELAY_USB 0x11
	sbic __w1_port-2,__w1_bit
	rjmp __w1_write_bit0
	sbrs r23,0
	rjmp __w1_write_bit1
	ret
__w1_write_bit0:
	sbrs r23,0
	ret
__w1_write_bit1:
	__DELAY_USB 0x64
	cbi  __w1_port-1,__w1_bit
	__DELAY_USB 0x7
	set
	ret

_w1_read:
	ldi  r22,8
	__w1_read0:
	rcall __w1_read_bit
	dec  r22
	brne __w1_read0
	ret

_w1_write:
	mov  r23,r26
	ldi  r22,8
	clr  r30
__w1_write0:
	rcall __w1_write_bit
	brtc __w1_write1
	ror  r23
	dec  r22
	brne __w1_write0
	inc  r30
__w1_write1:
	ret

_w1_search:
	push r20
	push r21
	clr  r1
	clr  r20
__w1_search0:
	mov  r0,r1
	clr  r1
	rcall _w1_init
	tst  r30
	breq __w1_search7
	push r26
	ld   r26,y
	rcall _w1_write
	pop  r26
	ldi  r21,1
__w1_search1:
	cp   r21,r0
	brsh __w1_search6
	rcall __w1_read_bit
	sbrc r30,7
	rjmp __w1_search2
	rcall __w1_read_bit
	sbrc r30,7
	rjmp __w1_search3
	rcall __sel_bit
	and  r24,r25
	brne __w1_search3
	mov  r1,r21
	rjmp __w1_search3
__w1_search2:
	rcall __w1_read_bit
__w1_search3:
	rcall __sel_bit
	and  r24,r25
	ldi  r23,0
	breq __w1_search5
__w1_search4:
	ldi  r23,1
__w1_search5:
	rcall __w1_write_bit
	rjmp __w1_search13
__w1_search6:
	rcall __w1_read_bit
	sbrs r30,7
	rjmp __w1_search9
	rcall __w1_read_bit
	sbrs r30,7
	rjmp __w1_search8
__w1_search7:
	mov  r30,r20
	pop  r21
	pop  r20
	adiw r28,1
	ret
__w1_search8:
	set
	rcall __set_bit
	rjmp __w1_search4
__w1_search9:
	rcall __w1_read_bit
	sbrs r30,7
	rjmp __w1_search10
	rjmp __w1_search11
__w1_search10:
	cp   r21,r0
	breq __w1_search12
	mov  r1,r21
__w1_search11:
	clt
	rcall __set_bit
	clr  r23
	rcall __w1_write_bit
	rjmp __w1_search13
__w1_search12:
	set
	rcall __set_bit
	ldi  r23,1
	rcall __w1_write_bit
__w1_search13:
	inc  r21
	cpi  r21,65
	brlt __w1_search1
	rcall __w1_read_bit
	rol  r30
	rol  r30
	andi r30,1
	adiw r26,8
	st   x,r30
	sbiw r26,8
	inc  r20
	tst  r1
	breq __w1_search7
	ldi  r21,9
__w1_search14:
	ld   r30,x
	adiw r26,9
	st   x,r30
	sbiw r26,8
	dec  r21
	brne __w1_search14
	rjmp __w1_search0

__sel_bit:
	mov  r30,r21
	dec  r30
	mov  r22,r30
	lsr  r30
	lsr  r30
	lsr  r30
	clr  r31
	add  r30,r26
	adc  r31,r27
	ld   r24,z
	ldi  r25,1
	andi r22,7
__sel_bit0:
	breq __sel_bit1
	lsl  r25
	dec  r22
	rjmp __sel_bit0
__sel_bit1:
	ret

__set_bit:
	rcall __sel_bit
	brts __set_bit2
	com  r25
	and  r24,r25
	rjmp __set_bit3
__set_bit2:
	or   r24,r25
__set_bit3:
	st   z,r24
	ret

_w1_dow_crc8:
	clr  r30
	tst  r26
	breq __w1_dow_crc83
	mov  r24,r26
	ldi  r22,0x18
	ld   r26,y
	ldd  r27,y+1
__w1_dow_crc80:
	ldi  r25,8
	ld   r31,x+
__w1_dow_crc81:
	mov  r23,r31
	eor  r23,r30
	ror  r23
	brcc __w1_dow_crc82
	eor  r30,r22
__w1_dow_crc82:
	ror  r30
	lsr  r31
	dec  r25
	brne __w1_dow_crc81
	dec  r24
	brne __w1_dow_crc80
__w1_dow_crc83:
	adiw r28,2
	ret

__ROUND_REPACK:
	TST  R21
	BRPL __REPACK
	CPI  R21,0x80
	BRNE __ROUND_REPACK0
	SBRS R30,0
	RJMP __REPACK
__ROUND_REPACK0:
	ADIW R30,1
	ADC  R22,R25
	ADC  R23,R25
	BRVS __REPACK1

__REPACK:
	LDI  R21,0x80
	EOR  R21,R23
	BRNE __REPACK0
	PUSH R21
	RJMP __ZERORES
__REPACK0:
	CPI  R21,0xFF
	BREQ __REPACK1
	LSL  R22
	LSL  R0
	ROR  R21
	ROR  R22
	MOV  R23,R21
	RET
__REPACK1:
	PUSH R21
	TST  R0
	BRMI __REPACK2
	RJMP __MAXRES
__REPACK2:
	RJMP __MINRES

__UNPACK:
	LDI  R21,0x80
	MOV  R1,R25
	AND  R1,R21
	LSL  R24
	ROL  R25
	EOR  R25,R21
	LSL  R21
	ROR  R24

__UNPACK1:
	LDI  R21,0x80
	MOV  R0,R23
	AND  R0,R21
	LSL  R22
	ROL  R23
	EOR  R23,R21
	LSL  R21
	ROR  R22
	RET

__CFD1U:
	SET
	RJMP __CFD1U0
__CFD1:
	CLT
__CFD1U0:
	PUSH R21
	RCALL __UNPACK1
	CPI  R23,0x80
	BRLO __CFD10
	CPI  R23,0xFF
	BRCC __CFD10
	RJMP __ZERORES
__CFD10:
	LDI  R21,22
	SUB  R21,R23
	BRPL __CFD11
	NEG  R21
	CPI  R21,8
	BRTC __CFD19
	CPI  R21,9
__CFD19:
	BRLO __CFD17
	SER  R30
	SER  R31
	SER  R22
	LDI  R23,0x7F
	BLD  R23,7
	RJMP __CFD15
__CFD17:
	CLR  R23
	TST  R21
	BREQ __CFD15
__CFD18:
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R23
	DEC  R21
	BRNE __CFD18
	RJMP __CFD15
__CFD11:
	CLR  R23
__CFD12:
	CPI  R21,8
	BRLO __CFD13
	MOV  R30,R31
	MOV  R31,R22
	MOV  R22,R23
	SUBI R21,8
	RJMP __CFD12
__CFD13:
	TST  R21
	BREQ __CFD15
__CFD14:
	LSR  R23
	ROR  R22
	ROR  R31
	ROR  R30
	DEC  R21
	BRNE __CFD14
__CFD15:
	TST  R0
	BRPL __CFD16
	RCALL __ANEGD1
__CFD16:
	POP  R21
	RET

__CDF1U:
	SET
	RJMP __CDF1U0
__CDF1:
	CLT
__CDF1U0:
	SBIW R30,0
	SBCI R22,0
	SBCI R23,0
	BREQ __CDF10
	CLR  R0
	BRTS __CDF11
	TST  R23
	BRPL __CDF11
	COM  R0
	RCALL __ANEGD1
__CDF11:
	MOV  R1,R23
	LDI  R23,30
	TST  R1
__CDF12:
	BRMI __CDF13
	DEC  R23
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R1
	RJMP __CDF12
__CDF13:
	MOV  R30,R31
	MOV  R31,R22
	MOV  R22,R1
	PUSH R21
	RCALL __REPACK
	POP  R21
__CDF10:
	RET

__ZERORES:
	CLR  R30
	CLR  R31
	CLR  R22
	CLR  R23
	POP  R21
	RET

__MINRES:
	SER  R30
	SER  R31
	LDI  R22,0x7F
	SER  R23
	POP  R21
	RET

__MAXRES:
	SER  R30
	SER  R31
	LDI  R22,0x7F
	LDI  R23,0x7F
	POP  R21
	RET

__MULF12:
	PUSH R21
	RCALL __UNPACK
	CPI  R23,0x80
	BREQ __ZERORES
	CPI  R25,0x80
	BREQ __ZERORES
	EOR  R0,R1
	SEC
	ADC  R23,R25
	BRVC __MULF124
	BRLT __ZERORES
__MULF125:
	TST  R0
	BRMI __MINRES
	RJMP __MAXRES
__MULF124:
	PUSH R0
	PUSH R17
	PUSH R18
	PUSH R19
	PUSH R20
	CLR  R17
	CLR  R18
	CLR  R25
	MUL  R22,R24
	MOVW R20,R0
	MUL  R24,R31
	MOV  R19,R0
	ADD  R20,R1
	ADC  R21,R25
	MUL  R22,R27
	ADD  R19,R0
	ADC  R20,R1
	ADC  R21,R25
	MUL  R24,R30
	RCALL __MULF126
	MUL  R27,R31
	RCALL __MULF126
	MUL  R22,R26
	RCALL __MULF126
	MUL  R27,R30
	RCALL __MULF127
	MUL  R26,R31
	RCALL __MULF127
	MUL  R26,R30
	ADD  R17,R1
	ADC  R18,R25
	ADC  R19,R25
	ADC  R20,R25
	ADC  R21,R25
	MOV  R30,R19
	MOV  R31,R20
	MOV  R22,R21
	MOV  R21,R18
	POP  R20
	POP  R19
	POP  R18
	POP  R17
	POP  R0
	TST  R22
	BRMI __MULF122
	LSL  R21
	ROL  R30
	ROL  R31
	ROL  R22
	RJMP __MULF123
__MULF122:
	INC  R23
	BRVS __MULF125
__MULF123:
	RCALL __ROUND_REPACK
	POP  R21
	RET

__MULF127:
	ADD  R17,R0
	ADC  R18,R1
	ADC  R19,R25
	RJMP __MULF128
__MULF126:
	ADD  R18,R0
	ADC  R19,R1
__MULF128:
	ADC  R20,R25
	ADC  R21,R25
	RET

__DIVF21:
	PUSH R21
	RCALL __UNPACK
	CPI  R23,0x80
	BRNE __DIVF210
	TST  R1
__DIVF211:
	BRPL __DIVF219
	RJMP __MINRES
__DIVF219:
	RJMP __MAXRES
__DIVF210:
	CPI  R25,0x80
	BRNE __DIVF218
__DIVF217:
	RJMP __ZERORES
__DIVF218:
	EOR  R0,R1
	SEC
	SBC  R25,R23
	BRVC __DIVF216
	BRLT __DIVF217
	TST  R0
	RJMP __DIVF211
__DIVF216:
	MOV  R23,R25
	PUSH R17
	PUSH R18
	PUSH R19
	PUSH R20
	CLR  R1
	CLR  R17
	CLR  R18
	CLR  R19
	CLR  R20
	CLR  R21
	LDI  R25,32
__DIVF212:
	CP   R26,R30
	CPC  R27,R31
	CPC  R24,R22
	CPC  R20,R17
	BRLO __DIVF213
	SUB  R26,R30
	SBC  R27,R31
	SBC  R24,R22
	SBC  R20,R17
	SEC
	RJMP __DIVF214
__DIVF213:
	CLC
__DIVF214:
	ROL  R21
	ROL  R18
	ROL  R19
	ROL  R1
	ROL  R26
	ROL  R27
	ROL  R24
	ROL  R20
	DEC  R25
	BRNE __DIVF212
	MOVW R30,R18
	MOV  R22,R1
	POP  R20
	POP  R19
	POP  R18
	POP  R17
	TST  R22
	BRMI __DIVF215
	LSL  R21
	ROL  R30
	ROL  R31
	ROL  R22
	DEC  R23
	BRVS __DIVF217
__DIVF215:
	RCALL __ROUND_REPACK
	POP  R21
	RET

__ANEGW1:
	NEG  R31
	NEG  R30
	SBCI R31,0
	RET

__ANEGD1:
	COM  R31
	COM  R22
	COM  R23
	NEG  R30
	SBCI R31,-1
	SBCI R22,-1
	SBCI R23,-1
	RET

__CWD1:
	MOV  R22,R31
	ADD  R22,R22
	SBC  R22,R22
	MOV  R23,R22
	RET

__LNEGB1:
	TST  R30
	LDI  R30,1
	BREQ __LNEGB1F
	CLR  R30
__LNEGB1F:
	RET

__MULW12U:
	MUL  R31,R26
	MOV  R31,R0
	MUL  R30,R27
	ADD  R31,R0
	MUL  R30,R26
	MOV  R30,R0
	ADD  R31,R1
	RET

__MULW12:
	RCALL __CHKSIGNW
	RCALL __MULW12U
	BRTC __MULW121
	RCALL __ANEGW1
__MULW121:
	RET

__DIVW21U:
	CLR  R0
	CLR  R1
	LDI  R25,16
__DIVW21U1:
	LSL  R26
	ROL  R27
	ROL  R0
	ROL  R1
	SUB  R0,R30
	SBC  R1,R31
	BRCC __DIVW21U2
	ADD  R0,R30
	ADC  R1,R31
	RJMP __DIVW21U3
__DIVW21U2:
	SBR  R26,1
__DIVW21U3:
	DEC  R25
	BRNE __DIVW21U1
	MOVW R30,R26
	MOVW R26,R0
	RET

__DIVW21:
	RCALL __CHKSIGNW
	RCALL __DIVW21U
	BRTC __DIVW211
	RCALL __ANEGW1
__DIVW211:
	RET

__MODW21:
	CLT
	SBRS R27,7
	RJMP __MODW211
	COM  R26
	COM  R27
	ADIW R26,1
	SET
__MODW211:
	SBRC R31,7
	RCALL __ANEGW1
	RCALL __DIVW21U
	MOVW R30,R26
	BRTC __MODW212
	RCALL __ANEGW1
__MODW212:
	RET

__CHKSIGNW:
	CLT
	SBRS R31,7
	RJMP __CHKSW1
	RCALL __ANEGW1
	SET
__CHKSW1:
	SBRS R27,7
	RJMP __CHKSW2
	COM  R26
	COM  R27
	ADIW R26,1
	BLD  R0,0
	INC  R0
	BST  R0,0
__CHKSW2:
	RET

__PUTDP1:
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	RET

__PUTPARD2:
	ST   -Y,R25
	ST   -Y,R24
	ST   -Y,R27
	ST   -Y,R26
	RET

__SAVELOCR6:
	ST   -Y,R21
__SAVELOCR5:
	ST   -Y,R20
__SAVELOCR4:
	ST   -Y,R19
__SAVELOCR3:
	ST   -Y,R18
__SAVELOCR2:
	ST   -Y,R17
	ST   -Y,R16
	RET

__LOADLOCR6:
	LDD  R21,Y+5
__LOADLOCR5:
	LDD  R20,Y+4
__LOADLOCR4:
	LDD  R19,Y+3
__LOADLOCR3:
	LDD  R18,Y+2
__LOADLOCR2:
	LDD  R17,Y+1
	LD   R16,Y
	RET

;END OF CODE MARKER
__END_OF_CODE:
