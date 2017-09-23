EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "CV Sequencer"
Date "2017-9-20"
Rev "0"
Comp "sound in jars"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATMEGA48A-PU U2
U 1 1 59C556AB
P 2200 3700
F 0 "U2" H 1450 4950 50  0000 L BNN
F 1 "ATMEGA48A-PU" H 2600 2300 50  0000 L BNN
F 2 "DIL28" H 2200 3700 50  0001 C CIN
F 3 "" H 2200 3700 50  0001 C CNN
	1    2200 3700
	1    0    0    -1  
$EndComp
$Comp
L MCP4922-E/P U3
U 1 1 59C55710
P 5000 2800
F 0 "U3" H 4600 3200 50  0000 L CNN
F 1 "MCP4922-E/P" H 5150 3200 50  0000 L CNN
F 2 "" H 5000 2800 50  0001 C CIN
F 3 "" H 5000 2800 50  0001 C CNN
	1    5000 2800
	1    0    0    -1  
$EndComp
$Comp
L 7805 U1
U 1 1 59C558C6
P 3050 1350
F 0 "U1" H 3200 1154 50  0000 C CNN
F 1 "7805" H 3050 1550 50  0000 C CNN
F 2 "" H 3050 1350 50  0001 C CNN
F 3 "" H 3050 1350 50  0001 C CNN
	1    3050 1350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR10
U 1 1 59C55952
P 3450 1300
F 0 "#PWR10" H 3450 1150 50  0001 C CNN
F 1 "+5V" H 3450 1440 50  0000 C CNN
F 2 "" H 3450 1300 50  0001 C CNN
F 3 "" H 3450 1300 50  0001 C CNN
	1    3450 1300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR7
U 1 1 59C55978
P 3050 1600
F 0 "#PWR7" H 3050 1350 50  0001 C CNN
F 1 "GND" H 3050 1450 50  0000 C CNN
F 2 "" H 3050 1600 50  0001 C CNN
F 3 "" H 3050 1600 50  0001 C CNN
	1    3050 1600
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 J1
U 1 1 59C55995
P 1400 1500
F 0 "J1" H 1400 1800 50  0000 C CNN
F 1 "CONN_02X05" H 1400 1200 50  0000 C CNN
F 2 "" H 1400 300 50  0001 C CNN
F 3 "" H 1400 300 50  0001 C CNN
	1    1400 1500
	1    0    0    -1  
$EndComp
$Comp
L -12VA #PWR5
U 1 1 59C55B91
P 1900 1700
F 0 "#PWR5" H 1900 1550 50  0001 C CNN
F 1 "-12VA" H 1900 1840 50  0000 C CNN
F 2 "" H 1900 1700 50  0001 C CNN
F 3 "" H 1900 1700 50  0001 C CNN
	1    1900 1700
	0    1    1    0   
$EndComp
$Comp
L +12V #PWR3
U 1 1 59C55BB1
P 1900 1300
F 0 "#PWR3" H 1900 1150 50  0001 C CNN
F 1 "+12V" H 1900 1440 50  0000 C CNN
F 2 "" H 1900 1300 50  0001 C CNN
F 3 "" H 1900 1300 50  0001 C CNN
	1    1900 1300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR4
U 1 1 59C55C40
P 1900 1500
F 0 "#PWR4" H 1900 1250 50  0001 C CNN
F 1 "GND" H 1900 1350 50  0000 C CNN
F 2 "" H 1900 1500 50  0001 C CNN
F 3 "" H 1900 1500 50  0001 C CNN
	1    1900 1500
	0    -1   -1   0   
$EndComp
$Comp
L +12V #PWR6
U 1 1 59C55DBB
P 2650 1300
F 0 "#PWR6" H 2650 1150 50  0001 C CNN
F 1 "+12V" H 2650 1440 50  0000 C CNN
F 2 "" H 2650 1300 50  0001 C CNN
F 3 "" H 2650 1300 50  0001 C CNN
	1    2650 1300
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR1
U 1 1 59C57C38
P 900 2450
F 0 "#PWR1" H 900 2300 50  0001 C CNN
F 1 "+5V" H 900 2590 50  0000 C CNN
F 2 "" H 900 2450 50  0001 C CNN
F 3 "" H 900 2450 50  0001 C CNN
	1    900  2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 59C57CF5
P 950 4900
F 0 "#PWR2" H 950 4650 50  0001 C CNN
F 1 "GND" H 950 4750 50  0000 C CNN
F 2 "" H 950 4900 50  0001 C CNN
F 3 "" H 950 4900 50  0001 C CNN
	1    950  4900
	1    0    0    -1  
$EndComp
$Comp
L 4051 U4
U 1 1 59C57D9F
P 6850 5550
F 0 "U4" H 6950 5550 50  0000 C CNN
F 1 "4051" H 6950 5350 50  0000 C CNN
F 2 "" H 6850 5550 60  0001 C CNN
F 3 "" H 6850 5550 60  0001 C CNN
	1    6850 5550
	-1   0    0    1   
$EndComp
$Comp
L 4051 U5
U 1 1 59C58001
P 6850 4000
F 0 "U5" H 6950 4000 50  0000 C CNN
F 1 "4051" H 6950 3800 50  0000 C CNN
F 2 "" H 6850 4000 60  0001 C CNN
F 3 "" H 6850 4000 60  0001 C CNN
	1    6850 4000
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR11
U 1 1 59C581BA
P 4300 3350
F 0 "#PWR11" H 4300 3100 50  0001 C CNN
F 1 "GND" H 4300 3200 50  0000 C CNN
F 2 "" H 4300 3350 50  0001 C CNN
F 3 "" H 4300 3350 50  0001 C CNN
	1    4300 3350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR12
U 1 1 59C5826F
P 4900 2100
F 0 "#PWR12" H 4900 1950 50  0001 C CNN
F 1 "+5V" H 4900 2240 50  0000 C CNN
F 2 "" H 4900 2100 50  0001 C CNN
F 3 "" H 4900 2100 50  0001 C CNN
	1    4900 2100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR15
U 1 1 59C582D7
P 5450 3300
F 0 "#PWR15" H 5450 3150 50  0001 C CNN
F 1 "+5V" H 5450 3440 50  0000 C CNN
F 2 "" H 5450 3300 50  0001 C CNN
F 3 "" H 5450 3300 50  0001 C CNN
	1    5450 3300
	1    0    0    -1  
$EndComp
$Comp
L POT RV1
U 1 1 59C586AE
P 8050 4100
F 0 "RV1" V 7875 4100 50  0000 C CNN
F 1 "POT" V 7950 4100 50  0000 C CNN
F 2 "" H 8050 4100 50  0001 C CNN
F 3 "" H 8050 4100 50  0001 C CNN
	1    8050 4100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR8
U 1 1 59C58730
P 9350 3700
F 0 "#PWR8" H 9350 3550 50  0001 C CNN
F 1 "+5V" H 9350 3840 50  0000 C CNN
F 2 "" H 9350 3700 50  0001 C CNN
F 3 "" H 9350 3700 50  0001 C CNN
	1    9350 3700
	0    1    1    0   
$EndComp
$Comp
L GND #PWR9
U 1 1 59C58759
P 9050 3700
F 0 "#PWR9" H 9050 3450 50  0001 C CNN
F 1 "GND" H 9050 3550 50  0000 C CNN
F 2 "" H 9050 3700 50  0001 C CNN
F 3 "" H 9050 3700 50  0001 C CNN
	1    9050 3700
	0    1    1    0   
$EndComp
Text GLabel 7900 5150 2    60   Input ~ 0
MUX_SEL_A
Text GLabel 7900 5050 2    60   Input ~ 0
MUX_SEL_B
Text GLabel 7900 4950 2    60   Input ~ 0
MUX_SEL_C
Text GLabel 7950 3600 2    60   Input ~ 0
MUX_SEL_A
Text GLabel 7950 3500 2    60   Input ~ 0
MUX_SEL_B
Text GLabel 7950 3400 2    60   Input ~ 0
MUX_SEL_C
Text GLabel 3350 4700 2    60   Input ~ 0
MUX_SEL_A
Text GLabel 3350 4800 2    60   Input ~ 0
MUX_SEL_B
Text GLabel 3350 4900 2    60   Input ~ 0
MUX_SEL_C
$Comp
L GND #PWR18
U 1 1 59C5BCAC
P 7650 3700
F 0 "#PWR18" H 7650 3450 50  0001 C CNN
F 1 "GND" H 7650 3550 50  0000 C CNN
F 2 "" H 7650 3700 50  0001 C CNN
F 3 "" H 7650 3700 50  0001 C CNN
	1    7650 3700
	1    0    0    -1  
$EndComp
$Comp
L AVR-ISP-6 CON1
U 1 1 59C5D823
P 4800 1450
F 0 "CON1" H 4695 1690 50  0000 C CNN
F 1 "AVR-ISP-6" H 4535 1220 50  0000 L BNN
F 2 "AVR-ISP-6" V 4280 1490 50  0001 C CNN
F 3 "" H 4775 1450 50  0001 C CNN
	1    4800 1450
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR13
U 1 1 59C5DBED
P 5200 1200
F 0 "#PWR13" H 5200 1050 50  0001 C CNN
F 1 "+5V" H 5200 1340 50  0000 C CNN
F 2 "" H 5200 1200 50  0001 C CNN
F 3 "" H 5200 1200 50  0001 C CNN
	1    5200 1200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR14
U 1 1 59C5DC7A
P 5200 1550
F 0 "#PWR14" H 5200 1300 50  0001 C CNN
F 1 "GND" H 5200 1400 50  0000 C CNN
F 2 "" H 5200 1550 50  0001 C CNN
F 3 "" H 5200 1550 50  0001 C CNN
	1    5200 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 59C5DDC0
P 6150 3700
F 0 "#PWR16" H 6150 3450 50  0001 C CNN
F 1 "GND" H 6150 3550 50  0000 C CNN
F 2 "" H 6150 3700 50  0001 C CNN
F 3 "" H 6150 3700 50  0001 C CNN
	1    6150 3700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR17
U 1 1 59C5DE3F
P 6150 5250
F 0 "#PWR17" H 6150 5000 50  0001 C CNN
F 1 "GND" H 6150 5100 50  0000 C CNN
F 2 "" H 6150 5250 50  0001 C CNN
F 3 "" H 6150 5250 50  0001 C CNN
	1    6150 5250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR19
U 1 1 59C5DF0F
P 7650 5250
F 0 "#PWR19" H 7650 5000 50  0001 C CNN
F 1 "GND" H 7650 5100 50  0000 C CNN
F 2 "" H 7650 5250 50  0001 C CNN
F 3 "" H 7650 5250 50  0001 C CNN
	1    7650 5250
	1    0    0    -1  
$EndComp
Text GLabel 5900 4600 0    60   Input ~ 0
MUX1
Text GLabel 3350 3450 2    60   Input ~ 0
MUX1
Text GLabel 3350 3550 2    60   Input ~ 0
MUX2
Text GLabel 5900 6150 0    60   Input ~ 0
MUX2
Wire Wire Line
	7550 3600 7950 3600
Wire Wire Line
	7550 3500 7950 3500
Wire Wire Line
	7550 3400 7950 3400
Wire Wire Line
	7550 5150 7900 5150
Wire Wire Line
	7550 5050 7900 5050
Wire Wire Line
	7550 4950 7900 4950
Connection ~ 1700 1500
Wire Wire Line
	1700 1650 1700 1500
Wire Wire Line
	1100 1650 1700 1650
Wire Wire Line
	1100 1500 1100 1650
Wire Wire Line
	1150 1500 1100 1500
Connection ~ 1750 1700
Wire Wire Line
	1750 1900 1750 1700
Wire Wire Line
	1150 1900 1750 1900
Wire Wire Line
	1150 1700 1150 1900
Connection ~ 1750 1300
Wire Wire Line
	1750 1150 1750 1300
Wire Wire Line
	1150 1150 1750 1150
Wire Wire Line
	1150 1300 1150 1150
Connection ~ 1150 1500
Wire Wire Line
	1150 1400 1150 1600
Wire Wire Line
	5450 3400 5450 3300
Wire Wire Line
	5100 3400 5450 3400
Wire Wire Line
	5100 3300 5100 3400
Connection ~ 4900 2200
Wire Wire Line
	5100 2200 5100 2300
Wire Wire Line
	4900 2200 5100 2200
Wire Wire Line
	4900 2100 4900 2300
Connection ~ 4300 3300
Wire Wire Line
	4300 3300 4900 3300
Connection ~ 4300 3000
Wire Wire Line
	4400 3000 4300 3000
Wire Wire Line
	4300 2900 4300 3350
Wire Wire Line
	4400 2900 4300 2900
Connection ~ 900  2600
Wire Wire Line
	900  2900 1300 2900
Wire Wire Line
	950  4900 1300 4900
Wire Wire Line
	900  2600 1300 2600
Wire Wire Line
	900  2450 900  3200
Connection ~ 1650 1500
Wire Wire Line
	1650 1500 1950 1500
Wire Wire Line
	1650 1400 1650 1600
Wire Wire Line
	1650 1700 1900 1700
Wire Wire Line
	1650 1300 1900 1300
Wire Wire Line
	7550 3700 7650 3700
Wire Wire Line
	900  3200 1300 3200
Connection ~ 900  2900
Wire Wire Line
	4900 1350 5200 1350
Wire Wire Line
	5200 1350 5200 1200
Wire Wire Line
	4900 1550 5200 1550
Wire Wire Line
	6150 3500 6150 3700
Wire Wire Line
	6150 5050 6150 5250
Wire Wire Line
	7550 5250 7650 5250
Wire Wire Line
	1300 4800 1200 4800
Wire Wire Line
	1200 4800 1200 4900
Connection ~ 1200 4900
Wire Wire Line
	5900 4600 6150 4600
Wire Wire Line
	3350 3450 3200 3450
Wire Wire Line
	3200 3550 3350 3550
Wire Wire Line
	5900 6150 6150 6150
$Comp
L POT RV?
U 1 1 59C5ED74
P 8500 4100
F 0 "RV?" V 8325 4100 50  0000 C CNN
F 1 "POT" V 8400 4100 50  0000 C CNN
F 2 "" H 8500 4100 50  0001 C CNN
F 3 "" H 8500 4100 50  0001 C CNN
	1    8500 4100
	1    0    0    -1  
$EndComp
$Comp
L POT RV?
U 1 1 59C5EE02
P 8950 4100
F 0 "RV?" V 8775 4100 50  0000 C CNN
F 1 "POT" V 8850 4100 50  0000 C CNN
F 2 "" H 8950 4100 50  0001 C CNN
F 3 "" H 8950 4100 50  0001 C CNN
	1    8950 4100
	1    0    0    -1  
$EndComp
$Comp
L POT RV?
U 1 1 59C5EE08
P 9400 4100
F 0 "RV?" V 9225 4100 50  0000 C CNN
F 1 "POT" V 9300 4100 50  0000 C CNN
F 2 "" H 9400 4100 50  0001 C CNN
F 3 "" H 9400 4100 50  0001 C CNN
	1    9400 4100
	1    0    0    -1  
$EndComp
Text GLabel 3350 4050 2    60   Input ~ 0
RST
Wire Wire Line
	3350 4050 3200 4050
Text GLabel 3350 4400 2    60   Input ~ 0
MCU_TRIGIN
Wire Wire Line
	3200 4700 3350 4700
Wire Wire Line
	3200 4800 3350 4800
Wire Wire Line
	3200 4900 3350 4900
Text GLabel 3350 2900 2    60   Input ~ 0
MOSI
Wire Wire Line
	3350 2900 3200 2900
Text GLabel 3350 3000 2    60   Input ~ 0
MISO
Wire Wire Line
	3350 3000 3200 3000
Wire Wire Line
	3350 4400 3200 4400
Text GLabel 3350 3100 2    60   Input ~ 0
SPI_CK
Wire Wire Line
	3350 3100 3200 3100
$EndSCHEMATC