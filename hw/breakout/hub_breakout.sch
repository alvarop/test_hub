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
LIBS:ostur
LIBS:test_hub
LIBS:hub_breakout-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB2514B U2
U 1 1 58D1BF23
P 3600 3100
F 0 "U2" H 3750 3350 60  0000 R CNN
F 1 "USB2514B" H 3750 3250 60  0000 R CNN
F 2 "footprints:SQFN36" H 3600 3100 60  0001 C CNN
F 3 "" H 3600 3100 60  0000 C CNN
F 4 "USB2514B/M2" H 3600 3100 60  0001 C CNN "MPN"
	1    3600 3100
	1    0    0    -1  
$EndComp
$Comp
L MIC550X U1
U 1 1 58D1BFBB
P 3050 1450
F 0 "U1" H 3150 1400 60  0000 C CNN
F 1 "MIC550X" H 3050 1850 60  0000 C CNN
F 2 "footprints:SOT-23(M5)" H 3050 1450 60  0001 C CNN
F 3 "" H 3050 1450 60  0000 C CNN
F 4 "MIC5504-3.3YM5-TR" H 3050 1450 60  0001 C CNN "MPN"
	1    3050 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 58D1C332
P 4900 5400
F 0 "#PWR01" H 4900 5150 50  0001 C CNN
F 1 "GND" H 4900 5250 50  0000 C CNN
F 2 "" H 4900 5400 50  0001 C CNN
F 3 "" H 4900 5400 50  0001 C CNN
	1    4900 5400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR02
U 1 1 58D1C456
P 5200 2300
F 0 "#PWR02" H 5200 2150 50  0001 C CNN
F 1 "+3.3V" H 5200 2440 50  0000 C CNN
F 2 "" H 5200 2300 50  0001 C CNN
F 3 "" H 5200 2300 50  0001 C CNN
	1    5200 2300
	1    0    0    -1  
$EndComp
NoConn ~ 3450 1300
$Comp
L GND #PWR03
U 1 1 58D1C572
P 3050 1750
F 0 "#PWR03" H 3050 1500 50  0001 C CNN
F 1 "GND" H 3050 1600 50  0000 C CNN
F 2 "" H 3050 1750 50  0001 C CNN
F 3 "" H 3050 1750 50  0001 C CNN
	1    3050 1750
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 58D1C63B
P 2400 1400
F 0 "C3" H 2425 1500 50  0000 L CNN
F 1 "1uF" H 2425 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2438 1250 50  0001 C CNN
F 3 "" H 2400 1400 50  0001 C CNN
F 4 "GRM188R61C105KA93D" H 2400 1400 60  0001 C CNN "MPN"
	1    2400 1400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR04
U 1 1 58D1C76D
P 3750 1150
F 0 "#PWR04" H 3750 1000 50  0001 C CNN
F 1 "+3.3V" H 3750 1290 50  0000 C CNN
F 2 "" H 3750 1150 50  0001 C CNN
F 3 "" H 3750 1150 50  0001 C CNN
	1    3750 1150
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR05
U 1 1 58D1C79E
P 2200 1150
F 0 "#PWR05" H 2200 1000 50  0001 C CNN
F 1 "+5V" H 2200 1290 50  0000 C CNN
F 2 "" H 2200 1150 50  0001 C CNN
F 3 "" H 2200 1150 50  0001 C CNN
	1    2200 1150
	1    0    0    -1  
$EndComp
$Comp
L Crystal Y1
U 1 1 58D1C826
P 2450 4450
F 0 "Y1" H 2450 4600 50  0000 C CNN
F 1 "24MHz" H 2450 4300 50  0000 C CNN
F 2 "footprints:HCM49" H 2450 4450 50  0001 C CNN
F 3 "" H 2450 4450 50  0001 C CNN
F 4 "HCM4924000000ABJT" H 2450 4450 60  0001 C CNN "MPN"
	1    2450 4450
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 58D1CAE9
P 2050 4450
F 0 "R1" V 2130 4450 50  0000 C CNN
F 1 "1M" V 2050 4450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1980 4450 50  0001 C CNN
F 3 "" H 2050 4450 50  0001 C CNN
F 4 "ERJ-3GEYJ105V" V 2050 4450 60  0001 C CNN "MPN"
	1    2050 4450
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 58D1CBCB
P 1750 4300
F 0 "C1" H 1775 4400 50  0000 L CNN
F 1 "18pF" H 1775 4200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1788 4150 50  0001 C CNN
F 3 "" H 1750 4300 50  0001 C CNN
F 4 "CL10C180JB8NNNC" H 1750 4300 60  0001 C CNN "MPN"
	1    1750 4300
	0    1    1    0   
$EndComp
$Comp
L C C2
U 1 1 58D1CD24
P 1750 4600
F 0 "C2" H 1775 4700 50  0000 L CNN
F 1 "18pF" H 1775 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1788 4450 50  0001 C CNN
F 3 "" H 1750 4600 50  0001 C CNN
F 4 "CL10C180JB8NNNC" H 1750 4600 60  0001 C CNN "MPN"
	1    1750 4600
	0    1    1    0   
$EndComp
$Comp
L GND #PWR06
U 1 1 58D1CEDB
P 1350 4450
F 0 "#PWR06" H 1350 4200 50  0001 C CNN
F 1 "GND" H 1350 4300 50  0000 C CNN
F 2 "" H 1350 4450 50  0001 C CNN
F 3 "" H 1350 4450 50  0001 C CNN
	1    1350 4450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR07
U 1 1 58D1D515
P 2500 3600
F 0 "#PWR07" H 2500 3350 50  0001 C CNN
F 1 "GND" H 2500 3450 50  0000 C CNN
F 2 "" H 2500 3600 50  0001 C CNN
F 3 "" H 2500 3600 50  0001 C CNN
	1    2500 3600
	1    0    0    -1  
$EndComp
NoConn ~ 6500 3900
$Comp
L R R4
U 1 1 58D1D9EE
P 3300 4850
F 0 "R4" V 3380 4850 50  0000 C CNN
F 1 "12k" V 3300 4850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3230 4850 50  0001 C CNN
F 3 "" H 3300 4850 50  0001 C CNN
F 4 "ERJ-3EKF1202V" V 3300 4850 60  0001 C CNN "MPN"
	1    3300 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 58D1DB4D
P 2950 5250
F 0 "#PWR08" H 2950 5000 50  0001 C CNN
F 1 "GND" H 2950 5100 50  0000 C CNN
F 2 "" H 2950 5250 50  0001 C CNN
F 3 "" H 2950 5250 50  0001 C CNN
	1    2950 5250
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG J1
U 1 1 58D1DEB1
P 1350 5950
F 0 "J1" H 1150 6400 50  0000 L CNN
F 1 "USB_OTG" H 1150 6300 50  0000 L CNN
F 2 "alvarop:USB_MICRO_B" H 1500 5900 50  0001 C CNN
F 3 "" H 1500 5900 50  0001 C CNN
F 4 "10118193-0001LF" H 1350 5950 60  0001 C CNN "MPN"
	1    1350 5950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 58D1DF99
P 1350 6600
F 0 "#PWR09" H 1350 6350 50  0001 C CNN
F 1 "GND" H 1350 6450 50  0000 C CNN
F 2 "" H 1350 6600 50  0001 C CNN
F 3 "" H 1350 6600 50  0001 C CNN
	1    1350 6600
	1    0    0    -1  
$EndComp
NoConn ~ 1650 6150
$Comp
L +5V #PWR010
U 1 1 58D1E2D2
P 1850 5650
F 0 "#PWR010" H 1850 5500 50  0001 C CNN
F 1 "+5V" H 1850 5790 50  0000 C CNN
F 2 "" H 1850 5650 50  0001 C CNN
F 3 "" H 1850 5650 50  0001 C CNN
	1    1850 5650
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR011
U 1 1 58D1E447
P 1250 6400
F 0 "#PWR011" H 1250 6200 50  0001 C CNN
F 1 "GNDPWR" H 1250 6270 50  0000 C CNN
F 2 "" H 1250 6350 50  0001 C CNN
F 3 "" H 1250 6350 50  0001 C CNN
	1    1250 6400
	1    0    0    -1  
$EndComp
$Comp
L Ferrite_Bead_Small FB2
U 1 1 58D1E7B8
P 2100 6100
F 0 "FB2" V 2200 6150 50  0000 L CNN
F 1 "Ferrite" V 2250 6000 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 2030 6100 50  0001 C CNN
F 3 "" H 2100 6100 50  0001 C CNN
F 4 "BLM18EG221SN1D" V 2100 6100 60  0001 C CNN "MPN"
	1    2100 6100
	0    1    1    0   
$EndComp
$Comp
L Ferrite_Bead_Small FB1
U 1 1 58D1E8DF
P 2100 5900
F 0 "FB1" V 2050 5750 50  0000 L CNN
F 1 "Ferrite" V 1950 5800 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 2030 5900 50  0001 C CNN
F 3 "" H 2100 5900 50  0001 C CNN
F 4 "BLM18EG221SN1D" V 2100 5900 60  0001 C CNN "MPN"
	1    2100 5900
	0    1    1    0   
$EndComp
Text Label 2400 6100 0    60   ~ 0
USB_UP_N
Text Label 2400 5900 0    60   ~ 0
USB_UP_P
Text Label 3400 4300 2    60   ~ 0
USB_UP_P
Text Label 3400 4200 2    60   ~ 0
USB_UP_N
Text Label 6500 4700 0    60   ~ 0
USB1_N
Text Label 6500 4600 0    60   ~ 0
USB1_P
Text Label 6500 4100 0    60   ~ 0
USB4_N
Text Label 6500 4000 0    60   ~ 0
USB4_P
Text Label 6500 4500 0    60   ~ 0
USB2_N
Text Label 6500 4400 0    60   ~ 0
USB2_P
Text Label 6500 4300 0    60   ~ 0
USB3_N
Text Label 6500 4200 0    60   ~ 0
USB3_P
Text Label 6500 3800 0    60   ~ 0
OCS_N1
Text Label 6500 3700 0    60   ~ 0
OCS_N2
Text Label 6500 3600 0    60   ~ 0
OCS_N3
Text Label 6500 3500 0    60   ~ 0
OCS_N4
Text Label 3400 3100 2    60   ~ 0
PRTPWR1
Text Label 3400 3300 2    60   ~ 0
PRTPWR2
Text Label 3400 3400 2    60   ~ 0
PRTPWR3
Text Label 3400 3500 2    60   ~ 0
PRTPWR4
Text Label 3400 3600 2    60   ~ 0
SDA
Text Label 3400 3700 2    60   ~ 0
SCL
Text Label 3400 3800 2    60   ~ 0
CFG_SEL1
Text Label 3400 3900 2    60   ~ 0
RESET_N
Text Label 3400 4000 2    60   ~ 0
VBUS_DET
Text Label 3400 4100 2    60   ~ 0
SUSP_IND
$Comp
L R R2
U 1 1 58D1FD10
P 2750 6800
F 0 "R2" V 2830 6800 50  0000 C CNN
F 1 "100k" V 2750 6800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2680 6800 50  0001 C CNN
F 3 "" H 2750 6800 50  0001 C CNN
F 4 "ERJ-3GEYJ104V" V 2750 6800 60  0001 C CNN "MPN"
	1    2750 6800
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR012
U 1 1 58D201CA
P 2750 6550
F 0 "#PWR012" H 2750 6400 50  0001 C CNN
F 1 "+5V" H 2750 6690 50  0000 C CNN
F 2 "" H 2750 6550 50  0001 C CNN
F 3 "" H 2750 6550 50  0001 C CNN
	1    2750 6550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 58D20208
P 2750 7450
F 0 "#PWR013" H 2750 7200 50  0001 C CNN
F 1 "GND" H 2750 7300 50  0000 C CNN
F 2 "" H 2750 7450 50  0001 C CNN
F 3 "" H 2750 7450 50  0001 C CNN
	1    2750 7450
	1    0    0    -1  
$EndComp
Text Label 2950 7000 0    60   ~ 0
VBUS_DET
$Comp
L CONN_01X10 J2
U 1 1 58D20851
P 7000 1400
F 0 "J2" H 7000 1950 50  0000 C CNN
F 1 "CONN_01X10" V 7100 1400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x10_Pitch2.54mm" H 7000 1400 50  0001 C CNN
F 3 "" H 7000 1400 50  0001 C CNN
	1    7000 1400
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X10 J3
U 1 1 58D209C4
P 7000 2550
F 0 "J3" H 7000 3100 50  0000 C CNN
F 1 "CONN_01X10" V 7100 2550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x10_Pitch2.54mm" H 7000 2550 50  0001 C CNN
F 3 "" H 7000 2550 50  0001 C CNN
	1    7000 2550
	-1   0    0    1   
$EndComp
Text Label 7400 1650 0    60   ~ 0
OCS_N1
Text Label 7400 1450 0    60   ~ 0
OCS_N2
Text Label 7400 1250 0    60   ~ 0
OCS_N3
Text Label 7400 1050 0    60   ~ 0
OCS_N4
Text Label 7400 1150 0    60   ~ 0
PRTPWR4
Text Label 7400 1350 0    60   ~ 0
PRTPWR3
Text Label 7400 1550 0    60   ~ 0
PRTPWR2
Text Label 7400 1750 0    60   ~ 0
PRTPWR1
Text Label 7400 2700 0    60   ~ 0
SDA
Text Label 7400 2600 0    60   ~ 0
SCL
Text Label 7400 2500 0    60   ~ 0
CFG_SEL1
Text Label 7400 2400 0    60   ~ 0
RESET_N
Text Label 7400 2300 0    60   ~ 0
VBUS_DET
Text Label 7400 2200 0    60   ~ 0
SUSP_IND
$Comp
L GND #PWR014
U 1 1 58D22231
P 7500 850
F 0 "#PWR014" H 7500 600 50  0001 C CNN
F 1 "GND" H 7500 700 50  0000 C CNN
F 2 "" H 7500 850 50  0001 C CNN
F 3 "" H 7500 850 50  0001 C CNN
	1    7500 850 
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR015
U 1 1 58D222FE
P 7550 2000
F 0 "#PWR015" H 7550 1750 50  0001 C CNN
F 1 "GND" H 7550 1850 50  0000 C CNN
F 2 "" H 7550 2000 50  0001 C CNN
F 3 "" H 7550 2000 50  0001 C CNN
	1    7550 2000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR016
U 1 1 58D22451
P 7600 2800
F 0 "#PWR016" H 7600 2550 50  0001 C CNN
F 1 "GND" H 7600 2650 50  0000 C CNN
F 2 "" H 7600 2800 50  0001 C CNN
F 3 "" H 7600 2800 50  0001 C CNN
	1    7600 2800
	0    -1   -1   0   
$EndComp
$Comp
L +3.3V #PWR017
U 1 1 58D22605
P 7500 1850
F 0 "#PWR017" H 7500 1700 50  0001 C CNN
F 1 "+3.3V" H 7500 1990 50  0000 C CNN
F 2 "" H 7500 1850 50  0001 C CNN
F 3 "" H 7500 1850 50  0001 C CNN
	1    7500 1850
	0    1    1    0   
$EndComp
$Comp
L +3.3V #PWR018
U 1 1 58D226E9
P 7600 3000
F 0 "#PWR018" H 7600 2850 50  0001 C CNN
F 1 "+3.3V" H 7600 3140 50  0000 C CNN
F 2 "" H 7600 3000 50  0001 C CNN
F 3 "" H 7600 3000 50  0001 C CNN
	1    7600 3000
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR019
U 1 1 58D2289E
P 7450 2900
F 0 "#PWR019" H 7450 2750 50  0001 C CNN
F 1 "+5V" H 7450 3040 50  0000 C CNN
F 2 "" H 7450 2900 50  0001 C CNN
F 3 "" H 7450 2900 50  0001 C CNN
	1    7450 2900
	0    1    1    0   
$EndComp
$Comp
L USB_A J8
U 1 1 58D22BAB
P 9900 2650
F 0 "J8" H 9700 3100 50  0000 L CNN
F 1 "USB_A" H 9700 3000 50  0000 L CNN
F 2 "footprints:292303-1" H 10050 2600 50  0001 C CNN
F 3 "" H 10050 2600 50  0001 C CNN
F 4 "292303-1" H 9900 2650 60  0001 C CNN "MPN"
	1    9900 2650
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR020
U 1 1 58D230F8
P 9900 2200
F 0 "#PWR020" H 9900 1950 50  0001 C CNN
F 1 "GND" H 9900 2050 50  0000 C CNN
F 2 "" H 9900 2200 50  0001 C CNN
F 3 "" H 9900 2200 50  0001 C CNN
	1    9900 2200
	-1   0    0    1   
$EndComp
$Comp
L GNDPWR #PWR021
U 1 1 58D2313F
P 10000 2050
F 0 "#PWR021" H 10000 1850 50  0001 C CNN
F 1 "GNDPWR" H 10000 1920 50  0000 C CNN
F 2 "" H 10000 2000 50  0001 C CNN
F 3 "" H 10000 2000 50  0001 C CNN
	1    10000 2050
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 J6
U 1 1 58D23321
P 9150 3050
F 0 "J6" H 9150 3200 50  0000 C CNN
F 1 "CONN_01X02" V 9250 3050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 9150 3050 50  0001 C CNN
F 3 "" H 9150 3050 50  0001 C CNN
	1    9150 3050
	0    1    1    0   
$EndComp
Text Label 9100 2700 2    60   ~ 0
USB4_P
Text Label 9100 2500 2    60   ~ 0
USB4_N
$Comp
L +5V #PWR022
U 1 1 58D235DE
P 8800 2850
F 0 "#PWR022" H 8800 2700 50  0001 C CNN
F 1 "+5V" H 8800 2990 50  0000 C CNN
F 2 "" H 8800 2850 50  0001 C CNN
F 3 "" H 8800 2850 50  0001 C CNN
	1    8800 2850
	0    -1   -1   0   
$EndComp
$Comp
L USB_A J9
U 1 1 58D2380E
P 9950 4400
F 0 "J9" H 9750 4850 50  0000 L CNN
F 1 "USB_A" H 9750 4750 50  0000 L CNN
F 2 "footprints:292303-1" H 10100 4350 50  0001 C CNN
F 3 "" H 10100 4350 50  0001 C CNN
F 4 "292303-1" H 9950 4400 60  0001 C CNN "MPN"
	1    9950 4400
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR023
U 1 1 58D23817
P 9950 3950
F 0 "#PWR023" H 9950 3700 50  0001 C CNN
F 1 "GND" H 9950 3800 50  0000 C CNN
F 2 "" H 9950 3950 50  0001 C CNN
F 3 "" H 9950 3950 50  0001 C CNN
	1    9950 3950
	-1   0    0    1   
$EndComp
$Comp
L GNDPWR #PWR024
U 1 1 58D2381D
P 10050 3800
F 0 "#PWR024" H 10050 3600 50  0001 C CNN
F 1 "GNDPWR" H 10050 3670 50  0000 C CNN
F 2 "" H 10050 3750 50  0001 C CNN
F 3 "" H 10050 3750 50  0001 C CNN
	1    10050 3800
	-1   0    0    1   
$EndComp
$Comp
L CONN_01X02 J7
U 1 1 58D23825
P 9200 4800
F 0 "J7" H 9200 4950 50  0000 C CNN
F 1 "CONN_01X02" V 9300 4800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 9200 4800 50  0001 C CNN
F 3 "" H 9200 4800 50  0001 C CNN
	1    9200 4800
	0    1    1    0   
$EndComp
Text Label 9100 4450 2    60   ~ 0
USB1_P
Text Label 9100 4250 2    60   ~ 0
USB1_N
$Comp
L +5V #PWR025
U 1 1 58D2382E
P 8850 4600
F 0 "#PWR025" H 8850 4450 50  0001 C CNN
F 1 "+5V" H 8850 4740 50  0000 C CNN
F 2 "" H 8850 4600 50  0001 C CNN
F 3 "" H 8850 4600 50  0001 C CNN
	1    8850 4600
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 J4
U 1 1 58D23AA3
P 8050 3850
F 0 "J4" H 8050 4000 50  0000 C CNN
F 1 "CONN_01X02" V 8150 3850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8050 3850 50  0001 C CNN
F 3 "" H 8050 3850 50  0001 C CNN
	1    8050 3850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 J5
U 1 1 58D23B4D
P 8050 4450
F 0 "J5" H 8050 4600 50  0000 C CNN
F 1 "CONN_01X02" V 8150 4450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 8050 4450 50  0001 C CNN
F 3 "" H 8050 4450 50  0001 C CNN
	1    8050 4450
	1    0    0    -1  
$EndComp
Text Label 7850 4500 2    60   ~ 0
USB2_P
Text Label 7850 4400 2    60   ~ 0
USB2_N
Text Label 7850 3900 2    60   ~ 0
USB3_P
Text Label 7850 3800 2    60   ~ 0
USB3_N
$Comp
L C C8
U 1 1 58D24041
P 4250 1700
F 0 "C8" H 4275 1800 50  0000 L CNN
F 1 "0.1uF" H 4275 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4288 1550 50  0001 C CNN
F 3 "" H 4250 1700 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 4250 1700 60  0001 C CNN "MPN"
	1    4250 1700
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 58D2418A
P 4750 1700
F 0 "C10" H 4775 1800 50  0000 L CNN
F 1 "4.7uF" H 4775 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4788 1550 50  0001 C CNN
F 3 "" H 4750 1700 50  0001 C CNN
F 4 "CL10B475KQ8NQNC" H 4750 1700 60  0001 C CNN "MPN"
	1    4750 1700
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR026
U 1 1 58D250F2
P 5000 1350
F 0 "#PWR026" H 5000 1200 50  0001 C CNN
F 1 "+3.3V" H 5000 1490 50  0000 C CNN
F 2 "" H 5000 1350 50  0001 C CNN
F 3 "" H 5000 1350 50  0001 C CNN
	1    5000 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 58D253E5
P 5000 2000
F 0 "#PWR027" H 5000 1750 50  0001 C CNN
F 1 "GND" H 5000 1850 50  0000 C CNN
F 2 "" H 5000 2000 50  0001 C CNN
F 3 "" H 5000 2000 50  0001 C CNN
	1    5000 2000
	1    0    0    -1  
$EndComp
$Comp
L Ferrite_Bead_Small FB5
U 1 1 58D25EDA
P 9450 4250
F 0 "FB5" V 9400 4100 50  0000 L CNN
F 1 "Ferrite" V 9300 4150 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 9380 4250 50  0001 C CNN
F 3 "" H 9450 4250 50  0001 C CNN
F 4 "BLM18EG221SN1D" V 9450 4250 60  0001 C CNN "MPN"
	1    9450 4250
	0    1    1    0   
$EndComp
$Comp
L Ferrite_Bead_Small FB6
U 1 1 58D265BA
P 9450 4450
F 0 "FB6" V 9550 4500 50  0000 L CNN
F 1 "Ferrite" V 9550 4200 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 9380 4450 50  0001 C CNN
F 3 "" H 9450 4450 50  0001 C CNN
F 4 "BLM18EG221SN1D" V 9450 4450 60  0001 C CNN "MPN"
	1    9450 4450
	0    1    1    0   
$EndComp
$Comp
L Ferrite_Bead_Small FB3
U 1 1 58D2721C
P 9450 2500
F 0 "FB3" V 9400 2350 50  0000 L CNN
F 1 "Ferrite" V 9300 2400 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 9380 2500 50  0001 C CNN
F 3 "" H 9450 2500 50  0001 C CNN
F 4 "BLM18EG221SN1D" V 9450 2500 60  0001 C CNN "MPN"
	1    9450 2500
	0    1    1    0   
$EndComp
$Comp
L Ferrite_Bead_Small FB4
U 1 1 58D272C8
P 9450 2700
F 0 "FB4" V 9550 2750 50  0000 L CNN
F 1 "Ferrite" V 9550 2450 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 9380 2700 50  0001 C CNN
F 3 "" H 9450 2700 50  0001 C CNN
F 4 "BLM18EG221SN1D" V 9450 2700 60  0001 C CNN "MPN"
	1    9450 2700
	0    1    1    0   
$EndComp
$Comp
L GND #PWR028
U 1 1 58D283EE
P 5000 6550
F 0 "#PWR028" H 5000 6300 50  0001 C CNN
F 1 "GND" H 5000 6400 50  0000 C CNN
F 2 "" H 5000 6550 50  0001 C CNN
F 3 "" H 5000 6550 50  0001 C CNN
	1    5000 6550
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR029
U 1 1 58D28462
P 4500 6550
F 0 "#PWR029" H 4500 6350 50  0001 C CNN
F 1 "GNDPWR" H 4500 6420 50  0000 C CNN
F 2 "" H 4500 6500 50  0001 C CNN
F 3 "" H 4500 6500 50  0001 C CNN
	1    4500 6550
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 58D28A79
P 7600 5350
F 0 "R7" V 7680 5350 50  0000 C CNN
F 1 "10k" V 7600 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7530 5350 50  0001 C CNN
F 3 "" H 7600 5350 50  0001 C CNN
F 4 "RC0603FR-0710KL" V 7600 5350 60  0001 C CNN "MPN"
	1    7600 5350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR030
U 1 1 58D28BB3
P 7600 5100
F 0 "#PWR030" H 7600 4950 50  0001 C CNN
F 1 "+3.3V" H 7600 5240 50  0000 C CNN
F 2 "" H 7600 5100 50  0001 C CNN
F 3 "" H 7600 5100 50  0001 C CNN
	1    7600 5100
	1    0    0    -1  
$EndComp
Text Label 7750 5650 0    60   ~ 0
RESET_N
Text Label 7750 5800 0    60   ~ 0
SDA
Text Label 7750 5950 0    60   ~ 0
SCL
$Comp
L C C7
U 1 1 58D2A1AA
P 3650 1400
F 0 "C7" H 3675 1500 50  0000 L CNN
F 1 "1uF" H 3675 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3688 1250 50  0001 C CNN
F 3 "" H 3650 1400 50  0001 C CNN
F 4 "GRM188R61C105KA93D" H 3650 1400 60  0001 C CNN "MPN"
	1    3650 1400
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 58D2A470
P 2450 7000
F 0 "C4" H 2475 7100 50  0000 L CNN
F 1 "1uF" H 2475 6900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2488 6850 50  0001 C CNN
F 3 "" H 2450 7000 50  0001 C CNN
F 4 "GRM188R61C105KA93D" H 2450 7000 60  0001 C CNN "MPN"
	1    2450 7000
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 58D2A871
P 4500 1700
F 0 "C9" H 4525 1800 50  0000 L CNN
F 1 "0.1uF" H 4525 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4538 1550 50  0001 C CNN
F 3 "" H 4500 1700 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 4500 1700 60  0001 C CNN "MPN"
	1    4500 1700
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 58D2A8F5
P 5000 1700
F 0 "C12" H 5025 1800 50  0000 L CNN
F 1 "0.1uF" H 5025 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5038 1550 50  0001 C CNN
F 3 "" H 5000 1700 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 5000 1700 60  0001 C CNN "MPN"
	1    5000 1700
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 58D2A98A
P 5250 1700
F 0 "C13" H 5275 1800 50  0000 L CNN
F 1 "0.1uF" H 5275 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5288 1550 50  0001 C CNN
F 3 "" H 5250 1700 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 5250 1700 60  0001 C CNN "MPN"
	1    5250 1700
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 58D2AA18
P 5500 1700
F 0 "C14" H 5525 1800 50  0000 L CNN
F 1 "0.1uF" H 5525 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5538 1550 50  0001 C CNN
F 3 "" H 5500 1700 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 5500 1700 60  0001 C CNN "MPN"
	1    5500 1700
	1    0    0    -1  
$EndComp
$Comp
L C C15
U 1 1 58D2AAA9
P 5750 1700
F 0 "C15" H 5775 1800 50  0000 L CNN
F 1 "0.1uF" H 5775 1600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5788 1550 50  0001 C CNN
F 3 "" H 5750 1700 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 5750 1700 60  0001 C CNN "MPN"
	1    5750 1700
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 58D2ADB8
P 4750 6400
F 0 "C11" H 4775 6500 50  0000 L CNN
F 1 "0.1uF" H 4775 6300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4788 6250 50  0001 C CNN
F 3 "" H 4750 6400 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 4750 6400 60  0001 C CNN "MPN"
	1    4750 6400
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 58D2C8A5
P 2750 7200
F 0 "R3" V 2830 7200 50  0000 C CNN
F 1 "100k" V 2750 7200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 2680 7200 50  0001 C CNN
F 3 "" H 2750 7200 50  0001 C CNN
F 4 "ERJ-3GEYJ104V" V 2750 7200 60  0001 C CNN "MPN"
	1    2750 7200
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 58D2D713
P 2500 3400
F 0 "C5" H 2525 3500 50  0000 L CNN
F 1 "0.1uF" H 2525 3300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2538 3250 50  0001 C CNN
F 3 "" H 2500 3400 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 2500 3400 60  0001 C CNN "MPN"
	1    2500 3400
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 58D2D800
P 2950 4900
F 0 "C6" H 2975 5000 50  0000 L CNN
F 1 "0.1uF" H 2975 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2988 4750 50  0001 C CNN
F 3 "" H 2950 4900 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 2950 4900 60  0001 C CNN "MPN"
	1    2950 4900
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 58D2DF1A
P 7400 5350
F 0 "R6" V 7480 5350 50  0000 C CNN
F 1 "10k" V 7400 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7330 5350 50  0001 C CNN
F 3 "" H 7400 5350 50  0001 C CNN
F 4 "RC0603FR-0710KL" V 7400 5350 60  0001 C CNN "MPN"
	1    7400 5350
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 58D2DFD4
P 7200 5350
F 0 "R5" V 7280 5350 50  0000 C CNN
F 1 "10k" V 7200 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 7130 5350 50  0001 C CNN
F 3 "" H 7200 5350 50  0001 C CNN
F 4 "RC0603FR-0710KL" V 7200 5350 60  0001 C CNN "MPN"
	1    7200 5350
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 58D3157A
P 9450 5200
F 0 "C17" H 9475 5300 50  0000 L CNN
F 1 "0.1uF" H 9475 5100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9488 5050 50  0001 C CNN
F 3 "" H 9450 5200 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 9450 5200 60  0001 C CNN "MPN"
	1    9450 5200
	1    0    0    -1  
$EndComp
$Comp
L CP C19
U 1 1 58D31CB0
P 9750 5200
F 0 "C19" H 9775 5300 50  0000 L CNN
F 1 "100uF" H 9775 5100 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_6.3x5.7" H 9788 5050 50  0001 C CNN
F 3 "" H 9750 5200 50  0001 C CNN
F 4 "UWR1A101MCL1GB" H 9750 5200 60  0001 C CNN "MPN"
	1    9750 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR031
U 1 1 58D320F6
P 9600 5550
F 0 "#PWR031" H 9600 5300 50  0001 C CNN
F 1 "GND" H 9600 5400 50  0000 C CNN
F 2 "" H 9600 5550 50  0001 C CNN
F 3 "" H 9600 5550 50  0001 C CNN
	1    9600 5550
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 58D3231B
P 9400 3450
F 0 "C16" H 9425 3550 50  0000 L CNN
F 1 "0.1uF" H 9425 3350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 9438 3300 50  0001 C CNN
F 3 "" H 9400 3450 50  0001 C CNN
F 4 "CC0603ZRY5V9BB104" H 9400 3450 60  0001 C CNN "MPN"
	1    9400 3450
	1    0    0    -1  
$EndComp
$Comp
L CP C18
U 1 1 58D32328
P 9700 3450
F 0 "C18" H 9725 3550 50  0000 L CNN
F 1 "100uF" H 9725 3350 50  0000 L CNN
F 2 "Capacitors_SMD:CP_Elec_6.3x5.7" H 9738 3300 50  0001 C CNN
F 3 "" H 9700 3450 50  0001 C CNN
F 4 "UWR1A101MCL1GB" H 9700 3450 60  0001 C CNN "MPN"
	1    9700 3450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR032
U 1 1 58D32332
P 9550 3800
F 0 "#PWR032" H 9550 3550 50  0001 C CNN
F 1 "GND" H 9550 3650 50  0000 C CNN
F 2 "" H 9550 3800 50  0001 C CNN
F 3 "" H 9550 3800 50  0001 C CNN
	1    9550 3800
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG033
U 1 1 58D3342C
P 4500 6400
F 0 "#FLG033" H 4500 6475 50  0001 C CNN
F 1 "PWR_FLAG" H 4500 6550 50  0000 C CNN
F 2 "" H 4500 6400 50  0001 C CNN
F 3 "" H 4500 6400 50  0001 C CNN
	1    4500 6400
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG034
U 1 1 58D3359E
P 5000 6400
F 0 "#FLG034" H 5000 6475 50  0001 C CNN
F 1 "PWR_FLAG" H 5000 6550 50  0000 C CNN
F 2 "" H 5000 6400 50  0001 C CNN
F 3 "" H 5000 6400 50  0001 C CNN
	1    5000 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3100 3400 3100
Wire Wire Line
	2200 3200 3600 3200
Wire Wire Line
	3600 3300 3400 3300
Wire Wire Line
	3600 3400 3400 3400
Wire Wire Line
	3600 3500 3400 3500
Wire Wire Line
	3600 3600 3400 3600
Wire Wire Line
	3600 3700 3400 3700
Wire Wire Line
	3600 3800 3400 3800
Wire Wire Line
	3600 3900 3400 3900
Wire Wire Line
	3600 4000 3400 4000
Wire Wire Line
	3600 4100 3400 4100
Wire Wire Line
	3600 4200 3400 4200
Wire Wire Line
	3600 4300 3400 4300
Wire Wire Line
	2850 4400 3600 4400
Wire Wire Line
	2850 4500 3600 4500
Wire Wire Line
	2950 4600 3600 4600
Wire Wire Line
	3300 4700 3600 4700
Wire Wire Line
	6300 3500 6500 3500
Wire Wire Line
	6300 3600 6500 3600
Wire Wire Line
	6300 3700 6500 3700
Wire Wire Line
	6300 3800 6500 3800
Wire Wire Line
	6300 3900 6500 3900
Wire Wire Line
	6300 4000 6500 4000
Wire Wire Line
	6300 4100 6500 4100
Wire Wire Line
	6300 4200 6500 4200
Wire Wire Line
	6300 4300 6500 4300
Wire Wire Line
	6300 4400 6500 4400
Wire Wire Line
	6300 4500 6500 4500
Wire Wire Line
	6300 4600 6500 4600
Wire Wire Line
	6300 4700 6500 4700
Wire Wire Line
	4900 5400 4900 5300
Wire Wire Line
	4900 2400 5400 2400
Connection ~ 5300 2400
Connection ~ 5200 2400
Connection ~ 5100 2400
Connection ~ 5000 2400
Wire Wire Line
	5200 2400 5200 2300
Wire Wire Line
	2200 1200 2650 1200
Wire Wire Line
	2550 1300 2650 1300
Wire Wire Line
	2550 1100 2550 1300
Connection ~ 2550 1200
Wire Wire Line
	2200 1200 2200 1150
Wire Wire Line
	3450 1200 3750 1200
Wire Wire Line
	3750 1200 3750 1150
Wire Wire Line
	3050 1650 3050 1750
Wire Wire Line
	2400 1250 2400 1200
Connection ~ 2400 1200
Wire Wire Line
	3650 1250 3650 1200
Connection ~ 3650 1200
Wire Wire Line
	3650 1700 3650 1550
Wire Wire Line
	2400 1700 3650 1700
Connection ~ 3050 1700
Wire Wire Line
	2400 1700 2400 1550
Wire Wire Line
	2850 4300 2850 4400
Wire Wire Line
	1900 4300 2850 4300
Connection ~ 2450 4300
Wire Wire Line
	2850 4600 2850 4500
Wire Wire Line
	1900 4600 2850 4600
Connection ~ 2450 4600
Connection ~ 2050 4300
Connection ~ 2050 4600
Wire Wire Line
	1600 4300 1450 4300
Wire Wire Line
	1450 4300 1450 4600
Wire Wire Line
	1450 4600 1600 4600
Wire Wire Line
	1450 4450 1350 4450
Connection ~ 1450 4450
Wire Wire Line
	2500 3200 2500 3250
Wire Wire Line
	2500 3600 2500 3550
Wire Wire Line
	2950 4600 2950 4750
Wire Wire Line
	3300 5150 3300 5000
Wire Wire Line
	2700 5150 3300 5150
Wire Wire Line
	2950 5050 2950 5250
Connection ~ 2950 5150
Wire Wire Line
	1350 6350 1350 6600
Wire Wire Line
	1650 6050 1850 6050
Wire Wire Line
	1650 5950 1850 5950
Wire Wire Line
	1650 5750 1850 5750
Wire Wire Line
	1850 5750 1850 5650
Wire Wire Line
	1250 6350 1250 6400
Wire Wire Line
	1850 5950 1850 5900
Wire Wire Line
	1850 5900 2000 5900
Wire Wire Line
	1850 6050 1850 6100
Wire Wire Line
	1850 6100 2000 6100
Wire Wire Line
	2200 5900 2400 5900
Wire Wire Line
	2200 6100 2400 6100
Wire Wire Line
	2750 6950 2750 7050
Wire Wire Line
	2450 6850 2450 6600
Wire Wire Line
	2450 6600 2750 6600
Wire Wire Line
	2750 6550 2750 6650
Wire Wire Line
	2450 7150 2450 7400
Wire Wire Line
	2450 7400 2750 7400
Wire Wire Line
	2750 7350 2750 7450
Wire Wire Line
	2750 7000 2950 7000
Connection ~ 2750 7000
Connection ~ 2750 6600
Connection ~ 2750 7400
Wire Wire Line
	7200 950  7400 950 
Wire Wire Line
	7200 1050 7400 1050
Wire Wire Line
	7200 1150 7400 1150
Wire Wire Line
	7200 1250 7400 1250
Wire Wire Line
	7200 1350 7400 1350
Wire Wire Line
	7200 1450 7400 1450
Wire Wire Line
	7200 1550 7400 1550
Wire Wire Line
	7200 1650 7400 1650
Wire Wire Line
	7200 1750 7400 1750
Wire Wire Line
	7200 1850 7500 1850
Wire Wire Line
	7200 2100 7450 2100
Wire Wire Line
	7200 2200 7400 2200
Wire Wire Line
	7200 3000 7600 3000
Wire Wire Line
	7200 2900 7450 2900
Wire Wire Line
	7200 2800 7600 2800
Wire Wire Line
	7200 2700 7400 2700
Wire Wire Line
	7200 2600 7400 2600
Wire Wire Line
	7200 2500 7400 2500
Wire Wire Line
	7200 2400 7400 2400
Wire Wire Line
	7200 2300 7400 2300
Wire Wire Line
	7500 850  7400 850 
Wire Wire Line
	7400 850  7400 950 
Wire Wire Line
	7550 2000 7450 2000
Wire Wire Line
	7450 2000 7450 2100
Wire Wire Line
	9200 2850 9600 2850
Wire Wire Line
	9350 2700 9100 2700
Wire Wire Line
	9100 2500 9350 2500
Wire Wire Line
	10000 2050 10000 2250
Wire Wire Line
	9900 2200 9900 2250
Wire Wire Line
	8800 2850 9100 2850
Wire Wire Line
	9250 4600 9650 4600
Wire Wire Line
	9350 4450 9100 4450
Wire Wire Line
	9100 4250 9350 4250
Wire Wire Line
	10050 3800 10050 4000
Wire Wire Line
	9950 3950 9950 4000
Wire Wire Line
	8850 4600 9150 4600
Wire Wire Line
	5500 1900 5500 1850
Wire Wire Line
	4250 1900 5750 1900
Wire Wire Line
	4250 1900 4250 1850
Wire Wire Line
	4500 1850 4500 1900
Connection ~ 4500 1900
Wire Wire Line
	4750 1850 4750 1900
Connection ~ 4750 1900
Wire Wire Line
	5000 1850 5000 2000
Connection ~ 5000 1900
Wire Wire Line
	5250 1850 5250 1900
Connection ~ 5250 1900
Wire Wire Line
	5500 1500 5500 1550
Wire Wire Line
	4250 1500 5750 1500
Wire Wire Line
	4250 1500 4250 1550
Wire Wire Line
	4500 1550 4500 1500
Connection ~ 4500 1500
Wire Wire Line
	4750 1550 4750 1500
Connection ~ 4750 1500
Wire Wire Line
	5000 1350 5000 1550
Connection ~ 5000 1500
Wire Wire Line
	5250 1500 5250 1550
Connection ~ 5250 1500
Wire Wire Line
	9650 4300 9550 4300
Wire Wire Line
	9550 4300 9550 4250
Wire Wire Line
	9550 4450 9550 4400
Wire Wire Line
	9550 4400 9650 4400
Wire Wire Line
	9550 2700 9550 2650
Wire Wire Line
	9550 2650 9600 2650
Wire Wire Line
	9600 2550 9550 2550
Wire Wire Line
	9550 2550 9550 2500
Wire Wire Line
	4500 6550 4500 6400
Wire Wire Line
	4500 6400 4600 6400
Wire Wire Line
	4900 6400 5000 6400
Wire Wire Line
	5000 6400 5000 6550
Wire Wire Line
	7600 5100 7600 5200
Wire Wire Line
	7600 5500 7600 5650
Wire Wire Line
	7600 5650 7750 5650
Wire Wire Line
	7400 5500 7400 5800
Wire Wire Line
	7400 5800 7750 5800
Wire Wire Line
	7200 5500 7200 5950
Wire Wire Line
	7200 5950 7750 5950
Wire Wire Line
	7200 5150 7200 5200
Connection ~ 7600 5150
Wire Wire Line
	7400 5150 7400 5200
Connection ~ 7400 5150
Wire Wire Line
	5750 1500 5750 1550
Connection ~ 5500 1500
Wire Wire Line
	5750 1900 5750 1850
Connection ~ 5500 1900
Wire Wire Line
	9750 5400 9750 5350
Wire Wire Line
	9450 5400 9750 5400
Wire Wire Line
	9450 5400 9450 5350
Wire Wire Line
	9450 5050 9450 5000
Wire Wire Line
	9450 5000 9750 5000
Wire Wire Line
	9750 5000 9750 5050
Wire Wire Line
	9600 4600 9600 5000
Connection ~ 9600 4600
Connection ~ 9600 5000
Wire Wire Line
	9600 5400 9600 5550
Connection ~ 9600 5400
Wire Wire Line
	9700 3650 9700 3600
Wire Wire Line
	9400 3650 9700 3650
Wire Wire Line
	9400 3650 9400 3600
Wire Wire Line
	9400 3300 9400 3250
Wire Wire Line
	9400 3250 9700 3250
Wire Wire Line
	9700 3250 9700 3300
Wire Wire Line
	9550 2850 9550 3250
Connection ~ 9550 3250
Wire Wire Line
	9550 3650 9550 3800
Connection ~ 9550 3650
Connection ~ 9550 2850
Connection ~ 4500 6400
Connection ~ 5000 6400
$Comp
L PWR_FLAG #FLG035
U 1 1 58D34194
P 2550 1100
F 0 "#FLG035" H 2550 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 1250 50  0000 C CNN
F 2 "" H 2550 1100 50  0001 C CNN
F 3 "" H 2550 1100 50  0001 C CNN
	1    2550 1100
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG036
U 1 1 58D34A5B
P 9600 4900
F 0 "#FLG036" H 9600 4975 50  0001 C CNN
F 1 "PWR_FLAG" H 9600 5050 50  0000 C CNN
F 2 "" H 9600 4900 50  0001 C CNN
F 3 "" H 9600 4900 50  0001 C CNN
	1    9600 4900
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG037
U 1 1 58D34B42
P 9550 3100
F 0 "#FLG037" H 9550 3175 50  0001 C CNN
F 1 "PWR_FLAG" H 9550 3250 50  0000 C CNN
F 2 "" H 9550 3100 50  0001 C CNN
F 3 "" H 9550 3100 50  0001 C CNN
	1    9550 3100
	0    1    1    0   
$EndComp
Connection ~ 9550 3100
Connection ~ 9600 4900
$Comp
L C C20
U 1 1 58D365A7
P 2200 3400
F 0 "C20" H 2225 3500 50  0000 L CNN
F 1 "1uF" H 2225 3300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2238 3250 50  0001 C CNN
F 3 "" H 2200 3400 50  0001 C CNN
F 4 "GRM188R61C105KA93D" H 2200 3400 60  0001 C CNN "MPN"
	1    2200 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 3250 2200 3200
Connection ~ 2500 3200
Wire Wire Line
	2500 3550 2200 3550
Connection ~ 2500 3550
$Comp
L C C21
U 1 1 58D369F9
P 2700 4900
F 0 "C21" H 2725 5000 50  0000 L CNN
F 1 "1uF" H 2725 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2738 4750 50  0001 C CNN
F 3 "" H 2700 4900 50  0001 C CNN
F 4 "GRM188R61C105KA93D" H 2700 4900 60  0001 C CNN "MPN"
	1    2700 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4750 2700 4700
Wire Wire Line
	2700 4700 2950 4700
Connection ~ 2950 4700
Wire Wire Line
	2700 5150 2700 5050
$Comp
L R R9
U 1 1 58D41393
P 7000 5350
F 0 "R9" V 7080 5350 50  0000 C CNN
F 1 "10k" V 7000 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6930 5350 50  0001 C CNN
F 3 "" H 7000 5350 50  0001 C CNN
F 4 "RC0603FR-0710KL" V 7000 5350 60  0001 C CNN "MPN"
	1    7000 5350
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 58D41469
P 6800 5350
F 0 "R8" V 6880 5350 50  0000 C CNN
F 1 "10k" V 6800 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 6730 5350 50  0001 C CNN
F 3 "" H 6800 5350 50  0001 C CNN
F 4 "RC0603FR-0710KL" V 6800 5350 60  0001 C CNN "MPN"
	1    6800 5350
	1    0    0    -1  
$EndComp
Connection ~ 7200 5150
Wire Wire Line
	6800 5150 6800 5200
Wire Wire Line
	7000 5200 7000 5150
Connection ~ 7000 5150
Wire Wire Line
	7000 5500 7000 6100
Wire Wire Line
	7000 6100 7750 6100
Wire Wire Line
	6800 5500 6800 6250
Wire Wire Line
	6800 6250 7750 6250
Text Label 7750 6100 0    60   ~ 0
CFG_SEL1
Text Label 7750 6250 0    60   ~ 0
SUSP_IND
Wire Wire Line
	6800 5150 7600 5150
$EndSCHEMATC
