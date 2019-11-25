EESchema Schematic File Version 4
LIBS:controlBoard-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 3550 950  1000 3050
U 5DD3C100
F0 "power" 50
F1 "power.sch" 50
F2 "BATT_VDD" I L 3550 3800 50 
F3 "GND" U L 3550 3900 50 
F4 "3V3" O L 3550 3200 50 
F5 "INT" O L 3550 2700 50 
F6 "USB_TO_BUS_EN" I R 4550 1200 50 
F7 "BATT_TO_BUS_EN" I R 4550 1100 50 
F8 "USB_C_VDD" I R 4550 1300 50 
F9 "GND" I R 4550 1400 50 
$EndSheet
$Sheet
S 650  2600 750  1400
U 5DFADF1E
F0 "battery" 50
F1 "battery.sch" 50
F2 "CHG_EN" I R 1400 2800 50 
F3 "CHG_AC_OK" O R 1400 3300 50 
F4 "CHG_OK" O R 1400 3400 50 
F5 "INT" I R 1400 2700 50 
F6 "BAT_VDD" O R 1400 3800 50 
F7 "BATT_VOLTAGE" O R 1400 2900 50 
F8 "PWR_SHUTDOWN" I R 1400 3000 50 
F9 "PWR_START" I R 1400 3100 50 
F10 "3V3" I R 1400 3200 50 
F11 "GND" U R 1400 3900 50 
$EndSheet
$Sheet
S 6450 950  800  3100
U 5E2C3773
F0 "bios" 50
F1 "bios.sch" 50
F2 "GND" U L 6450 3900 50 
F3 "3V3" I L 6450 3200 50 
F4 "USB_C_VDD" I L 6450 1300 50 
F5 "INT" I L 6450 2700 50 
F6 "CHG_EN" O L 6450 2800 50 
F7 "CHG_OK" I L 6450 3400 50 
F8 "CHG_AC_OK" I L 6450 3300 50 
F9 "BATT_VOLTAGE" I L 6450 2900 50 
F10 "PWR_START" O L 6450 3100 50 
F11 "PWR_SHUTDOWN" O L 6450 3000 50 
F12 "BATT_TO_BUS_EN" I L 6450 1100 50 
F13 "USB_TO_BUS_EN" I L 6450 1200 50 
F14 "SW_RIGHT" I R 7250 2500 50 
F15 "SW_LEFT" I R 7250 2300 50 
F16 "SW_MID" I R 7250 2400 50 
F17 "PWR_SHUTDOWN" I R 7250 2200 50 
F18 "PWR_START" I R 7250 2100 50 
F19 "GND" I R 7250 1900 50 
F20 "3V3" I R 7250 2000 50 
F21 "GND" I L 6450 1400 50 
F22 "USB_C_VDD" I R 7250 4000 50 
$EndSheet
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 5DDD52B8
P 2300 3000
AR Path="/5E8D0C73/5DDD52B8" Ref="J?"  Part="1" 
AR Path="/5DDD52B8" Ref="J11"  Part="1" 
F 0 "J11" H 2380 2992 50  0000 L CNN
F 1 "BATT_3" H 2380 2901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 2300 3000 50  0001 C CNN
F 3 "~" H 2300 3000 50  0001 C CNN
	1    2300 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3400 1400 3400
Wire Wire Line
	1400 3300 2100 3300
Wire Wire Line
	2100 3200 2050 3200
Wire Wire Line
	1400 3100 2100 3100
Wire Wire Line
	1400 2900 2100 2900
Wire Wire Line
	2100 2800 1400 2800
Wire Wire Line
	1400 2700 1600 2700
Wire Wire Line
	2100 3800 2050 3800
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J12
U 1 1 5DDEFEDA
P 2300 4500
F 0 "J12" H 2350 4717 50  0000 C CNN
F 1 "BATT_2" H 2350 4626 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 2300 4500 50  0001 C CNN
F 3 "~" H 2300 4500 50  0001 C CNN
	1    2300 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 3800 2600 3500
Wire Wire Line
	2600 3500 2050 3500
Wire Wire Line
	2050 3500 2050 3800
Wire Wire Line
	2100 4500 2100 4600
Wire Wire Line
	2600 4500 2600 4600
Wire Wire Line
	2050 3800 1400 3800
Connection ~ 2050 3800
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 5DE2B3FC
P 3000 3000
AR Path="/5E8D0C73/5DE2B3FC" Ref="J?"  Part="1" 
AR Path="/5DE2B3FC" Ref="J15"  Part="1" 
F 0 "J15" H 3080 2992 50  0000 L CNN
F 1 "PWR_3" H 3080 2901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 3000 3000 50  0001 C CNN
F 3 "~" H 3000 3000 50  0001 C CNN
	1    3000 3000
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J13
U 1 1 5DE2C490
P 3050 3800
F 0 "J13" H 3100 4017 50  0000 C CNN
F 1 "PWR_1" H 3100 3926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 3050 3800 50  0001 C CNN
F 3 "~" H 3050 3800 50  0001 C CNN
	1    3050 3800
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J14
U 1 1 5DE2C913
P 3050 4500
F 0 "J14" H 3100 4717 50  0000 C CNN
F 1 "BATT_2" H 3100 4626 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 3050 4500 50  0001 C CNN
F 3 "~" H 3050 4500 50  0001 C CNN
	1    3050 4500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J10
U 1 1 5DDECEF3
P 2300 3800
F 0 "J10" H 2350 4017 50  0000 C CNN
F 1 "BATT_1" H 2350 3926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 2300 3800 50  0001 C CNN
F 3 "~" H 2300 3800 50  0001 C CNN
	1    2300 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3900 2050 3900
Wire Wire Line
	2050 3900 2050 4100
Wire Wire Line
	2050 4100 2600 4100
Wire Wire Line
	2600 4100 2600 3900
Connection ~ 2050 3900
Wire Wire Line
	2050 3900 2100 3900
Wire Wire Line
	2600 4100 2600 4500
Connection ~ 2600 4100
Connection ~ 2600 4500
Wire Wire Line
	2050 4100 2050 4500
Wire Wire Line
	2050 4500 2100 4500
Connection ~ 2050 4100
Connection ~ 2100 4500
Wire Wire Line
	2850 3800 2850 3500
Wire Wire Line
	2850 3500 3350 3500
Wire Wire Line
	3350 3500 3350 3800
Wire Wire Line
	2850 3900 2850 4100
Wire Wire Line
	2850 4100 3350 4100
Wire Wire Line
	3350 4100 3350 3900
Connection ~ 3350 4100
Wire Wire Line
	3350 4500 3350 4600
Connection ~ 3350 4500
Wire Wire Line
	2850 4600 2850 4500
Wire Wire Line
	2850 4500 2850 4100
Connection ~ 2850 4500
Connection ~ 2850 4100
Wire Wire Line
	3550 3800 3350 3800
Connection ~ 3350 3800
Wire Wire Line
	3350 3900 3550 3900
Connection ~ 3350 3900
Wire Wire Line
	5350 2700 6200 2700
NoConn ~ 3200 2800
NoConn ~ 3200 2900
NoConn ~ 3200 3400
NoConn ~ 3200 3300
NoConn ~ 3200 3000
NoConn ~ 3200 3100
Wire Wire Line
	5350 3300 6450 3300
Wire Wire Line
	6450 3400 5350 3400
Wire Wire Line
	5350 3200 5750 3200
Wire Wire Line
	5350 2800 6450 2800
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J17
U 1 1 5DE2DB0D
P 5150 3800
F 0 "J17" H 5200 4017 50  0000 C CNN
F 1 "CTL_1" H 5200 3926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 5150 3800 50  0001 C CNN
F 3 "~" H 5150 3800 50  0001 C CNN
	1    5150 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 3800 4950 3500
Wire Wire Line
	4950 3500 5450 3500
Wire Wire Line
	5450 3500 5450 3800
Wire Wire Line
	4950 3900 4950 4050
Wire Wire Line
	4950 4050 5450 4050
Wire Wire Line
	5450 4050 5450 3900
Wire Wire Line
	5450 3900 5700 3900
Connection ~ 5450 3900
Connection ~ 5450 3800
Connection ~ 6300 2700
Wire Wire Line
	6300 2700 6450 2700
Wire Wire Line
	6200 3800 5700 3800
Connection ~ 6100 3900
Wire Wire Line
	6100 3900 6450 3900
$Comp
L Connector_Generic:Conn_02x02_Odd_Even J18
U 1 1 5DE421FA
P 5150 4500
F 0 "J18" H 5200 4717 50  0000 C CNN
F 1 "CTL_2" H 5200 4626 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical" H 5150 4500 50  0001 C CNN
F 3 "~" H 5150 4500 50  0001 C CNN
	1    5150 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 4600 4950 4500
Wire Wire Line
	4950 4500 4950 4050
Connection ~ 4950 4500
Connection ~ 4950 4050
Wire Wire Line
	5450 4050 5450 4500
Connection ~ 5450 4050
Wire Wire Line
	5450 4500 5450 4600
Connection ~ 5450 4500
$Sheet
S 9500 950  700  4150
U 5E8D0C73
F0 "interface" 50
F1 "interface.sch" 50
F2 "GND" I L 9500 1900 50 
F3 "3V3" I L 9500 2000 50 
F4 "SW_LEFT" I L 9500 2300 50 
F5 "SW_MID" I L 9500 2400 50 
F6 "SW_RIGHT" I L 9500 2500 50 
F7 "STOP_BUTTON" I L 9500 2200 50 
F8 "START_BUTTON" I L 9500 2100 50 
F9 "SDA" B L 9500 2700 50 
F10 "SCL" I L 9500 2600 50 
$EndSheet
Wire Wire Line
	6300 4300 6450 4300
Wire Wire Line
	6200 4400 6450 4400
Wire Wire Line
	6100 4500 6450 4500
Wire Wire Line
	6000 4600 6450 4600
Wire Wire Line
	6100 3900 6100 4500
Wire Wire Line
	6200 3800 6200 4400
Wire Wire Line
	6300 2700 6300 4300
$Comp
L power:PWR_FLAG #FLG0108
U 1 1 5DF30A5A
P 3350 3500
F 0 "#FLG0108" H 3350 3575 50  0001 C CNN
F 1 "PWR_FLAG" V 3350 3628 50  0000 L CNN
F 2 "" H 3350 3500 50  0001 C CNN
F 3 "~" H 3350 3500 50  0001 C CNN
	1    3350 3500
	0    1    1    0   
$EndComp
Connection ~ 3350 3500
$Comp
L power:PWR_FLAG #FLG0109
U 1 1 5DF33064
P 3350 4500
F 0 "#FLG0109" H 3350 4575 50  0001 C CNN
F 1 "PWR_FLAG" V 3350 4628 50  0000 L CNN
F 2 "" H 3350 4500 50  0001 C CNN
F 3 "~" H 3350 4500 50  0001 C CNN
	1    3350 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 4100 3350 4500
$Comp
L power:PWR_FLAG #FLG0110
U 1 1 5DF37EA8
P 6200 2500
F 0 "#FLG0110" H 6200 2575 50  0001 C CNN
F 1 "PWR_FLAG" H 6200 2673 50  0000 C CNN
F 2 "" H 6200 2500 50  0001 C CNN
F 3 "~" H 6200 2500 50  0001 C CNN
	1    6200 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2500 6200 2700
Connection ~ 6200 2700
Wire Wire Line
	6200 2700 6300 2700
$Comp
L power:PWR_FLAG #FLG0111
U 1 1 5DF3B19D
P 5750 2500
F 0 "#FLG0111" H 5750 2575 50  0001 C CNN
F 1 "PWR_FLAG" H 5750 2673 50  0000 C CNN
F 2 "" H 5750 2500 50  0001 C CNN
F 3 "~" H 5750 2500 50  0001 C CNN
	1    5750 2500
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0112
U 1 1 5DF3BEE1
P 5700 3800
F 0 "#FLG0112" H 5700 3875 50  0001 C CNN
F 1 "PWR_FLAG" H 5700 3973 50  0000 C CNN
F 2 "" H 5700 3800 50  0001 C CNN
F 3 "~" H 5700 3800 50  0001 C CNN
	1    5700 3800
	1    0    0    -1  
$EndComp
Connection ~ 5700 3800
Wire Wire Line
	5700 3800 5450 3800
$Comp
L power:PWR_FLAG #FLG0113
U 1 1 5DF3C8FB
P 5700 3900
F 0 "#FLG0113" H 5700 3975 50  0001 C CNN
F 1 "PWR_FLAG" H 5700 4073 50  0000 C CNN
F 2 "" H 5700 3900 50  0001 C CNN
F 3 "~" H 5700 3900 50  0001 C CNN
	1    5700 3900
	-1   0    0    1   
$EndComp
Connection ~ 5700 3900
Wire Wire Line
	5700 3900 6100 3900
$Comp
L power:PWR_FLAG #FLG0106
U 1 1 5DF49198
P 1600 2600
F 0 "#FLG0106" H 1600 2675 50  0001 C CNN
F 1 "PWR_FLAG" H 1600 2773 50  0000 C CNN
F 2 "" H 1600 2600 50  0001 C CNN
F 3 "~" H 1600 2600 50  0001 C CNN
	1    1600 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2700 3550 2700
Wire Wire Line
	1600 2600 1600 2700
Connection ~ 1600 2700
Wire Wire Line
	1600 2700 2100 2700
$Comp
L Connector_Generic:Conn_01x09 J21
U 1 1 5DF98F2D
P 8550 2300
F 0 "J21" H 8500 2900 50  0000 L CNN
F 1 "CTL_4" H 8500 2800 50  0000 L CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_1x09_P2.00mm_Vertical" H 8550 2300 50  0001 C CNN
F 3 "~" H 8550 2300 50  0001 C CNN
	1    8550 2300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x09 J22
U 1 1 5DFA8CEF
P 9000 2300
F 0 "J22" H 8918 2917 50  0000 C CNN
F 1 "INT_1" H 8918 2826 50  0000 C CNN
F 2 "Connector_PinHeader_2.00mm:PinHeader_1x09_P2.00mm_Vertical" H 9000 2300 50  0001 C CNN
F 3 "~" H 9000 2300 50  0001 C CNN
	1    9000 2300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	9500 2100 9200 2100
Wire Wire Line
	9200 2200 9500 2200
Wire Wire Line
	9500 2300 9200 2300
Wire Wire Line
	9200 2400 9500 2400
Wire Wire Line
	9500 2500 9200 2500
Wire Wire Line
	9500 1900 9200 1900
Wire Wire Line
	9200 2000 9500 2000
Wire Wire Line
	9500 2600 9200 2600
Wire Wire Line
	9200 2700 9500 2700
$Sheet
S 6450 4250 800  1500
U 5E080FD6
F0 "control" 50
F1 "control.sch" 50
F2 "INT" I L 6450 4300 50 
F3 "BATT_VDD" I L 6450 4400 50 
F4 "GND" U L 6450 4500 50 
F5 "3V3" I L 6450 4600 50 
F6 "USB_C_VDD" I R 7250 4750 50 
F7 "SDA" B R 7250 5000 50 
F8 "SCL" I R 7250 4900 50 
$EndSheet
Wire Wire Line
	7250 5000 8350 5000
Wire Wire Line
	8350 5000 8350 2700
Wire Wire Line
	7250 4900 8250 4900
Wire Wire Line
	8250 4900 8250 2600
Wire Wire Line
	8250 2600 8350 2600
Wire Wire Line
	7250 2300 8350 2300
Wire Wire Line
	7250 2200 8350 2200
Wire Wire Line
	7250 2100 8350 2100
Wire Wire Line
	7250 2500 8350 2500
Wire Wire Line
	7250 2400 8350 2400
Wire Wire Line
	7250 2000 8350 2000
Wire Wire Line
	8350 1900 7250 1900
$Comp
L Connector_Generic:Conn_01x04 J19
U 1 1 5DFEC693
P 4800 1200
F 0 "J19" H 4750 1500 50  0000 L CNN
F 1 "PWR_4" H 4750 1400 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4800 1200 50  0001 C CNN
F 3 "~" H 4800 1200 50  0001 C CNN
	1    4800 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J20
U 1 1 5DFF03B1
P 5250 1200
F 0 "J20" H 5200 1500 50  0000 L CNN
F 1 "CTL_5" H 5200 1400 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 5250 1200 50  0001 C CNN
F 3 "~" H 5250 1200 50  0001 C CNN
	1    5250 1200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5450 1100 6450 1100
Wire Wire Line
	6450 1200 5450 1200
Wire Wire Line
	5450 1300 6450 1300
Wire Wire Line
	6450 1400 5450 1400
Wire Wire Line
	4600 1100 4550 1100
Wire Wire Line
	4550 1200 4600 1200
Wire Wire Line
	4600 1300 4550 1300
Wire Wire Line
	4550 1400 4600 1400
Wire Wire Line
	7250 4000 7400 4000
Wire Wire Line
	7400 4000 7400 4750
Wire Wire Line
	7400 4750 7250 4750
Wire Wire Line
	1400 3000 2100 3000
$Comp
L power:PWR_FLAG #FLG0107
U 1 1 5DF497C9
P 2050 2600
F 0 "#FLG0107" H 2050 2675 50  0001 C CNN
F 1 "PWR_FLAG" H 2050 2773 50  0000 C CNN
F 2 "" H 2050 2600 50  0001 C CNN
F 3 "~" H 2050 2600 50  0001 C CNN
	1    2050 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 2600 2050 3200
Connection ~ 2050 3200
Wire Wire Line
	2050 3200 1400 3200
Wire Wire Line
	3550 3200 3200 3200
Wire Wire Line
	5750 2500 5750 3200
Connection ~ 5750 3200
Wire Wire Line
	5750 3200 6000 3200
$Comp
L Connector_Generic:Conn_01x08 J?
U 1 1 5DDEAB72
P 5150 3000
AR Path="/5E8D0C73/5DDEAB72" Ref="J?"  Part="1" 
AR Path="/5DDEAB72" Ref="J16"  Part="1" 
F 0 "J16" H 5230 2992 50  0000 L CNN
F 1 "CTL_3" H 5230 2901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical" H 5150 3000 50  0001 C CNN
F 3 "~" H 5150 3000 50  0001 C CNN
	1    5150 3000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6450 2900 5350 2900
Wire Wire Line
	6450 3100 5350 3100
Wire Wire Line
	5350 3000 6450 3000
Wire Wire Line
	6000 4600 6000 3200
Connection ~ 6000 3200
Wire Wire Line
	6000 3200 6450 3200
$EndSCHEMATC
