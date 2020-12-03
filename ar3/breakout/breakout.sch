EESchema Schematic File Version 4
EELAYER 30 0
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
L Teensy:Teensy3.5 U1
U 1 1 5FC45479
P 5600 3750
F 0 "U1" H 5600 6383 60  0000 C CNN
F 1 "Teensy3.5" H 5600 6277 60  0000 C CNN
F 2 "Teensy35:Teensy35_36" H 5500 6000 60  0001 C CNN
F 3 "https://www.pjrc.com/teensy/card8a_rev2.pdf" H 5600 6171 60  0000 C CNN
F 4 "https://www.pjrc.com/teensy/pinout.html" H 5600 6073 50  0000 C CNN "Pinouts"
	1    5600 3750
	1    0    0    -1  
$EndComp
$Comp
L Connector:RJ45 J?
U 1 1 5FC47CF7
P 1100 1300
F 0 "J?" H 1157 1967 50  0000 C CNN
F 1 "RJ45" H 1157 1876 50  0000 C CNN
F 2 "" V 1100 1325 50  0001 C CNN
F 3 "~" V 1100 1325 50  0001 C CNN
	1    1100 1300
	1    0    0    -1  
$EndComp
Text Label 1500 900  0    50   ~ 0
J1A
Text Label 1500 1000 0    50   ~ 0
J1B
Text Label 1500 1100 0    50   ~ 0
J2A
Text Label 1500 1200 0    50   ~ 0
J2B
Text Label 1500 1300 0    50   ~ 0
J3A
Text Label 1500 1400 0    50   ~ 0
J3B
$EndSCHEMATC
