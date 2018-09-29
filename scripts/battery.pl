#!/usr/bin/perl
#Baterijos itampos skaiciavimas
#
# define MB_DIVIDER_R1  10000
# define MB_DIVIDER_R2  1000
# MB_ADC_VOLTAGE 3.2f
#
#
#    const double _mbMaxVoltage = (((double) MB_DIVIDER_R1 + (double) MB_DIVIDER_R2) * (double) MB_ADC_VOLTAGE) / (double) MB_DIVIDER_R2;
#
#
#this->_battery_voltage = ((double) (adcValue * _mbMaxVoltage) / 1024);

use strict;

my $r1 =10000;		#  MB_DIVIDER_R1
my $r2 =10000;		#  MB_DIVIDER_R2

my $mbAdcVoltage = 3.2;	# MB_ADC_VOLTAGE

my $mbMaxVoltage = ($r1 + $r2) * $mbAdcVoltage / $r2; 

print "R1 - $r1\n";
print "R2 - $r2\n";
print "ADC voltage: $mbAdcVoltage\n";
print "Max voltage: $mbMaxVoltage\n";
print "Resolution : " . $mbMaxVoltage / 1024 ." V.\n";


