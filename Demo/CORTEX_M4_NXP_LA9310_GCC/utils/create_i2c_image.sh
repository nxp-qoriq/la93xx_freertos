#!/bin/sh
#SPDX-License-Identifier: BSD-3-Clause
#Copyright 2021 NXP

if [ "$#" -ne 1 ];then
        echo "Usage: create_i2cimage.sh <binary file>"
        echo "Ex: ./create_i2cimage.sh la9310.bin"
        exit 1
fi

i=0

outfile=i2c_write.xml
rm -rf $outfile
rm -rf file_hex
`xxd -p -c 1 $1 > file_hex`

sz=`xxd -p -c 1 $1 | wc -l`
sz_hx=`printf "%08x\n" $sz`
sz_d=`echo $sz_hx | cut -c 1-2`
sz_c=`echo $sz_hx | cut -c 3-4`
sz_b=`echo $sz_hx | cut -c 5-6`
sz_a=`echo $sz_hx | cut -c 7-8`

if [ -z "$sz_a" ] && [ -z "$sz_b" ] && [ -z "$sz_c" ] && [ -z "$sz_d" ];then
   sz_a=0
   sz_b=0
   sz_c=0
   sz_d=0
fi

if [ -n "$sz_d" ] && [ -z "$sz_c" ] && [ -z "$sz_b" ] && [ -z "$sz_a" ];then
   sz_a="$sz_d"
   sz_b=0
   sz_c=0
   sz_d=0
fi

if [ -n "$sz_d" ] && [ -n "$sz_c" ] && [ -z "$sz_b" ] && [ -z "$sz_a" ];then
   sz_a="$sz_c"
   sz_b="$sz_d"
   sz_c=0
   sz_d=0
fi

if [ -n "$sz_d" ] && [ -n "$sz_c" ] && [ -n "$sz_b" ] && [ -z "$sz_a" ];then
   sz_a="$sz_b"
   sz_b="$sz_c"
   sz_c="$sz_d"
   sz_d=0
fi

echo "<adapter>" >> $outfile
echo "  <configure i2c=\"1\" spi=\"1\" gpio=\"0\" tpower=\"1\" pullups=\"1\"/>" >> $outfile
echo "  <i2c_bitrate khz=\"100\"/>" >> $outfile

#Header
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 00 55</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 01 AA</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 02 55</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 03 AA</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile

echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 04 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 05 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 06 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 07 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile

echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 08 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 09 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 0a 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 0b 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile

echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 0c $sz_a</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 0d $sz_b</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 0e $sz_c</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 0f $sz_d</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile

echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 10 30</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 11 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 12 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 13 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile

echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 14 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 15 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 16 80</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 17 1f</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile

echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 18 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 19 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 1a 80</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 1b 1f</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile

echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 1c 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 1d 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 1e 01</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile
echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">00 1f 00</i2c_write>" >> $outfile
echo "  <sleep ms=\"10\"/>" >> $outfile


i=48

while read line
do
j=`printf "%08x\n" $i`
sz_d=`echo $j | cut -c 1-2`
sz_c=`echo $j | cut -c 3-4`
sz_b=`echo $j | cut -c 5-6`
sz_a=`echo $j | cut -c 7-8`

if [ "$i" -lt "65536" ]
then
  echo "  <i2c_write addr=\"0x50\" count=\"3\" radix=\"16\">$sz_b $sz_a $line</i2c_write>" >> $outfile
  echo "  <sleep ms=\"10\"/>" >> $outfile
elif [ "$i" -lt "16777216" ]
then
  echo "  <i2c_write addr=\"0x50\" count=\"4\" radix=\"16\">$sz_c $sz_b $sz_a $line</i2c_write>" >> $outfile
  echo "  <sleep ms=\"10\"/>" >> $outfile
else
  echo "  <i2c_write addr=\"0x50\" count=\"5\" radix=\"16\">$sz_d $sz_c $sz_b $sz_a $line</i2c_write>" >> $outfile
  echo "  <sleep ms=\"10\"/>" >> $outfile
fi

i=$((i+1))
done < file_hex

echo "</adapter>" >> $outfile
