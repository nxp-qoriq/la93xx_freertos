#!/bin/bash
#SPDX-License-Identifier: BSD-3-Clause
#Copyright 2021-2022 NXP

# Default options

# These address must be write_size aligned (128/256/512) based on EEPROM device used
freertos_image_offset="0x1000"
vspa_bin_offset="0x20000"
vspa_table_offset="0x34000"

####################################################

################### Boot Header ####################

#struct header {
#	uint32_t preamble;
#	uint32_t plugin_size;
#	uint32_t plugin_offset;
#	uint32_t bl_size;
#	uint32_t bl_src_offset;
#	uint32_t bl_dest;
#	uint32_t bl_entry;
#	uint32_t flags;
#};

################### Extended Boot Header #############

#struct header {
#	uint32_t page_write_size;
#	uint32_t addr_shift;
#	uint32_t freertos_image_offset;
#	uint32_t vspa_bin_location;
#	uint32_t vspa_table_location;
#	uint32_t reserved;
#	uint32_t reserved;
#	uint32_t reserved;
#};

########################################################


sleep_time=10
write_size=128
addr_shift=7
slave_addr_orig=0x50
bitrate=100
supported_eeproms="CAT24C512 , AT24CM02 , CAT24M01, M24M02"
supported_os="Ubuntu"
images=""
A7="(A15 - A7)"
A8="(A17 - A8)"
eeprom=
tcm_addr="0x1f800000"
amax=0
test_page_end_addr=262144
#gen_sanity_img="y"

vspa_table_images="PSS_REF_XCORR.bin PSSDET_REF_TD.bin SSSDET_REF.bin"
vspa_table_header_size="512"

vspa_bin_images=".ipram .vpram .idram .vpram_overlay .overlay_1 .overlay_2"
vspa_bin_header_size="512"

R='\033[0;31m'
G='\033[0;32m'
Y='\033[0;33m'
P='\033[0;35m'
NC='\033[0m'
C='\033[0;36m'
B='\e[5m'

outfile_frtos=freertos.xml
outfile_vspa=vspa.xml
outfile_vspa_table=vspa_table.xml
outfile_combine=combined_image.xml
outfile_bootstrap=bootstrap.xml
hex_frtos=file_hex_frtos
hex_vspa=file_hex_vspa
hex_vspa_table=file_hex_vspa_table
hex_bootstrap=file_hex_bootstrap

tmp_vspa_table_file=tmp_vtable_out.bin
tmp_vspa_table_header_file=tmp_vtable_header.bin

tmp_vspa_bin_file=tmp_vbin_out.bin
tmp_vspa_bin_header_file=tmp_vbin_header.bin

print_boundary() {
	echo ""
	echo -e "${C}###################################################################################${NC}"
	echo ""
}

do_exit() {
	echo -e "${C}###################################################################################${NC}"
	echo ""
	exit
}

do_err() {
	echo -e "${R}${B}ERROR${NC} : $1"
	do_exit
}

do_usage_err() {
	echo -e "${R}${B}ERROR${NC} : $1"
	usage
	do_exit
}

do_war() {
	echo -e "${R}${B}WARNING${NC} : ${Y}$1${NC}"
}

do_print() {
	echo -e "${Y}$1${NC}"
}

get_word_le() {
	local address
	local le_address
	address=`printf "%08x\n" $1`
	le_address=`echo ${address:6:2}${address:4:2}${address:2:2}${address:0:2}`
	echo $le_address
}

get_size() {
	local size=`stat -c '%s' $1`
	echo $size
}

truncated_file() {
	local size=`stat -c '%s' $1`
	local tsize=$((size-1))
	truncate -s $tsize $1
}

to_lefile() {
       local input=$1
       local output=$2
       xxd -e -g4 $input | xxd -r > $output
}

create_padded_file() {
	dd if=/dev/zero of=$1 bs=$2 count=1 > /dev/null 2>&1
}

get_word() {
	local address
	address=`printf "%08x\n" $1`
	echo $address
}

check_and_update_range() {
	local s_page=$1
	local e_page=$2
	local addr_shift=$4
    local rem_bytes=$3
    local dec_address
	local s_address
	local e_address

	s_address=$((s_page<<$addr_shift))
	s_address=`printf "%08x\n" $s_address`
    e_page=$((s_page+e_page))
	e_address=$((e_page<<$addr_shift))
	e_address=$((e_address+rem_bytes))
	e_address=`printf "%08x\n" $e_address`

    dec_address=$(hex_to_dec $s_address)
    if [[ "$dec_address" -lt "$amax" ]];then
        do_err "found address overlapped in generating image"
    else
        amax=$(hex_to_dec $e_address)
    fi
}

update_header_uint() {
	local value=$1
	local offset=$2
	local outfile=$3
	local ext_value

	ext_value=$(get_word_le $value)
	echo "$ext_value" > tmpfile
	xxd -r -p tmpfile tmpfile1
	dd if=tmpfile1 of=$outfile conv=notrunc bs=1 seek=$((offset)) > /dev/null 2>&1
	rm -rf tmpfile tmpfile1
}

update_header_string() {
	local value=$1
	local size=$2
	local offset=$3
	local outfile=$4 # in decimal

	echo "$value" | xxd -p > tmpfile
	xxd -r -p tmpfile tmpfile1
	truncated_file tmpfile1
	dd if=tmpfile1 of=$outfile conv=notrunc bs=1 seek=$((offset)) > /dev/null 2>&1
	rm -rf tmpfile tmpfile1
}

check_Ubuntu_version() {
	version_str=`lsb_release -r | cut -d : -f 2 | sed 's/\t//' | sed 's/^ *//'`
	version=`echo $version_str | cut -d . -f 1`
	if [ "$version" -lt 14 ] || [ "$version" -gt 20 ]; then
		do_war "Tool is not tested on distribution $1 ($version_str)"
	fi
}

check_machine() {
	local match=""
	for word in $supported_os;do
		machine=`lsb_release -d | grep "$word"`
		[ -n "$machine" ] && {
			match="y"
			check_"$word"_version $word
		}
	done
	[ -z "$match" ] && {
		do_war "Tool is not tested on distribution (`lsb_release -i | cut -d : -f 2 | sed 's/\t//' | sed 's/^ *//'`)"
	}
}

eeprom_info() {
	if [ -n "$eeprom" ];then
	do_print "EEPROM = $eeprom"
	else
		write_size=256
		addr_shift=8
		addr_bits="$A8"
		if [ -z "$eeprom_device" ];then
			do_war "'-d (--device)' option not used for EEPROM device (Using default options for 'Page Size' & 'Page Identification bits')"
		else
			do_war "Unmatched EEPROM device (Using default options for 'Page Size' & 'Page Identification bits')"
		fi
	fi
	do_print ""
	do_print "Page Size = $write_size"
	do_print "Page Identification bits = $addr_bits"
}

print_details() {
	#do_print "I2C Slave Address = $slave_addr_orig"
	do_print "Read/Write Delay = $sleep_time ms"
	do_print "Bitrate = $bitrate"
	do_print ""

	if [ "$gen_bootstrap_img" = "y" ];then
		do_print "Bootstrap header offset = 0x0"
	fi
	[ "$gen_frtos_img" = "y" ] && {
		do_print "freertos header offset = $freertos_image_offset"
	}
	[ "$gen_vspa_bin" = "y" ] && {
		do_print "VSPA bin offset = $vspa_bin_offset"
	}
	[ "$gen_vspa_img" = "y" ] && {
		do_print "VSPA bin offset = $vspa_bin_offset"
	}
	[ "$gen_vspa_table" = "y" ] && {
		do_print "VSPA table offset = $vspa_table_offset"
	}
	do_print ""
}

function ProgressBar {
    let _progress=(${1}*100/${2}*100)/100
    let _done=(${_progress}*4)/10
    let _left=40-$_done
    _fill=$(printf "%${_done}s")
    _empty=$(printf "%${_left}s")
    printf "\r                 [${_fill// /#}${_empty// /-}] ${_progress}%%"
}

usage ()
{
cat <<EOF

Usage:    $0 [ -b <boot_strapper.bin> ] [ -f <la9310.bin> ] [ -e <apm.elf> ] [ -d <eeprom device> ] [ -t ] [ -c ]
Example:
          $0 -b boot_strapper.bin -f la9310.bin -e apm.eld -d CAT24M01 -t -c
          $0 -b boot_strapper.bin -f la9310.bin -v apm_vspa.bin -d CAT24M01 -t -c

OPTION:
 -b,  --bootstrap    Prepend bootstrap image and header (Address 0x0)
 -f,  --freertos     Generate XML for FreeRTOS image including boot header
 -e,  --vspaelf      Generate XML after parsing ELF formatted vspa image for sections ($vspa_bin_images)
                     Note: default vspa binary offset is $vspa_bin_offset, change manually for different offset
 -d,  --device       EEPROM device (Ex: $supported_eeproms)
 -t,  --vspatable    Generate XML for vspa table with header using images ($vspa_table_images)
                     Note: default vspa table offset is $vspa_table_offset, change manually for different offset
 -v,  --vspa         Generate XML for vspa image (in binary formary)
 -c,  --combined     Merge all images to a single XML file

EOF
    do_exit
}

print_boundary

[ -z "$1" -o "$1" = "-h" -o "$1" = "--help" ] && usage && do_exit

ARGS=$(getopt -a -o f:v:d:b:e:cht \
       -l freertos:,vspa:,device:,bootstrap:vspaelf:,combined:,vspatable,help -- "$@")

[ $? -ne 0 ] && usage && exit
eval set -- "${ARGS}"
while true
do
	case "$1" in
	-f|--freertos)
		freertos_img=$2;
		[ ! -f "$freertos_img" ] && do_usage_err "freertos image \"$freertos_img\" does not exists"
		gen_frtos_img="y"
		images=$images" $outfile_frtos"
		shift;;
	-b|--bootstrap)
		bootstrap_img=$2;
		[ ! -f "$bootstrap_img" ] && do_usage_err "bootstrap image \"$bootstrap_img\" does not exists"
		gen_bootstrap_img="y"
		images=$images" $outfile_bootstrap"
		shift;;
	-v|--vspa)
		vspa_img=$2;
		[ ! -f "$vspa_img" ] && do_usage_err "vspa image \"$vspa_img\" does not exists"
		gen_vspa_img="y"
		images=$images" $outfile_vspa"
		shift;;
	-c|--combined)
		combine_img="y"
		;;
	-t|--vspatable)
		gen_vspa_table="y"
		images=$images" $outfile_vspa_table"
		;;
	-e|--vspaelf)
		vspa_bin_img=$2;
		[ ! -f "$vspa_bin_img" ] && do_usage_err "vspa elf image \"$vspa_bin_img\" does not exists"
		gen_vspa_bin="y"
		images=$images" $outfile_vspa"
		shift;;
	-d|--device)
		eeprom_device=$2;
		case $eeprom_device in
			CAT24C512)
				write_size=128
				addr_shift=7
				addr_bits="$A7"
				eeprom="CAT24C512"
				;;
			AT24CM02)
				write_size=256
				addr_shift=8
				addr_bits="$A8"
				eeprom="AT24CM02"
				;;
			CAT24M01)
				write_size=256
				addr_shift=8
				addr_bits="$A8"
				eeprom="CAT24M01"
				;;
			M24M02)
				write_size=256
				addr_shift=8
				addr_bits="$A8"
				eeprom="M24M02"
				;;
			*)
				write_size=256
				addr_shift=8
				addr_bits="$A8"
				eeprom=""
				;;
		esac

		;;
	-h|--help)
		usage;;
	--)
		shift; break;;
	esac
shift
done

[ -z "$images" ] && do_usage_err "Insufficient options"
check_machine
eeprom_info

cleanup_hex() {
	rm -rf $hex_frtos
	rm -rf $hex_vspa
	rm -rf $hex_vspa_table
	rm -rf $hex_bootstrap
}

cleanup_files() {
	rm -rf $outfile_frtos
	rm -rf $outfile_vspa
	rm -rf $outfile_vspa_bin
	rm -rf $outfile_combine
	rm -rf $outfile_bootstrap
}

cleanup_this_file() {
	rm -rf $1
}

get_address() {
	local page=$1
	local addr_shift=$2
	local address
	address=$((page<<$addr_shift))
	address=`printf "%08x\n" $address | sed 's/../& /g' | sed 's/ *$//'`
	echo $address
}

get_tcm_address() {
	local page=$1
	local addr_shift=$2
	local address
	address=$((page<<$addr_shift))
	tcm_address=$((tcm_addr+address))
	address=`printf "%08x\n" $tcm_address | sed 's/../& /g' | sed 's/ *$//'`
	echo $address
}

generate_xml_header() {
	local outfile=$1
	local bitrate=$2

	echo "<adapter>" >> $outfile
	echo "  <configure i2c=\"1\" spi=\"1\" gpio=\"0\" tpower=\"1\" pullups=\"1\"/>" >> $outfile
	echo "  <i2c_bitrate khz=\"$bitrate\"/>" >> $outfile
}

generate_xml_footer() {
	local outfile=$1
	echo "</adapter>" >> $outfile
}

generate_test_body() {
	local page_start=$1
	local page_end=$2
	local outfile=$3
	local count=4
	local str="AA 55 BB 66"
	local dummywrite=" $str"
	local bytes=0

	do_print "Appending test image"
	while [ "$count" -lt "$write_size" ]
	do
		count=$((count+4))
		str=$str$dummywrite
	done

    while [ "$page_start" -lt "$page_end" ]
	do
		read -r addr4 addr3 addr2 addr1<<< $(get_address $page_start $addr_shift)
		slave_addr=`printf "%02x\n" $((slave_addr_orig+addr4+addr3))`
		bytes=$((write_size+2))
		page_start=$((page_start+1))
		echo "  <i2c_write addr=\"0x$slave_addr\" count=\"$bytes\" radix=\"16\">$addr2 $addr1 $str</i2c_write>" >> $outfile
		echo "  <sleep ms=\"$sleep_time\"/>" >> $outfile
		ProgressBar ${page_start} ${page_end}
	done
	echo ""
}

generate_body() {
	local page=$1
	local num_pages=$2
	local rem_bytes=$3
	local hex_file=$4
	local outfile=$5
	local num_pages_prog=$num_pages

	if [ "$rem_bytes" -gt "0" ];then
		num_pages_prog=$((num_pages_prog+1))
	fi

	page_count=1
	while read line
	do
		read -r addr4 addr3 addr2 addr1<<< $(get_address $page $addr_shift)
		slave_addr=`printf "%02x\n" $((slave_addr_orig+addr4+addr3))`

		if [ "$page_count" -le "$num_pages" ]
		then
			bytes=$((write_size+2))
		else
			bytes=$((rem_bytes+2))
		fi
		echo "  <i2c_write addr=\"0x$slave_addr\" count=\"$bytes\" radix=\"16\">$addr2 $addr1 $line</i2c_write>" >> $outfile
		echo "  <sleep ms=\"$sleep_time\"/>" >> $outfile
		ProgressBar ${page_count} ${num_pages_prog}
		page=$((page+1))
		page_count=$((page_count+1))
	done < $hex_file
	echo ""
}

generate_hex() {
	local image=$1
	local outfile=$2
	`xxd -p -c $write_size $image | sed 's/../& /g' > $outfile`
}

generate_bootstrap() {
	local eh_1=0
	local eh_2=0
	local eh_3=0
	local eh_4=0
	local eh_5=0
	local eh_6=0
	local eh_7=0
	local eh_8=0

	do_print "Generating image ($outfile_bootstrap)"
	cleanup_this_file $outfile_bootstrap

	page=0
	read -r addr4 addr3 addr2 addr1<<< $(get_address $page $addr_shift)

	page=1
	read -r saddr4 saddr3 saddr2 saddr1<<< $(get_address $page $addr_shift)

	generate_hex $bootstrap_img $hex_bootstrap
	sz=`xxd -p -c 1 $bootstrap_img | wc -l`
	sz_hx=`printf "%08x\n" $sz | sed 's/../& /g' | sed 's/ *$//'`
	read -r sz_d sz_c sz_b sz_a<<< $(echo $sz_hx)

	#Extended Header
	eh1=$(get_word_le $write_size)
	eh1=`echo $eh1| sed 's/../& /g' | sed 's/ *$//'`

	eh2=$(get_word_le $addr_shift)
	eh2=`echo $eh2| sed 's/../& /g' | sed 's/ *$//'`

	eh3=$(get_word_le $freertos_image_offset)
	eh3=`echo $eh3| sed 's/../& /g' | sed 's/ *$//'`

	eh4=$(get_word_le $vspa_bin_offset)
	eh4=`echo $eh4| sed 's/../& /g' | sed 's/ *$//'`

	eh5=$(get_word_le $vspa_table_offset)
	eh5=`echo $eh5| sed 's/../& /g' | sed 's/ *$//'`

	eh6=$(get_word_le $eh6)
	eh6=`echo $eh6| sed 's/../& /g' | sed 's/ *$//'`

	eh7=$(get_word_le $eh7)
	eh7=`echo $eh7| sed 's/../& /g' | sed 's/ *$//'`

	eh8=$(get_word_le $eh8)
	eh8=`echo $eh8| sed 's/../& /g' | sed 's/ *$//'`

	num_pages=$(($sz/$write_size))
	rem_bytes=$(($sz - num_pages*write_size))
	page=1
	
	slave_addr=`printf "%02x\n" $((slave_addr_orig+addr4+addr3))`
	generate_xml_header $outfile_bootstrap $bitrate

	echo "  <i2c_write addr=\"0x$slave_addr\" count=\"64\" radix=\"16\">00 00 55 AA 55 AA 00 00 00 00 00 00 00 00 $sz_a $sz_b $sz_c $sz_d $saddr1 $saddr2 $saddr3 $saddr4 00 00 80 1f 00 00 80 1f 00 00 01 00 $eh1 $eh2 $eh3 $eh4 $eh5 $eh6 $eh7 $eh8</i2c_write>" >> $outfile_bootstrap
	echo "  <sleep ms=\"$sleep_time\"/>" >> $outfile_bootstrap
	
	generate_body $page $num_pages $rem_bytes $hex_bootstrap $outfile_bootstrap
	generate_xml_footer $outfile_bootstrap
}

generate_freertos() {
	local page_header=$1
	local page=$((page_header+1))
    local test_page_start=0

	do_print "Generating image ($outfile_frtos)"
	cleanup_this_file $outfile_frtos

	read -r addr4 addr3 addr2 addr1<<< $(get_address $page_header $addr_shift)
	read -r saddr4 saddr3 saddr2 saddr1<<< $(get_address $page $addr_shift)

	read -r dst4 dst3 dst2 dst1<<< $(get_tcm_address $page_header $addr_shift)

	generate_hex $freertos_img $hex_frtos
	sz=`xxd -p -c 1 $freertos_img | wc -l`
	sz_hx=`printf "%08x\n" $sz | sed 's/../& /g' | sed 's/ *$//'`
	read -r sz_d sz_c sz_b sz_a<<< $(echo $sz_hx)

	num_pages=$(($sz/$write_size))
	rem_bytes=$(($sz - num_pages*write_size))
	slave_addr=`printf "%02x\n" $((slave_addr_orig+addr4+addr3))`

    check_and_update_range $page_header $num_pages $rem_bytes $addr_shift
	generate_xml_header $outfile_frtos $bitrate

	echo "  <i2c_write addr=\"0x$slave_addr\" count=\"34\" radix=\"16\">$addr2 $addr1 55 AA 55 AA 00 00 00 00 00 00 00 00 $sz_a $sz_b $sz_c $sz_d $saddr1 $saddr2 $saddr3 $saddr4 $dst1 $dst2 $dst3 $dst4 $dst1 $dst2 $dst3 $dst4 00 00 01 00</i2c_write>" >> $outfile_frtos
	echo "  <sleep ms=\"$sleep_time\"/>" >> $outfile_frtos

	generate_body $page $num_pages $rem_bytes $hex_frtos $outfile_frtos
    [ "$gen_sanity_img" = "y" ] && {
        test_page_start=$((page+num_pages))
        test_page_end=$((test_page_end_addr/$write_size))
        generate_test_body $test_page_start $test_page_end $outfile_frtos
    }
	generate_xml_footer $outfile_frtos
}

generate_vspa() {
	do_print "Generating image ($outfile_vspa)"
	cleanup_this_file $outfile_vspa

	image_offset=`printf "0x%x\n" $vspa_bin_offset`
	mod=$((image_offset%write_size))
	[ "$mod" -ne "0" ] && do_err "Unaligned vspa binary offset $vspa_bin_offset"
	
	generate_hex $vspa_img $hex_vspa
	sz=`xxd -p -c 1 $vspa_img | wc -l`
	num_pages=$(($sz/$write_size))
	rem_bytes=$(($sz - num_pages*write_size))
	page=$((image_offset/write_size))

	generate_xml_header $outfile_vspa $bitrate
    check_and_update_range $page $num_pages $rem_bytes $addr_shift
	generate_body $page $num_pages $rem_bytes $hex_vspa $outfile_vspa
	generate_xml_footer $outfile_vspa
}


prepend_vspa_table_header() {
	local header_size=$1
	local num_image
	local current_header_offset=0
	local tbl_size=0
	local tbl_tcl_loc=0

	rm -rf $tmp_vspa_table_header_file
	create_padded_file $tmp_vspa_table_header_file $vspa_table_header_size

	# num_vspa_tbls
	num_image=`echo $vspa_table_images | awk '{print NF}'`
	update_header_uint $num_image $current_header_offset $tmp_vspa_table_header_file
	current_header_offset=$((current_header_offset+4))

    for vspa_file in $vspa_table_images
    do
		# tbl_name
		update_header_string $vspa_file 32 $current_header_offset $tmp_vspa_table_header_file
		current_header_offset=$((current_header_offset+32))

		#tbl_size
		tbl_size=$(get_size $vspa_file)
		update_header_uint $tbl_size $current_header_offset $tmp_vspa_table_header_file
		current_header_offset=$((current_header_offset+4))

		#tbl_tcl_loc
		tbl_tcl_loc=0
		update_header_uint $tbl_tcl_loc $current_header_offset $tmp_vspa_table_header_file
		current_header_offset=$((current_header_offset+4))
	done
}

generate_vspa_table() {
	local image_count

	for vspa_file in $vspa_table_images
	do
		[ ! -f "$vspa_file" ] && do_usage_err "vspa table image \"$vspa_file\" does not exists"
	done

	image_offset=`printf "0x%x\n" $vspa_table_offset`
	mod=$((image_offset%write_size))
	[ "$mod" -ne "0" ] && do_err "Unaligned vspa table offset $vspa_table_offset"

	do_print "Generating image ($outfile_vspa_table)"
	cleanup_this_file $outfile_vspa_table

	prepend_vspa_table_header $vspa_table_header_size
	mv -f $tmp_vspa_table_header_file $tmp_vspa_table_file

	image_count=`echo $vspa_table_images | awk '{print NF}'`
     [ "$image_count" -gt 1 ] && {
         for vspa_file in $vspa_table_images
         do
			 cat $vspa_file >> $tmp_vspa_table_file
         done
     }

	generate_hex $tmp_vspa_table_file $hex_vspa_table
	sz=`xxd -p -c 1 $tmp_vspa_table_file | wc -l`
	num_pages=$(($sz/$write_size))
	rem_bytes=$(($sz - num_pages*write_size))
	page=$((image_offset/write_size))

	generate_xml_header $outfile_vspa_table $bitrate
    check_and_update_range $page $num_pages $rem_bytes $addr_shift
	generate_body $page $num_pages $rem_bytes $hex_vspa_table $outfile_vspa_table
	generate_xml_footer $outfile_vspa_table
	rm -rf $tmp_vspa_table_file
}

hex_to_dec() {
	local value=`printf "%d\n" 0x$1`
	echo $value
}

dec_to_hex() {
	local value=`printf "%x\n" $1`
	echo $value
}

prepend_vspa_bin_header() {
	local header_size=$1
	local tmpfile="$2"
	local num_image=0
	local current_header_offset=0
	local is_overlay=0
	local dmem_addr=0
	local byte_cnt=0
	local xfr_ctrl=0
	local axi_tcm_addr=0
	local eeprom_rel_addr_offset=0
	local sec_name_length=20
	local alignment=0
	local afilesize=0
	local current_rel_addr=0

	rm -rf $tmp_vspa_bin_header_file
	create_padded_file $tmp_vspa_bin_header_file $vspa_bin_header_size

	# num_sections
	num_image=`echo $vspa_bin_images | awk '{print NF}'`
	update_header_uint $num_image $current_header_offset $tmp_vspa_bin_header_file
	current_header_offset=$((current_header_offset+4))

	for vspa_section in $vspa_bin_images
	do
		# section_name
		update_header_string $vspa_section $sec_name_length $current_header_offset $tmp_vspa_bin_header_file
		current_header_offset=$((current_header_offset+sec_name_length))

		#is_overlay
		is_overlay=`echo $vspa_section | grep "\.overlay_"`
		if [ -z "$is_overlay" ];then
			is_overlay=0
		else
			is_overlay=1
		fi
		update_header_uint $is_overlay $current_header_offset $tmp_vspa_bin_header_file
		current_header_offset=$((current_header_offset+4))

		#dmem_addr
		dmem_addr=`cat $tmpfile | grep "$vspa_section " | cut -d ' ' -f 6`
		dmem_addr=$(hex_to_dec $dmem_addr)
		update_header_uint $dmem_addr $current_header_offset $tmp_vspa_bin_header_file
		current_header_offset=$((current_header_offset+4))

		#byte_cnt : fixme : alignment from readelf is not correct, vspa driver is expecting 16 byte align images
		byte_cnt=`cat $tmpfile | grep "$vspa_section " | cut -d ' ' -f 8`
		alignment="0x10"
		afilesize=$(get_aligned_size $byte_cnt $alignment)
		update_header_uint $afilesize $current_header_offset $tmp_vspa_bin_header_file
		current_header_offset=$((current_header_offset+4))

		#xfr_ctrl : fixme : need logic for xfr_ctrl calculation
		xfr_ctrl=0
		case $vspa_section in
			.ipram)
				xfr_ctrl="300" #hex
				;;
			.vpram)
				xfr_ctrl="200" #hex
				;;
			.idram)
				xfr_ctrl="0"  #hex
				;;
			.vpram_overlay)
				xfr_ctrl="200" #hex
				;;
			.overlay_1)
				xfr_ctrl="200" #hex
				;;
			.overlay_2)
				xfr_ctrl="200" #hex
				;;
			*)
				xfr_ctrl="200" #hex
				;;
		esac

		xfr_ctrl=$(hex_to_dec $xfr_ctrl)
		update_header_uint $xfr_ctrl $current_header_offset $tmp_vspa_bin_header_file
		current_header_offset=$((current_header_offset+4))

		#axi_tcm_addr
		axi_tcm_addr=0xffffffff
		update_header_uint $axi_tcm_addr $current_header_offset $tmp_vspa_bin_header_file
		current_header_offset=$((current_header_offset+4))

		#eeprom_rel_addr_offset
		eeprom_rel_addr_offset=$current_rel_addr
		update_header_uint $eeprom_rel_addr_offset $current_header_offset $tmp_vspa_bin_header_file
		current_header_offset=$((current_header_offset+4))
		current_rel_addr=$((current_rel_addr+$afilesize))
	done
}

get_aligned_size() {
	local size=$(hex_to_dec $1)
	local alignment=$2
	local mod=0
	local diff=0

	mod=$((size%alignment))
	if [ "$mod" -ne "0" ];then
		diff=$((alignment-mod))
		size=$((size+diff))
	fi
	echo "$size"
}

copy_elf_to_bin() {
	local infile=$1
	local offset=$(hex_to_dec $2)
	local size=$(hex_to_dec $3)
	local outfile=$4

	xxd -p -l $size -s $offset $infile > tmpfile
	xxd -r -p tmpfile tmpfile1
	dd if=tmpfile1 of=$outfile conv=notrunc bs=1 > /dev/null 2>&1
	rm -rf tmpfile tmpfile1
}

extract_bin() {
	local elffile=$1
	local section=$2
	local tmpfile=$3
	local outfile=$4
	local size=0
	local alignment=0
	local afilesize=0
	local tmp_bin_file="tmpbinfile"

	offset=`cat $tmpfile | grep "$section " | cut -d ' ' -f 7`
	size=`cat $tmpfile | grep "$section " | cut -d ' ' -f 8`
	alignment=`cat $tmpfile | grep "$section " | cut -d ' ' -f 13`

	#fixme: readelf alignment is not correct, vspa code is expecting 16 byte alignment
	alignment="0x10"
	afilesize=$(get_aligned_size $size $alignment)

	#echo "section=$section offset=$offset size=$size alignment=$alignment afilesize=$(dec_to_hex $afilesize)"
	create_padded_file $tmp_bin_file $afilesize
	copy_elf_to_bin $elffile $offset $size $tmp_bin_file
	cat $tmp_bin_file >> $outfile
	rm -rf $tmp_bin_file
}

generate_vspa_bin() {
	local image_count
	local tmpfile="readelf_output"

	image_offset=`printf "0x%x\n" $vspa_bin_offset`
	mod=$((image_offset%write_size))
	[ "$mod" -ne "0" ] && do_err "Unaligned vspa image offset $vspa_bin_offset"

	do_print "Generating image ($outfile_vspa) from elf file $vspa_bin_img"
	cleanup_this_file $outfile_vspa

	readelf -sSdD $vspa_bin_img | tr -s ' ' > $tmpfile
	prepend_vspa_bin_header $vspa_bin_header_size $tmpfile

	mv -f $tmp_vspa_bin_header_file $tmp_vspa_bin_file

    for vspa_section in $vspa_bin_images
    do
		extract_bin $vspa_bin_img $vspa_section $tmpfile $tmp_vspa_bin_file 
	done
	rm -rf $tmpfile

	generate_hex $tmp_vspa_bin_file $hex_vspa
	sz=`xxd -p -c 1 $tmp_vspa_bin_file | wc -l`
	num_pages=$(($sz/$write_size))
	rem_bytes=$(($sz - num_pages*write_size))
	page=$((image_offset/write_size))

	generate_xml_header $outfile_vspa $bitrate
    check_and_update_range $page $num_pages $rem_bytes $addr_shift
	generate_body $page $num_pages $rem_bytes $hex_vspa $outfile_vspa
	generate_xml_footer $outfile_vspa
	rm -rf $tmp_vspa_bin_file
}

generate_combined_image() {
	local count=1
	cleanup_this_file $outfile_combine

	for word in $images
	do
		if [ $count -eq 1 ]; then
			cp -f $word $outfile_combine
		else
			sed -i '$d' $outfile_combine
			sed 1,3d $word >> $outfile_combine
		fi
		count=$((count+1))

		[ $count -eq 2 ] && {
			do_print "Generating combined image ($outfile_combine)"
		}
	done
}

#cleanup_files
print_details

if [ "$gen_bootstrap_img" = "y" ];then
	generate_bootstrap
fi

if [ "$gen_frtos_img" = "y" ];then
	fpage=$(($freertos_image_offset/$write_size))
	generate_freertos $fpage
fi

if [ "$gen_vspa_img" = "y" ];then
	generate_vspa
fi

if [ "$gen_vspa_bin" = "y" ];then
	if [ "$gen_vspa_img" = "y" ];then
		do_usage_err "VSPA binary (-v option) and VSPA ELF (-e option) support are mutually exclusive, please use either of them"
	else
		generate_vspa_bin
	fi
fi

if [ "$gen_vspa_table" = "y" ];then
	generate_vspa_table
fi

if [ "$combine_img" = "y" ];then
	generate_combined_image
fi

print_boundary

cleanup_hex
