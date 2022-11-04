This directory contain various utilities to create composite image 
to be flashed into EEPROM.

vspa_bin_tlb_gen:
===============
This is an x86 based utility to geneate vspa binary "apm_vspa.bin"
and composite vspa tables "apm_table.bin" from various vsp tables 
(Example PSSDET_REF_TD.bin, PSS_REF_XCORR.bin,SSSDET_REF.bin)
 
Below is the command to generate executable from C source code

gcc -o vspa_bin_tlb_gen vspa_bin_tlb_gen.c

