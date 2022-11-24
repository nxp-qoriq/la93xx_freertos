#!/usr/bin/python3
#SPDX-License-Identifier: BSD-3-Clause
#Copyright 2022 NXP
#
# Usage : ./memory_calc.py -i la9310.map -o la9310_memory_map 
# Dependency:  pip3 install tabulate

import sys
import copy
import enum
import tabulate
import getopt
import os.path
from os import path

subsections={"m_startup":{"start_up"},"m_interrupts":{"isr_vector"},"m_text":{"text","rodata","glue_7","glue_7t","eh_frame","init","fini"},"m_data":{"data","bss","heap","stack"}}

#globals
lmc_version = "1.0"
lmc_elf_filename = "la9310.map"
lmc_output_filename = "la9310_memory.dump"
test = False
search_specific_mode = False
specific_symbol_name = ""

usage = "Usage: \n \
              -i <path>/la9310.map : Input mapfile to check, if not given \"release/la9310.map\" is taken by default \n \
              -o output_file          : Outout file to dump the report \n \
              -t                      : Test all sections in single run\n \
              -s <symbol_name>        : Search symbol across all memory sections \n"

# strings
MECONFIG_STRING = "Memory Configuration"
DEFAULT_STRING = "*default*"
START_STRING = "_start ="
END_STRING = "_end ="
PREFIX_STRING = "__"
POWERPC_LIBRARY_STRING = "arm-none-eabi"
FILL_STRING = "*fill*"
FREERTOS_STRING = "la9310.dir/"
CMAKEFILES_STRING = "CMakeFiles/"

# elf table headers
NAME_STRING = "Name"
START_ADDR_STRING = "StartAddr"
RSV_END_ADDR_STRING = "RsvEndAddr"
END_ADDR_STRING = "EndAddr"
ATTRIBUTES_STRING = "Attributes"
AVAILABLE_KB_LEN_STRING = "Avail (KB)"
USED_KB_LEN_STRING = "Used (KB)"
FREE_KB_LEN_STRING = "Free (KB)"

# section table headers
SYMBOL_NAME_STRING = "Symbol Name"
SYMBOL_START_ADDR_STRING = "Symbol StartAddr"
SYMBOL_LEN_STRING = "Symbol Len (B)"
SYMBOL_FILE_NAME_STRING = "Symbol Filename"

class Mem_Config(enum.Enum):
    MEMCONFIG_IDLE = 0
    MEMCONFIG_START = 1
    MEMCONFIG_STOP = 2

class Sec_Config(enum.Enum):
    SECTIONCONFIG_IDLE = 0
    SECTIONCONFIG_START = 1
    SECTIONCONFIG_STOP = 2

class LA12xx_Memory_Calc:
    def __init__(self):
        self.version = lmc_version
        self.elf_filename = lmc_elf_filename
        self.output_filename = lmc_output_filename
        self.output_fp = None
        self.memconfig_flag = Mem_Config.MEMCONFIG_IDLE
        self.memconfig_map = []
        self.memconfig_count = 0
        self.sectionconfig_map = []
        self.sectionconfig_count = 0
        if not path.exists(self.elf_filename):
            print ("Map file do not exist:", self.elf_filename)
            print (usage)
            exit()

    def process_section_mem_config(self, section_name, line, prev_line):
        # case 1 - len(prev_len)
        if line.find(PREFIX_STRING + section_name + START_STRING) != -1:
            return
        if line.find(POWERPC_LIBRARY_STRING) != -1 or line.find(FILL_STRING) != -1:
            return
        values = line.split()

        if len(values) == 4: #case 2:  .text          0x000000001f800420        0xc CMakeFiles/la9310.dir/platform/ARM_CM4/start.S
            if values[3].find(CMAKEFILES_STRING) == -1:
                return
            self.gcc_lines_continue = False
            secconfig_map_item = {SYMBOL_NAME_STRING: "Unknown", SYMBOL_START_ADDR_STRING: 0, SYMBOL_LEN_STRING: 0, SYMBOL_FILE_NAME_STRING: "Unknown"}
            secconfig_map_item[SYMBOL_NAME_STRING] = values[0]
            secconfig_map_item[SYMBOL_START_ADDR_STRING] = values[1]
            secconfig_map_item[SYMBOL_FILE_NAME_STRING] = values[3].split(FREERTOS_STRING)[1] if len(values[3].split(FREERTOS_STRING)) > 1 else values[3]
            symbol_name = values[0]
            try:
                symbol_len = int(values[2], 16)
            except ValueError:
                symbol_len = -1
            if symbol_len > 0:
                secconfig_map_item[SYMBOL_LEN_STRING] = symbol_len
                self.sectionconfig_map.append(secconfig_map_item)
                self.sectionconfig_count += 1
        elif len(values) == 3: # case 3: 0x000000001f807310      0x13c CMakeFiles/la9310.dir/drivers/timesync/sync_timing_device.c.obj
            if values[2].find(CMAKEFILES_STRING) == -1:
                return
            self.gcc_lines_continue = False
            secconfig_map_item = {SYMBOL_NAME_STRING: "Unknown", SYMBOL_START_ADDR_STRING: 0, SYMBOL_LEN_STRING: 0, SYMBOL_FILE_NAME_STRING: "Unknown"}
            secconfig_map_item[SYMBOL_START_ADDR_STRING] = values[0]
            secconfig_map_item[SYMBOL_FILE_NAME_STRING] = values[2].split(FREERTOS_STRING)[1] if len(values[2].split(FREERTOS_STRING)) > 1 else values[2]
            try:
                symbol_len = int(values[1], 16)
            except ValueError:
                symbol_len = -1
            if symbol_len > 0:
                secconfig_map_item[SYMBOL_LEN_STRING] = symbol_len
                self.sectionconfig_map.append(secconfig_map_item)
                self.sectionconfig_count += 1
        elif len(values) == 2: # case 4: 0x000000001f805bb0                iLa9310_I2C_Init

            if len(prev_line.split()) < 2: # check if previous line length
                return
            if prev_line.find(POWERPC_LIBRARY_STRING) != -1 or self.gcc_lines_continue:
                self.gcc_lines_continue = True
                return 
            
            self.gcc_lines_continue = False
            
            if self.sectionconfig_count >= 1:

                for i in subsections[section_name]:
                    if self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_NAME_STRING].find(i) != -1:
                        self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_START_ADDR_STRING] = values[0]
                        self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_NAME_STRING] = values[1]
                        return

                if self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_NAME_STRING] == "Unknown" \
                    or self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_NAME_STRING].find("shared") != -1:
                    self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_START_ADDR_STRING] = values[0]
                    self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_NAME_STRING] = values[1]
            else:
                total_length = self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_LEN_STRING]
                secconfig_map_item =  copy.copy(self.sectionconfig_map[self.sectionconfig_count - 1])
                secconfig_map_item[SYMBOL_START_ADDR_STRING] = values[0]
                secconfig_map_item[SYMBOL_NAME_STRING] = values[1]
                self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_LEN_STRING] = int(secconfig_map_item[SYMBOL_START_ADDR_STRING], 16) - int(self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_START_ADDR_STRING], 16)
                secconfig_map_item[SYMBOL_LEN_STRING] = total_length - self.sectionconfig_map[self.sectionconfig_count - 1][SYMBOL_LEN_STRING]
                self.sectionconfig_map.append(secconfig_map_item)
                self.sectionconfig_count += 1
        else:
            return

    def parse_elf_section(self, section_name):
        self.sectionconfig_map = []
        self.sectionconfig_count = 0
        fp = None
        try:
            fp = open(self.elf_filename, 'r')
            try:
                prev_line = None
                while True:
                    line = fp.readline()
                    if not line:
                        break

                    if not line.strip():
                        continue
                    if line.find("*(") != -1:
                        continue

                    if line.find(PREFIX_STRING + section_name + START_STRING) != -1:
                        self.memconfig_flag = Sec_Config.SECTIONCONFIG_START

                    if self.memconfig_flag == Sec_Config.SECTIONCONFIG_START:
                        if line.find(PREFIX_STRING + section_name + END_STRING) != -1:
                            self.memconfig_flag = Sec_Config.SECTIONCONFIG_STOP
                        else:
                            values = line.split()
                            if values[0][0] == '.' and len(values[0]) >= 14:
                                line = line.strip('\n') + " " + fp.readline().strip()
                            self.process_section_mem_config(section_name, line, prev_line)
                    prev_line = copy.copy(line)
            finally:
                fp.close()
        except IOError:
            print ("file open fail, elf_filename=", self.elf_filename)
            print (usage)

    def print_section_config(self):
        if self.sectionconfig_map:
            newlist = sorted(self.sectionconfig_map, key=lambda d: d[SYMBOL_LEN_STRING], reverse=True)
            header = newlist[0].keys()
            rows =  [x.values() for x in newlist]
            total = sum([abs(list(x.values())[2]) for x in newlist])
            if header and rows:
                table = tabulate.tabulate(rows, header, showindex="always", tablefmt="grid")
                print (table)
                self.output_fp.write(table.encode("utf-8"))
                self.output_fp.write("\n".encode("utf-8"))
                print("Total size shown: ", total)
        else:
             print ("No symbols found!")

    def search_specific_symbol(self, symbol_name):
        if self.sectionconfig_count == 0:
            return -1
        header = self.sectionconfig_map[0].keys()
        rows = []
        for ind in range(self.sectionconfig_count):
            if self.sectionconfig_map[ind][SYMBOL_NAME_STRING] == symbol_name:
                rows =  [self.sectionconfig_map[ind].values()]
        if header and rows:
            print ("")
            table = tabulate.tabulate(rows, header, showindex="always", tablefmt="grid")
            print (table)
            self.output_fp.write(table.encode("utf-8"))
            self.output_fp.write("\n".encode("utf-8"))
        else:
            return -1

    def print_mem_config(self):
        header = self.memconfig_map[0].keys()
        rows =  [x.values() for x in self.memconfig_map]
        print ("")
        table = tabulate.tabulate(rows, header, showindex="always", tablefmt="grid")
        print (table)
        self.output_fp.write(table.encode("utf-8"))
        self.output_fp.write("\n".encode("utf-8"))

    def print_specific_section(self, section_name):
        header = self.memconfig_map[0].keys()
        for ind in range(self.memconfig_count):
            if self.memconfig_map[ind]['Name'] == section_name:
                rows = [self.memconfig_map[ind].values()]
        if header and rows:
            print ("")
            table = tabulate.tabulate(rows, header, showindex="always", tablefmt="grid")
            print (table)
            self.output_fp.write(table.encode("utf-8"))
            self.output_fp.write("\n".encode("utf-8"))
        else:
            return -1

    def process_elf_mem_config(self, line):
        if line.find(MECONFIG_STRING) != -1:
            return
        if line.find(NAME_STRING) != -1:
            return
        values = line.split()
        memconfig_map_item = {NAME_STRING: values[0], START_ADDR_STRING: values[1], RSV_END_ADDR_STRING: hex(int(values[1], 16) + int(values[2], 16)), ATTRIBUTES_STRING: values[3],END_ADDR_STRING: "NA", AVAILABLE_KB_LEN_STRING: "NA", USED_KB_LEN_STRING: "NA", FREE_KB_LEN_STRING: "NA"}
        self.memconfig_map.append(memconfig_map_item)
        self.memconfig_count += 1

    def update_elf_mem_config(self, line):
        section_end_addr, section_end_tag = None, None
        items=line.split()
        section_end_addr = items[0]
        section_end_tag_str = items[1]
        section_end_tag = section_end_tag_str[section_end_tag_str.find('_')+len('_'):section_end_tag_str.rfind('_')]
        section_end_tag = section_end_tag[1:] if section_end_tag[0] == '_' else section_end_tag
        #special case
        section_end_tag = "shared" if section_end_tag == "shdata" else section_end_tag
        for i in range(self.memconfig_count):
            if section_end_tag == self.memconfig_map[i]['Name']:
                self.memconfig_map[i][END_ADDR_STRING] = section_end_addr
                self.memconfig_map[i][AVAILABLE_KB_LEN_STRING] = "{:.2f}".format((int(self.memconfig_map[i][RSV_END_ADDR_STRING], 16) - int(self.memconfig_map[i][START_ADDR_STRING], 16)) / 1024)
                self.memconfig_map[i][USED_KB_LEN_STRING] = "{:.2f}".format((int(self.memconfig_map[i][END_ADDR_STRING], 16) - int(self.memconfig_map[i][START_ADDR_STRING], 16)) / 1024)
                self.memconfig_map[i][FREE_KB_LEN_STRING] = "{:.2f}".format(float(self.memconfig_map[i][AVAILABLE_KB_LEN_STRING]) - float(self.memconfig_map[i][USED_KB_LEN_STRING]))

    def parse_elf_map(self):
        fp = None
        try:
            fp = open(self.elf_filename, 'r')
            print ("elf_filename = ", self.elf_filename)
            self.output_fp = open(self.output_filename, 'wb')
            print ("output_filename = ", self.output_filename)
            try:
                while True:
                    line = fp.readline()
                    if not line:
                        break

                    if not line.strip():
                        continue

                    if line.find(MECONFIG_STRING) != -1:
                        self.memconfig_flag = Mem_Config.MEMCONFIG_START

                    if self.memconfig_flag == Mem_Config.MEMCONFIG_START:
                        if line.find(DEFAULT_STRING) != -1:
                            self.memconfig_flag = Mem_Config.MEMCONFIG_STOP
                        else:
                            self.process_elf_mem_config(line)

                    if line.find(END_STRING) != -1:
                            self.update_elf_mem_config(line)
            finally:
                fp.close()
        except IOError:
            print ("file open fail, elf_filename=", self.elf_filename)
            print (usage)

if __name__ == "__main__":
    argv = sys.argv[1:]
    try:
        opts, args = getopt.getopt(argv, "i:o:s:t")
    except:
        print("OptError")
        print(usage)
        exit()

    for opt, arg in opts:
        if opt in ['-i']:
            lmc_elf_filename = arg
        elif opt in ['-o']:
            lmc_output_filename = arg
        elif opt in ['-t']:
            test = True
        elif opt in ['-s']:
            search_specific_mode = True
            specific_symbol_name = arg

    lm = LA12xx_Memory_Calc()
    print ("LA12xx Memory Calculator, version", lm.version)
    lm.parse_elf_map()
    while True:
        if test:
            lm.print_mem_config()
            for mc in lm.memconfig_map:
                section_name = mc['Name']
                if section_name == "shared":
                    section_name = "shdata"
                print ("Section Name: ", section_name)
                lm.parse_elf_section(section_name)
                lm.print_section_config()
                print ("")
            break
        elif search_specific_mode:
            found = False
            for mc in lm.memconfig_map:
                section_name = mc['Name']
                if section_name == "shared":
                    section_name = "shdata"
                lm.parse_elf_section(section_name)
                if(lm.search_specific_symbol(specific_symbol_name) != -1):
                    found = True
                    print("Symbol found in the Section ", mc['Name'])
                    lm.print_specific_section(mc['Name'])
                    break
            if (found == False) :
                print("Cannot find Symbol :", specific_symbol_name)

            break
        else:
            lm.print_mem_config()
            try:
                print ("")
                option = int(input("Type 99 to exit, Choose option: "))
            except ValueError:
                option = -1
          
            if option == 99:
                break
            elif option < 0 or option >= len(lm.memconfig_map):
                print ("Invalid option!")
            else:
                # special case
                section_name = "shdata" if lm.memconfig_map[option]['Name'] == "shared" else lm.memconfig_map[option]['Name']
                print ("Section Name: ", section_name)
                lm.parse_elf_section(section_name)
                lm.print_section_config()
                print ("")
                input("Press any key to continue. ")

    lm.output_fp.close()
    print ("LA12xx Memory Calculator Exited!")
