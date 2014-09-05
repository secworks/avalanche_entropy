#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=======================================================================
#
# logic_data_extract.py
# --------------------
# Reads file with CSV data generated byt Salea Logic and extract
# the binary 8-bit data. Hard coded to grab from the second element.
#
# 
# Author: Joachim Strömbergson
# Copyright (c) 2014, Secworks Sweden AB
# 
# Redistribution and use in source and binary forms, with or 
# without modification, are permitted provided that the following 
# conditions are met: 
# 
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer. 
# 
# 2. Redistributions in binary form must reproduce the above copyright 
#    notice, this list of conditions and the following disclaimer in 
#    the documentation and/or other materials provided with the 
#    distribution. 
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#=======================================================================
 
#-------------------------------------------------------------------
# Python module imports.
#-------------------------------------------------------------------
import sys
import os

 
#-------------------------------------------------------------------
# Defines.
#-------------------------------------------------------------------
# Verbose operation on/off
VERBOSE = False
GEN_FILE = True
NUM_BYTES = 2


#-------------------------------------------------------------------
# main()
#
# Parse any arguments and run the tests.
# We skip the first line
#-------------------------------------------------------------------
def main():
    file_name = sys.argv[1]

    with open(file_name, 'r') as my_file:
        my_bytes = []
        i = 0
        for line in my_file:
            if i > 0:
                my_byte = line.split(',')[1][0:4]
                if VERBOSE:
                    print(my_byte, int(my_byte, 16), chr(int(my_byte, 16)))
                my_bytes.append(int(my_byte, 16))
            i = 1

    if VERBOSE:
        print(my_bytes)
        for my_byte in my_bytes:
            print(type(my_byte), my_byte, chr(my_byte), hex(my_byte))

    if GEN_FILE:
        with open(file_name + '.bin', 'wb') as my_file:
            for my_byte in my_bytes:
                my_file.write(bytes(chr(my_byte), 'latin_1'))


#-------------------------------------------------------------------
# __name__
# Python thingy which allows the file to be run standalone as
# well as parsed from within a Python interpreter.
#-------------------------------------------------------------------
if __name__=="__main__": 
    # Run the main function.
    sys.exit(main())


#=======================================================================
# EOF binextract.py
#=======================================================================
