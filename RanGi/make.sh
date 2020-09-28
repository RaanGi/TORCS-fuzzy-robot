#!/bin/sh

fuzzylite='/src/drivers/RanGi/fuzzylite'		
fuzzylite_lib='/src/drivers/RanGi/fuzzylite/release/bin'	
fuzzylite=${TORCS_BASE}${fuzzylite}			# Fuzzylite source path
fuzzlite_lib=${TORCS_BASE}${fuzzylite_lib}		# Fuzzylite lib path

g++ -o fuzzyengines fuzzyengines.cpp -I${fuzzylite} -L${fuzzylite_lib} -lfuzzylite-static -lrt -lpthread

make install

make clean
rm -f fuzzyengines.o fuzzyengines
