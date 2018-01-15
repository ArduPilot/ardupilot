#!/usr/bin/env python

import sys, fnmatch
import importlib

# peripheral types that can be shared, wildcard patterns
SHARED_MAP = [ "I2C*", "USART*_TX", "UART*_TX", "SPI*" ]

ignore_list = []
dma_map = None

debug = False

def check_possibility(periph, dma_stream, curr_dict, dma_map, check_list):
	for other_periph in curr_dict:
		if other_periph != periph:
			if curr_dict[other_periph] == dma_stream:
				ignore_list.append(periph)
				check_str = "%s(%d,%d) %s(%d,%d)" % (
										other_periph,
										curr_dict[other_periph][0],
										curr_dict[other_periph][1],
										periph,
										dma_stream[0],
										dma_stream[1])
				#check if we did this before
				if check_str in check_list:
					return False
				check_list.append(check_str)
                                if debug:
                                        print("Trying to Resolve Conflict: ", check_str)
				#check if we can resolve by swapping with other periphs
				for stream in dma_map[other_periph]:
					if stream != curr_dict[other_periph] and \
					   check_possibility(other_periph, stream, curr_dict, dma_map, check_list):
						curr_dict[other_periph] = stream
						return True
				return False
	return True

def can_share(periph):
        '''check if a peripheral is in the SHARED_MAP list'''
        for f in SHARED_MAP:
                if fnmatch.fnmatch(periph, f):
                        return True
        if debug:
                print("%s can't share" % periph)
        return False

def chibios_dma_define_name(key):
        '''return define name needed for board.h for ChibiOS'''
        if key.startswith('ADC'):
                return 'STM32_ADC_%s_DMA_STREAM' % key
        elif key.startswith('SPI'):
                return 'STM32_SPI_%s_DMA_STREAM' % key
        elif key.startswith('I2C'):
                return 'STM32_I2C_%s_DMA_STREAM' % key
        elif key.startswith('USART'):
                return 'STM32_UART_%s_DMA_STREAM' % key
        elif key.startswith('UART'):
                return 'STM32_UART_%s_DMA_STREAM' % key
        elif key.startswith('SDIO'):
                return 'STM32_SDC_%s_DMA_STREAM' % key
        else:
                print("Error: Unknown key type %s" % key)
                sys.exit(1)

def write_dma_header(f, peripheral_list, mcu_type):
        '''write out a DMA resolver header file'''
        global dma_map
        try:
                lib = importlib.import_module(mcu_type)
                dma_map = lib.DMA_Map
        except ImportError:
                print("Unable to find module for MCU %s" % mcu_type)
                sys.exit(1)
        
        print("Writing DMA map")
        unassigned = []
        curr_dict = {}

	for periph in peripheral_list:
		assigned = False
		check_list = []
                if not periph in dma_map:
                        print("Unknown peripheral function %s in DMA map for %s" % (periph, mcu_type))
                        sys.exit(1)
		for stream in dma_map[periph]:
			if check_possibility(periph, stream, curr_dict, dma_map, check_list):
				curr_dict[periph] = stream
				assigned = True
				break
		if assigned == False:
			unassigned.append(periph)

	# now look for shared DMA possibilities
	stream_assign = {}
	for k in curr_dict.iterkeys():
	        stream_assign[curr_dict[k]] = [k]

	unassigned_new = unassigned[:]
	for periph in unassigned:
		for stream in dma_map[periph]:
	                share_ok = True
	                for periph2 in stream_assign[stream]:
	                        if not can_share(periph) or not can_share(periph2):
	                                share_ok = False
	                if share_ok:
                                if debug:
                                        print("Sharing %s on %s with %s" % (periph, stream, stream_assign[stream]))
	                        curr_dict[periph] = stream
	                        stream_assign[stream].append(periph)
	                        unassigned_new.remove(periph)
	                        break
	unassigned = unassigned_new


	f.write("// auto-generated DMA mapping from dma_resolver.py\n")

	if unassigned:
	        f.write("\n// Note: The following peripherals can't be resolved for DMA: %s\n\n" % unassigned)

	for key in sorted(curr_dict.iterkeys()):
	        stream = curr_dict[key]
	        shared = ''
	        if len(stream_assign[stream]) > 1:
	                shared = ' // shared %s' % ','.join(stream_assign[stream])
	        f.write("#define %-30s STM32_DMA_STREAM_ID(%u, %u)%s\n" % (chibios_dma_define_name(key),
                                                                           curr_dict[key][0],
                                                                           curr_dict[key][1],
                                                                           shared))

	# now generate UARTDriver.cpp DMA config lines
	f.write("\n\n// generated UART DMA configuration lines\n")
	for u in range(1,9):
	        key = None
	        if 'USART%u_TX' % u in peripheral_list:
	                key = 'USART%u' % u
	        if 'UART%u_TX' % u in peripheral_list:
	                key = 'UART%u' % u
	        if 'USART%u_RX' % u in peripheral_list:
	                key = 'USART%u' % u
	        if 'UART%u_RX' % u in peripheral_list:
	                key = 'UART%u' % u
	        if key is None:
	                continue
	        f.write("#define STM32_%s_RX_DMA_CONFIG " % key)
	        if key + "_RX" in curr_dict:
	                f.write("true, STM32_UART_%s_RX_DMA_STREAM, STM32_%s_RX_DMA_CHN\n" % (key, key))
	        else:
	                f.write("false, 0, 0\n")
	        f.write("#define STM32_%s_TX_DMA_CONFIG " % key)
	        if key + "_TX" in curr_dict:
	                f.write("true, STM32_UART_%s_TX_DMA_STREAM, STM32_%s_TX_DMA_CHN\n" % (key, key))
	        else:
	                f.write("false, 0, 0\n")


if __name__ == '__main__':
	import optparse

	parser = optparse.OptionParser("dma_resolver.py")
	parser.add_option("-M", "--mcu", default=None, help='MCU type')
	parser.add_option("-D", "--debug", action='store_true', help='enable debug')
	parser.add_option("-P", "--peripherals", default=None, help='peripheral list (comma separated)')

	opts, args = parser.parse_args()

        if opts.peripherals is None:
                print("Please provide a peripheral list with -P")
                sys.exit(1)

        if opts.mcu is None:
                print("Please provide a MCU type with -<")
                sys.exit(1)

        debug = opts.debug

        plist = opts.peripherals.split(',')
        mcu_type = opts.mcu

        f = open("dma.h", "w")
        write_dma_header(f, plist, mcu_type)
