# GDB configuration for iMX6 SabreLite (Dual CortexA9)
# On Board RAM starts at 0x900000

echo Setting up for the iMX6 SabreLite Board.\n

# SETUP GDB :
#
# Common gdb setup for ARM CPUs
set complaints 1
set output-radix 10
set input-radix 10
set prompt (arm-gdb) 
set endian little
dir .
echo Sset remote memory-write-packet-size 1024.\n
# Tell GDB to use 1024 bytes packes when downloading, this
# reduces load image download times
set remote memory-write-packet-size 1024
set remote memory-write-packet-size fixed


# DEFINE MACROS	:
#
# Create a "refresh" macro to update gdb's screens after the cpu
# has been stopped by the other CPU or following an "monitor allstop" 
define refresh
	monitor set hbreak
	cont
	monitor clear hbreak
end


# CONNECT TO TARGET :
#
# Tell GDB where send it's "gdb monitor" commands:

# Tell GDB to send them to the OcdRemote monitoring port 8888
# that is running on your Linux Host
target remote 127.0.0.1:8888

# Or Tell GDB to send them to the WifiDemon/mpDemon/OcdRemote
# monitoring port 8888 at ethernet address 192.168.1.222
#target remote 192.168.1.222:8888


# SETUP OCDREMOTE/TARGET :
#
# Setup for a little Endian ARM CPU, 
monitor endian little

# Reset and Halt the Target (only Core 0 will be active)
monitor reset
set prompt (arm-gdb core 0)

# Read the iMX6;s System Reset Control:Control Status Register
monitor long 0x20d8000
# Modify this register to enable cores 1, 2 & 3
monitor long 0x20d8000 = 0x1c00521

# Switch to each newly enabled core, halting each one
monitor set core 1
set prompt (arm-gdb core 1) 
monitor status
monitor halt 

monitor set core 2
set prompt (arm-gdb core 2)
monitor status
monitor halt

monitor set core 3
set prompt (arm-gdb core 3)
monitor status
monitor halt

# Switch back to core 0
monitor set core 0
set prompt (arm-gdb core 0)

# Force GDB to reacquire the register cache.
flushregs

# Halt Target 
monitor halt

# Setup CONTROL AND CPSR cp15 Registers
monitor reg CONTROL = 0x00050078
monitor reg CPSR    = 0x1D3


# set registers to reset values
set $r8  = 0
set $r9  = 0
set $r10 = 0
set $r11 = 0
set $r12 = 0
set $r13 = 0
set $lr  = 0
set $cpsr = 0x1d3

# TEST MEMORY, LOAD IMAGE :
#
# Test memory 
monitor test endian 0x900000

# Load the program executable called "test"
load /home/ahmad/iMX6_Platform_SDK/output/mx6dq/smp_primes/sabre_lite_rev_a/smp_primes.elf

# Load the symbols for the program.
symbol-file /home/ahmad/iMX6_Platform_SDK/output/mx6dq/smp_primes/sabre_lite_rev_a/smp_primes.elf

# RUN TO MAIN :
#
# Set a breakpoint at main().
#b main

# Run to the breakpoint.
#c
