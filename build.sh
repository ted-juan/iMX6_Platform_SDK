TOOLCHAIN_PATH=/home/ted/ti/ccsv6/tools/compiler/gcc-arm-none-eabi-4_9-2015q3/
export PATH=${TOOLCHAIN_PATH}bin:$PATH
#export DEBUG="1"
export FREERTOS="1"
export CFLAGS="-g"
export CXXFLAGS="-g"
export CROSS_COMPILE=arm-none-eabi-
export CROSS_LIB1=${TOOLCHAIN_PATH}arm-none-eabi/lib
export CROSS_LIB2=${TOOLCHAIN_PATH}lib/gcc/arm-none-eabi/5.4.1

#./tools/build_sdk -t mx6sdl -b evb -rev a -r all
./tools/build_sdk -t mx6sdl -b sabre_ai -rev a -a freertos
