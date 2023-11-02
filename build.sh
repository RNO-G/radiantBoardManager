#! /bin/sh

#this builds all the different variants and sample rates automagically 
# you can pass additonal arguments that are treated as extra flags to the C compiler (e.g. -DENABLE_INTERRUPT_HISTORY) 

# extract version 
ver_extract() {
 grep "#define VER_$1" radiantBoardManager.ino | sed "s/#define VER_$1//" | sed "s/\r//" | tr -d ' '
}


VER_MAJOR=`ver_extract MAJOR`
VER_MINOR=`ver_extract MINOR`
VER_REV=`ver_extract REV`



VERSTRING=v${VER_MAJOR}r${VER_MINOR}p${VER_REV} 


mkdir -p ${VERSTRING} 

# did we pass any arguments? pass them to arduino-cli as extra flags

EXTRA_ARGS="" 

for arg in "$@" ; 
do 
  EXTRA_ARGS="$EXTRA_ARGS  --build-property compiler.cpp.extra_flags=$arg"
done 
echo "$EXTRA_ARGS" 

for variant in v1 v2 v3
  do 
  for sample_rate in 2400 3200 ; 
    do
    echo "Building $variant@$sample_rate" 
    samplerate_args="--build-property compiler.cpp.extra_flags=-DRADIANT_SAMPLE_RATE=$sample_rate"
    echo cmdline: arduino-cli compile -v -e -b osu-boards:samd:radiant_$variant $EXTRA_ARGS $samplerate_args . || exit 1
    arduino-cli compile -v -e -b osu-boards:samd:radiant_$variant $EXTRA_ARGS $EXTRA_ARGS $samplerate_args . || exit 1
    BINFILE=${VERSTRING}/radiantBoardManager_${VERSTRING}.ino.radiant_${variant}.${sample_rate}MHz.bin
    ELFFILE=${VERSTRING}/radiantBoardManager_${VERSTRING}.ino.radiant_${variant}.${sample_rate}MHz.elf
    cp build/osu-boards.samd.radiant_${variant}/radiantBoardManager.ino.bin ${BINFILE}
    cp build/osu-boards.samd.radiant_${variant}/radiantBoardManager.ino.elf ${ELFFILE}
    echo "Converting to uf2" 
    UF2FILE=${VERSTRING}/radiant${variant}BoardManager_${VERSTRING}.${sample_rate}MHz.uf2
    uf2conv.py -o ${UF2FILE} ${BINFILE}
    args_file=${VERSTRING}/extra_args.txt
    echo "$EXTRA_ARGS" > $args_file
  done 
done




