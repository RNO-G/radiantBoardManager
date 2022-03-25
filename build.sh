#! /bin/sh

# extract version 

ver_extract() {
 grep "#define VER_$1" radiantBoardManager.ino | sed "s/#define VER_$1//" | sed "s/\r//" | tr -d ' '
}


VER_MAJOR=`ver_extract MAJOR`
VER_MINOR=`ver_extract MINOR`
VER_REV=`ver_extract REV`



VERSTRING=v${VER_MAJOR}r${VER_MINOR}p${VER_REV} 

mkdir -p ${VERSTRING} 

for variant in v1 v2  
do
  echo "Building $variant" 
  arduino-cli compile -e -b osu-boards:samd:radiant_$variant || exit 1
  BINFILE=${VERSTRING}/radiantBoardManager_${VERSTRING}.ino.radiant_${variant}.bin
  cp build/osu-boards.samd.radiant_${variant}/radiantBoardManager.ino.bin ${BINFILE}
  echo "Converting to uf2" 
  UF2FILE=${VERSTRING}/radiant${variant}BoardManager_${VERSTRING}.uf2
  uf2conv.py -o ${UF2FILE} ${BINFILE}
done




