#!/bin/bash
export CHVERSION=21.11.5

function makeclean() {
  rm *.log 2> /dev/null
}

#function makechm() {
#  rm html/* 2> /dev/null
#  cp ../common/rsc/logo_small.png html/logo_small.png
#  if ! doxygen Doxyfile_chm 2> tmp.log
#  then
#    echo "Doxygen failed"
#    exit
#  fi
#  cat tmp.log | fgrep -v "warning: return type of member" > chibios_${CHVERSION}_${CHNAME}_chm2.log
#  if [ -s chibios_${CHVERSION}_${CHNAME}_chm2.log ]
#  then
#    echo "  * Doxygen warnings or errors"
##    exit
#  fi
#  rm tmp.log 2> /dev/null
#  cp html/refman.chm chibios_${CHVERSION}_${CHNAME}.chm
#  rm html/* 2> /dev/null
#  echo "  - CHM generated"
#}

function makepdf() {
  rm latex/* 2> /dev/null
  cp ../common/tex/chibios.sty ./latex
  if ! doxygen Doxyfile_pdf #2> /dev/null
  then
    echo "Doxygen failed"
    exit
  fi
#  cp ../common/tex/Makefile ./latex
  cd latex
  # Patches a Doxygen bug.
#  if grep "input{hierarchy}" < refman.tex > /dev/null
#  then
#    cp refman.tex refman2.tex
#    sed -e '0,/input{hierarchy}/s/Data Structure/Class Hierarchy/' < refman2.tex > refman.tex
#  fi
  # PDF generation.
  if ! make all 1> ../chibios_${CHVERSION}_${CHNAME}_rm_pdf1.log 2> ../chibios_${CHVERSION}_${CHNAME}_rm_pdf2.log
  then
    echo "Make failed"
    exit
  fi
  cd ..
  cp latex/refman.pdf ../../manuals/chibios_${CHVERSION}_${CHNAME}_rm.pdf
  rm latex/* 2> /dev/null
  echo "  - PDF generated"
}

function makearchive() {
#  tar -pczf chibios_${CHVERSION}_${CHNAME}.tar.gz chibios_${CHVERSION}_${CHNAME}.chm chibios_${CHVERSION}_${CHNAME}.pdf
  tar -pczf ../../manuals/chibios_${CHVERSION}_${CHNAME}_rm.tar.gz ../../manuals/chibios_${CHVERSION}_${CHNAME}_rm.pdf 2> /dev/null
}

export CHNAME=hal
echo ${CHNAME}
cd ${CHNAME}
makeclean
#makechm
makepdf
makearchive
cd ..

export CHNAME=nil
echo ${CHNAME}
cd ${CHNAME}
makeclean
#makechm
makepdf
makearchive
cd ..

export CHNAME=rt
echo ${CHNAME}
cd ${CHNAME}
makeclean
#makechm
makepdf
makearchive
cd ..
