#!/bin/bash
export XOPT XDEFS

XOPT="-ggdb -O0 -fomit-frame-pointer -DTEST_DELAY_BETWEEN_TESTS=0 -fprofile-arcs -ftest-coverage"
XDEFS=""

function clean() {
  echo -n "  * Cleaning..."
  make clean > /dev/null
  echo "OK"
}

function compile() {
  echo -n "  * Building..."
  if ! make > buildlog.txt
  then
    echo "failed"
    clean
    exit
  fi
  mv -f buildlog.txt ./reports/${1}_build.txt
  echo "OK"
}

function execute_test() {
  echo -n "  * Testing..."
  if ! ./build/ch > testlog.txt
  then
    echo "failed"
    clean
    exit
  fi
  mv -f testlog.txt ./reports/${1}_test.txt
  echo "OK"
}

function coverage() {
  echo -n "  * Coverage..."
  mkdir reports/${1}_gcov 2> /dev/null
  echo "Configuration $2" > gcovlog.txt
  echo "----------------------------------------------------------------" >> gcovlog.txt
  if ! make gcov >> gcovlog.txt 2> /dev/null
  then
    echo "failed"
    clean
    exit
  fi
  mv -f gcovlog.txt ./reports/${1}_gcov.txt
  mv -f *.gcov ./reports/${1}_gcov
  echo "OK"
}

function misra() {
  echo -n "  * Analysing..."
  if ! make misra > misralog.txt 2> misraerrlog.txt
  then
    echo "failed"
    clean
    exit
  fi
  echo "OK"
}

function test() {
  if [ -z "$2" ]
  then
    msg=$1": Default Settings"
    XDEFS=
  else
    msg=$1": "$2
    XDEFS=$2
  fi
  echo $msg
  compile $1
  execute_test $1
  coverage $1 "$msg"
  misra
  clean
}

function partial() {
  compile
  execute_test
  misra
  clean
}

mkdir reports 2> /dev/null

test cfg1 ""
test cfg2 "-DCH_CFG_OPTIMIZE_SPEED=FALSE"
test cfg3 "-DCH_CFG_TIME_QUANTUM=0"
test cfg4 "-DCH_CFG_USE_REGISTRY=FALSE -DCH_CFG_USE_DYNAMIC=FALSE"
test cfg5 "-DCH_CFG_USE_TM=FALSE"
test cfg6 "-DCH_CFG_USE_SEMAPHORES=FALSE -DCH_CFG_USE_MAILBOXES=FALSE -DCH_CFG_USE_OBJ_FIFOS=FALSE -DCH_CFG_USE_OBJ_CACHES=FALSE -DCH_CFG_USE_JOBS=FALSE"
test cfg7 "-DCH_CFG_USE_SEMAPHORES_PRIORITY=TRUE"
test cfg8 "-DCH_CFG_USE_MUTEXES=FALSE -DCH_CFG_USE_CONDVARS=FALSE"
test cfg9 "-DCH_CFG_USE_MUTEXES_RECURSIVE=TRUE"
test cfg10 "-DCH_CFG_USE_CONDVARS=FALSE"
test cfg11 "-DCH_CFG_USE_CONDVARS_TIMEOUT=FALSE"
test cfg12 "-DCH_CFG_USE_EVENTS=FALSE"
test cfg13 "-DCH_CFG_USE_EVENTS_TIMEOUT=FALSE"
test cfg14 "-DCH_CFG_USE_MESSAGES=FALSE -DCH_CFG_USE_DELEGATES=FALSE"
test cfg15 "-DCH_CFG_USE_MESSAGES_PRIORITY=TRUE"
test cfg16 "-DCH_CFG_USE_MAILBOXES=FALSE -DCH_CFG_USE_OBJ_FIFOS=FALSE -DCH_CFG_USE_JOBS=FALSE"
test cfg17 "-DCH_CFG_USE_MEMCORE=FALSE -DCH_CFG_USE_MEMPOOLS=FALSE -DCH_CFG_USE_HEAP=FALSE -DCH_CFG_USE_DYNAMIC=FALSE -DCH_CFG_USE_OBJ_FIFOS=FALSE -DCH_CFG_USE_JOBS=FALSE -DCH_CFG_USE_FACTORY=FALSE"
test cfg18 "-DCH_CFG_USE_MEMPOOLS=FALSE -DCH_CFG_USE_HEAP=FALSE -DCH_CFG_USE_DYNAMIC=FALSE -DCH_CFG_USE_OBJ_FIFOS=FALSE -DCH_CFG_USE_JOBS=FALSE -DCH_CFG_USE_FACTORY=FALSE"
test cfg19 "-DCH_CFG_USE_MEMPOOLS=FALSE -DCH_CFG_USE_OBJ_FIFOS=FALSE -DCH_CFG_USE_JOBS=FALSE -DCH_CFG_USE_FACTORY=FALSE"
test cfg20 "-DCH_CFG_USE_HEAP=FALSE -DCH_CFG_USE_FACTORY=FALSE"
test cfg21 "-DCH_CFG_USE_DYNAMIC=FALSE"
test cfg22 "-DCH_DBG_STATISTICS=TRUE"
test cfg23 "-DCH_DBG_SYSTEM_STATE_CHECK=TRUE"
test cfg24 "-DCH_DBG_ENABLE_CHECKS=TRUE"
test cfg25 "-DCH_DBG_ENABLE_ASSERTS=TRUE"
test cfg26 "-DCH_DBG_TRACE_MASK=CH_DBG_TRACE_MASK_ALL"
#test cfg27 "-DCH_DBG_ENABLE_STACK_CHECK=TRUE"
test cfg28 "-DCH_DBG_FILL_THREADS=TRUE"
test cfg29 "-DCH_DBG_THREADS_PROFILING=FALSE"
test cfg30 "-DCH_DBG_SYSTEM_STATE_CHECK=TRUE -DCH_DBG_ENABLE_CHECKS=TRUE -DCH_DBG_ENABLE_ASSERTS=TRUE -DCH_DBG_TRACE_MASK=CH_DBG_TRACE_MASK_ALL -DCH_DBG_FILL_THREADS=TRUE"
test cfg31 "-DCH_CFG_ST_RESOLUTION=16"
test cfg32 "-DCH_CFG_ST_RESOLUTION=16 -DCH_CFG_INTERVALS_SIZE=64"
test cfg33 "-DCH_CFG_INTERVALS_SIZE=64"
test cfg34 "-DCH_CFG_USE_OBJ_FIFOS=FALSE"
test cfg35 "-DCH_CFG_USE_FACTORY=FALSE"

rm *log.txt 2> /dev/null
echo
echo "Done"
