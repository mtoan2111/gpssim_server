#!/bin/bash


# Color Code
GREEN='\033[1;32m'
RED='\033[1;31m'
OFF='\033[0m'
YELLOW='\033[1;33m'
CYAN='\033[1;36m'
PURPLE='\033[1;35m'

# Variables
SIM_PATH=``
RF_PATH=``
GPSSIM_EXE="/gps-sdr-sim"
FIFO1="/tmp/myfifo1"
FIFO2="/tmp/myfifo2"


#kill all previous process
H_P=$(ps ax | grep hackrf_transfer | grep -v grep | wc -l)
if [ "${H_P}" -gt "0" ]
then
  echo -e \
    "${GREEN}$0${OFF}: ${RED}[K]${OFF} kill all hackrf_transfer"
  killall hackrf_transfer
fi

S_P=$(ps ax | grep gps-sdr-sim | grep -v grep | wc -l)
if [ "${S_P}" -gt "0" ]
then
  echo -e \
    "${GREEN}$0${OFF}: ${RED}[K]${OFF} kill all gps-sim-sdr"
  killall gps-sdr-sim 
fi
 

#check GPSSIM path
echo -e \
 "${GREEN}$0${OFF}: ${YELLOW}[C]${OFF} GPSSIM path"

if [ -z "${GPSSIM_PATH}" ]
then
  echo -e \
    "${GREEN}$0${OFF}: ${RED}[W]${OFF} GPSSIM path is not set"
    exit 1
  else
    if ! [ -d "${GPSSIM_PATH}" ]
    then
      echo -e \
      "${GREEN}$0${OFF}: ${RED}[W]${OFF} GPSSIM path must be a directory"
      exit 1
    else
      SIM_PATH=${GPSSIM_PATH}
    fi
fi

# Check the gpssim binary file
echo -e \
 "${GREEN}$0${OFF}: ${YELLOW}[C]${OFF} GPSSIM binary file"
FULL_EXE_PATH="$GPSSIM_PATH$GPSSIM_EXE"
if ! [ -f "${FULL_EXE_PATH}" ] 
then
  echo -e \
    "${GREEN}$0${OFF}: ${RED}[W]${OFF} GPSSIM binary file doesn't exist"
  echo -e \
    "${GREEN}$0${OFF}: ${CYAN}[G]${OFF} gps-sim-sdr"
  C_FILE="$SIM_PATH/gpssim.c"
  if ! [ -f "${C_FILE}" ]
  then
    echo -e \
      "${GREEN}$0${OFF}: ${RED}[W]${OFF} gpssim.c doesn't exist"
  else
    gcc ${C_FILE} -lm -lpthread -O3 -o ${FULL_EXE_PATH} || exit
  fi
fi


echo -e \
 "${GREEN}$0${OFF}: ${YELLOW}[C]${OFF} HACKRF path"
if [ -z "${HACKRF_PATH}" ]
then
  echo -e \
    "${GREEN}$0${OFF}: ${RED}[W]${OFF} HACKRF path is not set"
    exit 1
  else
    if ! [ -d "${HACKRF_PATH}" ]
    then
      echo -e \
      "${GREEN}$0${OFF}: ${RED}[W]${OFF} HACKRF path must be a directory"
      exit 1
    else
      RF_PATH=${HACKRF_PATH}
    fi
fi

#Check HACKRF tool

 echo -e \
  "${GREEN}$0${OFF}: ${YELLOW}[C]${OFF} HACKRF tool"

FULL_TOOL_PATH="$RF_PATH/host/build/hackrf-tools/src/hackrf_transfer"
if ! [ -e "${FULL_TOOL_PATH}" ] 
then
  echo -e \
    "${GREEN}$0${OFF}: ${RED}[W]${OFF} hackrf_transfer binary file doesn't exist"
  echo -e \
    "${GREEN}$0${OFF}: ${CYAN}[G]${OFF} hackrf_tool"
  CMAKE_BUILD="$RF_PATH/host/build"
  CMAKE_DIR="$RF_PATH/host"
  mkdir -p "$CMAKE_BUILD" || exit 
  cmake -B$CMAKE_BUILD -H$CMAKE_DIR || exit
  make -C $CMAKE_BUILD || exit
fi

echo -e \
 "${GREEN}$0${OFF}: ${CYAN}[G]${OFF} llh stream"
if ! [ -e "${FIFO1}" ]
then
  mkfifo ${FIFO1} || exit
fi
echo -e \
 "${GREEN}$0${OFF}: ${CYAN}[G]${OFF} gps data stream"
if ! [ -e "${FIFO2}" ]
then
  mkfifo ${FIFO2} || exit
fi

echo -e \
 "${GREEN}$0${OFF}: ${PURPLE}[E]${OFF} running gpssim"

#find brdc
ephs=$(find ${SIM_PATH} -name "brdc*.*n") || exit
e=``
if [ "$(echo ${ephs} | wc -l)" -gt "0" ]
then
  echo -e \
    "${GREEN}$0${OFF}: ${YELLOW}[C]${OFF} searching ephemeris files"
#  exit 1
  e=$(find ${SIM_PATH} -name "brdc*.*n" | sort -n | tail -n 1) 
else
 echo -e \
    "${GREEN}$0${OFF}: ${RED}[W]${OFF} Can't find any eph files in gpssim folder"
fi

${FULL_EXE_PATH} -e ${e} -f ${FIFO1} -b 8 -t $(date -u +%Y/%m/%d,%H:%M:%S) -o ${FIFO2} &

${FULL_TOOL_PATH} -t ${FIFO2} -f 1575420000 -s 2600000 -a 1 -x 0 &


echo -e \
 "${GREEN}$0${OFF}: ${PURPLE}[E]${OFF} running hackrf"




#${FULL_EXE_PATH} -f ${FIFO2} -b 
