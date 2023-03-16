#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo $SCRIPT_DIR
source ${SCRIPT_DIR}/../../../../devel/setup.bash
echo ${SCRIPT_DIR}/../../../../devel/setup.bash

dataset_office=("P000"  "P001"  "P002"  "P003"  "P004"  "P005"  "P006")
dataset_neighborhood=("P000"  "P001"  "P002"  "P003"  "P004"  "P005"  "P007"
                      "P008"  "P009"  "P010"  "P012"  "P013"  "P014"  "P015"
                      "P016"  "P017"  "P018"  "P019"  "P020"  "P021")
dataset_carwelding=("P001"  "P002"  "P004"  "P005"  "P006"  "P007")
dataset_oldtown=("P000"  "P001"  "P002"  "P004"  "P005"  "P007")

# RUN OFFICE
#for i in "${!dataset_office[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="office" \
#  dataset:="${dataset_office[i]}"
#done

#for i in "${!dataset_carwelding[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="carwelding" \
#  dataset:="${dataset_carwelding[i]}" \
#  verbosity:="SILENT"
#done

#for i in "${!dataset_oldtown[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="oldtown" \
#  dataset:="${dataset_oldtown[i]}" \
#  verbosity:="SILENT"
#done

for i in "${!dataset_neighborhood[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="neighborhood" \
  dataset:="${dataset_neighborhood[i]}" \
  verbosity:="SILENT"
done
