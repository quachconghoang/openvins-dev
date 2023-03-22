#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo $SCRIPT_DIR
source ${SCRIPT_DIR}/../../../../devel/setup.bash
echo ${SCRIPT_DIR}/../../../../devel/setup.bash

#dataset_carwelding=("P001"  "P002"  "P004"  "P005"  "P006"  "P007")
#dataset_neighborhood=("P000"  "P001"  "P002"  "P003"  "P004"  "P005")
#dataset_office=("P000"  "P001"  "P002"  "P003"  "P004"  "P005"  "P006")
#dataset_oldtown=("P000"  "P001"  "P002"  "P004"  "P005"  "P007")

#for i in "${!dataset_carwelding[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="carwelding" \
#  dataset:="${dataset_carwelding[i]}" \
#  verbosity:="SILENT"
#done
#
#for i in "${!dataset_neighborhood[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="neighborhood" \
#  dataset:="${dataset_neighborhood[i]}" \
#  verbosity:="SILENT"
#done
#
#for i in "${!dataset_office[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="office" \
#  dataset:="${dataset_office[i]}" \
#  verbosity:="SILENT"
#done
#
#for i in "${!dataset_oldtown[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="oldtown" \
#  dataset:="${dataset_oldtown[i]}" \
#  verbosity:="SILENT"
#done

dataset_carwelding_hard=("P000"   "P001"  "P002"  "P003")
dataset_neighborhood_hard=("P000"  "P001"  "P002"  "P003"  "P004"  "P005"  "P006")
dataset_office_hard=("P000"  "P001"  "P002"  "P003"  "P004"  "P005"  "P006" "P007")
dataset_oldtown_hard=("P000"  "P001"  "P002"   "P003"  "P004"  "P005"  "P006"  "P007" "P008")

for i in "${!dataset_carwelding_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="carwelding" \
  data_level:="Hard" \
  dataset:="${dataset_carwelding_hard[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_neighborhood_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="neighborhood" \
  data_level:="Hard" \
  dataset:="${dataset_neighborhood_hard[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_office_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="office" \
  data_level:="Hard" \
  dataset:="${dataset_office_hard[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_oldtown_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="oldtown" \
  data_level:="Hard" \
  dataset:="${dataset_oldtown_hard[i]}" \
  verbosity:="SILENT"
done
