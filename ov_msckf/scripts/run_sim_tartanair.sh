#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
echo $SCRIPT_DIR
source ${SCRIPT_DIR}/../../../../devel/setup.bash
echo ${SCRIPT_DIR}/../../../../devel/setup.bash

dataset_carwelding_easy=("P001" "P002" "P004" "P005" "P006" "P007")
dataset_carwelding_hard=("P000" "P001" "P002" "P003")

dataset_abandonedfactory_easy=("P000" "P001" "P002" "P004" "P005" "P006" "P008" "P009" "P010" "P011")
dataset_abandonedfactory_hard=("P000" "P001" "P002" "P003" "P004" "P005" "P006" "P007" "P008" "P009" "P010" "P011")

dataset_office_easy=("P000"  "P001"  "P002"  "P003"  "P004"  "P005"  "P006")
dataset_office_hard=("P000"  "P001"  "P002"  "P003"  "P004"  "P005"  "P006" "P007")

dataset_office2_easy=("P000" "P003" "P004" "P005" "P006" "P007" "P008" "P009" "P010" "P011")
dataset_office2_hard=("P000" "P001" "P002" "P003" "P004" "P005" "P006" "P007" "P008" "P009" "P010")


#for i in "${!dataset_carwelding_hard[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="carwelding" \
#  data_level:="Hard" \
#  dataset:="${dataset_carwelding_hard[i]}" \
#  verbosity:="SILENT"
#done
#
#for i in "${!dataset_abandonedfactory_hard[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="abandonedfactory" \
#  data_level:="Hard" \
#  dataset:="${dataset_abandonedfactory_hard[i]}" \
#  verbosity:="SILENT"
#done
#
#for i in "${!dataset_office_hard[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="office" \
#  data_level:="Hard" \
#  dataset:="${dataset_office_hard[i]}" \
#  verbosity:="SILENT"
#done
#
#for i in "${!dataset_office2_hard[@]}"; do
#roslaunch ov_msckf sim_tartanair.launch \
#  data_scenario:="office2" \
#  data_level:="Hard" \
#  dataset:="${dataset_office2_hard[i]}" \
#  verbosity:="SILENT"
#done

for i in "${!dataset_abandonedfactory_easy[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="abandonedfactory" \
  data_level:="Easy" \
  dataset:="${dataset_abandonedfactory_easy[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_abandonedfactory_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="abandonedfactory" \
  data_level:="Hard" \
  dataset:="${dataset_abandonedfactory_hard[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_carwelding_easy[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="carwelding" \
  data_level:="Easy" \
  dataset:="${dataset_carwelding_easy[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_carwelding_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="carwelding" \
  data_level:="Hard" \
  dataset:="${dataset_carwelding_hard[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_office_easy[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="office" \
  data_level:="Easy" \
  dataset:="${dataset_office_easy[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_office_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="office" \
  data_level:="Hard" \
  dataset:="${dataset_office_hard[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_office2_easy[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="office2" \
  data_level:="Easy" \
  dataset:="${dataset_office2_easy[i]}" \
  verbosity:="SILENT"
done

for i in "${!dataset_office2_hard[@]}"; do
roslaunch ov_msckf sim_tartanair.launch \
  data_scenario:="office2" \
  data_level:="Hard" \
  dataset:="${dataset_office2_hard[i]}" \
  verbosity:="SILENT"
done