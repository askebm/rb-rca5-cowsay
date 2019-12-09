#!/bin/sh
DIR=$(readlink -f $(dirname $0))

PATH_TO_BIN=${1}

lambda="70"
episodes="10000"
nodes="20"
name="circle"
frequency="100"

FILE="parallelJobs.sh"
FOLDER="$(date)"

mkdir -p "${FOLDER}"

rm ${FILE}

for epsilon in $(seq 10 2 100); do
	for alpha in $(seq 10 2 100); do
#		for lambda in $(seq -w 10 2 90); do
			echo "${PATH_TO_BIN} ${name} ${nodes} ${episodes} ${lambda} ${epsilon} ${alpha} ${frequency}"\
				' > '"'""${FOLDER}""'""/test_${name}_episodes_${episodes}_nodes_${nodes}_lambda_${lambda}_epsilon_${epsilon}_alpha_${alpha}_freq_${frequency}.csv"\
				>> ${FILE}
#		done
	done
done
