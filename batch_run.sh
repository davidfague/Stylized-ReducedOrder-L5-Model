#!/bin/bash

#SBATCH -N 1
#SBATCH -n 60
#SBATCH --qos=normal
#SBATCH --job-name=CA1
#SBATCH --output=CA1.out
#SBATCH --time 0-12:00

START=$(date)
nrngui microcircuithoc
END=$(date)

{ printf "Start: $START \nEnd:   $END\n"}| mail -r jivhr5@umsystem.edu -s "Detailed L5 Results"

echo "Done running model at $(date)"