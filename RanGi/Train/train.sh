#!/bin/bash

file='/practice'
extension='.xml'

cores=4
for ((i=0; i < 10; i++))
do
    echo "Training RanGi $i"
    torcs -r ${PWD}${file}${i}${extension} &

    # Check how many background jobs there are, and if it
    # is equal to the number of cores, wait for anyone to
    # finish before continuing.
    background=( $(jobs -p) )
    if (( ${#background[@]} == cores )); then
        wait -n
    fi
done

echo "Training done"
