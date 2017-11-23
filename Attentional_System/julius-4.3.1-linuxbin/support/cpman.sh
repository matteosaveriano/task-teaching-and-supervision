#!/bin/sh
base=`basename $1 .man`
dir=`dirname $1`

cp $1 doc/man/${base}.1

if [ -f ${dir}/00readme.txt ]; then
    cp ${dir}/00readme.txt doc/txt/${base}.txt
fi
if [ -f ${dir}/00readme-${base}.txt ]; then
    cp ${dir}/00readme-${base}.txt doc/txt/${base}.txt
fi

echo $1
