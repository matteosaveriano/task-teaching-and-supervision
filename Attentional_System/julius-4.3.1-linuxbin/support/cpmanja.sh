#!/bin/sh
base=`basename $1 .man.ja`
dir=`dirname $1`

cp $1 doc/man.ja/${base}.1

if [ -f ${dir}/00readme-ja.txt ]; then
    cp ${dir}/00readme-ja.txt doc/txt.ja/${base}.txt
fi
if [ -f ${dir}/00readme-${base}-ja.txt ]; then
    cp ${dir}/00readme-${base}-ja.txt doc/txt.ja/${base}.txt
fi

echo $1
