#!/bin/sh
#
#### default directory to install
prefix="/usr/local"

echo
echo "--- Julius binary package installation script ---"
echo
echo "Please enter the target root directory: (enter to accept default)"
echo
echo -n "[${prefix}] "
read ANS
if [ -z $ANS ]; then
    dir=$prefix
else
    dir=$ANS
fi
echo
echo "Going to install"
echo
echo " executables to \"${dir}/bin\""
echo " libraries   to \"${dir}/lib\""
echo " headers     to \"${dir}/include\""
echo " manuals     to \"${dir}/man\""
echo
echo -n "Are you sure ? [y/N] "
read ANS
if [ -z $ANS ]; then
    echo process terminated
elif [ $ANS = 'Y' -o $ANS = 'y' -o $ANS = 'yes' ]; then
    mkdir -p ${dir}/bin ${dir}/lib ${dir}/include ${dir}/man ${dir}/man/man1
    echo copying binaries...
    cp -p bin/* ${dir}/bin
    echo copying libraries...
    cp -p lib/libsent.a lib/libjulius.a ${dir}/lib
    echo copying headers...
    cp -pr include/julius include/sent ${dir}/include
    echo copying manuals...
    cp -pr man/* ${dir}/man/man1
    echo setting libsent-config and libjulius-config...
    perl ./support/setprefix.pl ${dir} bin/libsent-config > ${dir}/bin/libsent-config
    perl ./support/setprefix.pl ${dir} bin/libjulius-config > ${dir}/bin/libjulius-config
    echo installation done
else
    echo process terminated
fi
