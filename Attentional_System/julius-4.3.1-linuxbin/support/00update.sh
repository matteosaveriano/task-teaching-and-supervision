#!/bin/sh
#
# Automatic update of binary package from Julius source archive.
# Supports 3.5 and later.
#
# !! Should be executed at "../"
#
# you should specify the target source archive .tar.gz file.
#
#

# expand archive
mv julius-simple aaa
rm -rf julius*
tar xzvf $1
ln -s julius-* julius
rm -rf include/sent include/julius
rm -rf man
mv aaa julius-simple

# update binaries
cd julius
sh ./support/build-all.sh
mv build/bin/* ../bin
mv build/lib/* ../lib
mv build/include/* ../include
mv julius-simple/julius-simple.c ../julius-simple
cd ..
# normalize libsent/libjulius directory for julius-simple
perl ./support/setprefix.pl .. bin/libsent-config > tmp
mv tmp bin/libsent-config
chmod +x bin/libsent-config
perl ./support/setprefix.pl .. bin/libjulius-config > tmp
mv tmp bin/libjulius-config
chmod +x bin/libjulius-config

# copy the documents
cp -p julius/00readme.txt 00readme-julius.txt
cp -p julius/00readme-ja.txt 00readme-julius-ja.txt
cp -p julius/LICENSE.txt .
cp -p julius/Release* .
cp -p julius/Sample*jconf* .

# copy manuals
cp -pr julius/man .
rm -f man/Makefile.in

# finished
rm -f julius
echo '#### FINISHED, you should remove julius-ver directory manually ###'
echo '#### Also copy "plugin" dir to "plugin-sample" and modify the Makefile'
echo '#### Also delete old binaries in bin.'
