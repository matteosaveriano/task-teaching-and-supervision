#!/usr/bin/perl

$str = shift;

while(<>) {
    if (/^prefix=/) {
	print "prefix=\"$str\"\n";
    } else {
	print;
    }
}
