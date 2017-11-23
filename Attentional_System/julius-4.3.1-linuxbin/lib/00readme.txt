--------------
These are static libraries of Juliuslib.

They were compiled with default configuration (i.e. no argument for
configure script).  If you want to enable other setting,
ex. decoder-based VAD or GMM-based VAD, you should compile them
from source code by your own.

--------------
How to link an application with juliuslib:
 (${dir} should be the parent directory)

1) see sample code in "julius-simple" directory.

2) Add the include directory to the search path, and
   set compiler flag as obtained by "${dir}/bin/libsent-config
   --cflags" and "${dir}/bin/libjulius-config --cflags".

   For example, when compile with gcc on Linux, do like this:

	gcc \
	    -I${dir}/include \
	    `${dir}/bin/libsent-config --cflags` \
	    `${dir}/bin/libjulius-config --cflags` \
	    -o obj src.c

3) Add the library directory to the library search path, and
   link both the libjulius.a and libsent.a.
   Set linker flag as obtained by "${dir}/bin/libsent-config
   --libs" and "${dir}/bin/libjulius-config --libs".

   For example,

        gcc \
	    -L${dir}/lib \
	    -ljulius -lsent \
	    `${dir}/bin/libsent-config --cflags` \
	    `${dir}/bin/libjulius-config --cflags` \
	    -o bin src.o

