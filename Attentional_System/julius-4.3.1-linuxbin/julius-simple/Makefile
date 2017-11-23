CC=gcc
CFLAGS=-g -O2 

PKGBIN=../bin

# if you want to use system-installed Julius libraries,
# delete all "$(PKGBIN)/" in the followings.

CPPFLAGS=`$(PKGBIN)/libsent-config --cflags` `$(PKGBIN)/libjulius-config --cflags`
LDFLAGS= `$(PKGBIN)/libjulius-config --libs` `$(PKGBIN)/libsent-config --libs` 

############################################################

all: julius-simple

julius-simple: julius-simple.c
	$(CC) $(CFLAGS) $(CPPFLAGS) -o julius-simple julius-simple.c $(LDFLAGS)

clean:
	$(RM) *.o *.bak *~ core TAGS

distclean:
	$(RM) *.o *.bak *~ core TAGS
	$(RM) julius-simple
