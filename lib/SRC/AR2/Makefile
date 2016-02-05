
INC_DIR= ../../../include
LIB_DIR= ../..
#
#  compiler
#
CC=gcc
CFLAGS=-O3 -I$(INC_DIR)
#
# For making the library
#
AR=ar
ARFLAGS=rs

#
#   products
#
LIB= ${LIB_DIR}/libAR2.a

INCLUDE= \
         ${INC_DIR}/AR2/config.h \
         ${INC_DIR}/AR2/coord.h \
         ${INC_DIR}/AR2/featureSet.h \
         ${INC_DIR}/AR2/imageFormat.h \
         ${INC_DIR}/AR2/imageSet.h \
         ${INC_DIR}/AR2/marker.h \
         ${INC_DIR}/AR2/searchPoint.h \
         ${INC_DIR}/AR2/template.h \
         ${INC_DIR}/AR2/tracking.h \
         ${INC_DIR}/AR2/util.h

#
#   compilation control
#
LIBOBJS= \
	 ${LIB}(handle.o) \
	 ${LIB}(imageSet.o) \
	 ${LIB}(jpeg.o) \
	 ${LIB}(marker.o) \
	 ${LIB}(featureMap.o) \
	 ${LIB}(featureSet.o) \
	 ${LIB}(selectTemplate.o) \
	 ${LIB}(surface.o) \
	 ${LIB}(tracking.o) \
	 ${LIB}(tracking2d.o) \
	 ${LIB}(matching.o) \
	 ${LIB}(matching2.o) \
	 ${LIB}(template.o) \
	 ${LIB}(searchPoint.o) \
	 ${LIB}(coord.o) \
	 ${LIB}(util.o)


all:	${LIBOBJS}

${LIBOBJS}:	${INCLUDE}

.c.a:
	${CC} -c ${CFLAGS} $<
	${AR} ${ARFLAGS} $@ $*.o
	rm -f $*.o

clean:
	rm -f *.o *~
	rm -f ${LIB}
