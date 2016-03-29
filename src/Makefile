#
#  Makefile
#  ARToolKit5
#
#  This file is part of ARToolKit.
#
#  ARToolKit is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  ARToolKit is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
AR2_HOME= ..
AR2_INC_DIR= $(AR2_HOME)/include
AR2_LIB_DIR= $(AR2_HOME)/lib

BIN_DIR= ../bin

CC=gcc
CXX=g++ -std=c++11
CFLAG= -O3 -I$(AR2_INC_DIR)
LDFLAG= -L$(AR2_LIB_DIR)
LIBS= -lAR2 -lAR -ljpeg -lKPM

OBJS =
HEADDERS =

all: $(BIN_DIR)/genTexData

$(BIN_DIR)/genTexData: genTexData.o $(OBJS)
	${CXX} -o $(BIN_DIR)/genTexData genTexData.o $(OBJS) $(LDFLAG) $(LIBS)

genTexData.o: genTexData.c $(HEADDERS)
	${CC} -c $(CFLAG) genTexData.c

clean_o:
	rm -f *.o *~ *.bak

clean: clean_o
	rm -f $(BIN_DIR)/genTexData
