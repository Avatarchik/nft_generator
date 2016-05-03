UNAME = $(shell uname)

AR_HOME = ../..
AR_CPPFLAGS = -I$(AR_HOME)/include

CC=gcc
CXX=g++ -std=c++11 -stdlib=libc++
CPPFLAGS = $(AR_CPPFLAGS)
CFLAGS = -O3
CXXFLAGS = -O3
AR=ar
ARFLAGS=rs
RANLIB = true

TARGET = $(AR_HOME)/lib/libAR.a

HEADERS = \
arLabelingSub/arLabelingPrivate.h \
arLabelingSub/arLabelingSub.h \
$(AR_HOME)/include/AR/ar.h \
$(AR_HOME)/include/AR/config.h \
$(AR_HOME)/include/AR/arConfig.h \
$(AR_HOME)/include/AR/matrix.h \
$(AR_HOME)/include/AR/icp.h \
$(AR_HOME)/include/AR/icpCore.h \
$(AR_HOME)/include/AR/matrix.h \
$(AR_HOME)/include/AR/param.h \
$(AR_HOME)/include/AR/arImageProc.h \
$(AR_HOME)/include/AR/arFilterTransMat.h \


OBJS = \
mAlloc.o \
mFree.o \
mAllocDup.o \
mDup.o \
mAllocTrans.o \
mTrans.o \
mAllocMul.o \
mMul.o \
mAllocInv.o \
mInv.o \
mSelfInv.o \
mAllocUnit.o \
mUnit.o \
mDisp.o \
mDet.o \
mPCA.o \
vAlloc.o \
vDisp.o \
vFree.o \
vHouse.o \
vInnerP.o \
vTridiag.o \
paramGetPerspective.o \
paramClear.o \
paramDecomp.o \
paramDistortion.o \
paramChangeSize.o \
paramFile.o \
paramLT.o \
paramDisp.o \
ar3DCreateHandle.o \
ar3DUtil.o \
arCreateHandle.o \
arDetectMarker.o \
arDetectMarker2.o \
arFilterTransMat.o \
arGetLine.o \
arGetMarkerInfo.o \
arGetTransMat.o \
arGetTransMatStereo.o \
arImageProc.o \
arLabeling.o \
arLabelingSub/arLabelingSubDBI3C.o \
arLabelingSub/arLabelingSubDBI3C565.o \
arLabelingSub/arLabelingSubDBI3CA.o \
arLabelingSub/arLabelingSubDBI3CA4444.o \
arLabelingSub/arLabelingSubDBI3CA5551.o \
arLabelingSub/arLabelingSubDBIA3C.o \
arLabelingSub/arLabelingSubDBIC.o \
arLabelingSub/arLabelingSubDBICY.o \
arLabelingSub/arLabelingSubDBIYC.o \
arLabelingSub/arLabelingSubDBR3C.o \
arLabelingSub/arLabelingSubDBR3C565.o \
arLabelingSub/arLabelingSubDBR3CA.o \
arLabelingSub/arLabelingSubDBR3CA4444.o \
arLabelingSub/arLabelingSubDBR3CA5551.o \
arLabelingSub/arLabelingSubDBRA3C.o \
arLabelingSub/arLabelingSubDBRC.o \
arLabelingSub/arLabelingSubDBRCY.o \
arLabelingSub/arLabelingSubDBRYC.o \
arLabelingSub/arLabelingSubDBZ.o \
arLabelingSub/arLabelingSubDWI3C.o \
arLabelingSub/arLabelingSubDWI3C565.o \
arLabelingSub/arLabelingSubDWI3CA.o \
arLabelingSub/arLabelingSubDWI3CA4444.o \
arLabelingSub/arLabelingSubDWI3CA5551.o \
arLabelingSub/arLabelingSubDWIA3C.o \
arLabelingSub/arLabelingSubDWIC.o \
arLabelingSub/arLabelingSubDWICY.o \
arLabelingSub/arLabelingSubDWIYC.o \
arLabelingSub/arLabelingSubDWR3C.o \
arLabelingSub/arLabelingSubDWR3C565.o \
arLabelingSub/arLabelingSubDWR3CA.o \
arLabelingSub/arLabelingSubDWR3CA4444.o \
arLabelingSub/arLabelingSubDWR3CA5551.o \
arLabelingSub/arLabelingSubDWRA3C.o \
arLabelingSub/arLabelingSubDWRC.o \
arLabelingSub/arLabelingSubDWRCY.o \
arLabelingSub/arLabelingSubDWRYC.o \
arLabelingSub/arLabelingSubDWZ.o \
arLabelingSub/arLabelingSubEBI3C.o \
arLabelingSub/arLabelingSubEBI3C565.o \
arLabelingSub/arLabelingSubEBI3CA.o \
arLabelingSub/arLabelingSubEBI3CA4444.o \
arLabelingSub/arLabelingSubEBI3CA5551.o \
arLabelingSub/arLabelingSubEBIA3C.o \
arLabelingSub/arLabelingSubEBIC.o \
arLabelingSub/arLabelingSubEBICY.o \
arLabelingSub/arLabelingSubEBIYC.o \
arLabelingSub/arLabelingSubEBR3C.o \
arLabelingSub/arLabelingSubEBR3C565.o \
arLabelingSub/arLabelingSubEBR3CA.o \
arLabelingSub/arLabelingSubEBR3CA4444.o \
arLabelingSub/arLabelingSubEBR3CA5551.o \
arLabelingSub/arLabelingSubEBRA3C.o \
arLabelingSub/arLabelingSubEBRC.o \
arLabelingSub/arLabelingSubEBRCY.o \
arLabelingSub/arLabelingSubEBRYC.o \
arLabelingSub/arLabelingSubEBZ.o \
arLabelingSub/arLabelingSubEWI3C.o \
arLabelingSub/arLabelingSubEWI3C565.o \
arLabelingSub/arLabelingSubEWI3CA.o \
arLabelingSub/arLabelingSubEWI3CA4444.o \
arLabelingSub/arLabelingSubEWI3CA5551.o \
arLabelingSub/arLabelingSubEWIA3C.o \
arLabelingSub/arLabelingSubEWIC.o \
arLabelingSub/arLabelingSubEWICY.o \
arLabelingSub/arLabelingSubEWIYC.o \
arLabelingSub/arLabelingSubEWR3C.o \
arLabelingSub/arLabelingSubEWR3C565.o \
arLabelingSub/arLabelingSubEWR3CA.o \
arLabelingSub/arLabelingSubEWR3CA4444.o \
arLabelingSub/arLabelingSubEWR3CA5551.o \
arLabelingSub/arLabelingSubEWRA3C.o \
arLabelingSub/arLabelingSubEWRC.o \
arLabelingSub/arLabelingSubEWRCY.o \
arLabelingSub/arLabelingSubEWRYC.o \
arLabelingSub/arLabelingSubEWZ.o \
arPattAttach.o \
arPattCreateHandle.o \
arPattGetID.o \
arPattLoad.o \
arPattSave.o \

ifneq "$(UNAME)" "Darwin"
OBJS += arUtil.o
endif

ifeq "$(UNAME)" "Darwin"
OBJS_OBJC = arUtil.o
endif


# Implicit rule, to compile C++ files with the .cpp suffix (rule already exists for .cc).
%.o : %.cpp
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

default build all: $(TARGET)

$(OBJS_OBJC): $(HEADERS)
	$(CC) $(CFLAGS) $(CPPFLAGS)  -c -ObjC -o $@ $*.c

$(OBJS): $(HEADERS)

$(TARGET): $(OBJS) $(OBJS_OBJC)
	$(AR) $(ARFLAGS) $@ $^
	$(RANLIB) $@

clean:
	-rm -f *.o *~ *.bak arLabelingSub/*.o arLabelingSub/*.bak
	-rm $(TARGET)
