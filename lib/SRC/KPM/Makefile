
UNAME = $(shell uname)

AR_HOME = ../../..
AR_CPPFLAGS = -I$(AR_HOME)/include -I$(AR_HOME)/lib/SRC/KPM/FreakMatcher

CC=cc
CXX=g++ -std=c++11
CPPFLAGS = $(AR_CPPFLAGS)
CFLAGS = -O3
CXXFLAGS = -O3
AR=ar
ARFLAGS=rs
RANLIB = true

TARGET = $(AR_HOME)/lib/libKPM.a

HEADERS = \
	$(AR_HOME)/include/KPM/kpm.h      \
	$(AR_HOME)/include/KPM/kpmType.h  \
	kpmFopen.h                        \
    kpmPrivate.h                      \
	FreakMatcher/detectors/DoG_scale_invariant_detector.h  \
	FreakMatcher/detectors/gaussian_scale_space_pyramid.h  \
	FreakMatcher/detectors/gradients.h  \
	FreakMatcher/detectors/harris-inline.h  \
	FreakMatcher/detectors/harris.h  \
	FreakMatcher/detectors/interpolate.h  \
	FreakMatcher/detectors/orientation_assignment.h  \
	FreakMatcher/detectors/pyramid-inline.h  \
	FreakMatcher/detectors/pyramid.h  \
	FreakMatcher/facade/visual_database_facade.h  \
	FreakMatcher/framework/date_time.h  \
	FreakMatcher/framework/error.h  \
	FreakMatcher/framework/exception.h  \
	FreakMatcher/framework/image.h  \
	FreakMatcher/framework/image_utils.h  \
	FreakMatcher/framework/logger.h  \
	FreakMatcher/framework/timers.h  \
	FreakMatcher/homography_estimation/homography_solver.h  \
	FreakMatcher/homography_estimation/robust_homography.h  \
	FreakMatcher/matchers/binary_hierarchical_clustering.h  \
	FreakMatcher/matchers/feature_matcher-inline.h  \
	FreakMatcher/matchers/feature_matcher.h  \
	FreakMatcher/matchers/feature_point.h  \
	FreakMatcher/matchers/feature_store.h  \
	FreakMatcher/matchers/freak.h  \
	FreakMatcher/matchers/freak84-inline.h  \
	FreakMatcher/matchers/hough_similarity_voting.h  \
	FreakMatcher/matchers/keyframe.h  \
	FreakMatcher/matchers/kmedoids.h  \
	FreakMatcher/matchers/matcher_types.h  \
	FreakMatcher/matchers/visual_database-inline.h  \
	FreakMatcher/matchers/visual_database.h  \
	FreakMatcher/matchers/visual_database_types.h  \
	FreakMatcher/math/cholesky.h  \
	FreakMatcher/math/cholesky_linear_solvers.h  \
	FreakMatcher/math/geometry.h  \
	FreakMatcher/math/hamming.h  \
	FreakMatcher/math/homography.h  \
	FreakMatcher/math/indexing.h  \
	FreakMatcher/math/linear_algebra.h  \
	FreakMatcher/math/linear_solvers.h  \
	FreakMatcher/math/math_utils.h  \
	FreakMatcher/math/matrix.h  \
	FreakMatcher/math/polynomial.h  \
	FreakMatcher/math/quaternion.h  \
	FreakMatcher/math/rand.h  \
	FreakMatcher/math/robustifiers.h  \
	FreakMatcher/utils/feature_drawing.h  \
	FreakMatcher/utils/partial_sort.h  \
	FreakMatcher/utils/point.h  \


OBJS = \
	kpmHandle.o      \
	kpmRefDataSet.o  \
	kpmMatching.o    \
	kpmResult.o      \
	kpmUtil.o        \
	kpmFopen.o       \
	FreakMatcher/detectors/DoG_scale_invariant_detector.o  \
	FreakMatcher/detectors/gaussian_scale_space_pyramid.o  \
	FreakMatcher/detectors/gradients.o  \
	FreakMatcher/detectors/harris.o  \
	FreakMatcher/detectors/orientation_assignment.o  \
	FreakMatcher/detectors/pyramid.o  \
	FreakMatcher/facade/visual_database_facade.o  \
	FreakMatcher/matchers/hough_similarity_voting.o  \
	FreakMatcher/matchers/freak.o  \
	FreakMatcher/framework/date_time.o  \
	FreakMatcher/framework/image.o  \
	FreakMatcher/framework/logger.o  \
	FreakMatcher/framework/timers.o  \


# Implicit rule, to compile C++ files with the .cpp suffix (rule already exists for .cc).
%.o : %.cpp
	$(CXX) -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

default build all: $(TARGET)

$(OBJS): $(HEADERS)

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) $@ $^
	$(RANLIB) $@ 

clean:
	-rm -f *.o *~ *.bak
	-rm $(TARGET)

