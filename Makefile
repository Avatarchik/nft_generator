all:
	(cd lib/SRC;    make -f Makefile)
	(cd src;       make -f Makefile)

clean:
	(cd lib/SRC;    make -f Makefile clean)
	(cd src;       make -f Makefile clean)
	rm -f *~ *.bak
