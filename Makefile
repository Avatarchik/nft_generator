all:
	(cd lib/SRC;    make -f Makefile)
	(cd src;       make -f Makefile)
	(cd lib/SRC;    make -f Makefile clean)
	(cd src;       make -f Makefile clean_o)

clean:
	(cd lib/SRC;    make -f Makefile clean)
	(cd src;       make -f Makefile clean)
	rm -f *~ *.bak
