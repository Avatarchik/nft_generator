all:
	(cd lib;       make -f Makefile)
	(cd src;       make -f Makefile)
	(cd lib;       make -f Makefile clean)
	(cd src;       make -f Makefile clean_o)

clean:
	(cd lib;       make -f Makefile clean)
	(cd src;       make -f Makefile clean)
	rm -f *~ *.bak
