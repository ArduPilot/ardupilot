This directory contains the support files for various board models. If you
want to support a new board:
- Create a new directory under ./os/hal/boards, give it the name of your board.
- Copy inside the new directory the files from a similar board.
- Customize board.c, board.h and board.mk in order to correctly initialize
  your board.

The files in those board directories containing:
- <board>/cfg/board.chcfg
- <board>/cfg/board.fmpp
are generated automatically, just run the "fmpp" tool from within <board>/cfg,
the download is available here: http://fmpp.sourceforge.net, note, it
requires Java.

All board files can be batch-regenerated automatically by running the
genboards.sh script in this same directory.

