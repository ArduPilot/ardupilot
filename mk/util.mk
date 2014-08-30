#
# The sketch splitter is an awk script used to split off the
# header and body of the concatenated .pde/.ino files.  It also
# inserts #line directives to help in backtracking from compiler
# and debugger messages to the original source file.
#
# Note that # and $ require special treatment here to avoid upsetting
# make.
#
# This script requires BWK or GNU awk.
#
define SKETCH_SPLITTER
  BEGIN { 							\
    scanning = 1; 						\
    printing = (mode ~ "header") ? 1 : 0;			\
  }								\
  { toggles = 1 }						\
  (FNR == 1) && printing { 					\
    printf "#line %d \"%s\"\n", FNR, FILENAME;			\
  }								\
  /^[[:space:]]*\/\*/,/\*\// { 					\
    toggles = 0;						\
  }								\
  /^[[:space:]]*$$/ || /^[[:space:]]*\/\/.*/ || /^\#.*$$/ { 	\
    toggles = 0;						\
  }								\
  scanning && toggles { 					\
    scanning = 0; 						\
    printing = !printing;					\
    if (printing) { 						\
      printf "#line %d \"%s\"\n", FNR, FILENAME;		\
    }								\
  }								\
  printing
endef


#
# The prototype scanner is an awk script used to generate function
# prototypes from the concantenated .pde/.ino files.
#
# Function definitions are expected to follow the form
#
#   <newline><type>[<qualifier>...]<name>([<arguments>]){
#
# with whitespace permitted between the various elements.  The pattern
# is assembled from separate subpatterns to make adjustments easier.
#
# Note that $ requires special treatment here to avoid upsetting make,
# and backslashes are doubled in the partial patterns to satisfy
# escaping rules.
#
# This script requires BWK or GNU awk.
#
define SKETCH_PROTOTYPER
  BEGIN {								\
    RS="{";								\
    type       = "((\\n)|(^))[[:space:]]*[[:alnum:]_]+[[:space:]]+";	\
    qualifiers = "([[:alnum:]_\\*&]+[[:space:]]*)*";			\
    name       = "[[:alnum:]_]+[[:space:]]*";				\
    args       = "\\([[:space:][:alnum:]_,&\\*\\[\\]]*\\)";		\
    bodycuddle = "[[:space:]]*$$";					\
    pattern    = type qualifiers name args bodycuddle;			\
  }									\
  match($$0, pattern) {							\
    proto = substr($$0, RSTART, RLENGTH);				\
    gsub("\n", " ", proto);						\
    printf "%s;\n", proto;						\
  }
endef

# common header for rules, prints what is being built
define RULEHDR
	@echo %% $(subst $(BUILDROOT)/,,$@)
	@mkdir -p $(dir $@)
endef