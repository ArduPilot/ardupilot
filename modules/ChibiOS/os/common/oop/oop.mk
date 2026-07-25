# OOP subsystem build.

# Required files.
ifndef OOPSRC
  ifndef OOPSELECT
    # If none is specified then including only oop_base_object.c by default.
    OOPSRC = ${CHIBIOS}/os/common/oop/src/oop_base_object.c
  else
    ifneq ($(filter base,$(OOPSELECT)),)
      OOPSRC += ${CHIBIOS}/os/common/oop/src/oop_base_object.c
    endif
    ifneq ($(filter referenced,$(OOPSELECT)),)
      OOPSRC += ${CHIBIOS}/os/common/oop/src/oop_referenced_object.c
    endif
    ifneq ($(filter synchronized,$(OOPSELECT)),)
      OOPSRC += ${CHIBIOS}/os/common/oop/src/oop_synchronized_object.c
    endif
    ifneq ($(filter memstreams,$(OOPSELECT)),)
      OOPSRC += ${CHIBIOS}/os/common/oop/src/oop_memstreams.c
    endif
    ifneq ($(filter nullstreams,$(OOPSELECT)),)
      OOPSRC += ${CHIBIOS}/os/common/oop/src/oop_nullstreams.c
    endif
    ifneq ($(filter chprintf,$(OOPSELECT)),)
      OOPSRC += ${CHIBIOS}/os/common/oop/src/oop_chprintf.c
    endif
    ifneq ($(filter chscanf,$(OOPSELECT)),)
      OOPSRC += ${CHIBIOS}/os/common/oop/src/oop_chscanf.c
    endif
  endif
  ALLCSRC += $(OOPSRC)
endif

# Required include directories
ifndef OOPINC
  OOPINC = ${CHIBIOS}/os/common/oop/include
  ALLINC += $(OOPINC)
endif
