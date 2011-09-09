#
# Process the parameter specification and produce the parameter enumerations and
# name table.
#
# See paramgen.in for details of the input format
#

BEGIN {
    paramIndex = 0
    typeIndex = 0
    firstInType = 0

    printf("//\n// THIS FILE WAS AUTOMATICALLY GENERATED - DO NOT EDIT\n//\n")		> "param_table.h"
    printf("/// @file param_table.h\n\n")						> "param_table.h"

    printf("//\n// THIS FILE WAS AUTOMATICALLY GENERATED - DO NOT EDIT\n//\n")		> "param_init.pde"
    printf("/// @file param_init.pde\n\n")						> "param_init.pde"
    printf("void param_reset_defaults(void)\n")						> "param_init.pde"
    printf("{\n")									> "param_init.pde"

    printf("//\n// THIS FILE WAS AUTOMATICALLY GENERATED - DO NOT EDIT\n//\n")		> "param_table.c"
    printf("/// @file param_table.c\n\n")						> "param_table.c"
    printf("#pragma pack(push)\n")							> "param_table.c"
    printf("#pragma pack(1)\n\n")							> "param_table.c"
    printf("#include <stdint.h>\n")							> "param_table.c"
    printf("#include <avr/pgmspace.h>\n")						> "param_table.c"
    printf("#include \"global_data.h\"\n\n")						> "param_table.c"
    printf("#define PARAM_NAME(_name)	static const char _param_ ## _name [] PROGMEM = #_name;\n") > "param_table.c"
    printf("#define PARAM_INDEX(_name)	[PARAM_ ## _name] = _param_ ## _name\n\n")	> "param_table.c"


}

function END_ENUM() {
    printf("    PARAM_LAST_%s\n", currentType)						> "param_table.h"
    printf("};\n")									> "param_table.h"
    printf("#define PARAM_%s_COUNT (PARAM_LAST_%s - PARAM_FIRST_%s)\n\n", 
	   currentType, currentType, currentType)					> "param_table.h"
}

#
# skip lines containing comments
#
$1=="#" { next }

#
# process a type section change
#
$1=="type" { 

    newType = toupper($2)

    # if there's a type already open, close it
    if (currentType == "") {
	# first enum opens at index zero
	printf("#define PARAM_FIRST_%s 0\n", newType)					> "param_table.h"
    } else {
	# finalise the preceding enum
	END_ENUM()

	# chain the next enum's starting value off the previous
	printf("#define PARAM_FIRST_%s PARAM_LAST_%s\n", newType, currentType)		> "param_table.h"
    }
    printf("enum %s_param_t {\n", tolower(newType))									> "param_table.h"

    currentType = newType
    firstInType = 1

    next 
}

#
# process a parameter name
#
NF >= 1 {
    
    paramName = $1
    paramInitial = $2

    # emit the parameter inside the enum for param_table.h
    if (firstInType) {
	printf("    PARAM_%s = PARAM_FIRST_%s,\n", paramName, currentType)		> "param_table.h"
	firstInType = 0
    } else {
	printf("    PARAM_%s,\n", paramName)						> "param_table.h"
    }

    # emit the call to the initialiser for param_init.pde
    if (paramInitial != "") {
	printf("    set(PARAM_%s, %s);\n", paramName, paramInitial)			> "param_init.pde"
    }

    # save name for param_table.c
    paramNames[paramIndex] = paramName
    paramIndex++
}

END {
    #
    # close out the current enum
    #
    END_ENUM()
    printf("#define PARAM_COUNT PARAM_LAST_%s\n", currentType)				> "param_table.h"

    #
    # close the initialiser function
    #
    printf("}\n")									> "param_init.pde"


    #
    # Generate param_table.c
    #
    # emit the PARAM_NAME invocations
    for (name in paramNames) {
	printf("PARAM_NAME(%s);\n", paramNames[name])					> "param_table.c"
    }

    # emit the PARAM_INDEX array
    printf("\nconst char *param_nametab[] PROGMEM = {\n")				> "param_table.c"
    for (name in paramNames) {
	printf("    PARAM_INDEX(%s),\n", paramNames[name])				> "param_table.c"
    }
    printf("};\n")									> "param_table.c"
}

