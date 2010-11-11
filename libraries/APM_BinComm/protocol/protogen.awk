#
# Process the protocol specification and emit functions to pack and unpack buffers.
#
# See protocol.def for a description of the definition format.
#

BEGIN {
    printf("//\n// THIS FILE WAS AUTOMATICALLY GENERATED - DO NOT EDIT\n//\n")
    printf("/// @file protocol.h\n")
    printf("#pragma pack(push)\n");
    printf("#pragma pack(1)\n");
    
    currentMessage = ""
    structureCount = 0
}

END {
    # finalise the last message definition
    EMIT_MESSAGE()

    #
    # emit the MessageID enum
    #
    # XXX it would be elegant to sort the array here, but not
    #     everyone has GNU awk.
    #
    printf("\n//////////////////////////////////////////////////////////////////////\n")
    printf("/// Message ID values\n")
    printf("enum MessageID {\n")
    for (opcode in opcodes) {
	printf("\t%s = 0x%x,\n", opcodes[opcode], opcode)
    }
    printf("\tMSG_ANY = 0xfe,\n")
    printf("\tMSG_NULL = 0xff\n")
    printf("};\n")    

    printf("\n//////////////////////////////////////////////////////////////////////\n")
    printf("/// Message buffer sizing\n")
    printf("union _binCommBufferSizer {\n");
    for (i = 0; i < structureCount; i++) {
	printf("\tstruct %s %s;\n", structs[i], structs[i]);
    }
    printf("};\n");
    printf("#define BINCOMM_MAX_MESSAGE_SIZE sizeof(union _binCommBufferSizer)\n\n");

    printf("#pragma pack(pop)\n")
}

#
# Emit definitions for one message
#
function EMIT_MESSAGE(payloadSize)
{
    if (currentMessage != "") {
	printf("\n//////////////////////////////////////////////////////////////////////\n")
	printf("/// @name %s \n//@{\n\n", currentMessage)

	#
	# emit a structure defining the message payload
	#
	printf("/// Structure describing the payload section of the %s message\n", currentMessage)
	printf("struct %s {\n", tolower(currentMessage))
	for (i = 0; i < fieldCount; i++) {
	    printf("\t%s %s", types[i], names[i])
	    if (counts[i])
		printf("[%s]", counts[i])
	    printf(";\n")
	}
	printf("};\n\n")
	
	#
	# Record the structure name for later use sizing the receive buffer.
	#
	structs[structureCount] = tolower(currentMessage);
	structureCount++;

	#
	# emit a routine to emit the message payload from a set of variables
	#
	printf("/// Send a %s message\n", currentMessage)
	printf("inline void\nsend_%s(\n", tolower(currentMessage))
	for (i = 0; i < fieldCount; i++) {
	    if (counts[i]) {
		printf("\tconst %s (&%s)[%d]", types[i], names[i], counts[i])
	    } else {
		printf("\tconst %s %s", types[i], names[i])
	    }
	    if (i < (fieldCount -1))
		printf(",\n");
	}	
	printf(")\n{\n")
	printf("\t_startMessage(%s,", currentMessage);
	for (i = 0; i < fieldCount; i++) {
	    if (counts[i]) {
		printf("\n\t\t(sizeof(%s[0]) * %d) +", names[i], counts[i]);
	    } else {
		printf("\n\t\tsizeof(%s) +", names[i]);
	    }
	}
	printf(" 0);\n");
	for (i = 0; i < fieldCount; i++) {
	    if (counts[i]) {
		printf("\t_emit(%s, %s);\n", names[i], counts[i])
	    } else {
		printf("\t_emit(%s);\n", names[i])
	    }
	}
	printf("\t_endMessage();\n")
	printf("};\n\n")

	#
	# emit a routine to unpack the current message into a set of variables
	#
	printf("/// Unpack a %s message\n", currentMessage)
	printf("inline void\nunpack_%s(\n", tolower(currentMessage))
	for (i = 0; i < fieldCount; i++) {
	    if (counts[i]) {
		printf("\t%s (&%s)[%d]", types[i], names[i], counts[i])
	    } else {
		printf("\t%s &%s", types[i], names[i])
	    }
	    if (i < (fieldCount -1))
		printf(",\n");
	}	
	printf(")\n{\n")
	printf("\tuint8_t *__p = &_decodeBuf.payload[0];\n")
	for (i = 0; i < fieldCount; i++) {
	    if (counts[i]) {
		printf("\t_unpack(__p, %s, %s);\n", names[i], counts[i])
	    } else {
		printf("\t_unpack(__p, %s);\n", names[i])
	    }
	}
	printf("};\n")

	# close the Doxygen group
	printf("//@}\n")
    }
}

# skip lines containing comments
$1=="#" {
    next
}

#
# process a new message declaration
#
$1=="message" {
    
    # emit any previous message
    EMIT_MESSAGE()

    # save the current opcode and message name
    currentOpcode = $2
    currentMessage = $3
    opcodes[$2] = $3

    # set up for the coming fields
    fieldCount = 0
    delete types
    delete names
    delete counts

    next
}

#
# process a field inside a message definition
#
NF >= 2 {
    
    # save the field definition
    types[fieldCount] = $1
    names[fieldCount] = $2

    # if an array size was supplied, save it
    if (NF >= 3) {
	counts[fieldCount] = $3
    }

    fieldCount++
}
