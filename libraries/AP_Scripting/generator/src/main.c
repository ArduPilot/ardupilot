#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

char keyword_alias[]               = "alias";
char keyword_rename[]              = "rename";
char keyword_ap_object[]           = "ap_object";
char keyword_comment[]             = "--";
char keyword_depends[]             = "depends";
char keyword_enum[]                = "enum";
char keyword_field[]               = "field";
char keyword_include[]             = "include";
char keyword_method[]              = "method";
char keyword_operator[]            = "operator";
char keyword_read[]                = "read";
char keyword_scheduler_semaphore[] = "scheduler-semaphore";
char keyword_semaphore[]           = "semaphore";
char keyword_semaphore_pointer[]   = "semaphore-pointer";
char keyword_singleton[]           = "singleton";
char keyword_userdata[]            = "userdata";
char keyword_write[]               = "write";
char keyword_literal[]             = "literal";
char keyword_reference[]           = "reference";
char keyword_deprecate[]           = "deprecate";
char keyword_manual[]              = "manual";
char keyword_global[]              = "global";
char keyword_creation[]            = "creation";
char keyword_manual_operator[]     = "manual_operator";

// attributes (should include the leading ' )
char keyword_attr_enum[]    = "'enum";
char keyword_attr_literal[] = "'literal";
char keyword_attr_null[]    = "'Null";
char keyword_attr_reference[]  = "'Ref";
char keyword_attr_array[]      = "'array";
char keyword_attr_no_range_check[] = "'skip_check";

// type keywords
char keyword_boolean[]  = "boolean";
char keyword_float[]    = "float";
char keyword_int8_t[]   = "int8_t";
char keyword_int16_t[]  = "int16_t";
char keyword_int32_t[]  = "int32_t";
char keyword_string[]   = "string";
char keyword_uint8_t[]  = "uint8_t";
char keyword_uint16_t[] = "uint16_t";
char keyword_uint32_t[] = "uint32_t";
char keyword_void[]     = "void";

enum error_codes {
  ERROR_OUT_OF_MEMORY   = 1, // ran out of memory
  ERROR_HEADER          = 2, // header keyword not followed by a header to include
  ERROR_UNKNOWN_KEYWORD = 3, // a keyword we didn't know how to handle
  ERROR_USERDATA        = 4, // userdata
  ERROR_INTERNAL        = 5, // internal error of some form
  ERROR_GENERAL         = 6, // general error
  ERROR_SINGLETON       = 7, // singletons
  ERROR_DEPENDS         = 8, // dependencies
  ERROR_DOCS            = 9, // Documentation
  ERROR_GLOBALS         = 10,
};

struct header {
  struct header *next;
  char *name; // name of the header to include (not sanatized)
  int line; // line of the file declared on
  char *dependency;
};

struct generator_state {
  char line[1<<14];
  int line_num; // current line read in
  int token_num; // current token on the current line
  char *token;
};

FILE *description;
FILE *header;
FILE *source;
FILE *docs;

static struct generator_state state;
static struct header * headers;

enum trace_level {
  TRACE_TOKENS    = (1 << 0),
  TRACE_HEADER    = (1 << 1),
  TRACE_GENERAL   = (1 << 2),
  TRACE_USERDATA  = (1 << 3),
  TRACE_SINGLETON = (1 << 4),
  TRACE_DEPENDS   = (1 << 5),
};

enum access_flags {
  ACCESS_FLAG_READ  = (1 << 0),
  ACCESS_FLAG_WRITE = (1 << 1),
};

enum field_type {
  TYPE_BOOLEAN = 0,
  TYPE_FLOAT,
  TYPE_INT8_T,
  TYPE_INT16_T,
  TYPE_INT32_T,
  TYPE_UINT8_T,
  TYPE_UINT16_T,
  TYPE_UINT32_T,
  TYPE_NONE,
  TYPE_STRING,
  TYPE_ENUM,
  TYPE_LITERAL,
  TYPE_USERDATA,
  TYPE_AP_OBJECT
};

const char * type_labels[TYPE_USERDATA + 1] = { "bool",
                                                "float",
                                                "int8_t",
                                                "int16_t",
                                                "int32_t",
                                                "uint8_t",
                                                "uint16_t",
                                                "void",
                                                "string",
                                                "enum",
                                                "userdata",
                                                "ap_object",
                                              };

enum operator_type {
  OP_ADD  = (1U << 0),
  OP_SUB  = (1U << 1),
  OP_MUL  = (1U << 2),
  OP_DIV  = (1U << 3),
  OP_MANUAL = (1U << 4),
  OP_LAST
};

enum access_type {
  ACCESS_VALUE = 0,
  ACCESS_REFERENCE,
};

struct range_check {
  // store the requested range check as a string
  // we will check that it's a numeric form of some type, but keep it as a string rather then a casted version
  char *low;
  char *high;
};

enum type_flags {
  TYPE_FLAGS_NULLABLE = (1U << 1),
  TYPE_FLAGS_ENUM     = (1U << 2),
  TYPE_FLAGS_REFERNCE = (1U << 3),
  TYPE_FLAGS_NO_RANGE_CHECK = (1U << 4),
};

struct type {
  struct range_check *range;
  enum field_type type;
  enum access_type access;
  uint32_t flags;
  union {
    struct ud {
      char *name;
      char *sanatized_name;
    } ud;
    char *enum_name;
    char *literal;
  } data;
};

int TRACE_LEVEL = 0;

void trace(const int trace, const char *message, ...) {
  if (trace & TRACE_LEVEL) {
    char * fmt = malloc(strlen(message)+1024);
    if (fmt == NULL) {
      exit(ERROR_OUT_OF_MEMORY);
    }
  
    sprintf(fmt, "TRACE: %s\n", message);
  
    va_list args;
    va_start(args, message);
    vfprintf(stderr, fmt, args);
    va_end(args);
    free(fmt);
    fmt = NULL;
  }
}

void error(const int code, const char *message, ...) __attribute__ ((noreturn));
void error(const int code, const char *message, ...) {
  char * fmt = malloc(strlen(message)+1024);
  if (fmt == NULL) {
    exit(ERROR_OUT_OF_MEMORY);
  }

  if (state.line_num >= 0) {
    sprintf(fmt, "Error (line %d): %s\n", state.line_num, message);
  } else {
    sprintf(fmt, "Error: %s\n", message);
  }

  va_list args;
  va_start(args, message);
  vfprintf(stderr, fmt, args);
  va_end(args);
  free(fmt);
  fmt = NULL;
  exit(code);
}

char *token_delimiters = " \n";

char * next_token(void) {
  state.token = strtok(NULL, token_delimiters);
  state.token_num++;
  trace(TRACE_TOKENS, "Token %d:%d %s", state.line_num, state.token_num, state.token);
  if ((state.token!= NULL) && (strcmp(state.token, keyword_comment) == 0)) {
    trace(TRACE_TOKENS, "Detected comment %d", state.line_num);
    state.token = NULL; // burn the line
  }
  return state.token;
}

char * start_line(void) {
  while (fgets(state.line, sizeof(state.line)/sizeof(state.line[0]), description) != NULL) {//state.line = readline(NULL))) {
      state.line_num++;
    
      const size_t length = strlen(state.line);
      if (length > 1 && state.line[length - 2] == '\r') {
        trace(TRACE_TOKENS, "Discarding carriage return");
        if (length == 2) { // empty line of just carriage return, loop again
          continue;
        }
        state.line[length - 2] = '\0';
      } else if (length > 0 && state.line[length - 1] == '\n') {
        trace(TRACE_TOKENS, "Discarding newline");
        if (length == 1) { // empty line of just carriage return, loop again
          continue;
        }
        state.line[length - 1] = '\0';
      }

      state.token = strtok(state.line, token_delimiters);
      state.token_num = 1;
      trace(TRACE_TOKENS, "Start of line token %d:%d %s", state.line_num, state.token_num, state.token);

      if (state.token != NULL) {
        break;
      }
  }

  return state.token;
}

// thin wrapper for malloc that exits if we can't allocate memory, and memsets the allocated chunk
void *allocate(const size_t size) {
  void *data = malloc(size);
  if (data == NULL) {
    error(ERROR_OUT_OF_MEMORY, "Out of memory.");
  } else {
    memset(data, 0, size);
  }
  return data;
}

// lazy helper that allocates a storage buffer and does strcpy for us
void string_copy(char **dest, const char * src) {
  *dest = (char *)allocate(strlen(src) + 1);
  strcpy(*dest, src);
}

void handle_header(void) {
  trace(TRACE_HEADER, "Parsing a header");

  // find the new header
  char * name = next_token();
  if (name == NULL) {
    error(ERROR_HEADER, "Header must be followed by the name of the header to include");
  }

  // search for duplicates
  struct header *node = headers;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }
  if (node != NULL) {
    error(ERROR_HEADER, "Header %s was already included on line %d", name, node->line);
  }

  // add to the list of headers
  node = (struct header *)allocate(sizeof(struct header));
  node->next = headers;
  node->line = state.line_num;
  node->name = (char *)allocate(strlen(name) + 1);
  strcpy(node->name, name);

  // add depedns
  char * key_word = next_token();
  char *depends = NULL;
  if (key_word != NULL) {
    if (strcmp(key_word, keyword_depends) == 0) {
        depends = strtok(NULL, "");
        if (depends == NULL) {
          error(ERROR_HEADER, "Expected a depends string for %s", name);
        }
        string_copy(&(node->dependency), depends);
    } else {
      error(ERROR_HEADER, "Received an unsupported keyword on a header: %s", key_word);
    }
  }

  headers = node;

  trace(TRACE_HEADER, "Added header %s", name);

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_HEADER, "Header contained an unexpected extra token: %s", state.token);
  }
}

enum userdata_type {
  UD_USERDATA,
  UD_SINGLETON,
  UD_AP_OBJECT,
  UD_GLOBAL,
};

struct argument {
  struct argument * next;
  struct type type;
  int line_num; // line read from
  int token_num; // token number on the line
};

struct method {
  struct method * next;
  char *name;
  char *sanatized_name;  // sanatized name of the C++ singleton
  char *rename; // (optional) used for scripting access
  char *deprecate; // (optional) issue deprecateion warning string on first call
  int line; // line declared on
  struct type return_type;
  struct argument * arguments;
  uint32_t flags; // filled out with TYPE_FLAGS
};

enum alias_type {
  ALIAS_TYPE_NONE,
  ALIAS_TYPE_MANUAL,
  ALIAS_TYPE_MANUAL_OPERATOR,
};

struct method_alias {
  struct method_alias *next;
  char *name;
  char *alias;
  int line;
  enum alias_type type;
};

struct userdata_field {
  struct userdata_field * next;
  char * name;
  char * rename;
  struct type type; // field type, points to a string
  int line; // line declared on
  unsigned int access_flags;
  char * array_len; // literal array length
};

enum userdata_flags {
  UD_FLAG_SEMAPHORE         = (1U << 0),
  UD_FLAG_SCHEDULER_SEMAPHORE = (1U << 1),
  UD_FLAG_LITERAL = (1U << 2),
  UD_FLAG_SEMAPHORE_POINTER = (1U << 3),
  UD_FLAG_REFERENCE = (1U << 4),
};

struct userdata_enum {
  struct userdata_enum * next;
  char * name;     // enum name
};

struct userdata {
  struct userdata * next;
  char *name;  // name of the C++ singleton
  char *sanatized_name;  // sanatized name of the C++ singleton
  char *rename; // (optional) used for scripting access
  struct userdata_field *fields;
  struct method *methods;
  struct method_alias *method_aliases;
  struct userdata_enum *enums;
  enum userdata_type ud_type;
  uint32_t operations; // bitset of enum operation_types
  int flags; // flags from the userdata_flags enum
  char *dependency;
  char *creation; // name of a manual creation function if set, note that this will not be used internally
};

static struct userdata *parsed_userdata;
static struct userdata *parsed_ap_objects;
static struct userdata *parsed_globals;

void sanitize_character(char **str, char character) {
  char *position = strchr(*str, character);
  while (position) {
    *position = '_';
    position = strchr(position, character);
  }
}

void sanatize_name(char **dest, char *src) {
  *dest = (char *)allocate(strlen(src) + 1);
  strcpy(*dest, src);

  sanitize_character(dest, ':');
  sanitize_character(dest, '.');
  sanitize_character(dest, '<');
  sanitize_character(dest, '>');
  sanitize_character(dest, '(');
  sanitize_character(dest, ')');
  sanitize_character(dest, '-');

};


struct range_check *parse_range_check(enum field_type type) {
  char * low = next_token();
  if (low == NULL) {
    error(ERROR_USERDATA, "Missing low value for a range check (type: %s)", type_labels[type]);
  }

  trace(TRACE_TOKENS, "Range check: Low: %s", low);

  char * high  = next_token();
  if (high == NULL) {
    error(ERROR_USERDATA, "Missing high value for a range check");
  }

  trace(TRACE_TOKENS, "Range check: High: %s", high);

  struct range_check *check = allocate(sizeof(struct range_check));

  string_copy(&(check->low), low);
  string_copy(&(check->high), high);

  return check;
}

// parses one or more access flags, leaves the token on the first non access token
// throws an error if no flags were found
unsigned int parse_access_flags(struct type * type) {
  unsigned int flags = 0;

  next_token();

  while(state.token != NULL) {
    trace(TRACE_TOKENS, "Possible access: %s", state.token);
    char *p;
    if ((p = strstr(state.token, keyword_attr_no_range_check)) != NULL) {
        *p = 0;
        type->flags |= TYPE_FLAGS_NO_RANGE_CHECK;
    }
    if (strcmp(state.token, keyword_read) == 0) {
      flags |= ACCESS_FLAG_READ;
    } else if (strcmp(state.token, keyword_write) == 0) {
      flags |= ACCESS_FLAG_WRITE;
      if ((type->flags & TYPE_FLAGS_NO_RANGE_CHECK) == 0) {
        switch (type->type) {
          case TYPE_FLOAT:
          case TYPE_INT8_T:
          case TYPE_INT16_T:
          case TYPE_INT32_T:
          case TYPE_UINT8_T:
          case TYPE_UINT16_T:
          case TYPE_UINT32_T:
          case TYPE_ENUM:
            type->range = parse_range_check(type->type);
            break;
          case TYPE_AP_OBJECT:
          case TYPE_USERDATA:
          case TYPE_BOOLEAN:
          case TYPE_STRING:
          case TYPE_LITERAL:
            // a range check is illogical
            break;
          case TYPE_NONE:
            error(ERROR_INTERNAL, "Can't access a NONE type");
        }
      }
    } else {
      break;
    }
    next_token();
  }

  trace(TRACE_TOKENS, "Parsed access flags: 0x%x", flags);

  if (flags == 0) {
    error(ERROR_USERDATA, "Expected to find an access specifier");
  }

  return flags;
}

#define TRUE  1
#define FALSE 0

enum type_restriction {
  TYPE_RESTRICTION_NONE         = 0,
  TYPE_RESTRICTION_OPTIONAL     = (1U << 1),
  TYPE_RESTRICTION_NOT_NULLABLE = (1U << 2),
};

enum range_check_type {
  RANGE_CHECK_NONE,
  RANGE_CHECK_MANDATORY,
};

int parse_type(struct type *type, const uint32_t restrictions, enum range_check_type range_type) {
  char *data_type = next_token();

  if (data_type == NULL) {
    if (restrictions & TYPE_RESTRICTION_OPTIONAL) {
      return FALSE;
    } else {
      error(ERROR_USERDATA, "Data type must be specified");
    }
  }

  if (data_type[0] == '&') {
    type->access = ACCESS_REFERENCE;
    data_type++; // drop the reference character
  } else {
    type->access = ACCESS_VALUE;
  }

  char *attribute = strchr(data_type, '\'');
  if (attribute != NULL) {
    if (strcmp(attribute, keyword_attr_enum) == 0) {
      type->flags |= TYPE_FLAGS_ENUM;
    } else if (strcmp(attribute, keyword_attr_literal) == 0) {
      type->type = TYPE_LITERAL;
    } else if (strcmp(attribute, keyword_attr_null) == 0) {
      if (restrictions & TYPE_RESTRICTION_NOT_NULLABLE) {
        error(ERROR_USERDATA, "%s is not nullable in this context", data_type);
      }
      type->flags |= TYPE_FLAGS_NULLABLE;
    } else if (strcmp(attribute, keyword_attr_reference) == 0) {
      type->flags |= TYPE_FLAGS_REFERNCE;
    } else if (strcmp(attribute, keyword_attr_no_range_check) == 0) {
      type->flags |= TYPE_FLAGS_NO_RANGE_CHECK;
    } else {
      error(ERROR_USERDATA, "Unknown attribute: %s", attribute);
    }
    attribute[0] = 0;
  }

  if ((type->access == ACCESS_REFERENCE) && ((type->flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE)) == 0)) {
      error(ERROR_USERDATA, "Only support refences access will 'Null or 'Ref keyword");
  }

  if (strcmp(data_type, keyword_boolean) == 0) {
    type->type = TYPE_BOOLEAN;
  } else if (strcmp(data_type, keyword_float) == 0) {
    type->type = TYPE_FLOAT;
  } else if (strcmp(data_type, keyword_int8_t) == 0) {
    type->type = TYPE_INT8_T;
  } else if (strcmp(data_type, keyword_int16_t) == 0) {
    type->type = TYPE_INT16_T;
  } else if (strcmp(data_type, keyword_int32_t) == 0) {
    type->type = TYPE_INT32_T;
  } else if (strcmp(data_type, keyword_uint8_t) == 0) {
    type->type = TYPE_UINT8_T;
  } else if (strcmp(data_type, keyword_uint16_t) == 0) {
    type->type = TYPE_UINT16_T;
  } else if (strcmp(data_type, keyword_uint32_t) == 0) {
    type->type = TYPE_UINT32_T;
  } else if (strcmp(data_type, keyword_string) == 0) {
    type->type = TYPE_STRING;
  } else if (strcmp(data_type, keyword_void) == 0) {
    type->type = TYPE_NONE;
  } else if (type->flags & TYPE_FLAGS_ENUM) {
    type->type = TYPE_ENUM;
    string_copy(&(type->data.enum_name), data_type);
  } else if (type->type == TYPE_LITERAL) {
    string_copy(&(type->data.literal), data_type);
  } else if (strcmp(data_type, keyword_rename) == 0) {
    error(ERROR_USERDATA, "Cant add rename to unknown function");
  } else {
    // this must be a user data or an ap_object, check if it's already been declared as an object
    struct userdata *node = parsed_ap_objects;
    while (node != NULL && strcmp(node->name, data_type)) {
      node = node->next;
    }
    if (node != NULL) {
      type->type = TYPE_AP_OBJECT;
    } else {
      // assume that this is a user data, we can't validate this until later though
      type->type = TYPE_USERDATA;
    }
    string_copy(&(type->data.ud.name), data_type);
    sanatize_name(&(type->data.ud.sanatized_name), type->data.ud.name);
  }

  // only allow no range check on float, int32 and uint32
  if (type->flags & TYPE_FLAGS_NO_RANGE_CHECK) {
    switch (type->type) {
      case TYPE_FLOAT:
      case TYPE_INT32_T:
      case TYPE_UINT32_T:
        break;
      default:
        error(ERROR_USERDATA, "%s types cannot skip range check", data_type);
        break;
    }
  }

  // sanity check that only supported types are nullable
  if (type->flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE)) {
    // a switch is a very verbose way to do this, but forces users to consider new types added
    switch (type->type) {
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_UINT32_T:
      case TYPE_BOOLEAN:
      case TYPE_STRING:
      case TYPE_ENUM:
      case TYPE_USERDATA:
        break;
      case TYPE_AP_OBJECT:
      case TYPE_LITERAL:
      case TYPE_NONE:
        if (type->flags & TYPE_FLAGS_NULLABLE) {
          error(ERROR_USERDATA, "%s types cannot be nullable", data_type);
        } else {
          error(ERROR_USERDATA, "%s types cannot be passed as reference", data_type);
        }
        break;
    }
  }

  // add range checks, unless disabled or a nullable type
  if (range_type != RANGE_CHECK_NONE && !(type->flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE | TYPE_FLAGS_NO_RANGE_CHECK))) {
    switch (type->type) {
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_UINT32_T:
      case TYPE_ENUM:
        type->range = parse_range_check(type->type);
        break;
      case TYPE_AP_OBJECT:
      case TYPE_BOOLEAN:
      case TYPE_NONE:
      case TYPE_STRING:
      case TYPE_USERDATA:
      case TYPE_LITERAL:
        // no sane range checks, so we can ignore this
        break;
    }
  }
  return TRUE;
}

void handle_userdata_enum(struct userdata *data) {
  trace(TRACE_USERDATA, "Adding a userdata enum");

  char * enum_name;
  while ((enum_name = next_token()) != NULL) {
    trace(TRACE_USERDATA, "Adding enum %s", enum_name);
    struct userdata_enum *ud_enum = (struct userdata_enum *) allocate(sizeof(struct userdata_enum));
    ud_enum->next = data->enums;
    string_copy(&(ud_enum->name), enum_name);
    data->enums = ud_enum;
  }
}

void handle_userdata_field(struct userdata *data) {
  trace(TRACE_USERDATA, "Adding a userdata field");

  // find the field name
  char * token = next_token();
  if (token == NULL) {
    error(ERROR_USERDATA, "Missing a field name for userdata %s", data->name);
  }

  size_t split = strcspn(token, "\'");
  char * field_name;
  if (split == strlen(token)) {
    string_copy(&field_name, token);
  } else {
    field_name = (char *)allocate(split+1);
    memcpy(field_name, token, split);
  }

  struct userdata_field * field = data->fields;
  while (field != NULL && strcmp(field->name, field_name)) {
    field = field-> next;
  }
  if (field != NULL) {
    char *token = next_token();
    if (strcmp(token, keyword_rename) != 0) {
      error(ERROR_USERDATA, "Field %s already exists in userdata %s (declared on %d)", field_name, data->name, field->line);
    }
    char *rename = next_token();
    string_copy(&(field->rename), rename);
    return;
  }

  trace(TRACE_USERDATA, "Adding field %s", field_name);
  field = (struct userdata_field *)allocate(sizeof(struct userdata_field));
  field->next = data->fields;
  data->fields = field;
  field->line = state.line_num;
  string_copy(&(field->name), field_name);

  char *attribute = strchr(token, '\'');
  if (attribute != NULL) {
    if (strcmp(attribute, keyword_attr_array) != 0) {
      error(ERROR_USERDATA, "Unknown field attribute %s for userdata %s field %s", attribute, data->name, field_name);
    }
    char * token = next_token();
    string_copy(&(field->array_len), token);
    trace(TRACE_USERDATA, "userdata %s field %s array length %s", data->name, field->name, field->array_len);
  }

  parse_type(&(field->type), TYPE_RESTRICTION_NOT_NULLABLE, RANGE_CHECK_NONE);
  field->access_flags = parse_access_flags(&(field->type));
}

void handle_method(struct userdata *node) {
  trace(TRACE_USERDATA, "Adding a method");
  char * parent_name = node->name;

  // find the field name
  char * name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Missing method name for %s", parent_name);
  }
  
  struct method * method = node->methods;
  while (method != NULL && strcmp(method->name, name)) {
    method = method-> next;
  }
  if (method != NULL) {
    char *token = next_token();
    if (strcmp(token, keyword_rename) == 0) {
      char *rename = next_token();
      string_copy(&(method->rename), rename);
      return;

    } else if (strcmp(token, keyword_alias) == 0) {
      char *alias_name = next_token();

      struct method * method = node->methods;
      while (method != NULL && strcmp(method->name, alias_name)) {
        method = method-> next;
      }
      if (method != NULL) {
        error(ERROR_USERDATA, "Method %s already exists for %s (declared on %d) cannot alias to %s", alias_name, parent_name, method->line, name);
      }


      struct method_alias *alias = allocate(sizeof(struct method_alias));
      string_copy(&(alias->name), name);
      string_copy(&(alias->alias), alias_name);
      alias->line = state.line_num;
      alias->next = node->method_aliases;
      node->method_aliases = alias;
      return;

    } else if (strcmp(token, keyword_deprecate) == 0) {
      char *deprecate = strtok(NULL, "");
      if (deprecate == NULL) {
        error(ERROR_USERDATA, "Expected a deprecate string for %s %s on line", parent_name, name, state.line_num);
      }
      string_copy(&(method->deprecate), deprecate);
      return;
    }
    error(ERROR_USERDATA, "Method %s already exists for %s (declared on %d)", name, parent_name, method->line);
  }

  struct method_alias * alias = node->method_aliases;
  while (alias != NULL && strcmp(alias->alias, name)) {
    alias = alias-> next;
  }
  if (alias != NULL) {
    error(ERROR_USERDATA, "alias %s already exists for %s (declared on %d)", name, alias->name, alias->line);
  }

  trace(TRACE_USERDATA, "Adding method %s", name);
  method = allocate(sizeof(struct method));
  method->next = node->methods;
  node->methods = method;
  string_copy(&(method->name), name);
  sanatize_name(&(method->sanatized_name), name);
  method->line = state.line_num;

  parse_type(&(method->return_type), TYPE_RESTRICTION_NONE, RANGE_CHECK_NONE);

  // iterate the arguments
  struct type arg_type = {};
  while (parse_type(&arg_type, TYPE_RESTRICTION_OPTIONAL, RANGE_CHECK_MANDATORY)) {
    if (arg_type.type == TYPE_NONE) {
      error(ERROR_USERDATA, "Can't pass an empty argument to a method");
    }
    if ((method->return_type.type != TYPE_BOOLEAN) && (arg_type.flags & TYPE_FLAGS_NULLABLE)) {
      error(ERROR_USERDATA, "Nullable arguments are only available on a boolean method");
    }
    if ((method->return_type.type == TYPE_BOOLEAN) && (arg_type.flags & TYPE_FLAGS_REFERNCE)) {
      error(ERROR_USERDATA, "Use Nullable arguments on a boolean method, not 'Ref");
    }
    if (arg_type.flags & TYPE_FLAGS_NULLABLE) {
      method->flags |= TYPE_FLAGS_NULLABLE;
    }
    if (arg_type.flags & TYPE_FLAGS_REFERNCE) {
      method->flags |= TYPE_FLAGS_REFERNCE;
    }
    struct argument * arg = allocate(sizeof(struct argument));
    memcpy(&(arg->type), &arg_type, sizeof(struct type));
    arg->line_num = state.line_num;
    arg->token_num = state.token_num;
    if (method->arguments == NULL) {
      method->arguments = arg;
    } else {
      struct argument *tail = method->arguments;
      while (tail->next != NULL) {
        tail = tail->next;
      }
      tail->next = arg;
    }

    // reset the stack arg_type
    memset(&arg_type, 0, sizeof(struct type));
  }
}

void handle_manual(struct userdata *node, enum alias_type type) {
  char *name = next_token();
  if (name == NULL) {
    error(ERROR_SINGLETON, "Expected a lua name for manual %s method",node->name);
  }
  char *cpp_function_name = next_token();
  if (cpp_function_name == NULL) {
    error(ERROR_SINGLETON, "Expected a cpp name for manual %s method",node->name);
  }
  struct method_alias *alias = allocate(sizeof(struct method_alias));
  string_copy(&(alias->name), cpp_function_name);
  string_copy(&(alias->alias), name);
  alias->line = state.line_num;
  alias->type = type;
  alias->next = node->method_aliases;
  node->method_aliases = alias;
}

void handle_operator(struct userdata *data) {
  trace(TRACE_USERDATA, "Adding a operator");

  if (data->ud_type != UD_USERDATA) {
    error(ERROR_USERDATA, "Operators are only allowed on userdata objects");
  }

  char *operator = next_token();
  if (operator == NULL) {
    error(ERROR_USERDATA, "Needed a symbol for the operator");
  }

  enum operator_type operation = OP_ADD;
  if (strcmp(operator, "+") == 0) {
    operation = OP_ADD;
  } else if (strcmp(operator, "-") == 0) {
    operation = OP_SUB;
  } else if (strcmp(operator, "*") == 0) {
    operation = OP_MUL;
  } else if (strcmp(operator, "/") == 0) {
    operation = OP_DIV;
  } else {
    error(ERROR_USERDATA, "Unknown operation type: %s", operator);
  }

  if ((data->operations) & operation) {
    error(ERROR_USERDATA, "Operation %s was already defined for %s", operator, data->name);
  }

  trace(TRACE_USERDATA, "Adding operation %d to %s", operation, data->name);

  data->operations |= operation;

  if (next_token() != NULL) {
    error(ERROR_USERDATA, "Extra token on operation %s", operator);
  }
}

void handle_userdata(void) {
  trace(TRACE_USERDATA, "Adding a userdata");

  char *name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Expected a name for the userdata");
  }

  struct userdata *node = parsed_userdata;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }
  if (node == NULL) {
    trace(TRACE_USERDATA, "Allocating new userdata for %s", name);
    node = (struct userdata *)allocate(sizeof(struct userdata));
    node->ud_type = UD_USERDATA;
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    sanatize_name(&(node->sanatized_name), node->name);
    node->next = parsed_userdata;
    parsed_userdata = node;
  } else {
    trace(TRACE_USERDATA, "Found existing userdata for %s", name);
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_USERDATA, "Expected a access type for userdata %s", name);
  }

  // match type
  if (strcmp(type, keyword_field) == 0) {
    handle_userdata_field(node);
  } else if (strcmp(type, keyword_operator) == 0) {
    handle_operator(node);
  } else if (strcmp(type, keyword_method) == 0) {
    handle_method(node);
  } else if (strcmp(type, keyword_enum) == 0) {
    handle_userdata_enum(node);
  } else if (strcmp(type, keyword_rename) == 0) {
    const char *rename = next_token();
    if (rename == NULL) {
      error(ERROR_SINGLETON, "Missing the name of the rename for userdata %s", node->name);
    }
    node->rename = (char *)allocate(strlen(rename) + 1);
    strcpy(node->rename, rename);

  } else if (strcmp(type, keyword_depends) == 0) {
      if (node->dependency != NULL) {
        error(ERROR_SINGLETON, "Userdata only support a single depends");
      }
      char *depends = strtok(NULL, "");
      if (depends == NULL) {
        error(ERROR_DEPENDS, "Expected a depends string for %s",node->name);
      }
      string_copy(&(node->dependency), depends);

  } else if (strcmp(type, keyword_manual) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL);

  } else if (strcmp(type, keyword_creation) == 0) {
      if (node->creation != NULL) {
        error(ERROR_SINGLETON, "Userdata only support a single creation function");
      }
      char *creation = strtok(NULL, "");
      if (creation == NULL) {
        error(ERROR_USERDATA, "Expected a creation string for %s",node->name);
      }
      string_copy(&(node->creation), creation);

  } else if (strcmp(type, keyword_manual_operator) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL_OPERATOR);
    node->operations |= OP_MANUAL;

  } else {
    error(ERROR_USERDATA, "Unknown or unsupported type for userdata: %s", type);
  }

}

struct userdata *parsed_singletons = NULL;

void handle_singleton(void) {
  trace(TRACE_SINGLETON, "Adding a singleton");

  char *name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Expected a name for the singleton");
  }

  struct userdata *node = parsed_singletons;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }

  if (node == NULL) {
    trace(TRACE_SINGLETON, "Allocating new singleton for %s", name);
    node = (struct userdata *)allocate(sizeof(struct userdata));
    node->ud_type = UD_SINGLETON;
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    sanatize_name(&(node->sanatized_name), node->name);
    node->next = parsed_singletons;
    parsed_singletons = node;
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_SINGLETON, "Expected a access type for userdata %s", name);
  }

  if (strcmp(type, keyword_rename) == 0) {
    if (node->rename != NULL) {
      error(ERROR_SINGLETON, "rename of %s was already declared for %s", node->rename, node->name);
    }
    const char *rename = next_token();
    if (rename == NULL) {
      error(ERROR_SINGLETON, "Missing the name of the rename for %s", node->name);
    }
    node->rename = (char *)allocate(strlen(rename) + 1);
    strcpy(node->rename, rename);

  } else if (strcmp(type, keyword_semaphore) == 0) {
    node->flags |= UD_FLAG_SEMAPHORE;
  } else if (strcmp(type, keyword_scheduler_semaphore) == 0) {
    node->flags |= UD_FLAG_SCHEDULER_SEMAPHORE;
  } else if (strcmp(type, keyword_semaphore_pointer) == 0) {
    node->flags |= UD_FLAG_SCHEDULER_SEMAPHORE;
  } else if (strcmp(type, keyword_method) == 0) {
    handle_method(node);
  } else if (strcmp(type, keyword_enum) == 0) {
    handle_userdata_enum(node);
  } else if (strcmp(type, keyword_depends) == 0) {
    if (node->dependency != NULL) {
      error(ERROR_SINGLETON, "Singletons only support a single depends");
    }
    char *depends = strtok(NULL, "");
    if (depends == NULL) {
      error(ERROR_DEPENDS, "Expected a depends string for %s",node->name);
    }
    string_copy(&(node->dependency), depends);
  } else if (strcmp(type, keyword_literal) == 0) {
    node->flags |= UD_FLAG_LITERAL;
  } else if (strcmp(type, keyword_field) == 0) {
    handle_userdata_field(node);
  } else if (strcmp(type, keyword_reference) == 0) {
    node->flags |= UD_FLAG_REFERENCE;
  } else if (strcmp(type, keyword_manual) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL);
  } else {
    error(ERROR_SINGLETON, "Singletons only support renames, methods, semaphore, depends, literal or manual keywords (got %s)", type);
  }

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_HEADER, "Singleton contained an unexpected extra token: %s", state.token);
  }
}

void handle_ap_object(void) {
  trace(TRACE_SINGLETON, "Adding a ap_object");

  char *name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Expected a name for the ap_object");
  }

  struct userdata *node = parsed_ap_objects;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }

  if (node == NULL) {
    trace(TRACE_USERDATA, "Allocating new ap_object for %s", name);
    node = (struct userdata *)allocate(sizeof(struct userdata));
    node->ud_type = UD_AP_OBJECT;
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    sanatize_name(&(node->sanatized_name), node->name);
    node->next = parsed_ap_objects;
    parsed_ap_objects = node;
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_SINGLETON, "Expected a access type for ap_object %s", name);
  }

  if (strcmp(type, keyword_rename) == 0) {
    if (node->rename != NULL) {
      error(ERROR_SINGLETON, "Rename of %s was already declared for %s", node->rename, node->name);
    }
    const char *rename = next_token();
    if (rename == NULL) {
      error(ERROR_SINGLETON, "Missing the name of the rename for %s", node->name);
    }
    node->rename = (char *)allocate(strlen(rename) + 1);
    strcpy(node->rename, rename);

  } else if (strcmp(type, keyword_semaphore) == 0) {
    node->flags |= UD_FLAG_SEMAPHORE;
  } else if (strcmp(type, keyword_scheduler_semaphore) == 0) {
    node->flags |= UD_FLAG_SCHEDULER_SEMAPHORE;
  } else if (strcmp(type, keyword_semaphore_pointer) == 0) {
    node->flags |= UD_FLAG_SEMAPHORE_POINTER;
  } else if (strcmp(type, keyword_method) == 0) {
    handle_method(node);
  } else if (strcmp(type, keyword_depends) == 0) {
      if (node->dependency != NULL) {
        error(ERROR_SINGLETON, "AP_Objects only support a single depends");
      }
      char *depends = strtok(NULL, "");
      if (depends == NULL) {
        error(ERROR_DEPENDS, "Expected a depends string for %s",node->name);
      }
      string_copy(&(node->dependency), depends);

  } else if (strcmp(type, keyword_manual) == 0) {
    handle_manual(node, ALIAS_TYPE_MANUAL);

  } else {
    error(ERROR_SINGLETON, "AP_Objects only support renames, methods, semaphore or manual keywords (got %s)", type);
  }

  // check that we didn't just add 2 singleton flags
  int semaphore = (node->flags & UD_FLAG_SEMAPHORE)?1:0;
  semaphore += (node->flags & UD_FLAG_SCHEDULER_SEMAPHORE)?1:0;
  semaphore += (node->flags & UD_FLAG_SEMAPHORE_POINTER)?1:0;
  if (semaphore > 1) {
    error(ERROR_SINGLETON, "Taking multiple types of semaphore is prohibited");
  }

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_HEADER, "Singleton contained an unexpected extra token: %s", state.token);
  }
}

void handle_global(void) {

  if (parsed_globals == NULL) {
    parsed_globals = (struct userdata *)allocate(sizeof(struct userdata));
    parsed_globals->ud_type = UD_GLOBAL;
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_GLOBALS, "Expected a access type for global");
  }

  if (strcmp(type, keyword_manual) == 0) {
    handle_manual(parsed_globals, ALIAS_TYPE_MANUAL);
  } else {
    error(ERROR_GLOBALS, "globals only support manual keyword (got %s)", type);
  }

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_GLOBALS, "global contained an unexpected extra token: %s", state.token);
  }
}

void sanity_check_userdata(void) {
  struct userdata * node = parsed_userdata;
  while(node) {
    if ((node->fields == NULL) && (node->methods == NULL) && (node->method_aliases == NULL)) {
      error(ERROR_USERDATA, "Userdata %s has no fields or methods", node->name);
    }
    node = node->next;
  }
}

void start_dependency(FILE *f, const char *dependency) {
  if (dependency != NULL) {
    fprintf(f, "#if %s\n", dependency);
  }
}

void end_dependency(FILE *f, const char *dependency) {
  if (dependency != NULL) {
    fprintf(f, "#endif // %s\n", dependency);
  }
}

void emit_headers(FILE *f) {
  struct header *node = headers;
  while (node) {
    start_dependency(f, node->dependency);
    fprintf(f, "#include <%s>\n", node->name);
    end_dependency(f, node->dependency);
    node = node->next;
  }
}

void emit_userdata_allocators(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    start_dependency(source, node->dependency);
    fprintf(source, "int new_%s(lua_State *L) {\n", node->sanatized_name);
    fprintf(source, "    luaL_checkstack(L, 2, \"Out of stack\");\n"); // ensure we have sufficent stack to push the return
    fprintf(source, "    void *ud = lua_newuserdata(L, sizeof(%s));\n", node->name);
    fprintf(source, "    memset(ud, 0, sizeof(%s));\n", node->name);
    fprintf(source, "    new (ud) %s();\n", node->name);
    fprintf(source, "    luaL_getmetatable(L, \"%s\");\n", node->rename ? node->rename :  node->name);
    fprintf(source, "    lua_setmetatable(L, -2);\n");
    fprintf(source, "    return 1;\n");
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
    node = node->next;
  }
}

void emit_ap_object_allocators(void) {
  struct userdata * node = parsed_ap_objects;
  while (node) {
    start_dependency(source, node->dependency);
    fprintf(source, "int new_%s(lua_State *L) {\n", node->sanatized_name);
    fprintf(source, "    luaL_checkstack(L, 2, \"Out of stack\");\n"); // ensure we have sufficent stack to push the return
    fprintf(source, "    void *ud = lua_newuserdata(L, sizeof(%s *));\n", node->name);
    fprintf(source, "    memset(ud, 0, sizeof(%s *));\n", node->name); // FIXME: memset is a ridiculously large hammer here
    fprintf(source, "    luaL_getmetatable(L, \"%s\");\n", node->name);
    fprintf(source, "    lua_setmetatable(L, -2);\n");
    fprintf(source, "    return 1;\n");
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
    node = node->next;
  }
}

void emit_userdata_checkers(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    start_dependency(source, node->dependency);
    fprintf(source, "%s * check_%s(lua_State *L, int arg) {\n", node->name, node->sanatized_name);
    fprintf(source, "    void *data = luaL_checkudata(L, arg, \"%s\");\n",  node->rename ? node->rename :  node->name);
    fprintf(source, "    return (%s *)data;\n", node->name);
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
    node = node->next;
  }
}

void emit_ap_object_checkers(void) {
  struct userdata * node = parsed_ap_objects;
  while (node) {
    start_dependency(source, node->dependency);
    fprintf(source, "%s ** check_%s(lua_State *L, int arg) {\n", node->name, node->sanatized_name);
    fprintf(source, "    void *data = luaL_checkudata(L, arg, \"%s\");\n", node->name);
    fprintf(source, "    return (%s **)data;\n", node->name);
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
    node = node->next;
  }
}

void emit_userdata_declarations(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    start_dependency(header, node->dependency);
    fprintf(header, "int new_%s(lua_State *L);\n", node->sanatized_name);
    fprintf(header, "%s * check_%s(lua_State *L, int arg);\n", node->name, node->sanatized_name);
    end_dependency(header, node->dependency);
    node = node->next;
  }
}

void emit_ap_object_declarations(void) {
  struct userdata * node = parsed_ap_objects;
  while (node) {
    start_dependency(header, node->dependency);
    fprintf(header, "int new_%s(lua_State *L);\n", node->sanatized_name);
    fprintf(header, "%s ** check_%s(lua_State *L, int arg);\n", node->name, node->sanatized_name);
    end_dependency(header, node->dependency);
    node = node->next;
  }
}

#define NULLABLE_ARG_COUNT_BASE 5000
void emit_checker(const struct type t, int arg_number, int skipped, const char *indentation, const char *name) {
  assert(indentation != NULL);

  if (arg_number > NULLABLE_ARG_COUNT_BASE) {
    error(ERROR_INTERNAL, "Can't handle more then %d arguments to a function", NULLABLE_ARG_COUNT_BASE);
  }

  if (t.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE)) {
    arg_number = arg_number + NULLABLE_ARG_COUNT_BASE;
    switch (t.type) {
      case TYPE_BOOLEAN:
        fprintf(source, "%sbool data_%d;\n", indentation, arg_number);
        break;
      case TYPE_FLOAT:
        fprintf(source, "%sfloat data_%d;\n", indentation, arg_number);
        break;
      case TYPE_INT8_T:
        fprintf(source, "%sint8_t data_%d;\n", indentation, arg_number);
        break;
      case TYPE_INT16_T:
        fprintf(source, "%sint16_t data_%d;\n", indentation, arg_number);
        break;
      case TYPE_INT32_T:
        fprintf(source, "%sint32_t data_%d;\n", indentation, arg_number);
        break;
      case TYPE_UINT8_T:
        fprintf(source, "%suint8_t data_%d;\n", indentation, arg_number);
        break;
      case TYPE_UINT16_T:
        fprintf(source, "%suint16_t data_%d;\n", indentation, arg_number);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "%suint32_t data_%d;\n", indentation, arg_number);
        break;
      case TYPE_AP_OBJECT:
      case TYPE_NONE:
      case TYPE_LITERAL:
        return; // nothing to do here, this should potentially be checked outside of this, but it makes an easier implementation to accept it
      case TYPE_STRING:
        fprintf(source, "%schar * data_%d = {};\n", indentation, arg_number);
        break;
      case TYPE_ENUM:
        fprintf(source, "%suint32_t data_%d;\n", indentation, arg_number);
        break;
      case TYPE_USERDATA:
        fprintf(source, "%s%s data_%d = {};\n", indentation, t.data.ud.name, arg_number);
        break;
    }
  } else {
    // handle this in four stages
    //   - figure out any relevant minimum values for range checking
    //   - emit a non down casted version
    //   - then run range checks
    //   - then cast down as appropriate

    // select minimums
    char * forced_min;
    char * forced_max;
    switch (t.type) {
      case TYPE_FLOAT:
        forced_min = "-INFINITY";
        forced_max = "INFINITY";
        break;
      case TYPE_INT8_T:
        forced_min = "INT8_MIN";
        forced_max = "INT8_MAX";
        break;
      case TYPE_INT16_T:
        forced_min = "INT16_MIN";
        forced_max = "INT16_MAX";
        break;
      case TYPE_INT32_T:
        forced_min = "INT32_MIN";
        forced_max = "INT32_MAX";
        break;
      case TYPE_UINT8_T:
        forced_min = "0";
        forced_max = "UINT8_MAX";
        break;
      case TYPE_UINT16_T:
        forced_min = "0";
        forced_max = "UINT16_MAX";
        break;
      case TYPE_UINT32_T:
        forced_min = "0U";
        forced_max = "UINT32_MAX";
        break;
      case TYPE_ENUM:
        forced_min = forced_max = NULL;
        break;
      case TYPE_NONE:
        return; // nothing to do here, this should potentially be checked outside of this, but it makes an easier implementation to accept it
      case TYPE_AP_OBJECT:
      case TYPE_STRING:
      case TYPE_BOOLEAN:
      case TYPE_USERDATA:
      case TYPE_LITERAL:
        // these don't get range checked, so skip the raw_data phase
        assert(t.range == NULL); // we should have caught this during the parse phase
        break;
    }

    // non down cast
    switch (t.type) {
      case TYPE_FLOAT:
        fprintf(source, "%sconst float raw_data_%d = luaL_checknumber(L, %d);\n", indentation, arg_number, arg_number - skipped);
        break;
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_ENUM:
        fprintf(source, "%sconst lua_Integer raw_data_%d = luaL_checkinteger(L, %d);\n", indentation, arg_number, arg_number - skipped);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "%sconst uint32_t raw_data_%d = coerce_to_uint32_t(L, %d);\n", indentation, arg_number, arg_number - skipped);
        break;
      case TYPE_AP_OBJECT:
      case TYPE_NONE:
      case TYPE_STRING:
      case TYPE_BOOLEAN:
      case TYPE_USERDATA:
      case TYPE_LITERAL:
        // these don't get range checked, so skip the raw_data phase
        assert(t.range == NULL); // we should have caught this during the parse phase
        break;
    }

    // range check
    if (t.range != NULL) {
      if ((forced_min != NULL) && (forced_max != NULL)) {
        fprintf(source, "%sluaL_argcheck(L, ((raw_data_%d >= MAX(%s, %s)) && (raw_data_%d <= MIN(%s, %s))), %d, \"%s out of range\");\n",
                indentation,
                arg_number, t.range->low, forced_min,
                arg_number, t.range->high, forced_max,
                arg_number, name);
       } else {
         char * cast_target = "";

         switch (t.type) {
           case TYPE_FLOAT:
             cast_target = "float";
             break;
           case TYPE_INT8_T:
           case TYPE_INT16_T:
           case TYPE_INT32_T:
           case TYPE_UINT8_T:
           case TYPE_UINT16_T:
           case TYPE_ENUM:
             cast_target = "int32_t";
             break;
           case TYPE_UINT32_T:
             cast_target = "uint32_t";
             break;
           case TYPE_AP_OBJECT:
           case TYPE_NONE:
           case TYPE_STRING:
           case TYPE_BOOLEAN:
           case TYPE_USERDATA:
           case TYPE_LITERAL:
             assert(t.range == NULL); // we should have caught this during the parse phase
             break;
         }

         fprintf(source, "%sluaL_argcheck(L, ((raw_data_%d >= static_cast<%s>(%s)) && (raw_data_%d <= static_cast<%s>(%s))), %d, \"%s out of range\");\n",
                 indentation,
                 arg_number, cast_target, t.range->low,
                 arg_number, cast_target, t.range->high,
                 arg_number - skipped, name);
       }
    }

    // down cast
    switch (t.type) {
      case TYPE_FLOAT:
        // this is a trivial transformation, trust the compiler to resolve it for us
        fprintf(source, "%sconst float data_%d = raw_data_%d;\n", indentation, arg_number, arg_number);
        break;
      case TYPE_INT8_T:
        fprintf(source, "%sconst int8_t data_%d = static_cast<int8_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_INT16_T:
        fprintf(source, "%sconst int16_t data_%d = static_cast<int16_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_INT32_T:
        fprintf(source, "%sconst int32_t data_%d = raw_data_%d;\n", indentation, arg_number, arg_number);
        break;
      case TYPE_UINT8_T:
        fprintf(source, "%sconst uint8_t data_%d = static_cast<uint8_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_UINT16_T:
        fprintf(source, "%sconst uint16_t data_%d = static_cast<uint16_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "%sconst uint32_t data_%d = static_cast<uint32_t>(raw_data_%d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_BOOLEAN:
        fprintf(source, "%sconst bool data_%d = static_cast<bool>(lua_toboolean(L, %d));\n", indentation, arg_number, arg_number);
        break;
      case TYPE_STRING:
        fprintf(source, "%sconst char * data_%d = luaL_checkstring(L, %d);\n", indentation, arg_number, arg_number);
        break;
      case TYPE_ENUM:
        fprintf(source, "%sconst %s data_%d = static_cast<%s>(raw_data_%d);\n", indentation, t.data.enum_name, arg_number, t.data.enum_name, arg_number);
        break;
      case TYPE_USERDATA:
        fprintf(source, "%s%s & data_%d = *check_%s(L, %d);\n", indentation, t.data.ud.name, arg_number, t.data.ud.sanatized_name, arg_number);
        break;
      case TYPE_AP_OBJECT:
        fprintf(source, "%s%s * data_%d = *check_%s(L, %d);\n", indentation, t.data.ud.name, arg_number, t.data.ud.sanatized_name, arg_number);
        break;
      case TYPE_LITERAL:
        // literals are expected to be done directly later
        break;
      case TYPE_NONE:
        // nothing to do, we've either already emitted a reasonable value, or returned
        break;
    }
  }
}

void emit_userdata_field(const struct userdata *data, const struct userdata_field *field) {
  fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanatized_name, field->name);
  fprintf(source, "    %s *ud = check_%s(L, 1);\n", data->name, data->sanatized_name);

  char *index_string = "";
  int write_arg_number = 2;
  if (field->array_len != NULL) {
    index_string = "[index]";
    write_arg_number = 3;

    fprintf(source, "\n    const lua_Integer raw_index = luaL_checkinteger(L, 2);\n");
    fprintf(source, "    luaL_argcheck(L, ((raw_index >= 0) && (raw_index < MIN(%s, UINT8_MAX))), 2, \"index out of range\");\n",field->array_len);
    fprintf(source, "    const uint8_t index = static_cast<uint8_t>(raw_index);\n\n");

    fprintf(source, "    switch(lua_gettop(L)-1) {\n");

  } else {
    fprintf(source, "    switch(lua_gettop(L)) {\n");
  }

  if (field->access_flags & ACCESS_FLAG_READ) {
    fprintf(source, "        case 1:\n");
    switch (field->type.type) {
      case TYPE_BOOLEAN:
        fprintf(source, "            lua_pushinteger(L, ud->%s%s);\n", field->name, index_string);
        break;
      case TYPE_FLOAT:
        fprintf(source, "            lua_pushnumber(L, ud->%s%s);\n", field->name, index_string);
        break;
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_ENUM:
        fprintf(source, "            lua_pushinteger(L, ud->%s%s);\n", field->name, index_string);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "            new_uint32_t(L);\n");
        fprintf(source, "            *static_cast<uint32_t *>(luaL_checkudata(L, -1, \"uint32_t\")) = ud->%s%s;\n", field->name, index_string);
        break;
      case TYPE_NONE:
        error(ERROR_INTERNAL, "Can't access a NONE field");
        break;
      case TYPE_LITERAL:
        error(ERROR_INTERNAL, "Can't access a literal field");
        break;
      case TYPE_STRING:
        fprintf(source, "            lua_pushstring(L, ud->%s%s);\n", field->name, index_string);
        break;
      case TYPE_USERDATA:
        error(ERROR_USERDATA, "Userdata does not currently support access to userdata field's");
        break;
      case TYPE_AP_OBJECT: // FIXME: collapse the identical cases here, and use the type string function
        error(ERROR_USERDATA, "AP_Object does not currently support access to userdata field's");
        break;
    }
    fprintf(source, "            return 1;\n");
  }

  if (field->access_flags & ACCESS_FLAG_WRITE) {
    fprintf(source, "        case 2: {\n");
    emit_checker(field->type, write_arg_number, 0, "            ", field->name);
    fprintf(source, "            ud->%s%s = data_%i;\n", field->name, index_string, write_arg_number);
    fprintf(source, "            return 0;\n");
    fprintf(source, "         }\n");
  }

  fprintf(source, "        default:\n");
  fprintf(source, "            return luaL_argerror(L, lua_gettop(L), \"too many arguments\");\n");
  fprintf(source, "    }\n");
  fprintf(source, "}\n\n");
}

void emit_userdata_fields() {
  struct userdata * node = parsed_userdata;
  while(node) {
    struct userdata_field *field = node->fields;
    if (field) {
      start_dependency(source, node->dependency);
      while(field) {
        emit_userdata_field(node, field);
        field = field->next;
      }
    }
    end_dependency(source, node->dependency);
    node = node->next;
  }
}

void emit_singleton_field(const struct userdata *data, const struct userdata_field *field) {
  fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanatized_name, field->name);

  // emit comments on expected arg/type
  if (!(data->flags & UD_FLAG_LITERAL)) {
      // fetch and check the singleton pointer
      fprintf(source, "    %s * ud = %s::get_singleton();\n", data->name, data->name);
      fprintf(source, "    if (ud == nullptr) {\n");
      fprintf(source, "        return not_supported_error(L, %d, \"%s\");\n", 1, data->rename ? data->rename : data->name);
      fprintf(source, "    }\n\n");
  }
  const char *ud_name = (data->flags & UD_FLAG_LITERAL)?data->name:"ud";
  const char *ud_access = (data->flags & UD_FLAG_REFERENCE)?".":"->";

  char *index_string = "";
  int write_arg_number = 2;
  if (field->array_len != NULL) {
    index_string = "[index]";
    write_arg_number = 3;

    fprintf(source, "\n    const lua_Integer raw_index = luaL_checkinteger(L, 2);\n");
    fprintf(source, "    luaL_argcheck(L, ((raw_index >= 0) && (raw_index < MIN(%s, UINT8_MAX))), 2, \"index out of range\");\n",field->array_len);
    fprintf(source, "    const uint8_t index = static_cast<uint8_t>(raw_index);\n\n");

    fprintf(source, "    switch(lua_gettop(L)-1) {\n");

  } else {
    fprintf(source, "    switch(lua_gettop(L)) {\n");
  }

  if (field->access_flags & ACCESS_FLAG_READ) {
    fprintf(source, "        case 1:\n");
    switch (field->type.type) {
      case TYPE_BOOLEAN:
        fprintf(source, "            lua_pushinteger(L, %s%s%s%s);\n", ud_name, ud_access, field->name, index_string);
        break;
      case TYPE_FLOAT:
        fprintf(source, "            lua_pushnumber(L, %s%s%s%s);\n", ud_name, ud_access, field->name, index_string);
        break;
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_ENUM:
        fprintf(source, "            lua_pushinteger(L, %s%s%s%s);\n", ud_name, ud_access, field->name, index_string);
        break;
      case TYPE_UINT32_T:
        fprintf(source, "            new_uint32_t(L);\n");
        fprintf(source, "            *static_cast<uint32_t *>(luaL_checkudata(L, -1, \"uint32_t\")) = %s%s%s%s;\n", ud_name, ud_access, field->name, index_string);
        break;
      case TYPE_NONE:
        error(ERROR_INTERNAL, "Can't access a NONE field");
        break;
      case TYPE_LITERAL:
        error(ERROR_INTERNAL, "Can't access a literal field");
        break;
      case TYPE_STRING:
        fprintf(source, "            lua_pushstring(L, %s%s%s%s);\n", ud_name, ud_access, field->name, index_string);
        break;
      case TYPE_USERDATA:
        error(ERROR_USERDATA, "Userdata does not currently support access to userdata field's");
        break;
      case TYPE_AP_OBJECT: // FIXME: collapse the identical cases here, and use the type string function
        error(ERROR_USERDATA, "AP_Object does not currently support access to userdata field's");
        break;
    }
    fprintf(source, "            return 1;\n");
  }

  if (field->access_flags & ACCESS_FLAG_WRITE) {
    fprintf(source, "        case 2: {\n");
    emit_checker(field->type, write_arg_number, 0, "            ", field->name);
    fprintf(source, "            %s%s%s%s = data_%i;\n", ud_name, ud_access, field->name, index_string, write_arg_number);
    fprintf(source, "            return 0;\n");
    fprintf(source, "         }\n");
  }

  fprintf(source, "        default:\n");
  fprintf(source, "            return luaL_argerror(L, lua_gettop(L), \"too many arguments\");\n");
  fprintf(source, "    }\n");
  fprintf(source, "}\n\n");
}

void emit_singleton_fields() {
  struct userdata * node = parsed_singletons;
  while(node) {
    struct userdata_field *field = node->fields;
    if (field) {
      start_dependency(source, node->dependency);
      while(field) {
        emit_singleton_field(node, field);
        field = field->next;
      }
      end_dependency(source, node->dependency);
    }
    node = node->next;
  }
}

// emit refences functions for a call, return the number of arduments added
int emit_references(const struct argument *arg, const char * tab) {
  int arg_index = NULLABLE_ARG_COUNT_BASE + 2;
  int return_count = 0;
  while (arg != NULL) {
    if (arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE)) {
      return_count++;
      switch (arg->type.type) {
        case TYPE_BOOLEAN:
          fprintf(source, "%slua_pushboolean(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_FLOAT:
          fprintf(source, "%slua_pushnumber(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_INT8_T:
        case TYPE_INT16_T:
        case TYPE_INT32_T:
        case TYPE_UINT8_T:
        case TYPE_UINT16_T:
        case TYPE_ENUM:
          fprintf(source, "%slua_pushinteger(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_UINT32_T:
          fprintf(source, "%snew_uint32_t(L);\n", tab);
          fprintf(source, "%s*static_cast<uint32_t *>(luaL_checkudata(L, -1, \"uint32_t\")) = data_%d;\n", tab, arg_index);
          break;
        case TYPE_STRING:
          fprintf(source, "%slua_pushstring(L, data_%d);\n", tab, arg_index);
          break;
        case TYPE_USERDATA:
          // userdatas must allocate a new container to return
          fprintf(source, "%snew_%s(L);\n", tab, arg->type.data.ud.sanatized_name);
          fprintf(source, "%s*check_%s(L, -1) = data_%d;\n", tab, arg->type.data.ud.sanatized_name, arg_index);
          break;
        case TYPE_NONE:
          error(ERROR_INTERNAL, "Attempted to emit a nullable or reference  argument of type none");
          break;
        case TYPE_LITERAL:
          error(ERROR_INTERNAL, "Attempted to make a nullable or reference literal");
          break;
        case TYPE_AP_OBJECT: // FIXME: collapse these to a single failure case
          error(ERROR_INTERNAL, "Attempted to make a nullable or reference ap_object");
          break;
      }
    }
    arg_index++;
    arg = arg->next;
  }
  return return_count;
}

void emit_userdata_method(const struct userdata *data, const struct method *method) {
  int arg_count = 1;

  start_dependency(source, data->dependency);

  const char *access_name = data->rename ? data->rename : data->name;
  // bind ud early if it's a singleton, so that we can use it in the range checks
  fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanatized_name, method->sanatized_name);
  // emit comments on expected arg/type
  struct argument *arg = method->arguments;

  if ((data->ud_type == UD_SINGLETON) && !(data->flags & UD_FLAG_LITERAL)) {
      // fetch and check the singleton pointer
      fprintf(source, "    %s * ud = %s::get_singleton();\n", data->name, data->name);
      fprintf(source, "    if (ud == nullptr) {\n");
      fprintf(source, "        return not_supported_error(L, %d, \"%s\");\n", arg_count, access_name);
      fprintf(source, "    }\n\n");
  }

  // emit warning if configured
  if (method->deprecate != NULL) {
    fprintf(source, "    static bool warned = false;\n");
    fprintf(source, "    if (!warned) {\n");
    fprintf(source, "        lua_scripts::set_and_print_new_error_message(MAV_SEVERITY_WARNING, \"%s:%s %s\");\n", data->rename, method->rename ? method->rename : method->name, method->deprecate);
    fprintf(source, "        warned = true;\n");
    fprintf(source, "    }\n\n");
  }


  // sanity check number of args called with
  arg_count = 1;
  while (arg != NULL) {
    if (!(arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE)) && !(arg->type.type == TYPE_LITERAL)) {
      arg_count++;
    }
    arg = arg->next;
  }
  fprintf(source, "    binding_argcheck(L, %d);\n", arg_count);

  switch (data->ud_type) {
    case UD_USERDATA:
      // extract the userdata
      fprintf(source, "    %s * ud = check_%s(L, 1);\n", data->name, data->sanatized_name);
      break;
    case UD_SINGLETON:
    case UD_GLOBAL:
      // this was bound early
      break;
    case UD_AP_OBJECT:
      // extract the userdata, it was a pointer, so we need to grab it
      fprintf(source, "    %s * ud = *check_%s(L, 1);\n", data->name, data->sanatized_name);
      fprintf(source, "    if (ud == NULL) {\n");
      fprintf(source, "        return luaL_error(L, \"Internal error, null pointer\");\n");
      fprintf(source, "    }\n");
      break;
  }

  // extract the arguments
  arg = method->arguments;
  arg_count = 2;
  int skipped = 0;
  while (arg != NULL) {
    if (arg->type.type != TYPE_LITERAL) {
      // emit_checker will emit a nullable argument for us
      emit_checker(arg->type, arg_count, skipped, "    ", "argument");
      arg_count++;
    }
    if (//arg->type.type == TYPE_LITERAL ||
        arg->type.flags & (TYPE_FLAGS_NULLABLE| TYPE_FLAGS_REFERNCE)) {
      skipped++;
    }
    arg = arg->next;
  }

  const char *ud_name = (data->flags & UD_FLAG_LITERAL)?data->name:"ud";
  const char *ud_access = (data->flags & UD_FLAG_REFERENCE)?".":"->";

  if (data->flags & UD_FLAG_SEMAPHORE) {
    fprintf(source, "    %s%sget_semaphore().take_blocking();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SEMAPHORE_POINTER) {
    fprintf(source, "    %s%sget_semaphore()->take_blocking();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SCHEDULER_SEMAPHORE) {
    fprintf(source, "    AP::scheduler().get_semaphore().take_blocking();\n");
  }

  int static_cast = TRUE;

  switch (method->return_type.type) {
    case TYPE_STRING:
      fprintf(source, "    const char * data = %s%s%s(", ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_ENUM:
      fprintf(source, "    const %s &data = %s%s%s(", method->return_type.data.enum_name, ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_USERDATA:
      if (strcmp(method->name, "copy") == 0) {
          // special case for copy method
          fprintf(source, "    const %s data = (*%s", method->return_type.data.ud.name, ud_name);
      } else {
          fprintf(source, "    const %s &data = %s%s%s(", method->return_type.data.ud.name, ud_name, ud_access, method->name);
      }
      static_cast = FALSE;
      break;
    case TYPE_AP_OBJECT:
      fprintf(source, "    %s *data = %s%s%s(", method->return_type.data.ud.name, ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_NONE:
      fprintf(source, "    %s%s%s(", ud_name, ud_access, method->name);
      static_cast = FALSE;
      break;
    case TYPE_LITERAL:
      error(ERROR_USERDATA, "Can't return a literal from a method");
      break;
    case TYPE_BOOLEAN:
    case TYPE_FLOAT:
    case TYPE_INT8_T:
    case TYPE_INT16_T:
    case TYPE_INT32_T:
    case TYPE_UINT8_T:
    case TYPE_UINT16_T:
    case TYPE_UINT32_T:
      break;
  }

  if (static_cast) {
    char *var_type_name;
    switch (method->return_type.type) {
      case TYPE_BOOLEAN:
        var_type_name = "bool";
        break;
      case TYPE_FLOAT:
        var_type_name = "float";
        break;
      case TYPE_INT8_T:
        var_type_name = "int8_t";
        break;
      case TYPE_INT16_T:
        var_type_name = "int16_t";
        break;
      case TYPE_INT32_T:
        var_type_name = "int32_t";
        break;
      case TYPE_UINT8_T:
        var_type_name = "uint8_t";
        break;
      case TYPE_UINT16_T:
        var_type_name = "uint16_t";
        break;
      case TYPE_UINT32_T:
        var_type_name = "uint32_t";
        break;
    case TYPE_STRING:
    case TYPE_ENUM:
    case TYPE_USERDATA:
    case TYPE_AP_OBJECT:
    case TYPE_NONE:
    case TYPE_LITERAL:
        error(ERROR_USERDATA, "Unexpected type");
        break;
    }
    fprintf(source, "    const %s data = static_cast<%s>(%s%s%s(", var_type_name, var_type_name, ud_name, ud_access, method->name);
  }

  if (arg_count != 2) {
    fprintf(source, "\n");
  }

  arg = method->arguments;
  arg_count = 2;
  while (arg != NULL) {
    switch (arg->type.type) {
      case TYPE_BOOLEAN:
      case TYPE_FLOAT:
      case TYPE_INT8_T:
      case TYPE_INT16_T:
      case TYPE_INT32_T:
      case TYPE_STRING:
      case TYPE_UINT8_T:
      case TYPE_UINT16_T:
      case TYPE_UINT32_T:
      case TYPE_ENUM:
      case TYPE_USERDATA:
      case TYPE_AP_OBJECT:
        fprintf(source, "            %sdata_%d", (arg->type.access == ACCESS_REFERENCE)?"&":"", arg_count + ((arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE)) ? NULLABLE_ARG_COUNT_BASE : 0));
        break;
      case TYPE_LITERAL:
        fprintf(source, "            %s", arg->type.data.literal);
        break;
      case TYPE_NONE:
        error(ERROR_INTERNAL, "Can't pass nil as an argument");
        break;
    }
    if (arg->type.type != TYPE_LITERAL) {
      arg_count++;
    }
    arg = arg->next;
    if (arg != NULL) {
            fprintf(source, ",\n");
    }
  }
  if (static_cast) {
    fprintf(source, "%s));\n\n", "");
  } else {
    fprintf(source, "%s);\n\n", "");
  }

  if (data->flags & UD_FLAG_SEMAPHORE) {
    fprintf(source, "    %s%sget_semaphore().give();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SEMAPHORE_POINTER) {
    fprintf(source, "    %s%sget_semaphore()->give();\n", ud_name, ud_access);
  } else if (data->flags & UD_FLAG_SCHEDULER_SEMAPHORE) {
    fprintf(source, "    AP::scheduler().get_semaphore().give();\n");
  }

  // we need to emit out refernce arguments, iterate the args again, creating and copying objects, while keeping a new count
  int return_count = 1; 
  if (method->flags & TYPE_FLAGS_REFERNCE) {
    arg = method->arguments;
    // number of arguments to return
    return_count += emit_references(arg,"    ");
  }

  switch (method->return_type.type) {
    case TYPE_BOOLEAN:
      if (method->flags & TYPE_FLAGS_NULLABLE) {
        fprintf(source, "    if (data) {\n");
        // we need to emit out nullable arguments, iterate the args again, creating and copying objects, while keeping a new count
        arg = method->arguments;
        return_count = emit_references(arg,"        ");
        fprintf(source, "        return %d;\n", return_count);
        fprintf(source, "    }\n");
        fprintf(source, "    return 0;\n");
      } else {
        fprintf(source, "    lua_pushboolean(L, data);\n");
      }
      break;
    case TYPE_FLOAT:
      fprintf(source, "    lua_pushnumber(L, data);\n");
      break;
    case TYPE_INT8_T:
    case TYPE_INT16_T:
    case TYPE_INT32_T:
    case TYPE_UINT8_T:
    case TYPE_UINT16_T:
    case TYPE_ENUM:
      fprintf(source, "    lua_pushinteger(L, data);\n");
      break;
    case TYPE_UINT32_T:
      fprintf(source, "        new_uint32_t(L);\n");
      fprintf(source, "        *static_cast<uint32_t *>(luaL_checkudata(L, -1, \"uint32_t\")) = data;\n");
      break;
    case TYPE_STRING:
      fprintf(source, "    lua_pushstring(L, data);\n");
      break;
    case TYPE_USERDATA:
      // userdatas must allocate a new container to return
      fprintf(source, "    new_%s(L);\n", method->return_type.data.ud.sanatized_name);
      fprintf(source, "    *check_%s(L, -1) = data;\n", method->return_type.data.ud.sanatized_name);
      break;
    case TYPE_AP_OBJECT:
      fprintf(source, "    if (data == NULL) {\n");
      fprintf(source, "        return 0;\n");
      fprintf(source, "    }\n");
      fprintf(source, "    new_%s(L);\n", method->return_type.data.ud.sanatized_name);
      fprintf(source, "    *check_%s(L, -1) = data;\n", method->return_type.data.ud.sanatized_name);
      break;
    case TYPE_NONE:
    case TYPE_LITERAL:
      // no return value, so don't worry about pushing a value
      return_count--;
      break;
  }

  if ((method->return_type.type != TYPE_BOOLEAN) || ((method->flags & TYPE_FLAGS_NULLABLE) == 0)) {
      fprintf(source, "    return %d;\n", return_count);
  }

  fprintf(source, "}\n");
  end_dependency(source, data->dependency);
  fprintf(source, "\n");

}

const char * get_name_for_operation(enum operator_type op) {
  switch (op) {
    case OP_ADD:
      return "__add";
    case OP_SUB:
      return "__sub";
    case OP_MUL:
      return "__mul";
      break;
    case OP_DIV:
      return "__div";
      break;
    case OP_MANUAL:
    case OP_LAST:
      return NULL;
  }
  return NULL;
}

void emit_operators(struct userdata *data) {
  trace(TRACE_USERDATA, "Emitting operators for %s", data->name);

  assert(data->ud_type == UD_USERDATA);

  for (uint32_t i = 1; i < OP_LAST; i = (i << 1)) {
    const char * op_name = get_name_for_operation((data->operations) & i);
    if (op_name == NULL) {
      continue;
    }

    char op_sym;
    switch ((data->operations) & i) {
      case OP_ADD:
        op_sym = '+';
        break;
      case OP_SUB:
        op_sym = '-';
        break;
      case OP_MUL:
        op_sym = '*';
        break;
      case OP_DIV:
        op_sym = '/';
        break;
      case OP_MANUAL:
      case OP_LAST:
        return;
    }

    fprintf(source, "static int %s_%s(lua_State *L) {\n", data->sanatized_name, op_name);
    // check number of arguments
    fprintf(source, "    binding_argcheck(L, 2);\n");
    // check the pointers
    fprintf(source, "    %s *ud = check_%s(L, 1);\n", data->name, data->sanatized_name);
    fprintf(source, "    %s *ud2 = check_%s(L, 2);\n", data->name, data->sanatized_name);
    // create a container for the result
    fprintf(source, "    new_%s(L);\n", data->sanatized_name);
    fprintf(source, "    *check_%s(L, -1) = *ud %c *ud2;\n", data->sanatized_name, op_sym);
    // return the first pointer
    fprintf(source, "    return 1;\n");
    fprintf(source, "}\n\n");

  }
}

void emit_methods(struct userdata *node) {
  while(node) {
    // methods
    struct method *method = node->methods;
    while(method) {
      emit_userdata_method(node, method);
      method = method->next;
    }

    // operators
    if (node->operations) {
      emit_operators(node);
    }
    node = node->next;
  }
}

void emit_enum(struct userdata * data) {
    fprintf(source, "struct userdata_enum %s_enums[] = {\n", data->sanatized_name);
    struct userdata_enum *ud_enum = data->enums;
    while (ud_enum != NULL) {
      fprintf(source, "    {\"%s\", %s::%s},\n", ud_enum->name, data->name, ud_enum->name);
      ud_enum = ud_enum->next;
    }
    fprintf(source, "};\n\n");
}

void emit_index(struct userdata *head) {

  struct userdata * node = head;
  while(node) {
    start_dependency(source, node->dependency);

    fprintf(source, "const luaL_Reg %s_meta[] = {\n", node->sanatized_name);

    struct method *method = node->methods;
    while (method) {
      fprintf(source, "    {\"%s\", %s_%s},\n", method->rename ? method->rename :  method->name, node->sanatized_name, method->name);
      method = method->next;
    }

    struct userdata_field *field = node->fields;
    while(field) {
      fprintf(source, "    {\"%s\", %s_%s},\n", field->rename ? field->rename : field->name, node->sanatized_name, field->name);
      field = field->next;
    }

    struct method_alias *alias = node->method_aliases;
    while(alias) {
      if (alias->type == ALIAS_TYPE_MANUAL) {
        fprintf(source, "    {\"%s\", %s},\n", alias->alias, alias->name);
      } else if (alias->type == ALIAS_TYPE_NONE) {
        fprintf(source, "    {\"%s\", %s_%s},\n", alias->alias, node->sanatized_name, alias->name);
      }
      alias = alias->next;
    }

    fprintf(source, "};\n\n");

    if (node->operations) {
      fprintf(source, "const luaL_Reg %s_operators[] = {\n", node->sanatized_name);
      for (uint32_t i = 1; i < OP_LAST; i = (i << 1)) {
        const char * op_name = get_name_for_operation((node->operations) & i);
        if (op_name == NULL) {
          continue;
        }
        fprintf(source, "    {\"%s\", %s_%s},\n", op_name, node->sanatized_name, op_name);
      }
      struct method_alias *alias = node->method_aliases;
      while(alias) {
        if (alias->type == ALIAS_TYPE_MANUAL_OPERATOR) {
          fprintf(source, "    {\"%s\", %s},\n", alias->alias, alias->name);
        }
        alias = alias->next;
      }
      fprintf(source, "    {NULL, NULL},\n");
      fprintf(source, "};\n\n");
    }

    if (node->enums != NULL) {
      emit_enum(node);
    }

    fprintf(source, "static int %s_index(lua_State *L) {\n", node->sanatized_name);
    fprintf(source, "    const char * name = luaL_checkstring(L, 2);\n");
    fprintf(source, "    if (load_function(L,%s_meta,ARRAY_SIZE(%s_meta),name)",node->sanatized_name,node->sanatized_name);
    if (node->enums != NULL) {
      fprintf(source, " || load_enum(L,%s_enums,ARRAY_SIZE(%s_enums),name)",node->sanatized_name,node->sanatized_name);
    }
    fprintf(source, ") {\n");
    fprintf(source, "        return 1;\n");
    fprintf(source, "    }\n");
    fprintf(source, "    return 0;\n");
    fprintf(source, "}\n");
    end_dependency(source, node->dependency);
    fprintf(source, "\n");
    node = node->next;
  }
}

void emit_type_index(struct userdata * data, char * meta_name) {
  fprintf(source, "const struct luaL_Reg %s_fun[] = {\n", meta_name);
  while (data) {
    start_dependency(source, data->dependency);
    fprintf(source, "    {\"%s\", %s_index},\n", data->rename ? data->rename : data->name, data->sanatized_name);
    end_dependency(source, data->dependency);
    data = data->next;
  }
  fprintf(source, "};\n\n");
}

void emit_type_index_with_operators(struct userdata * data, char * meta_name) {
  fprintf(source, "const struct userdata_meta %s_fun[] = {\n", meta_name);
  while (data) {
    start_dependency(source, data->dependency);
    if (data->operations == 0) {
      fprintf(source, "    {\"%s\", %s_index, nullptr},\n", data->rename ? data->rename : data->name, data->sanatized_name);
    } else {
      fprintf(source, "    {\"%s\", %s_index, %s_operators},\n", data->rename ? data->rename : data->name, data->sanatized_name, data->sanatized_name);
    }
    end_dependency(source, data->dependency);
    data = data->next;
  }
  fprintf(source, "};\n\n");
}

void emit_loaders(void) {

  emit_type_index_with_operators(parsed_userdata, "userdata");
  emit_type_index(parsed_singletons, "singleton");
  emit_type_index(parsed_ap_objects, "ap_object");

  fprintf(source, "void load_generated_bindings(lua_State *L) {\n");
  fprintf(source, "    luaL_checkstack(L, 5, \"Out of stack\");\n"); // this is more stack space then we need, but should never fail
  fprintf(source, "    // userdata metatables\n");
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(userdata_fun); i++) {\n");
  fprintf(source, "        luaL_newmetatable(L, userdata_fun[i].name);\n");
  fprintf(source, "        lua_pushcclosure(L, userdata_fun[i].func, 0);\n");
  fprintf(source, "        lua_setfield(L, -2, \"__index\");\n");

  fprintf(source, "        if (userdata_fun[i].operators != nullptr) {\n");
  fprintf(source, "            luaL_setfuncs(L, userdata_fun[i].operators, 0);\n");
  fprintf(source, "        }\n");

  fprintf(source, "        lua_pushstring(L, \"__call\");\n");
  fprintf(source, "        lua_pushvalue(L, -2);\n");
  fprintf(source, "        lua_settable(L, -3);\n");

  fprintf(source, "        lua_pop(L, 1);\n");
  fprintf(source, "        lua_newuserdata(L, 0);\n");
  fprintf(source, "        luaL_getmetatable(L, userdata_fun[i].name);\n");
  fprintf(source, "        lua_setmetatable(L, -2);\n");
  fprintf(source, "        lua_setglobal(L, userdata_fun[i].name);\n");
  fprintf(source, "    }\n");
  fprintf(source, "\n");

  fprintf(source, "    // ap object metatables\n");
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(ap_object_fun); i++) {\n");
  fprintf(source, "        luaL_newmetatable(L, ap_object_fun[i].name);\n");
  fprintf(source, "        lua_pushcclosure(L, ap_object_fun[i].func, 0);\n");
  fprintf(source, "        lua_setfield(L, -2, \"__index\");\n");
  fprintf(source, "        lua_pushstring(L, \"__call\");\n");
  fprintf(source, "        lua_pushvalue(L, -2);\n");
  fprintf(source, "        lua_settable(L, -3);\n");

  fprintf(source, "        lua_pop(L, 1);\n");
  fprintf(source, "        lua_newuserdata(L, 0);\n");
  fprintf(source, "        luaL_getmetatable(L, ap_object_fun[i].name);\n");
  fprintf(source, "        lua_setmetatable(L, -2);\n");
  fprintf(source, "        lua_setglobal(L, ap_object_fun[i].name);\n");
  fprintf(source, "    }\n");
  fprintf(source, "\n");

  fprintf(source, "    // singleton metatables\n");
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(singleton_fun); i++) {\n");
  fprintf(source, "        luaL_newmetatable(L, singleton_fun[i].name);\n");
  fprintf(source, "        lua_pushcclosure(L, singleton_fun[i].func, 0);\n");
  fprintf(source, "        lua_setfield(L, -2, \"__index\");\n");
  fprintf(source, "        lua_pushstring(L, \"__call\");\n");
  fprintf(source, "        lua_pushvalue(L, -2);\n");
  fprintf(source, "        lua_settable(L, -3);\n");

  fprintf(source, "        lua_pop(L, 1);\n");
  fprintf(source, "        lua_newuserdata(L, 0);\n");
  fprintf(source, "        luaL_getmetatable(L, singleton_fun[i].name);\n");
  fprintf(source, "        lua_setmetatable(L, -2);\n");
  fprintf(source, "        lua_setglobal(L, singleton_fun[i].name);\n");
  fprintf(source, "    }\n");

  fprintf(source, "\n");
  fprintf(source, "}\n\n");
}

void emit_sandbox(void) {
  struct userdata *data = parsed_userdata;
  fprintf(source, "const struct userdata {\n");
  fprintf(source, "    const char *name;\n");
  fprintf(source, "    const lua_CFunction fun;\n");
  fprintf(source, "} new_userdata[] = {\n");
  while (data) {
    start_dependency(source, data->dependency);
    if (data->creation) {
      // expose custom creation function to user (not used internally)
      fprintf(source, "    {\"%s\", %s},\n", data->rename ? data->rename :  data->name, data->creation);
    } else {
      fprintf(source, "    {\"%s\", new_%s},\n", data->rename ? data->rename :  data->name, data->sanatized_name);
    }
    end_dependency(source, data->dependency);
    data = data->next;
  }
  data = parsed_ap_objects;
  while (data) {
    start_dependency(source, data->dependency);
    fprintf(source, "    {\"%s\", new_%s},\n", data->name, data->sanatized_name);
    end_dependency(source, data->dependency);
    data = data->next;
  }
  if (parsed_globals) {
    struct method_alias *manual_aliases = parsed_globals->method_aliases;
    while (manual_aliases) {
      if (manual_aliases->type != ALIAS_TYPE_MANUAL) {
        error(ERROR_GLOBALS, "Globals only support manual methods");
      }
      fprintf(source, "    {\"%s\", %s},\n", manual_aliases->alias, manual_aliases->name);
      manual_aliases = manual_aliases->next;
    }
  }
  fprintf(source, "};\n\n");

  fprintf(source, "void load_generated_sandbox(lua_State *L) {\n");
  // load the singletons
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(singleton_fun); i++) {\n");
  fprintf(source, "        lua_pushstring(L, singleton_fun[i].name);\n");
  fprintf(source, "        lua_getglobal(L, singleton_fun[i].name);\n");
  fprintf(source, "        lua_settable(L, -3);\n");
  fprintf(source, "    }\n");

  // load the userdata allactors and globals
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(new_userdata); i++) {\n");
  fprintf(source, "        lua_pushstring(L, new_userdata[i].name);\n");
  fprintf(source, "        lua_pushcfunction(L, new_userdata[i].fun);\n");
  fprintf(source, "        lua_settable(L, -3);\n");
  fprintf(source, "    }\n");

  fprintf(source, "\n");
  fprintf(source, "}\n");
}

void emit_argcheck_helper(void) {
  // tagging this with NOINLINE can save a large amount of flash
  // but until we need it we will allow the compilier to choose to inline this for us
  fprintf(source, "int binding_argcheck(lua_State *L, int expected_arg_count) {\n");
  fprintf(source, "    const int args = lua_gettop(L);\n");
  fprintf(source, "    if (args > expected_arg_count) {\n");
  fprintf(source, "        return luaL_argerror(L, args, \"too many arguments\");\n");
  fprintf(source, "    } else if (args < expected_arg_count) {\n");
  fprintf(source, "        return luaL_argerror(L, args, \"too few arguments\");\n");
  fprintf(source, "    }\n");
  fprintf(source, "    return 0;\n");
  fprintf(source, "}\n\n");
}

void emit_not_supported_helper(void) {
  fprintf(source, "static int not_supported_error(lua_State *L, int arg, const char* name) {\n");
  fprintf(source, "    char error_msg[50];\n");
  fprintf(source, "    snprintf(error_msg, sizeof(error_msg), \"%%s not supported on this firmware\", name);\n");
  fprintf(source, "    return luaL_argerror(L, arg, error_msg);\n");
  fprintf(source, "}\n\n");
}

void emit_docs_type(struct type type, const char *prefix, const char *suffix) {
  switch (type.type) {
    case TYPE_BOOLEAN:
      fprintf(docs, "%s boolean%s", prefix, suffix);
      break;
    case TYPE_FLOAT:
      fprintf(docs, "%s number%s", prefix, suffix);
      break;
    case TYPE_INT8_T:
    case TYPE_INT16_T:
    case TYPE_INT32_T:
    case TYPE_UINT8_T:
    case TYPE_UINT16_T:
    case TYPE_ENUM:
      fprintf(docs, "%s integer%s", prefix, suffix);
      break;
    case TYPE_STRING:
      fprintf(docs, "%s string%s", prefix, suffix);
      break;
    case TYPE_UINT32_T:
      fprintf(docs, "%s uint32_t_ud%s", prefix, suffix);
      break;
    case TYPE_USERDATA: {
      // userdata may have rename
      struct userdata *data = parsed_userdata;
      int found = 0;
      while (data) {
        if (strcmp(type.data.ud.sanatized_name, data->sanatized_name) == 0) {
          found = 1;
          break;
        }
        data = data->next;
      }
      if (found == 0) {
        error(ERROR_GENERAL, "Could not find userdata %s", type.data.ud.sanatized_name);
      }
      fprintf(docs, "%s %s_ud%s", prefix, data->rename ? data->rename : data->sanatized_name, suffix);
      break;
    }
    case TYPE_AP_OBJECT:
      fprintf(docs, "%s %s_ud%s", prefix, type.data.ud.sanatized_name, suffix);
      break;
    case TYPE_NONE:
    case TYPE_LITERAL:
      break;
  }
}

void emit_docs_method(const char *name, const char *method_name, struct method *method) {

  fprintf(docs, "-- desc\n");

  if (method->deprecate != NULL) {
    fprintf(docs, "---@deprecated %s\n", method->deprecate);
  }

  struct argument *arg = method->arguments;
  int count = 1;
  // input arguments
  while (arg != NULL) {
    if ((arg->type.type != TYPE_LITERAL) && (arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE)) == 0) {
      char *param_name = (char *)allocate(20);
      sprintf(param_name, "---@param param%i", count);
      emit_docs_type(arg->type, param_name, "\n");
      free(param_name);
      count++;
    }
    arg = arg->next;
  }

  // return type
  if ((method->flags & TYPE_FLAGS_NULLABLE) == 0) {
    emit_docs_type(method->return_type, "---@return", "\n");
  }

  arg = method->arguments;
  // nulable and refences returns
  while (arg != NULL) {
    if ((arg->type.type != TYPE_LITERAL) && (arg->type.flags & (TYPE_FLAGS_NULLABLE | TYPE_FLAGS_REFERNCE))) {
      if (arg->type.flags & TYPE_FLAGS_NULLABLE) {
        emit_docs_type(arg->type, "---@return", "|nil\n");
      } else {
        emit_docs_type(arg->type, "---@return", "\n");
      }
    }
    arg = arg->next;
  }

  // function name
  fprintf(docs, "function %s:%s(", name, method_name);
  for (int i = 1; i < count; ++i) {
    fprintf(docs, "param%i", i);
    if (i < count-1) {
      fprintf(docs, ", ");
    }
  }
  fprintf(docs, ") end\n\n");
}

void emit_docs(struct userdata *node, int is_userdata, int emit_creation) {
  while(node) {
    char *name = (char *)allocate(strlen(node->rename ? node->rename : node->sanatized_name) + 5);
    if (is_userdata) {
      sprintf(name, "%s_ud", node->rename ? node->rename : node->sanatized_name);
    } else {
      sprintf(name, "%s", node->rename ? node->rename : node->sanatized_name);
    }


    fprintf(docs, "-- desc\n");
    fprintf(docs, "---@class %s\n", name);

    // enums
    if (node->enums != NULL) {
      struct userdata_enum *ud_enum = node->enums;
      while (ud_enum != NULL) {
        fprintf(docs, "---@field %s number\n", ud_enum->name);
        ud_enum = ud_enum->next;
      }
    }

    if (is_userdata) {
      // local userdata
      fprintf(docs, "local %s = {}\n\n", name);

      if (emit_creation) {
        // creation function
        fprintf(docs, "---@return %s\n", name);
        fprintf(docs, "function %s() end\n\n", node->rename ? node->rename : node->sanatized_name);
      }
    } else {
      // global
      fprintf(docs, "%s = {}\n\n", name);
    }


    // fields
    if (node->fields != NULL) {
      struct userdata_field *field = node->fields;
      while(field) {
          if (field->array_len == NULL) {
            // single value field
            if (field->access_flags & ACCESS_FLAG_READ) {
              fprintf(docs, "-- get field\n");
              emit_docs_type(field->type, "---@return", "\n");
              fprintf(docs, "function %s:%s() end\n\n", name, field->rename ? field->rename : field->name);
            }
            if (field->access_flags & ACCESS_FLAG_WRITE) {
              fprintf(docs, "-- set field\n");
              emit_docs_type(field->type, "---@param value", "\n");
              fprintf(docs, "function %s:%s(value) end\n\n", name, field->rename ? field->rename : field->name);
            }
          } else {
            // array field
            if (field->access_flags & ACCESS_FLAG_READ) {
              fprintf(docs, "-- get array field\n");
              fprintf(docs, "---@param index integer\n");
              emit_docs_type(field->type, "---@return", "\n");
              fprintf(docs, "function %s:%s(index) end\n\n", name, field->rename ? field->rename : field->name);
            }
            if (field->access_flags & ACCESS_FLAG_WRITE) {
              fprintf(docs, "-- set array field\n");
              fprintf(docs, "---@param index integer\n");
              emit_docs_type(field->type, "---@param value", "\n");
              fprintf(docs, "function %s:%s(index, value) end\n\n", name, field->rename ? field->rename : field->name);
            }
          }
        field = field->next;
      }
    }

    // methods
    struct method *method = node->methods;
    while(method) {
      emit_docs_method(name, method->rename ? method->rename : method->name, method);

      method = method->next;
    }

    // aliases
    struct method_alias *alias = node->method_aliases;
    while(alias) {
      // dont do manual bindings
      if (alias->type == ALIAS_TYPE_NONE) {
        // find the method this is a alias of
        struct method * method = node->methods;
        while (method != NULL && strcmp(method->name, alias->name)) {
          method = method-> next;
        }
        if (method == NULL) {
          error(ERROR_DOCS, "Could not fine Method %s to alias to %s", alias->name, alias->alias);
        }

        emit_docs_method(name, alias->alias, method);
      }
      alias = alias->next;
    }
    fprintf(docs, "\n");
    free(name);
    node = node->next;
  }
}


void emit_index_helpers(void) {
  fprintf(source, "static bool load_function(lua_State *L, const luaL_Reg *list, const uint8_t length, const char* name) {\n");
  fprintf(source, "    for (uint8_t i = 0; i < length; i++) {\n");
  fprintf(source, "        if (strcmp(name,list[i].name) == 0) {\n");
  fprintf(source, "            lua_pushcfunction(L, list[i].func);\n");
  fprintf(source, "            return true;\n");
  fprintf(source, "        }\n");
  fprintf(source, "    }\n");
  fprintf(source, "    return false;\n");
  fprintf(source, "}\n\n");

  fprintf(source, "static bool load_enum(lua_State *L, const userdata_enum *list, const uint8_t length, const char* name) {\n");
  fprintf(source, "    for (uint8_t i = 0; i < length; i++) {\n");
  fprintf(source, "        if (strcmp(name,list[i].name) == 0) {\n");
  fprintf(source, "            lua_pushinteger(L, list[i].value);\n");
  fprintf(source, "            return true;\n");
  fprintf(source, "        }\n");
  fprintf(source, "    }\n");
  fprintf(source, "    return false;\n");
  fprintf(source, "}\n\n");
}

void emit_structs(void) {
  // emit the enum header
  fprintf(source, "struct userdata_enum {\n");
  fprintf(source, "    const char *name;\n");
  fprintf(source, "    int value;\n");
  fprintf(source, "};\n\n");

  // emit the meta table header
  fprintf(source, "struct userdata_meta {\n");
  fprintf(source, "    const char *name;\n");
  fprintf(source, "    lua_CFunction func;\n");
  fprintf(source, "    const luaL_Reg *operators;\n");
  fprintf(source, "};\n\n");
}

char * output_path = NULL;
char * docs_path = NULL;

int main(int argc, char **argv) {
  state.line_num = -1;

  int c;
  while ((c = getopt(argc, argv, "i:o:d:")) != -1) {
    switch (c) {
      case 'i':
        if (description != NULL) {
          error(ERROR_GENERAL, "Already loaded a description file");
        }
        trace(TRACE_GENERAL, "Loading a description file: %s", optarg);
        description = fopen(optarg, "r");
        if (description == NULL) {
          error(ERROR_GENERAL, "Unable to load the description file: %s", optarg);
        }
        break;
      case 'o':
        if (output_path != NULL) {
          error(ERROR_GENERAL, "An output path was already selected.");
        }
        output_path = optarg;
        trace(TRACE_GENERAL, "Loading an output path of %s", output_path);
        break;
      case 'd':
        if (docs_path != NULL) {
          error(ERROR_GENERAL, "An docs path was already selected.");
        }
        docs_path = optarg;
        trace(TRACE_GENERAL, "Loading an docs path of %s", docs_path);
        break;
    }
  }

  if (output_path == NULL) {
    error(ERROR_GENERAL, "An output path must be provided for the generated bindings");
  }

  state.line_num = 0;

  while (start_line()) {

    // identify line type
    if (state.token != NULL) {
      // we have input
      if (strcmp(state.token, keyword_comment) == 0) {
        while (next_token()) {}
        // nothing to do here, jump to the next line
      } else if (strcmp(state.token, keyword_include) == 0) {
        handle_header();
      } else if (strcmp (state.token, keyword_userdata) == 0){
        handle_userdata();
      } else if (strcmp (state.token, keyword_singleton) == 0){
        handle_singleton();
      } else if (strcmp (state.token, keyword_ap_object) == 0){
        handle_ap_object();
      } else if (strcmp (state.token, keyword_global) == 0){
        handle_global();
      } else {
        error(ERROR_UNKNOWN_KEYWORD, "Expected a keyword, got: %s", state.token);
      }

      if (next_token()) {
        error(ERROR_UNKNOWN_KEYWORD, "Extra token provided: %s", state.token);
      }
    }
  }

  state.line_num = -1;

  char *file_name = (char *)allocate(strlen(output_path) + 5);
  sprintf(file_name, "%s.cpp", output_path);
  source = fopen(file_name, "w");
  if (source == NULL) {
    error(ERROR_GENERAL, "Unable to open the output source file: %s", file_name);
  }

  fprintf(source, "// auto generated bindings, don't manually edit.  See README.md for details.\n");
  
  trace(TRACE_GENERAL, "Sanity checking parsed input");

  sanity_check_userdata();

  fprintf(source, "#pragma GCC optimize(\"Os\")\n");
  fprintf(source, "#include \"lua_generated_bindings.h\"\n");
  fprintf(source, "#include <AP_Scripting/lua_boxed_numerics.h>\n");
  fprintf(source, "#include <AP_Scripting/lua_bindings.h>\n");

  // for set_and_print_new_error_message deprecate warning
  fprintf(source, "#include <AP_Scripting/lua_scripts.h>\n");

  trace(TRACE_GENERAL, "Starting emission");

  emit_headers(source);

  fprintf(source, "\n\n");

  emit_argcheck_helper();

  emit_not_supported_helper();

  emit_structs();

  emit_index_helpers();

  emit_userdata_allocators();

  emit_userdata_checkers();

  emit_ap_object_allocators();

  emit_ap_object_checkers();



  emit_singleton_fields();
  emit_methods(parsed_singletons);
  emit_index(parsed_singletons);

  emit_userdata_fields();
  emit_methods(parsed_userdata);
  emit_index(parsed_userdata);


  emit_methods(parsed_ap_objects);
  emit_index(parsed_ap_objects);


  emit_loaders();

  emit_sandbox();

  fclose(source);
  source = NULL;

  sprintf(file_name, "%s.h", output_path);
  header = fopen(file_name, "w");
  if (header == NULL) {
    error(ERROR_GENERAL, "Unable to open the output header file: %s", file_name);
  }
  free(file_name);
  fprintf(header, "#pragma once\n");
  fprintf(header, "// auto generated bindings, don't manually edit.  See README.md for details.\n");
  fprintf(header, "#include <AP_Vehicle/AP_Vehicle_Type.h> // needed for APM_BUILD_TYPE #if\n");
  emit_headers(header);
  fprintf(header, "#include <AP_Scripting/lua/src/lua.hpp>\n");
  fprintf(header, "#include <new>\n\n");

  emit_userdata_declarations();
  emit_ap_object_declarations();

  fprintf(header, "void load_generated_bindings(lua_State *L);\n");
  fprintf(header, "void load_generated_sandbox(lua_State *L);\n");
  fprintf(header, "int binding_argcheck(lua_State *L, int expected_arg_count);\n");

  fclose(header);
  header = NULL;

  if (docs_path == NULL) {
    // no docs to generate, all done
    return 0;
  }

  docs = fopen(docs_path, "w");
  if (docs == NULL) {
    error(ERROR_GENERAL, "Unable to open the output docs file: %s", docs_path);
  }

  fprintf(docs, "-- ArduPilot lua scripting documentation in EmmyLua Annotations\n");
  fprintf(docs, "-- This file should be auto generated and then manual edited\n");
  fprintf(docs, "-- generate with --scripting-docs, eg  ./waf copter --scripting-docs\n");
  fprintf(docs, "-- see: https://github.com/sumneko/lua-language-server/wiki/EmmyLua-Annotations\n\n");

  emit_docs(parsed_userdata, TRUE, TRUE);

  emit_docs(parsed_ap_objects, TRUE, FALSE);

  emit_docs(parsed_singletons, FALSE, FALSE);

  fclose(docs);

  return 0;
}
