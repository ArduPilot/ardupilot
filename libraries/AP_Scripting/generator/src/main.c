#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <readline/readline.h>
#include <string.h>
#include <unistd.h>

char keyword_comment[]   = "--";
char keyword_field[]     = "field";
char keyword_include[]   = "include";
char keyword_method[]    = "method";
char keyword_read[]      = "read";
char keyword_singleton[] = "singleton";
char keyword_userdata[]  = "userdata";
char keyword_write[]     = "write";

// type keywords
char keyword_boolean[] = "boolean";
char keyword_float[]   = "float";
char keyword_int32_t[] = "int32_t";
char keyword_string[]  = "string";
char keyword_void[]    = "void";

enum error_codes {
  ERROR_OUT_OF_MEMORY   = 1, // ran out of memory
  ERROR_HEADER          = 2, // header keyword not followed by a header to include
  ERROR_UNKNOWN_KEYWORD = 3, // a keyword we didn't know how to handle
  ERROR_USERDATA        = 4, // userdata
  ERROR_INTERNAL        = 5, // internal error of some form
  ERROR_GENERAL         = 6, // general error
  ERROR_SINGLETON       = 7, // singletons
};

struct header {
  struct header *next;
  char *name; // name of the header to include (not sanatized)
  int line; // line of the file declared on
};

struct generator_state {
  char *line;
  int line_num; // current line read in
  int token_num; // current token on the current line
  char *token;
};

FILE *header;
FILE *source;

static struct generator_state state = {};
static struct header * headers = NULL;

enum trace_level {
  TRACE_TOKENS    = (1 << 0),
  TRACE_HEADER    = (1 << 1),
  TRACE_GENERAL   = (1 << 2),
  TRACE_USERDATA  = (1 << 3),
  TRACE_SINGLETON = (1 << 4),
};

enum access_flags {
  ACCESS_FLAG_READ  = (1 << 0),
  ACCESS_FLAG_WRITE = (1 << 1),
};

enum field_type {
  TYPE_BOOLEAN,
  TYPE_FLOAT,
  TYPE_INT32_T,
  TYPE_NONE,
  TYPE_STRING,
  TYPE_USERDATA,
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

struct type {
  struct range_check *range;
  enum field_type type;
  enum access_type access;
  union {
    char *userdata_name;
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
  }
}

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

  exit(code);
}

char * next_token(void) {
  state.token = strtok(NULL, " ");
  state.token_num++;
  trace(TRACE_TOKENS, "Token %d:%d %s", state.line_num, state.token_num, state.token);
  if ((state.token!= NULL) && (strcmp(state.token, keyword_comment) == 0)) {
    trace(TRACE_TOKENS, "Detected comment %d", state.line_num);
    while (next_token()) {} // burn all the symbols
  }
  return state.token;
}

char * start_line(void) {
  if (state.line != NULL) {
    free(state.line);
  }

  while ((state.line = readline(NULL))) {
      state.line_num++;
    
      state.token = strtok(state.line, " ");
      state.token_num = 1;
      trace(TRACE_TOKENS, "Token %d:%d %s", state.line_num, state.token_num, state.token);

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
  headers = node;

  trace(TRACE_HEADER, "Added header %s", name);

  // ensure no more tokens on the line
  if (next_token()) {
    error(ERROR_HEADER, "Header contained an unexpected extra token: %s", state.token);
  }
}

struct userdata_field {
  struct userdata_field * next;
  char * name;
  struct type type; // field type, points to a string
  int line; // line declared on
  unsigned int access_flags;
};

struct argument {
  struct argument * next;
  struct type type;
};

struct method {
  struct method * next;
  char *name;
  int line; // line declared on
  struct type return_type;
  struct argument * arguments;
};

struct userdata {
  struct userdata * next;
  char * name;
  struct userdata_field *fields;
  struct method *methods;
};

static struct userdata *parsed_userdata = NULL;

// lazy helper that allocates a storage buffer and does strcpy for us
void string_copy(char **dest, const char * src) {
  *dest = (char *)allocate(strlen(src) + 1);
  strcpy(*dest, src);
}

struct range_check *parse_range_check(void) {
  char * low = next_token();
  if (low == NULL) {
    error(ERROR_USERDATA, "Missing low value for a range check");
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
    if (strcmp(state.token, keyword_read) == 0) {
      flags |= ACCESS_FLAG_READ;
    } else if (strcmp(state.token, keyword_write) == 0) {
      flags |= ACCESS_FLAG_WRITE;
      switch (type->type) {
        case TYPE_FLOAT:
        case TYPE_INT32_T:
          type->range = parse_range_check();
          break;
        case TYPE_USERDATA:
        case TYPE_BOOLEAN:
        case TYPE_STRING:
          // a range check is illogical
          break;
        case TYPE_NONE:
          error(ERROR_INTERNAL, "Can't access a NONE type");
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
  TYPE_REQUIRED,
  TYPE_OPTIONAL,
};

enum range_check_type {
  RANGE_CHECK_NONE,
  RANGE_CHECK_MANDATORY,
};

int parse_type(struct type *type, const enum type_restriction restrictions, enum range_check_type range_type) {
  char *data_type = next_token();

  if (data_type == NULL) {
    if (restrictions == TYPE_REQUIRED) {
      error(ERROR_USERDATA, "Data type must be specified");
    } else {
      return FALSE;
    }
  }

  if (data_type[0] == '&') {
    type->access = ACCESS_REFERENCE;
    data_type++; // drop the reference character
  } else {
    type->access = ACCESS_VALUE;
  }

  if (strcmp(data_type, keyword_boolean) == 0) {
    type->type = TYPE_BOOLEAN;
  } else if (strcmp(data_type, keyword_float) == 0) {
    type->type = TYPE_FLOAT;
  } else if (strcmp(data_type, keyword_int32_t) == 0) {
    type->type = TYPE_INT32_T;
  } else if (strcmp(data_type, keyword_string) == 0) {
    type->type = TYPE_STRING;
  } else if (strcmp(data_type, keyword_void) == 0) {
    type->type = TYPE_NONE;
  } else {
    // assume that this is a user data, we can't validate this until later though
    type->type = TYPE_USERDATA;
    string_copy(&(type->data.userdata_name), data_type);
  }

  if (range_type != RANGE_CHECK_NONE) {
    switch (type->type) {
      case TYPE_FLOAT:
      case TYPE_INT32_T:
              printf("loading a range");
        type->range = parse_range_check();
        break;
      case TYPE_BOOLEAN:
      case TYPE_NONE:
      case TYPE_STRING:
      case TYPE_USERDATA:
        // no sane range checks, so we can ignore this
        break;
    }
  }
  return TRUE;
}

void handle_userdata_field(struct userdata *data) {
  trace(TRACE_USERDATA, "Adding a userdata field");

  // find the field name
  char * field_name = next_token();
  if (field_name == NULL) {
    error(ERROR_USERDATA, "Missing a field name for userdata %s", data->name);
  }
  
  struct userdata_field * field = data->fields;
  while (field != NULL && strcmp(field->name, field_name)) {
    field = field-> next;
  }
  if (field != NULL) {
    error(ERROR_USERDATA, "Field %s already exsists in userdata %s (declared on %d)", field_name, data->name, field->line);
  }

  trace(TRACE_USERDATA, "Adding field %s", field_name);
  field = (struct userdata_field *)allocate(sizeof(struct userdata_field));
  field->next = data->fields;
  data->fields = field;
  field->line = state.line_num;
  string_copy(&(field->name), field_name);

  parse_type(&(field->type), TYPE_REQUIRED, RANGE_CHECK_NONE);
  field->access_flags = parse_access_flags(&(field->type));
}

void handle_method(enum trace_level traceType, char *parent_name, struct method **methods) {
  trace(traceType, "Adding a method");

  // find the field name
  char * name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Missing method name for %s", parent_name);
  }
  
  struct method * method = *methods;
  while (method != NULL && strcmp(method->name, name)) {
    method = method-> next;
  }
  if (method != NULL) {
    error(ERROR_USERDATA, "Method %s already exsists for %s (declared on %d)", name, parent_name, method->line);
  }

  trace(traceType, "Adding method %s", name);
  method = allocate(sizeof(struct method));
  method->next = *methods;
  *methods = method;
  string_copy(&(method->name), name);
  method->line = state.line_num;

  parse_type(&(method->return_type), TYPE_REQUIRED, RANGE_CHECK_NONE);

  // iterate the arguments
  struct type arg_type = {};
  while (parse_type(&arg_type, TYPE_OPTIONAL, RANGE_CHECK_MANDATORY)) {
    if (arg_type.type == TYPE_NONE) {
      error(ERROR_USERDATA, "Can't pass an empty argument to a method");
    }
    struct argument * arg = allocate(sizeof(struct argument));
    memcpy(&(arg->type), &arg_type, sizeof(struct type));
    arg->next = method->arguments;
    method->arguments = arg;
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
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    node->next = parsed_userdata;
    parsed_userdata = node;
  } else {
    trace(TRACE_USERDATA, "Found exsisting userdata for %s", name);
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_USERDATA, "Expected an access type for userdata %s", name);
  }

  // match type
  if (strcmp(type, keyword_field) == 0) {
    handle_userdata_field(node);
  } else if (strcmp(type, keyword_method) == 0) {
    handle_method(TRACE_USERDATA, node->name, &(node->methods));
  } else {
    error(ERROR_USERDATA, "Unknown or unsupported type for userdata: %s", type);
  }

}

struct singleton {
  struct singleton *next;
  char *name;
  struct method * methods;
};

struct singleton *parsed_singletons = NULL;

void handle_singleton(void) {
  trace(TRACE_SINGLETON, "Adding a singleton");

  char *name = next_token();
  if (name == NULL) {
    error(ERROR_USERDATA, "Expected a name for the singleton");
  }

  struct singleton *node = parsed_singletons;
  while (node != NULL && strcmp(node->name, name)) {
    node = node->next;
  }

  if (node == NULL) {
    trace(TRACE_SINGLETON, "Allocating new singleton for %s", name);
    node = (struct singleton *)allocate(sizeof(struct singleton));
    node->name = (char *)allocate(strlen(name) + 1);
    strcpy(node->name, name);
    node->next = parsed_singletons;
    parsed_singletons = node;
  }

  // read type
  char *type = next_token();
  if (type == NULL) {
    error(ERROR_SINGLETON, "Expected an access type for userdata %s", name);
  }
  if (strcmp(type, keyword_method) != 0) {
    error(ERROR_SINGLETON, "Singletons only support method access types (got %s)", type);
  }

  // method name
  handle_method(TRACE_USERDATA, node->name, &(node->methods));

}

void sanity_check_userdata(void) {
  struct userdata * node = parsed_userdata;
  while(node) {
    if ((node->fields == NULL) && (node->methods == NULL)) {
      error(ERROR_USERDATA, "Userdata %s has no fields or methods", node->name);
    }
    node = node->next;
  }
}

void emit_headers(FILE *f) {
  struct header *node = headers;
  while (node) {
    fprintf(f, "#include <%s>\n", node->name);
    node = node->next;
  }
}

void emit_userdata_allocators(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    fprintf(source, "int new_%s(lua_State *L) {\n", node->name);
    fprintf(source, "    %s *ud = (%s *)lua_newuserdata(L, sizeof(%s));\n", node->name, node->name, node->name);
    fprintf(source, "    new (ud) %s();\n", node->name);
    fprintf(source, "    luaL_getmetatable(L, \"%s\");\n", node->name);
    fprintf(source, "    lua_setmetatable(L, -2);\n");
    fprintf(source, "    return 1;\n");
    fprintf(source, "}\n\n");
    node = node->next;
  }
}

void emit_userdata_checkers(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    fprintf(source, "%s * check_%s(lua_State *L, int arg) {\n", node->name, node->name);
    fprintf(source, "    void *data = luaL_checkudata(L, arg, \"%s\");\n", node->name);
    fprintf(source, "    return (%s *)data;\n", node->name);
    fprintf(source, "}\n\n");
    node = node->next;
  }
}

void emit_userdata_declarations(void) {
  struct userdata * node = parsed_userdata;
  while (node) {
    fprintf(header, "int new_%s(lua_State *L);\n", node->name);
    fprintf(header, "%s * check_%s(lua_State *L, int arg);\n", node->name, node->name);
    node = node->next;
  }
}

void emit_range_check(const struct range_check *range, const char * name, const char *internal_name, const char *indentation) {
  if (range == NULL) {
    error(ERROR_INTERNAL, "Internal Error: Attempted to emit a range check for %s but no range check information was found", name);
  }
  fprintf(source, "%sluaL_argcheck(L, ((%s >= %s) && (%s <= %s)), 2, \"%s out of range\");\n",
          indentation != NULL ? indentation : "",
          internal_name, range->low,
          internal_name, range->high,
          name);
}

void emit_checker(const struct type t, int arg_number, const char *indentation, const char *name) {
  assert(indentation != NULL);

  // consider the arg numberto provide both the name, and the stack position of the variable
  switch (t.type) {
    case TYPE_BOOLEAN:
      fprintf(source, "%sconst bool data_%d = lua_toboolean(L, %d);\n", indentation, arg_number, arg_number);
      break;
    case TYPE_FLOAT:
      fprintf(source, "%sconst float data_%d = luaL_checknumber(L, %d);\n", indentation, arg_number, arg_number);
      break;
    case TYPE_INT32_T:
      fprintf(source, "%sconst int32_t data_%d = luaL_checkinteger(L, %d);\n", indentation, arg_number, arg_number);
      break;
    case TYPE_NONE:
      return; // nothing to do here, this should potentially be checked outside of this, but it makes an easier implementation to accept it
    case TYPE_STRING:
      fprintf(source, "%sconst char * data_%d = luaL_checkstring(L, %d);\n", indentation, arg_number, arg_number);
      break;
    case TYPE_USERDATA:
      fprintf(source, "%s%s & data_%d = *check_%s(L, %d);\n", indentation, t.data.userdata_name, arg_number, t.data.userdata_name, arg_number);
      break;
  }

  if (t.range != NULL) {
    fprintf(source, "%sluaL_argcheck(L, ((data_%d >= %s) && (data_%d <= %s)), %d, \"%s out of range\");\n",
            indentation,
            arg_number, t.range->low,
            arg_number, t.range->high,
            arg_number, name);
  }
}

void emit_userdata_field(const struct userdata *data, const struct userdata_field *field) {
  fprintf(source, "int %s_%s(lua_State *L) {\n", data->name, field->name);
  fprintf(source, "    %s *ud = check_%s(L, 1);\n", data->name, data->name);
  fprintf(source, "    switch(lua_gettop(L)) {\n");

  if (field->access_flags & ACCESS_FLAG_READ) {
    fprintf(source, "        case 1:\n");
    switch (field->type.type) {
      case TYPE_BOOLEAN:
        fprintf(source, "            lua_pushinteger(L, ud->%s);\n", field->name);
        break;
      case TYPE_FLOAT:
        fprintf(source, "            lua_pushnumber(L, ud->%s);\n", field->name);
        break;
      case TYPE_INT32_T:
        fprintf(source, "            lua_pushinteger(L, ud->%s);\n", field->name);
        break;
      case TYPE_NONE:
        error(ERROR_INTERNAL, "Can't access a NONE field");
        break;
      case TYPE_STRING:
        fprintf(source, "            lua_pushstring(L, ud->%s);\n", field->name);
        break;
      case TYPE_USERDATA:
        error(ERROR_USERDATA, "Userdata does not currently support accss to userdata field's");
        break;
    }
    fprintf(source, "            return 1;\n");
  }

  if (field->access_flags & ACCESS_FLAG_WRITE) {
    fprintf(source, "        case 2: {\n");
    emit_checker(field->type, 2, "            ", field->name);
    fprintf(source, "            ud->%s = data_2;\n", field->name);
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
    while(field) {
      emit_userdata_field(node, field);
      field = field->next;
    }
    node = node->next;
  }
}

void emit_userdata_method(const struct userdata *data, const struct method *method) {
  int arg_count = 1;
  struct argument *arg = method->arguments;
  while (arg != NULL) {
    arg_count++;
    arg = arg->next;
  }

  fprintf(source, "int %s_%s(lua_State *L) {\n", data->name, method->name);
  fprintf(source, "    const int args = lua_gettop(L);\n");
  fprintf(source, "    if (args > %d) {\n", arg_count);
  fprintf(source, "        return luaL_argerror(L, args, \"too many arguments\");\n");
  fprintf(source, "    } else if (args < %d) {\n", arg_count);
  fprintf(source, "        return luaL_argerror(L, args, \"too few arguments\");\n");
  fprintf(source, "    }\n\n");

  // extract the userdata
  fprintf(source, "    %s * ud = check_%s(L, 1);\n", data->name, data->name);

  // extract the arguments
  arg = method->arguments;
  arg_count = 2;
  while (arg != NULL) {
    emit_checker(arg->type, arg_count, "    ", "argument");
    arg = arg->next;
    arg_count++;
  }

  // we have all the types checked, emit the call
  switch (method->return_type.type) {
    case TYPE_BOOLEAN:
      fprintf(source, "    const bool data = ud->%s(\n", method->name);
      break;
    case TYPE_FLOAT:
      fprintf(source, "    const float data = ud->%s(\n", method->name);
      break;
    case TYPE_INT32_T:
      fprintf(source, "    const int32_t data = ud->%s(\n", method->name);
      break;
    case TYPE_STRING:
      fprintf(source, "    const char * data = ud->%s(\n", method->name);
      break;
    case TYPE_USERDATA:
      error(ERROR_USERDATA, "Userdata methods may not currently return a userdata object");
      break;
    case TYPE_NONE:
      fprintf(source, "    ud->%s(\n", method->name);
      break;
  }

  arg = method->arguments;
  arg_count = 2;
  while (arg != NULL) {
    fprintf(source, "            data_%d", arg_count);
    arg = arg->next;
    if (arg != NULL) {
            fprintf(source, ",\n");
    }
    arg_count++;
  }
  fprintf(source, ");\n\n");

  switch (method->return_type.type) {
    case TYPE_BOOLEAN:
      fprintf(source, "    lua_pushboolean(L, data);\n");
      break;
    case TYPE_FLOAT:
      fprintf(source, "    lua_pushnumber(L, data);\n");
      break;
    case TYPE_INT32_T:
      fprintf(source, "    lua_pushinteger(L, data);\n");
      break;
    case TYPE_STRING:
      fprintf(source, "    lua_pushstring(L, data);\n");
      break;
    case TYPE_USERDATA:
      error(ERROR_INTERNAL, "Can't return a userdata currently");
      break;
    case TYPE_NONE:
      // no return value, so don't worry about pushing a value
      break;
  }

  fprintf(source, "    return %d;\n", method->return_type.type != TYPE_NONE ? 1 : 0);

  fprintf(source, "}\n\n");
}

void emit_userdata_methods(void) {
  struct userdata * node = parsed_userdata;
  while(node) {
    struct method *method = node->methods;
    while(method) {
      emit_userdata_method(node, method);
      method = method->next;
    }
    node = node->next;
  }
}

void emit_singleton_method(const struct singleton *data, const struct method *method) {
  int arg_count = 1;
  struct argument *arg = method->arguments;
  while (arg != NULL) {
    arg_count++;
    arg = arg->next;
  }
  const char * dereference = ".";

  fprintf(source, "int %s_%s(lua_State *L) {\n", data->name, method->name);
  fprintf(source, "    const int args = lua_gettop(L);\n");
  fprintf(source, "    if (args > %d) {\n", arg_count);
  fprintf(source, "        return luaL_argerror(L, args, \"too many arguments\");\n");
  fprintf(source, "    } else if (args < %d) {\n", arg_count);
  fprintf(source, "        return luaL_argerror(L, args, \"too few arguments\");\n");
  fprintf(source, "    }\n\n");
  fprintf(source, "    luaL_checkudata(L, 1, \"%s\");\n", data->name);

  // extract the arguments
  arg = method->arguments;
  arg_count = 2;
  while (arg != NULL) {
    emit_checker(arg->type, arg_count, "    ", "argument");
    arg = arg->next;
    arg_count++;
  }


  switch (method->return_type.type) {
    case TYPE_BOOLEAN:
      fprintf(source, "    const bool data = AP::%s()%s%s(\n", data->name, dereference, method->name);
      break;
    case TYPE_FLOAT:
      fprintf(source, "    const float data = AP::%s()%s%s(\n", data->name, dereference, method->name);
      break;
    case TYPE_INT32_T:
      fprintf(source, "    const int32_t data = AP::%s()%s%s(\n", data->name, dereference, method->name);
      break;
    case TYPE_STRING:
      fprintf(source, "    const char * data = AP::%s()%s%s(\n", data->name, dereference, method->name);
      break;
    case TYPE_USERDATA:
      fprintf(source, "    const %s &data = AP::%s()%s%s(\n", method->return_type.data.userdata_name, data->name, dereference, method->name);
      break;
    case TYPE_NONE:
      fprintf(source, "    AP::%s()%s%s(\n", data->name, dereference, method->name);
      break;
  }

  arg = method->arguments;
  arg_count = 2;
  while (arg != NULL) {
    fprintf(source, "            data_%d", arg_count);
    arg = arg->next;
    if (arg != NULL) {
            fprintf(source, ",\n");
    }
    arg_count++;
  }
  fprintf(source, ");\n\n");


  switch (method->return_type.type) {
    case TYPE_BOOLEAN:
      fprintf(source, "    lua_pushboolean(L, data);\n");
      break;
    case TYPE_FLOAT:
      fprintf(source, "    lua_pushnumber(L, data);\n");
      break;
    case TYPE_INT32_T:
      fprintf(source, "    lua_pushinteger(L, data);\n");
      break;
    case TYPE_STRING:
      fprintf(source, "    lua_pushstring(L, data);\n");
      break;
    case TYPE_USERDATA:
      // userdatas must allocate a new container to return
      fprintf(source, "    new_%s(L);\n", method->return_type.data.userdata_name);
      fprintf(source, "    *check_%s(L, -1) = data;\n", method->return_type.data.userdata_name);
      break;
    case TYPE_NONE:
      // no return value, so don't worry about pushing a value
      break;
  }

  fprintf(source, "    return %d;\n", method->return_type.type != TYPE_NONE ? 1 : 0);

  fprintf(source, "}\n\n");
}

void emit_singleton_methods(void) {
  struct singleton * node = parsed_singletons;
  while(node) {
    struct method *method = node->methods;
    while(method) {
      emit_singleton_method(node, method);
      method = method->next;
    }
    node = node->next;
  }
}

void emit_userdata_metatables(void) {
  struct userdata * node = parsed_userdata;
  while(node) {
    fprintf(source, "const luaL_Reg %s_meta[] = {\n", node->name);

    struct userdata_field *field = node->fields;
    while(field) {
      fprintf(source, "    {\"%s\", %s_%s},\n", field->name, node->name, field->name);
      field = field->next;
    }

    struct method *method = node->methods;
    while(method) {
      fprintf(source, "    {\"%s\", %s_%s},\n", method->name, node->name, method->name);
      method = method->next;
    }

    fprintf(source, "    {NULL, NULL}\n");
    fprintf(source, "};\n\n");

    node = node->next;
  }
}

void emit_singleton_metatables(void) {
  struct singleton * node = parsed_singletons;
  while(node) {
    fprintf(source, "const luaL_Reg %s_meta[] = {\n", node->name);

    struct method *method = node->methods;
    while(method) {
      fprintf(source, "    {\"%s\", %s_%s},\n", method->name, node->name, method->name);
      method = method->next;
    }

    fprintf(source, "    {NULL, NULL}\n");
    fprintf(source, "};\n\n");

    node = node->next;
  }
}

void emit_loaders(void) {
  fprintf(source, "const struct userdata_fun {\n");
  fprintf(source, "    const char *name;\n");
  fprintf(source, "    const luaL_Reg *reg;\n");
  fprintf(source, "} userdata_fun[] = {\n");
  struct userdata * data = parsed_userdata;
  while (data) {
    fprintf(source, "    {\"%s\", %s_meta},\n", data->name, data->name);
    data = data->next;
  }
  fprintf(source, "};\n\n");

  fprintf(source, "const struct singleton_fun {\n");
  fprintf(source, "    const char *name;\n");
  fprintf(source, "    const luaL_Reg *reg;\n");
  fprintf(source, "} singleton_fun[] = {\n");
  struct singleton * single = parsed_singletons;
  while (single) {
    fprintf(source, "    {\"%s\", %s_meta},\n", single->name, single->name);
    single = single->next;
  }
  fprintf(source, "};\n\n");

  fprintf(source, "void load_generated_bindings(lua_State *L) {\n");
  fprintf(source, "    // userdata metatables\n");
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(userdata_fun); i++) {\n");
  fprintf(source, "        luaL_newmetatable(L, userdata_fun[i].name);\n");
  fprintf(source, "        luaL_setfuncs(L, userdata_fun[i].reg, 0);\n");
  fprintf(source, "        lua_pushstring(L, \"__index\");\n");
  fprintf(source, "        lua_pushvalue(L, -2);\n");
  fprintf(source, "        lua_settable(L, -3);\n");
  fprintf(source, "        lua_pop(L, 1);\n");
  fprintf(source, "    }\n");
  fprintf(source, "\n");

  fprintf(source, "    // singleton metatables\n");
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(singleton_fun); i++) {\n");
  fprintf(source, "        luaL_newmetatable(L, singleton_fun[i].name);\n");
  fprintf(source, "        luaL_setfuncs(L, singleton_fun[i].reg, 0);\n");
  fprintf(source, "        lua_pushstring(L, \"__index\");\n");
  fprintf(source, "        lua_pushvalue(L, -2);\n");
  fprintf(source, "        lua_settable(L, -3);\n");
  fprintf(source, "        lua_pop(L, 1);\n");
  fprintf(source, "        lua_newuserdata(L, 0);\n");
  fprintf(source, "        luaL_getmetatable(L, singleton_fun[i].name);\n");
  fprintf(source, "        lua_setmetatable(L, -2);\n");
  fprintf(source, "        lua_setglobal(L, singleton_fun[i].name);\n");
  fprintf(source, "    }\n");

  fprintf(source, "}\n\n");
}

void emit_sandbox(void) {
  struct singleton *single = parsed_singletons;
  fprintf(source, "const char *singletons[] = {\n");
  while (single) {
    fprintf(source, "    \"%s\",\n", single->name);
    single = single->next;
  }
  fprintf(source, "};\n\n");

  struct userdata *data = parsed_userdata;
  fprintf(source, "const struct userdata {\n");
  fprintf(source, "    const char *name;\n");
  fprintf(source, "    const lua_CFunction fun;\n");
  fprintf(source, "} new_userdata[] = {\n");
  while (data) {
    fprintf(source, "    {\"%s\", new_%s},\n", data->name, data->name);
    data = data->next;
  }
  fprintf(source, "};\n\n");

  fprintf(source, "void load_generated_sandbox(lua_State *L) {\n");
  // load the singletons
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(singletons); i++) {\n");
  fprintf(source, "        lua_pushstring(L, singletons[i]);\n");
  fprintf(source, "        lua_getglobal(L, singletons[i]);\n");
  fprintf(source, "        lua_settable(L, -3);\n");
  fprintf(source, "    }\n");

  // load the userdata allactors
  fprintf(source, "    for (uint32_t i = 0; i < ARRAY_SIZE(new_userdata); i++) {\n");
  fprintf(source, "        lua_pushstring(L, new_userdata[i].name);\n");
  fprintf(source, "        lua_pushcfunction(L, new_userdata[i].fun);\n");
  fprintf(source, "        lua_settable(L, -3);\n");
  fprintf(source, "    }\n");

  // load the userdata complex functions
  fprintf(source, "}\n");
}

char * output_path = NULL;

int main(int argc, char **argv) {
  state.line_num = -1;

  int c;
  while ((c = getopt(argc, argv, "o:")) != -1) {
    switch (c) {
      case 'o':
        if (output_path != NULL) {
          error(ERROR_GENERAL, "An output path was already selected.");
        }
        output_path = optarg;
        trace(TRACE_GENERAL, "Loading an output path of %s", output_path);
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

  fprintf(source, "// auto generated bindings, don't manually edit\n");
  
  trace(TRACE_GENERAL, "Sanity checking parsed input");

  sanity_check_userdata();

  fprintf(source, "#include \"lua_generated_bindings.h\"\n");

  trace(TRACE_GENERAL, "Starting emission");

  emit_headers(source);

  fprintf(source, "\n\n");

  emit_userdata_allocators();

  emit_userdata_checkers();

  emit_userdata_fields();

  emit_userdata_methods();

  emit_userdata_metatables();

  emit_singleton_methods();

  emit_singleton_metatables();

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
  fprintf(header, "// auto generated bindings, don't manually edit\n");
  emit_headers(header);
  fprintf(header, "#include \"lua/src/lua.hpp\"\n");
  fprintf(header, "#include <new>\n\n");

  emit_userdata_declarations();

  fprintf(header, "void load_generated_bindings(lua_State *L);\n");
  fprintf(header, "void load_generated_sandbox(lua_State *L);\n");

  fclose(header);
  header = NULL;

  return 0;
}
