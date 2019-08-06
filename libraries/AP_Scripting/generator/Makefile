CC = gcc
CFLAGS = -Wall -Werror -Wextra -g -std=c99
LDFLAGS = -lreadline

OBJ_DIR = build/obj
SRC_DIR = src
OUT_DIR = build
OUT_NAME = gen-bindings
OUT = $(OUT_DIR)/$(OUT_NAME)

_OBJS = main.c.o
OBJS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

.PHONY: all
all: directories $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $(OUT)
	cp $(OUT) .

run: all
	./$(OUT) -o ../lua_generated_bindings -i description/bindings.desc

$(OBJ_DIR)/%.c.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: directories
directories:
	mkdir -p $(OUT_DIR)
	mkdir -p $(OBJ_DIR)

.PHONY: clean
clean:
	rm -rf $(OBJ_DIR)/*c.o
	rm -rf $(OUT_DIR)
	rm -rf $(OUT)
	rm -rf $(OUT_NAME)
