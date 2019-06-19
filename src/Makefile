TARGET := libhorizonlink.so
HEADERS := horizonlink.h

CC := gcc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -g

LIB_DIR := ~/.local/lib/
INCLUDE_DIR := ~/.local/include/

$(TARGET): horizonlink.o
	$(CC) -shared -o $@ $^
horizonlink.o: horizonlink.c horizonlink.h
	$(CC) $(CFLAGS) -c -fPIC -o $@ $<

.PHONY: install uninstall rebuild clean all
install:
	cp $(TARGET) $(LIB_DIR)
	cp $(HEADERS) $(INCLUDE_DIR)
uninstall:
	rm -f $(addprefix $(LIB_DIR), $(TARGET))
	rm -f $(addprefix $(INCLUDE_DIR), $(HEADERS))
rebuild: clean all
all: $(TARGET)
clean:
	rm -rf *.o *.so
