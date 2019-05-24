TARGET := libhorizonlink.so
HEADERS := horizon_link.h

CC := gcc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -g

LIB_DIR := ~/.local/lib/
INCLUDE_DIR := ~/.local/include/

$(TARGET): horizon_link.o
	$(CC) -shared -o $@ $^
horizon_link.o: horizon_link.c horizon_link.h
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
