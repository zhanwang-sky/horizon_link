TARGET := libhorizonlink.a
HEADERS := horizon_link.h

CC := gcc
AR := ar
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -g

$(TARGET): horizon_link.o
	$(AR) -rc $@ $^
horizon_link.o: horizon_link.c horizon_link.h
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: install rebuild clean all
install:
	cp $(TARGET) ~/.local/lib/
	cp $(HEADERS) ~/.local/include/
rebuild: clean all
all: $(TARGET)
clean:
	rm -rf *.o *.a
