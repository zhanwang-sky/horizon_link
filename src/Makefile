CC := gcc
AR := ar
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -g
TARGET := libhlink.a

$(TARGET): horizon_link.o
	$(AR) -rc $@ $^
horizon_link.o: horizon_link.c horizon_link.h
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: rebuild clean all
rebuild: clean all
all: $(TARGET)
clean:
	rm -rf *.o *.a
