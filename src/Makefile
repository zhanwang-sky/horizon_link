TARGET := libhorizonlink.so
HEADERS := horizon_link.h

CC := gcc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -g

$(TARGET): horizon_link.o
	$(CC) -shared -fPIC -o $@ $^
horizon_link.o: horizon_link.c horizon_link.h
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: install rebuild clean all
install:
	cp $(TARGET) /usr/local/lib/
	cp $(HEADERS) /usr/local/include/
rebuild: clean all
all: $(TARGET)
clean:
	rm -rf *.o *.so
