TARGET := fmc.out

CC := g++
LD := g++
CPPFLAGS := -std=c++11 -Wall -Wextra -Wpedantic -g
LIB_DIR := ~/.local/lib/
INCLUDE_DIR := ~/.local/include/
LIBS := -lapue -lhorizonlink

$(TARGET): main.o uart.o
	$(LD) -o $@ $^ $(addprefix -L , $(LIB_DIR)) $(LIBS)

main.o: main.cpp uart.h
	$(CC) $(CPPFLAGS) -c -o $@ $< $(addprefix -I , $(INCLUDE_DIR))

uart.o: uart.cpp uart.h
	$(CC) $(CPPFLAGS) -c -o $@ $< $(addprefix -I , $(INCLUDE_DIR))

.PHONY: rebuild clean all
rebuild: clean all
all: $(TARGET)
clean:
	rm -rf *.out *.o *.dSYM
