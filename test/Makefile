TARGET := test.out

CC := g++
CPPFLAGS := -std=c++11 -Wall -Wextra -Wpedantic -g
LIB_DIR := ~/.local/lib/
INCLUDE_DIR := ~/.local/include/
LIBS := -lhorizonlink

$(TARGET): main.cpp
	$(CC) $(CPPFLAGS) -o $@ $^ $(addprefix -I , $(INCLUDE_DIR)) $(addprefix -L , $(LIB_DIR)) $(LIBS)

.PHONY: rebuild clean all
rebuild: clean all
all: $(TARGET)
clean:
	rm -rf *.out *.o *.dSYM
