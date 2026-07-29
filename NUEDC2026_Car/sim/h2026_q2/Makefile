CC = gcc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -pedantic
CPPFLAGS = -I../../firmware/h2026_q2/core
LDLIBS = -lm

TARGET = h2026_q2_tests.exe
SOURCES = test_h2026_q2.c ../../firmware/h2026_q2/core/h2026_q2.c

.PHONY: all test clean

all: test

$(TARGET): $(SOURCES) ../../firmware/h2026_q2/core/h2026_q2.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $(SOURCES) $(LDLIBS)

test: $(TARGET)
	./$(TARGET)

clean:
	cmd /C "if exist $(TARGET) del /Q $(TARGET)"
