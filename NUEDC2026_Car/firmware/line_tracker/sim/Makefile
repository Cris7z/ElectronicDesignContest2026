CC ?= gcc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -I../core

.PHONY: test clean

test: test_line_tracker
	./test_line_tracker

test_line_tracker: test_line_tracker.c ../core/line_tracker.c ../core/line_tracker.h
	$(CC) $(CFLAGS) test_line_tracker.c ../core/line_tracker.c -o $@

clean:
	@if exist test_line_tracker.exe del /q test_line_tracker.exe
	@if exist test_line_tracker del /q test_line_tracker
