CC ?= gcc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -I../core

.PHONY: test clean

test: test_line_tracker test_wheel_speed_pi
	./test_line_tracker
	./test_wheel_speed_pi

test_line_tracker: test_line_tracker.c ../core/line_tracker.c ../core/line_tracker.h
	$(CC) $(CFLAGS) test_line_tracker.c ../core/line_tracker.c -o $@

test_wheel_speed_pi: test_wheel_speed_pi.c ../core/wheel_speed_pi.c ../core/wheel_speed_pi.h
	$(CC) $(CFLAGS) test_wheel_speed_pi.c ../core/wheel_speed_pi.c -o $@

clean:
	@if exist test_line_tracker.exe del /q test_line_tracker.exe
	@if exist test_line_tracker del /q test_line_tracker
	@if exist test_wheel_speed_pi.exe del /q test_wheel_speed_pi.exe
	@if exist test_wheel_speed_pi del /q test_wheel_speed_pi
