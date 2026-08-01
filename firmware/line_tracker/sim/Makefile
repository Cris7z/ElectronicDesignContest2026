CC ?= gcc
CFLAGS := -std=c11 -Wall -Wextra -Werror -pedantic -I../core

.PHONY: test clean

test: test_line_tracker test_wheel_speed_pi test_lap_monitor test_task_mode test_line_calibration
	./test_line_tracker
	./test_wheel_speed_pi
	./test_lap_monitor
	./test_task_mode
	./test_line_calibration

test_line_tracker: test_line_tracker.c ../core/line_tracker.c ../core/line_tracker.h
	$(CC) $(CFLAGS) test_line_tracker.c ../core/line_tracker.c -o $@

test_wheel_speed_pi: test_wheel_speed_pi.c ../core/wheel_speed_pi.c ../core/wheel_speed_pi.h
	$(CC) $(CFLAGS) test_wheel_speed_pi.c ../core/wheel_speed_pi.c -o $@

test_lap_monitor: test_lap_monitor.c ../core/lap_monitor.c ../core/lap_monitor.h
	$(CC) $(CFLAGS) test_lap_monitor.c ../core/lap_monitor.c -o $@

test_task_mode: test_task_mode.c ../core/task_mode.c ../core/task_mode.h
	$(CC) $(CFLAGS) test_task_mode.c ../core/task_mode.c -o $@

test_line_calibration: test_line_calibration.c ../core/line_calibration.c ../core/line_calibration.h ../core/line_calibration_session.c ../core/line_calibration_session.h
	$(CC) $(CFLAGS) test_line_calibration.c ../core/line_calibration.c ../core/line_calibration_session.c -o $@

clean:
	@if exist test_line_tracker.exe del /q test_line_tracker.exe
	@if exist test_line_tracker del /q test_line_tracker
	@if exist test_wheel_speed_pi.exe del /q test_wheel_speed_pi.exe
	@if exist test_wheel_speed_pi del /q test_wheel_speed_pi
	@if exist test_lap_monitor.exe del /q test_lap_monitor.exe
	@if exist test_lap_monitor del /q test_lap_monitor
	@if exist test_task_mode.exe del /q test_task_mode.exe
	@if exist test_task_mode del /q test_task_mode
	@if exist test_line_calibration.exe del /q test_line_calibration.exe
	@if exist test_line_calibration del /q test_line_calibration
