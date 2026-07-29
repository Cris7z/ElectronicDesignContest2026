CC = gcc
CFLAGS = -O2 -Wall -Wextra -lm
SRC = sim_main.c \
      ../firmware/control/pid.c \
      ../firmware/control/chassis.c \
      ../firmware/control/odometry.c \
      ../firmware/control/track_ctrl.c \
      ../firmware/sensors/gray8.c \
      ../firmware/sensors/encoder.c \
      ../firmware/sensors/imu_jy61p.c \
      ../firmware/sensors/openmv_link.c \
      ../firmware/comm/radio_link.c \
      ../firmware/app/app_mission.c

H2026_SRC = h2026_sim.c \
      ../firmware/h2026/ball_link.c \
      ../firmware/h2026/ball_balance.c \
      ../firmware/h2026/linefollower_8ch.c \
      ../firmware/h2026/lf04.c

sim: $(SRC)
	$(CC) -o sim $(SRC) $(CFLAGS)

test: sim
	./sim

h2026_sim: $(H2026_SRC)
	$(CC) -o h2026_sim $(H2026_SRC) $(CFLAGS)

test-h2026: h2026_sim
	./h2026_sim

test-all: test test-h2026

clean:
	rm -f sim h2026_sim
