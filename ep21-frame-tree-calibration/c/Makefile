CC=gcc
CFLAGS=-std=c99 -Wall -Wextra -O2
all: frame_chain pinhole stereo
%: %.c
	$(CC) $(CFLAGS) $< -o $@ -lm
