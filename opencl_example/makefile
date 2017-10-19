
PREFIX=/usr/local/cuda/targets/x86_64-linux
CFLAGS=-L$(PREFIX)/lib -I$(PREFIX)/include -lOpenCL

vecadd: vecadd.c
	$(CC) -o $(@) $(^) $(CFLAGS)
