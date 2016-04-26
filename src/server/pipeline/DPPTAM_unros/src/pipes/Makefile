CC=gcc
CFLAGS=-Wall -g -I.

pipetest: pipefib
	$(CC) $(CFLAGS) -o $@ $@.c

pipefib:
	$(CC) $(CFLAGS) -o $@ $@.c

.PHONY: clean
clean:
	rm -f *.o *~ pipefib pipetest
