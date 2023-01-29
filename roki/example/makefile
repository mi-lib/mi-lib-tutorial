CC=gcc
CFLAGS= -Wall -O3 `roki-config --cflags`
LINK=`roki-config -l`

%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean:
	-rm -f *.o *~
