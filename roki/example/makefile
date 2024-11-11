CC=gcc
CFLAGS=-Wall -O3 `roki-config --cflags`
LINK=`roki-config -l`

TARGET=$(shell ls *.c | xargs -i basename {} .c | tr -s "\n" " ")

all: $(TARGET)
%: %.c
	$(CC) $(CFLAGS) -o $@ $< $(LINK)
clean:
	-rm -f *.o *~ core $(TARGET)
