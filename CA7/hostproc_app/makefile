#Makefile

SRC = $(wildcard *.c)
OBJ = $(SRC:.c=.o)
TARGET = hostproc_app
CFLAGS = -Wall -O2

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $^ $(CFLAGS) -o $@

%.o: %.c
	$(CC) -c $^ $(CFLAGS) -o $@


clean:
	rm *.o $(TARGET)