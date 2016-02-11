CC = g++

CFLAGS = -std=c++11 -O3

all: main.cpp
	$(CC) $(CFLAGS) main.cpp $(OBJS) -o main $(LIBS)

