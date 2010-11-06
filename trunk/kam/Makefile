CC=g++
CFLAGS += -g $(shell pkg-config --cflags)
LIBS += $(shell pkg-config --cflags --libs opencv)
SOURCES=dc1394test.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLES=dc1394test boundary

all: $(EXECUTABLES)

$(EXECUTABLES): $(SOURCES) 
		$(CC) $(CFLAGS) $< -o $@ $(LIBS)
		
boundary: boundary.c
	gcc -g boundary.c -o boundary -lcurses

clean:
	rm $(EXECUTABLES)
