CC=g++
CFLAGS += -g $(shell pkg-config --cflags)
LIBS += $(shell pkg-config --cflags --libs opencv)
SOURCES=dc1394test.cpp init.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLES=dc1394test init

all: $(EXECUTABLES)

$(EXECUTABLES): $(SOURCES) 
		$(CC) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	rm $(EXECUTABLES)
