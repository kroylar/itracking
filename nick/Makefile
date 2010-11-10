CC=gcc
CFLAGS += -g $(shell pkg-config --cflags)
LIBS += $(shell pkg-config --cflags --libs opencv)
SOURCES=eyedetect.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLES=eyedetect

all: $(EXECUTABLES)

$(EXECUTABLES): $(SOURCES) 
		$(CC) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	rm $(EXECUTABLES)
