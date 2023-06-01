APP1 = uio-gpio-test
APP2 = uio-gpio-pwm-test

# Add any other object files to this list below
APP1_OBJS = uio-gpio-test.o
APP2_OBJS = uio-gpio-pwm-test.o

LDLIBS := $(LDLIBS) -luio

all: build

build: $(APP1) $(APP2)

$(APP1): $(APP1_OBJS)
	$(CC) -o $@ $(APP1_OBJS) $(LDFLAGS) $(LDLIBS)
clean:
	rm -f $(APP1) *.o

$(APP2): $(APP2_OBJS)
	$(CC) -o $@ $(APP2_OBJS) $(LDFLAGS) $(LDLIBS)
clean:
	rm -f $(APP2) *.o
