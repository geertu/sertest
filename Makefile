CC = $(CROSS_COMPILE)gcc
OFLAGS = -O3 -fomit-frame-pointer
CFLAGS = -Wall -Werror $(OFLAGS) -g
LFLAGS = -lbsd -lbrahe -lpthread -lm

TARGET =	sertest

SRCS += $(wildcard *.c)
OBJS += $(subst .c,.o,$(SRCS))
HDRS += $(wildcard *.h)

ifneq ($(V),1)
Q = @
endif

all:		$(TARGET)

.PHONY:		all clean

$(TARGET):	$(OBJS)
		@echo LD $@
		$(Q)$(CC) -o $@ $(OBJS) $(LFLAGS)

%.o:		%.c $(HDRS)
		@echo CC $<
		$(Q)$(CC) -c $(CFLAGS) -o $@ $<

clean:
		@echo CLEAN
		$(Q)$(RM) $(TARGET) $(OBJS)
