# general Makefile

include Makefile.common

DEVICE_NAME=/dev/ttyUSB0

LDFLAGS=$(COMMONFLAGS) -fno-exceptions -ffunction-sections -fdata-sections -L$(LIBDIR) -nostartfiles -Wl,--gc-sections,-Tlinker.ld
LDLIBS+=-lm
LDLIBS+=-lstm32

all: ctags mkdir src linux
	$(CC) -o $(PATH_TARGET)/$(PROGRAM).elf $(LDFLAGS) \
		-Wl,--whole-archive \
		src/$(PATH_TARGET)/app.a \
		-Wl,--no-whole-archive \
		$(LDLIBS)
	$(OBJCOPY) -O ihex $(PATH_TARGET)/$(PROGRAM).elf $(PATH_TARGET)/$(PROGRAM).hex
	$(OBJCOPY) -O binary $(PATH_TARGET)/$(PROGRAM).elf $(PATH_TARGET)/$(PROGRAM).bin
	arm-none-eabi-readelf -a $(PATH_TARGET)/$(PROGRAM).elf > $(PATH_TARGET)/$(PROGRAM).info_elf
	arm-none-eabi-size -d -B -t $(PATH_TARGET)/$(PROGRAM).elf > $(PATH_TARGET)/$(PROGRAM).info_size
	arm-none-eabi-objdump -S $(PATH_TARGET)/$(PROGRAM).elf > $(PATH_TARGET)/$(PROGRAM).info_code
	arm-none-eabi-nm -t d -S --size-sort -s $(PATH_TARGET)/$(PROGRAM).elf > $(PATH_TARGET)/$(PROGRAM).info_symbol

.PHONY: ctags mkdir src linux clean

ctags:
	ctags -R --c++-kinds=+p --fields=+iaS --extra=+q .
	
mkdir:
	mkdir -p $(PATH_TARGET)

src:
	$(MAKE) -C src $@
	
linux:
	$(MAKE) -C linux $@
	
install:
	stm32flash -w $(PATH_TARGET)/$(PROGRAM).bin -v -g 0x0 $(DEVICE_NAME)

clean:
	$(MAKE) -C src $@
	$(MAKE) -C linux $@
	rm -f $(PATH_TARGET)/*
