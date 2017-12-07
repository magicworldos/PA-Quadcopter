# libs Makefile
include ../Makefile.common
LIBS+=libstm32.a
CFLAGSlib+=-c

all: libs

libs: $(LIBS)

libstm32.a:
	@echo -n "Building $@ ..."
	@cd $(STMLIB)/CMSIS/Device/ST/$(LIBNAME_H)/Source/Templates && \
		$(CC) $(CFLAGSlib) \
			system_$(LIBNAME_L).c
	@cd $(STMLIB)/$(LIBNAME_H)_StdPeriph_Driver/src && \
		$(CC) $(CFLAGSlib) \
			-D"assert_param(expr)=((void)0)" \
			-I../../CMSIS/Include \
			-I../../CMSIS/Device/ST/$(LIBNAME_H)/Include \
			-I../inc \
			*.c
#	@cd $(STMLIB)/STM32_USB-FS-Device_Driver/src && \
#	$(CC) $(CFLAGSlib) \
#	-D"assert_param(expr)=((void)0)" \
#	-I../../CMSIS/Include \
#	-I../../CMSIS/Device/ST/$(LIBNAME_H)/Include \
#	-I../inc \
#	*.c
	@$(AR) cr $(LIBDIR)/$@ \
		$(STMLIB)/CMSIS/Device/ST/$(LIBNAME_H)/Source/Templates/system_$(LIBNAME_L).o \
		$(STMLIB)/$(LIBNAME_H)_StdPeriph_Driver/src/*.o \
#		$(STMLIB)/STM32_USB-FS-Device_Driver/src/*.o
		@echo "done."
.PHONY: libs clean tshow

clean:
	rm -f $(STMLIB)/CMSIS/Device/ST/$(LIBNAME_H)/Source/Templates/system_$(LIBNAME_L).o
	rm -f $(STMLIB)/$(LIBNAME_H)_StdPeriph_Driver/src/*.o
	rm -f $(STMLIB)/STM32_USB-FS-Device_Driver/src/*.o
	rm -f $(LIBS)
tshow:
	@echo "######################################################################################################"
	@echo "################# optimize settings: $(InfoTextLib), $(InfoTextSrc)"
	@echo "######################################################################################################"
