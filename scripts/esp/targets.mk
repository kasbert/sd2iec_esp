# architecture-dependent additional targets and manual dependencies

# Program the device.
program: bin hex eep
	$(ESPTOOLS) write_flash $(ESPTOOLS_FLAGS) 0x0 $(OBJDIR)/eagle.app.flash.bin $(ROM_ADDRESS) $(OBJDIR)/eagle.app.v6.irom0text.bin

bin: $(OBJDIR)/sd2iec.elf
	$(RM) -f $(OBJDIR)/eagle.S $(OBJDIR)/eagle.dump
	$(OBJDUMP) -x -s $< > $(OBJDIR)/eagle.dump
	$(OBJDUMP) -S $< > $(OBJDIR)/eagle.S
	$(OBJCOPY) --only-section .text -O binary $< $(OBJDIR)/eagle.app.v6.text.bin
	$(OBJCOPY) --only-section .data -O binary $< $(OBJDIR)/eagle.app.v6.data.bin
	$(OBJCOPY) --only-section .rodata -O binary $< $(OBJDIR)/eagle.app.v6.rodata.bin
	$(OBJCOPY) --only-section .irom0.text -O binary $< $(OBJDIR)/eagle.app.v6.irom0text.bin
	rm -f $(OBJDIR)/eagle.app.flash.bin
	cd $(OBJDIR); COMPILE=gcc python $(ESP_SDK)/tools/gen_appbin.py sd2iec.elf 0 0 0 4 0
