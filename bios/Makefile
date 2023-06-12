.PHONY: all
all: bios.ihx example.ihx randbcd.ihx break_test.ihx

bios.ihx: bin/bios.rel
	sdldz80 -n -i $@ $^

%.ihx: bin/bios.rel bin/init.rel bin/%.rel
	sdldz80 -nmw -b _CODE=0x100 -i $@ $^

bin/%.rel: src/%.s
	mkdir -p $(@D)
	sdasz80 -ol $@ $<

bin/%.rel: src/%.c
	mkdir -p $(@D)
	sdcc -mz80 -o $@ -c $<
