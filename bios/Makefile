.PHONY: all
all: bios.ihx example.ihx

bios.ihx: bin/bios.rel
	sdldz80 -nmw -i $@ $^

example.ihx: bin/bios.rel bin/init.rel bin/example.rel
	sdldz80 -nmw -b _CODE=0x100 -i $@ $^

bin/%.rel: src/%.s
	mkdir -p $(@D)
	sdasz80 -o $@ $<

bin/%.rel: src/%.c
	mkdir -p $(@D)
	sdcc -mz80 -o $@ -c $<
