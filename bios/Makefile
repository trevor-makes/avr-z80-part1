BIN = bios.ihx example.ihx randbcd.ihx break_test.ihx worm.ihx

.PHONY: all
all: $(addprefix bin/, $(BIN))

bin/bios.ihx: build/bios.rel
	mkdir -p $(@D)
	sdldz80 -n -i $@ $^

bin/%.ihx: build/bios.rel build/init.rel build/%.rel
	mkdir -p $(@D)
	sdldz80 -nmw -b _CODE=0x100 -i $@ $^

build/%.rel: src/%.s
	mkdir -p $(@D)
	sdasz80 -ol $@ $<

build/%.rel: src/%.c
	mkdir -p $(@D)
	sdcc -mz80 -o $@ -c $<
