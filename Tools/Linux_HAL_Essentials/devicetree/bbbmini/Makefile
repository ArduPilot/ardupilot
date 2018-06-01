all: BB-BBBMINI-00A0.dtbo

%.dtbo: %.dts
	dtc -O dtb -o $@ -b 0 -@ $<

install:
	cp *.dtbo /lib/firmware

clean:
	rm -f *.dtbo
