all: BB-BONE-PRU-05-00A0.dtbo BB-SPI0-01-00A0.dtbo BB-SPI0-SWP-01-00A0.dtbo BB-SPI1-SWP-01-00A0.dtbo

%.dtbo: %.dts
	dtc -O dtb -o $@ -b 0 -@ $<

install:
	cp *.dtbo /lib/firmware

clean:
	rm -f *.dtbo
