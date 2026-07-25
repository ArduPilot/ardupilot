

all:
	echo done.

clean:
	cd calc
		$(MAKE) clean
	cd ..
	cd dime
		$(MAKE) clean
	cd ..
