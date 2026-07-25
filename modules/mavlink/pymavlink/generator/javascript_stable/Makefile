all: clean
	npm install

test: all
	mocha test

ci: all
	mocha --reporter xunit test > mocha.xml

clean:
	rm -rf implementations