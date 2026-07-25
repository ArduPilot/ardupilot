int main(string[] args)
{
	var loop = new MainLoop();

	var foo = new Foo(loop);
	foo.an_event.connect(() => {stdout.printf("Foo"); stdout.flush(); return false;});

	var bar = new Bar(loop);
	bar.an_event.connect(() => {stdout.printf("Bar\n"); loop.quit(); return false;});

	loop.run();

	return 0;
}
