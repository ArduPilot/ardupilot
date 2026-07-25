using GLib;

class Foo : Object
{
	public signal bool an_event();
	
	public Foo(MainLoop loop)
	{
		var time = new TimeoutSource(5000);
		time.set_callback(() => {return an_event();});
		time.attach(loop.get_context());
	}
}

