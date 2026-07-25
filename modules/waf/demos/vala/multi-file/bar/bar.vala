class Bar : Object
{
	public signal bool an_event();

	public Bar(MainLoop loop)
	{
		var time = new TimeoutSource(8000);
		time.set_callback(() => {return an_event();});
		time.attach(loop.get_context());
	}
}

