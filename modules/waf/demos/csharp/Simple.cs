// taken from the gtk# samples
namespace MyApp
{
	using Gtk;
	using System;

	public class Simple
	{

		public static int Main(string[] args)
		{
			Application.Init();
			Window win = new Window("Simple gtk# app");
			win.DefaultWidth  = 300;
			win.DefaultHeight = 300;
			win.DeleteEvent += new DeleteEventHandler(Window_Delete);
			Button btn = new Button("Simple button");
			btn.Clicked += new EventHandler(print_line);
			win.Add(btn);
			win.ShowAll();
			Application.Run();
			return 0;
		}

		static void print_line(object obj, EventArgs args)
		{
			Console.WriteLine("Simple button was clicked!");
		}

		static void Window_Delete(object obj, DeleteEventArgs args)
		{
			Application.Quit();
			args.RetVal = true;
		}
	}
}

