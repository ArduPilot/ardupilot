using Gtk;

[GtkTemplate (ui="/org/test/appwindow.ui")]
class TestWindow : Gtk.ApplicationWindow {
    public TestWindow(Gtk.Application app) {
        Object(application: app);
    }
}

class TestApp : Gtk.Application {
    public override void activate() {
        var window = new TestWindow(this);

        window.show_all();
    }
}

int main(string[] args) {
    var app = new TestApp();

    return app.run();
}
