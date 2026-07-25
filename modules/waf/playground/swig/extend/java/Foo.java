package foo.bar.pouet;


class Foo {
	public Foo() {
		// TODO how to test the library from java?
	}

	public static void main(String[] args) {
		System.loadLibrary("_test_swig_waf");
		A test = new A();
		System.out.println(test.add(17, 28));
	}
}
