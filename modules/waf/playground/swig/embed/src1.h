#ifndef SWIGTOOLDEMO_HPP
#define SWIGTOOLDEMO_HPP

// singleton shared between test app and python
// (Note: this is a demo, remember singletons should not be used)
class TestClass
{
	public:
		static TestClass* instance()
		{
			if (_instance == 0)
				_instance = new TestClass();
			return _instance;
		}

		void destroy () { delete _instance; _instance = 0; }

	protected:
		TestClass() {};
		~TestClass(){};

	public:
		const char* test() { return "Hello World from C++\n"; }

	private:
		static TestClass* _instance;
};

#endif //SWIGTOOLDEMO_HPP

