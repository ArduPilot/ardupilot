#ifndef NONCOPYABLE_HPP_
#define NONCOPYABLE_HPP_

/**
 * To make a class non-copyable* simply inherit from NonCopyable.
 *
 * non-copyable:
 * -------------
 * the following code shows 3 examples, that use the copy-constructor / assignment-operator.
 * All of them will fail at compile-time, if A inherits from NonCopyable
 * <Code>
 * 	A a1, a2;
 *
 * 	A a3(a1);  // these two lines are equivalent:
 * 	A a4 = a2; //  	both invoke A's copy-constructor: A::A(const A&);
 *
 * 	a1 = a2; // this line calls A's assignment-operator
 * </Code>
 *
 * Error Messages:
 * ---------------
 * If you are reading this because you received an error message like
 *
 * 	error: 'NonCopyable::NonCopyable(const NonCopyable&)' is private
 * 	...
 * 	error: within this context
 * 	...
 * 	In copy constructor ...
 *
 * Then you tried to copy an object that isn't supposed to be copied.
 * You should propably try to pass it by const reference.
 *
 */
class NonCopyable {

	/**
	 * private copy-constructor & assignment-operator to prevent copying
	 */
private:
	NonCopyable(const NonCopyable&);
	NonCopyable &operator=(const NonCopyable&);

	/**
	 * protected constructor & destructor to prevent instantiation of NonCopyable / deletion of NonCopyable pointer
	 */
protected:
	NonCopyable(){}
	~NonCopyable(){}

};

#endif /* NONCOPYABLE_HPP_ */
