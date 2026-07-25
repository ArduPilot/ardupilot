/*
 * Copyright (c) 2007, Michael Feathers, James Grenning and Bas Vodde
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE EARLIER MENTIONED AUTHORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef D_MockNamedValue_h
#define D_MockNamedValue_h
/*
 * MockParameterComparator is an interface that needs to be used when creating Comparators.
 * This is needed when comparing values of non-native type.
 */

class MockNamedValueComparator
{
public:
	MockNamedValueComparator() {};
	virtual ~MockNamedValueComparator() {};

	virtual bool isEqual(void* object1, void* object2)=0;
	virtual SimpleString valueToString(void* object)=0;
};

class MockFunctionComparator : public MockNamedValueComparator
{
public:
	typedef bool (*isEqualFunction)(void*, void*);
	typedef SimpleString (*valueToStringFunction)(void*);

	MockFunctionComparator(isEqualFunction equal, valueToStringFunction valToString)
		: equal_(equal), valueToString_(valToString) {}
	virtual ~MockFunctionComparator(){};

	virtual bool isEqual(void* object1, void* object2){ return equal_(object1, object2); }
	virtual SimpleString valueToString(void* object) { return valueToString_(object); }
private:
	isEqualFunction equal_;
	valueToStringFunction valueToString_;
};

/*
 * MockNamedValue is the generic value class used. It encapsulates basic types and can use them "as if one"
 * Also it enables other types by putting object pointers. They can be compared with comparators.
 *
 * Basically this class ties together a Name, a Value, a Type, and a Comparator
 */

class MockNamedValue
{
public:
	MockNamedValue(const SimpleString& name);
	virtual ~MockNamedValue();

	virtual void setValue(int value);
	virtual void setValue(double value);
	virtual void setValue(void* value);
	virtual void setValue(const char* value);
	virtual void setObjectPointer(const SimpleString& type, void* objectPtr);

	virtual void setComparator(MockNamedValueComparator* comparator);
	virtual void setName(const char* name);

	virtual bool equals(const MockNamedValue& p) const;

	virtual SimpleString toString() const;

	virtual SimpleString getName() const;
	virtual SimpleString getType() const;

	virtual int getIntValue() const;
	virtual double getDoubleValue() const;
	virtual const char* getStringValue() const;
	virtual void* getPointerValue() const;
	virtual void* getObjectPointer() const;
private:
	SimpleString name_;
	SimpleString type_;
	union {
		int intValue_;
		double doubleValue_;
		const char* stringValue_;
		void* pointerValue_;
		void* objectPointerValue_;
	} value_;
	MockNamedValueComparator* comparator_;
};

class MockNamedValueListNode
{
public:
	MockNamedValueListNode(MockNamedValue* newValue);

	SimpleString getName() const;
	SimpleString getType() const;

	MockNamedValueListNode* next();
	MockNamedValue* item();

	void destroy();
	void setNext(MockNamedValueListNode* node);
private:
	MockNamedValue* data_;
	MockNamedValueListNode* next_;
};

class MockNamedValueList
{
public:
	MockNamedValueList();

	MockNamedValueListNode* begin();

	void add(MockNamedValue* newValue);
	void clear();

	MockNamedValue* getValueByName(const SimpleString& name);

private:
	MockNamedValueListNode* head_;
};

/*
 * MockParameterComparatorRepository is a class which stores comparators which can be used for comparing non-native types
 *
 */

struct MockNamedValueComparatorRepositoryNode;
class MockNamedValueComparatorRepository
{
	MockNamedValueComparatorRepositoryNode* head_;
public:
	MockNamedValueComparatorRepository();
	virtual ~MockNamedValueComparatorRepository();

	virtual void installComparator(const SimpleString& name, MockNamedValueComparator& comparator);
	virtual void installComparators(const MockNamedValueComparatorRepository& repository);
	virtual MockNamedValueComparator* getComparatorForType(const SimpleString& name);

	void clear();
};

#endif
