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

#ifndef D_MockSupport_c_h
#define D_MockSupport_c_h

typedef enum {
	MOCKVALUETYPE_INTEGER,
	MOCKVALUETYPE_DOUBLE,
	MOCKVALUETYPE_STRING,
	MOCKVALUETYPE_POINTER,
	MOCKVALUETYPE_OBJECT
} MockValueType_c;

typedef struct SMockValue_c
{
	MockValueType_c type;
	union  {
		int intValue;
		double doubleValue;
		const char* stringValue;
		void* pointerValue;
		void* objectValue;
	} value;
} MockValue_c;

typedef struct SMockFunctionCall_c MockFunctionCall_c;
struct SMockFunctionCall_c
{
	MockFunctionCall_c* (*withIntParameters)(const char* name, int value);
	MockFunctionCall_c* (*withDoubleParameters)(const char* name, double value);
	MockFunctionCall_c* (*withStringParameters)(const char* name, const char* value);
	MockFunctionCall_c* (*withPointerParameters)(const char* name, void* value);
	MockFunctionCall_c* (*withParameterOfType)(const char* type, const char* name, void* value);

	MockFunctionCall_c* (*andReturnIntValue)(int value);
	MockFunctionCall_c* (*andReturnDoubleValue)(double value);
	MockFunctionCall_c* (*andReturnStringValue)(const char* value);
	MockFunctionCall_c* (*andReturnPointerValue)(void* value);

	MockValue_c (*returnValue)();

};

typedef int (*MockTypeEqualFunction_c)(void* object1, void* object2);
typedef char* (*MockTypeValueToStringFunction_c)(void* object1);

typedef struct SMockSupport_c MockSupport_c;
struct SMockSupport_c
{
	MockFunctionCall_c* (*expectOneCall)(const char* name);
	MockFunctionCall_c* (*actualCall)(const char* name);
	MockValue_c (*returnValue)();

	void (*setIntData) (const char* name, int value);
	void (*setDoubleData) (const char* name, double value);
	void (*setStringData) (const char* name, const char* value);
	void (*setPointerData) (const char* name, void* value);
	void (*setDataObject) (const char* name, const char* type, void* value);
	MockValue_c (*getData)(const char* name);

	void (*checkExpectations)();
	int (*expectedCallsLeft)();

	void (*clear)();

	void (*installComparator) (const char* typeName, MockTypeEqualFunction_c isEqual, MockTypeValueToStringFunction_c valueToString);
	void (*removeAllComparators)();
};


MockSupport_c* mock_c();
MockSupport_c* mock_scope_c(const char* scope);

#endif
