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

#include "CppUTestExt/MockSupport_c.h"

static int typeNameIsEqual(void* object1, void* object2)
{
	return object1 == object2;

}

static char* typeNameValueToString(void* object)
{
	return (char*) object;
}

void all_mock_support_c_calls()
{
	mock_c()->expectOneCall("boo");
	mock_c()->actualCall("boo");
	mock_c()->checkExpectations();

	mock_c()->expectOneCall("boo")->withIntParameters("integer", 1)->withDoubleParameters("doube", 1.0)->
			withStringParameters("string", "string")->withPointerParameters("pointer", (void*) 1);
	mock_c()->actualCall("boo")->withIntParameters("integer", 1)->withDoubleParameters("doube", 1.0)->
			withStringParameters("string", "string")->withPointerParameters("pointer", (void*) 1);

	mock_c()->installComparator("typeName", typeNameIsEqual, typeNameValueToString);
	mock_c()->expectOneCall("boo")->withParameterOfType("typeName", "name", (void*) 1);
	mock_c()->actualCall("boo")->withParameterOfType("typeName", "name", (void*) 1);
	mock_c()->clear();
	mock_c()->removeAllComparators();

	mock_c()->expectOneCall("boo")->andReturnIntValue(10);
	mock_c()->actualCall("boo")->returnValue();
	mock_c()->returnValue();
	mock_c()->expectOneCall("boo2")->andReturnDoubleValue(1.0);
	mock_c()->actualCall("boo2")->returnValue();
	mock_c()->returnValue();

	mock_c()->expectOneCall("boo3")->andReturnStringValue("hello world");
	mock_c()->actualCall("boo3")->returnValue();
	mock_c()->returnValue();

	mock_c()->expectOneCall("boo4")->andReturnPointerValue((void*) 10);
	mock_c()->actualCall("boo4")->returnValue();
	mock_c()->returnValue();

	mock_scope_c("scope")->expectOneCall("boo");
	mock_scope_c("other")->expectedCallsLeft();
	mock_scope_c("scope")->expectedCallsLeft();
	mock_scope_c("scope")->actualCall("boo");
}
