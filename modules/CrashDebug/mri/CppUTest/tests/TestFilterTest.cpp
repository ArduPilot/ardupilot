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

#include "CppUTest/TestHarness.h"
#include "CppUTest/TestFilter.h"

TEST_GROUP(TestFilter)
{
};

TEST(TestFilter, emptyFilterMatchesEverything)
{
	TestFilter filter;
	CHECK(filter.match("random_name"));
	CHECK(filter.match(""));
	CHECK(filter.match("*&%#^&%$(*&^@#(&*@#^(&*$^@#"));
}

TEST(TestFilter, defaultAbsoluteMismatches)
{
	TestFilter filter("filtername");
	CHECK(!filter.match("notevenclose"));
	CHECK(!filter.match("filterrname"));
	CHECK(!filter.match(""));
}

TEST(TestFilter, strictMatching)
{
	TestFilter filter("filter");
	filter.strictMatching();
	CHECK(filter.match("filter"));
	CHECK(!filter.match("filterr"));
	CHECK(!filter.match(" filter"));
}

TEST(TestFilter, equality)
{
	TestFilter filter1("filter");
	TestFilter filter2("filter");
	TestFilter filter3("filter3");
	CHECK(filter1 == filter2);
	CHECK(! (filter1 == filter3));
}

TEST(TestFilter, equalityWithStrictness)
{
	TestFilter filter1("filter");
	TestFilter filter2("filter");
	filter2.strictMatching();
	CHECK(! (filter1 == filter2));
}

TEST(TestFilter, notEqual)
{
	TestFilter filter1("filter");
	TestFilter filter2("filter");
	TestFilter filter3("filter3");
	CHECK(filter1 != filter3);
	CHECK(! (filter1 != filter2));
}

TEST(TestFilter, stringFrom)
{
	TestFilter filter("filter");
	STRCMP_EQUAL("TestFilter: \"filter\"", StringFrom(filter).asCharString());
}

TEST(TestFilter, stringFromWithStrictMatching)
{
	TestFilter filter("filter");
	filter.strictMatching();
	STRCMP_EQUAL("TestFilter: \"filter\" with strict matching", StringFrom(filter).asCharString());
}
