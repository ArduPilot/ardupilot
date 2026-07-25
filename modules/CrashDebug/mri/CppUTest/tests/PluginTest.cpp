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
#include "CppUTest/TestRegistry.h"
#include "CppUTest/TestOutput.h"
#include "CppUTest/TestTestingFixture.h"

#define GENERIC_PLUGIN  "GenericPlugin"
#define GENERIC_PLUGIN2 "GenericPlugin2"
#define GENERIC_PLUGIN3 "GenericPlugin3"

static int sequenceNumber;

class DummyPlugin: public TestPlugin
{
public:
	DummyPlugin(const SimpleString& name) :
		TestPlugin(name), preAction(0), postAction(0)
	{
	}

	virtual ~DummyPlugin()
	{
	}

	virtual void preTestAction(UtestShell&, TestResult&)
	{
		preAction++;
		preActionSequence = sequenceNumber++;
	}

	virtual void postTestAction(UtestShell&, TestResult&)
	{
		postAction++;
		postActionSequence = sequenceNumber++;
	}

	int preAction;
	int preActionSequence;
	int postAction;
	int postActionSequence;
};

class DummyPluginWhichAcceptsParameters: public DummyPlugin
{
public:
	DummyPluginWhichAcceptsParameters(const SimpleString& name) :
		DummyPlugin(name)
	{
	}

	virtual bool parseArguments(int ac, const char** av, int index)
	{
		SimpleString argument (av[index]);
		if (argument == "-paccept")
			return true;
		return TestPlugin::parseArguments(ac, av, index);
	}

};


TEST_GROUP(PluginTest)
{
	DummyPlugin* firstPlugin;
	DummyPluginWhichAcceptsParameters* secondPlugin;
	DummyPlugin* thirdPlugin;
	TestTestingFixture *genFixture;
	TestRegistry *registry;
	void setup()
	{
		firstPlugin = new DummyPlugin(GENERIC_PLUGIN);
		secondPlugin = new DummyPluginWhichAcceptsParameters(GENERIC_PLUGIN2);
		thirdPlugin = new DummyPlugin(GENERIC_PLUGIN3);
		genFixture = new TestTestingFixture;
		registry = genFixture->registry_;
		registry->installPlugin(firstPlugin);
		sequenceNumber = 1;
	}

	void teardown()
	{
		delete firstPlugin;
		delete secondPlugin;
		delete thirdPlugin;
		delete genFixture;
	}
};

#define GENERIC_PLUGIN  "GenericPlugin"
#define GENERIC_PLUGIN4 "GenericPlugin4"

TEST(PluginTest, PluginHasName)
{
	CHECK_EQUAL(GENERIC_PLUGIN, firstPlugin->getName());
}

TEST(PluginTest, InstallPlugin)
{
	CHECK_EQUAL(firstPlugin, registry->getFirstPlugin());
	CHECK_EQUAL(firstPlugin, registry->getPluginByName(GENERIC_PLUGIN));
}

TEST(PluginTest, InstallMultiplePlugins)
{
	registry->installPlugin(thirdPlugin);
	CHECK_EQUAL(firstPlugin, registry->getPluginByName(GENERIC_PLUGIN));
	CHECK_EQUAL(thirdPlugin, registry->getPluginByName(GENERIC_PLUGIN3));
	CHECK_EQUAL(0, registry->getPluginByName("I do not exist"));
}

TEST(PluginTest, ActionsAllRun)
{
	genFixture->runAllTests();
	genFixture->runAllTests();
	CHECK_EQUAL(2, firstPlugin->preAction);
	CHECK_EQUAL(2, firstPlugin->postAction);
}

TEST(PluginTest, Sequence)
{
	registry->installPlugin(thirdPlugin);
	genFixture->runAllTests();
	CHECK_EQUAL(1, thirdPlugin->preActionSequence);
	CHECK_EQUAL(2, firstPlugin->preActionSequence);
	CHECK_EQUAL(3, firstPlugin->postActionSequence);
	CHECK_EQUAL(4, thirdPlugin->postActionSequence);
}

TEST(PluginTest, DisablesPluginsDontRun)
{
	registry->installPlugin(thirdPlugin);
	thirdPlugin->disable();
	genFixture->runAllTests();
	CHECK(!thirdPlugin->isEnabled());
	thirdPlugin->enable();
	genFixture->runAllTests();
	CHECK_EQUAL(2, firstPlugin->preAction);
	CHECK_EQUAL(1, thirdPlugin->preAction);
	CHECK(thirdPlugin->isEnabled());
}

TEST(PluginTest, ParseArgumentsForUnknownArgumentsFails)
{
	registry->installPlugin(secondPlugin);
	const char *cmd_line[] = {"nonsense", "andmorenonsense"};
	CHECK(registry->getFirstPlugin()->parseAllArguments(2, cmd_line, 0) == false	);
}

TEST(PluginTest, ParseArgumentsContinuesAndSucceedsWhenAPluginCanParse)
{
	registry->installPlugin(secondPlugin);
	const char *cmd_line[] = {"-paccept", "andmorenonsense"};
	CHECK(registry->getFirstPlugin()->parseAllArguments(2, cmd_line, 0));
}

