Convert CppUTest files to Unity test files
------------------------------------------

To convert a CppUTest test file to Unity, enter a command like this

CONVERT=$CPPUTEST_HOME/scripts/convertToUnity/cpp_u_test_to_unity.rb

ruby $CONVERT tests/some/dir/or/dirs/MyThink.cpp

This will create two files like this

    unity/some/dir/or/dirs/MyThing.c
    unity/some/dir/or/dirs/MyThing_runner.c
    
Things that need to be done manually once to your
unity test environment:

    Create directory 'unity/some/dir/or/dirs'
    Adjust unity makefile to refer to the needed files
    Unity main needs to call RUN_TEST_GROUP(MyThing)

Things not supported in converted test cases:
    CppUMock
    Hand built mocks that use CppUTest macros.  
    Formatting conventions can cause the script to fail  
