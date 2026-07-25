module abc.
def.	gh;

import std.stdio;       // for writefln()
import std .  // system2;
system  ;
//static import std.date, std.thread /+ /+ +/ , std.io +/ ;

import testlib.code;

int main(string[] args)   // string is a type alias for const(char)[]
{
	// Declare an associative array with string keys and
	// arrays of strings as data
	char[][] [char[]] container;

	int result = test_lib();


	// Add some people to the container and let them carry some items
	container["Anya"] ~= cast(char[]) "scarf";
	container["Dimitri"] ~= cast(char[]) "tickets";
	container["Anya"] ~= cast(char[]) "puppy";

	// Iterate over all the persons in the container
	foreach (const(char)[] person, char[][] items; container)
		display_item_count(cast(char[]) person, items);
	return 0;
}

void display_item_count(char[] person, char[][] items)
{
	writefln(person, " is carrying ", items.length, " items");
}

