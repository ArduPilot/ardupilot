-module(hello_eunit).
-include_lib("eunit/include/eunit.hrl").
-include("hello.hrl").

example_test_() ->
    [
     ?_assert(hello:say_hello(waf) =:= "Hello WAF, cool to see you!"),
     ?_assert(hello:say_hello(make) =:= "Oh Make, you again..."),
     ?_assert(hello:say_hello("Mike") =:= "Hi Mike")
    ].
