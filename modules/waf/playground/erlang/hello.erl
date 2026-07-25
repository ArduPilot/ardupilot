% what a weird language ... :-)
%%% @author Przemyslaw Rzepecki
%%% @version 0.01
  
%%% @doc == Hello World, Example Module ==
%%% This module contains some Erlang code for WAF build system support for
%%% Erlang language.
%%% @end

-module(hello).
-export([say_hello/1, hello_world/0]).
-include("hello.hrl").

%%% ###########################################################################
%% @doc Returns a greetings string
%%
%% Some more specific description of the function should be written here...
%% 
%% See http://erlang.org/doc/apps/edoc/users_guide.html for the complete Edoc
%% guide.
%%
%% @end
%%% ----------------------------------------------------------
say_hello(waf) -> "Hello WAF, cool to see you!";
say_hello(make) -> "Oh Make, you again...";
say_hello(Other) -> "Hi " ++ Other.


%%% ###########################################################################
%% @doc Print a 'Hello World' string to stdout of the program..
%%
%% This is an Erlang Version of the famous hello_world function. 
%%
%% @end
%%% ----------------------------------------------------------
hello_world() -> io:fwrite("~p~n", [?HELLO_WORLD]).
