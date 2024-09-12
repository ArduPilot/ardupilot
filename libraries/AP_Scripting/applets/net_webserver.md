# Web Server Application

This implements a web server for boards that have networking support.

# Parameters

The web server has a small number of parameters

## WEB_ENABLE

This must be set to 1 to enable the web server

## WEB_BIND_PORT

This sets the network port to use for the server. It defaults to 8080

## WEB_DEBUG

This enables verbose debugging

## WEB_BLOCK_SIZE

This sets the block size for network and file read/write
operations. Setting a larger value can increase performance at the
cost of more memory

## WEB_TIMEOUT

This sets the timeout in seconds for inactive client connections.

# Operation

By default the web server serves the root of your microSD card. You
can include html, javascript (*.js), image files etc on your microSD
to create a full web server with any structure you want.

## Server Side Scripting

The web server supports embedding lua script elements inside html
files for files with a filename of *.shtml. Here is an example:

```
<html>
  <head>
       <meta http-equiv="refresh" content="1">
  </head>
  <body>
    <h1>Server Side Scripting Test</h1>
    <table>
      <tr><th>Roll</th><th>Pitch</th><th>Yaw</th></tr>
      <tr>
      <td><?lua return tostring(math.deg(ahrs:get_roll()))?></td>
      <td><?lstr math.deg(ahrs:get_pitch())?></td>
      <td><?lstr math.deg(ahrs:get_yaw())?></td>
      </tr>
    </table>
  </body>
</html>
```
In this example we are using two forms of embedded lua scripts. The
first form starts with "<?lua" and requires you to have a return
statement at the end which returns a string. This form can use as many
lines as you like.

The second form starts with "<?lstr" and will automatically cast the
expression to a string. The second form is for short single value
elements.

You can use any of the normal ArduPilot lua script bindings inside
these lua embedded elements to access sensor data etc.

## CGI Scripts

You can have CGI scripts written in lua by putting them in a directory
called "cgi-bin" in the root of the microSD card. The files must have
a file extension of ".lua".

Here is an example of a simple cgi script:
```
--[[
example lua cgi file for cgi-bin/ folder
--]]
return [[
test-from-cgi
2nd line
and a third line
]]
```

You can use any of the normal ArduPilot lua script bindings inside CGI
scripts to access sensor data etc.
