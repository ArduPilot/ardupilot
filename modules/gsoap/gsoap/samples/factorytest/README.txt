
A remote object factory client.

See samples/factory for the server code.

Server objects are instantiated, manipulated, and destroyed remotely using SOAP
messages. The objects are persistent and can be named. Objects are purged when
their lease expires. A lease is renewed when the object is used.

The client-side message definitions do not include the server object
definitions, since only the messaging parts are needed.

