
A remote object factory.

Server objects are instantiated, manipulated, and destroyed remotely using SOAP
messages. The objects are persistent and can be named. Objects are purged when
their lease expires. A lease is renewed when the object is used.

This example defines a root object, an adder object, and a counter object.

See samples/factorytest for client code.
