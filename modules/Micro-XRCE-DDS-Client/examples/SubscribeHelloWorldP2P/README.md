# SubscribeHelloWorldP2P example

This example will show how to subscribe to data creating a client subscriber.
It is necessary to have the following profiles enabled to compile this example:

- `UCLIENT_PROFILE_UDP`

## Usage
1. Run an agent in a certain port, for example, *2018*: `MicroXRCEAgent udp4 -p 2018`.
2. Run the *PublisherHelloWorldP2P* example or some publisher that can send the *HelloWorld* topic.
3. Run the *SubscriberHelloWorldP2P* example.
   The example can be parameterized with the number of topics that will be listened to.
   If no number is given, the publisher will publish indefinitely.

## Topic

The *HelloWorld* topic has the following *IDL* representation:

```
struct HelloWorld
{
	unsigned long index;
	string message;
};
```

