# SubscribeHelloWorldBestEffort example

This example will show how to receive data from the DDS World creating a client subscriber in best effort mode.
In order to compile this example, the following profiles should be enabled:

- `UCLIENT_PROFILE_UDP`

## Usage
1. Run an agent in a certain port, for example, *2018*: `MicroXRCEAgent udp4 -p 2018`.
2. Run the *PublisherHelloWorldBestEffort* example or some publisher that can send the *HelloWorld* topic.
3. Run the *SubscriberHelloWorldBestEffort* example.
   The example expects first and second argument to be IP address and port where the Micro XRCE-DDS Agent is running. It can also be parameterized with the amount of times that the topic will be listened.
   If no number is given, the subscriber will listen indefinitely.

## Topic

The *HelloWorld* topic has the following *IDL* representation:

```
struct HelloWorld
{
	unsigned long index;
	string message;
};
```
