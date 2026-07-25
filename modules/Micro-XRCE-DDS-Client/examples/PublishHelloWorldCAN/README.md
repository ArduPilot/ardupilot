# PublishHelloWorldCAN example

This example will show how to send data to the DDS World creating a client publisher using a CAN bus.
In order to compile this example, the following profiles should be enabled:

- `UCLIENT_PROFILE_CAN`

To run the example, a virtual CAN FD bus can be created with the following commands:
```
ip link add dev vcan0 type vcan
ip link set vcan0 mtu 72
ip link set dev vcan0 up
```

## Usage
1. Run an agent in can mode, for example: `MicroXRCEAgent canfd --dev vcan0 --id 0x00000001`.
2. Run the *PublisherHelloWorldCAN* example on the same CAN bus with a different can id.
   The example expects the CAN device name as first argument and the client can identifier as second.
   A different can identifier should be used on the Agent and the Client.

   It can also be parameterized with the number of topics that will be sent.

## Topic

The *HelloWorld* topic has the following *IDL* representation:

```
struct HelloWorld
{
	unsigned long index;
	string message;
};
```
