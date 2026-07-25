# Deployment example

This example will show how deploy a real world Micro XRCE-DDS configuration.
The goal of this example is to create a minimal publisher and subscriber that can send and receive topics from DDS world.
To archive this minimal examples, it is necessary to split some logic.
This implies that the publisher and subscriber will be configured previously with another application: the configurator client.
The configurator client will create the necessary entities in the agent for a specific session.
In order to compile this example, the following profiles should be enabled:

- `UCLIENT_PROFILE_UDP`

## Usage

1. Run an agent in a certain port, for example, *2018*: `MicroXRCEAgent udp4 -p 2018`.
2. Run the *ConfiguratorClient* example to create both, a publisher and a subscriber.
   1. Configure the publisher: `ConfiguratorClient --key 1000 create pub --id 1`.
      This will create a publisher with the session key `1000`, all necessary entities to create the publisher will have `1` as id.
   2. Configure the subscriber: `ConfiguratorClient --key 2000 create sub --id 1`.
      This will create a subscriber with the session key `2000`, all necessary entities to create the subscriber will have `1` as id.

3. Run the *Publisher* example: `PublisherClient --key 1000 --id 1`.
   The publisher will use the configuration for key `1000` and will publish by the *DataWriter* entity `1`.

4. Run the *Subscriber* example: `SubscriberClient --key 2000 --id 1`.
   The subscriber will use the configuration for key `2000` and will subscribe by the *DataReader* entity `1`.

## Topic

The *HelloWorld* topic has the following *IDL* representation:

```
struct HelloWorld
{
	unsigned long index;
	string message;
};
```

