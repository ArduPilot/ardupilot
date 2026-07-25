# Guide to MAVLink2

MAVLink2 is a new variant of the MAVLink protocol designed to bring
more flexibility and security to MAVLink communication.

A background document on the MAVLink2 design is available here:

  https://docs.google.com/document/d/1XtbD0ORNkhZ8eKrsbSIZNLyg9sFRXMXbsR2mp37KbIg/edit?usp=sharing

it is suggested that you start by reading that document.

The key features of MAVLink2 are:

* support for more than 256 message IDs (24 bit message ID)
* support for packet signing
* support for extending existing MAVLink messages
* support for variable length arrays

The first 3 features in this list are implemented now and ready for
testing. The last feature is not implemented yet.

Most of the rest of this document is a guide for programmers
interested in using MAVLink2 in their applications. The two language
bindings that are covered are C and Python. Bindings for MAVLink2 for
other languages are not implemented yet.

## Using the C implementation

Using the C implementation of MAVLink2 is very similar to using the
existing MAVLink1 implementation. You start by generating the MAVLink2
headers using mavgen.py, but passing the --wire-protocol=2.0
option. For example:

```
 mavgen.py --lang C message_definitions/v1.0/ardupilotmega.xml -o generator/C/include_v2.0 --wire-protocol=2.0
```

This will generate a set of C headers in the generator/C/include_v2.0
directory. These headers offer the same range of APIs as was offered
by MAVLink1. The major changes from an API perspective are:

* you don't need to provide a message CRC table any more, or message
  length table. These have been folded into a single packed table,
  replacing the old table which was indexed by msgId. That was
  necessary to cope with the much larger 24 bit namespace of message
  IDs.

### Sending and receiving MAVLink1 packets

The C implementation of MAVLink2 supports sending and receiving
MAVLink1 packets as well. To force sending MAVLink1 packets on a
particular channel you change the flags field of the status
object. For example:

```
   mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
   status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
```

that will cause subsequent packets on the given channel to be sent as
MAVLink1.

Incoming MAVLink1 packets will be automatically handled as
MAVLink1. If you need to determine if a particular message was received as
MAVLink1 or MAVLink2 then you can use the magic field of the message,
like this:

```
  if (msg->magic = MAVLINK_STX_MAVLINK1) {
     printf("This is a MAVLink1 message\n");
  }
```

In most cases this should not be necessary as the xml message
definition files for MAVLink1 and MAVLink2 are the same, so you can
treat incoming MAVLink1 messages the same as MAVLink2 messages.

Note that MAVLink1 is restricted to messageIDs less than 256, so any
messages with a higher messageID won't be received as MAVLink1.

### Handling message extensions

The XML for messages can contain extensions that are optional in the
protocol. This allows for extra fields to be added to a message. For
example:

```
  <message id="152" name="MEMINFO">
    <description>state of APM memory</description>
    <field name="brkval" type="uint16_t">heap top</field>
    <field name="freemem" type="uint16_t">free memory</field>
    <extensions/>
    <field name="extrafield1" type="uint8_t">extension1</field>
    <field name="extrafield2" type="uint32_t">extension2</field>
  </message>
```

the fields after the extensions line are extended fields. The rules
for extended messages are:

* if sent as a MAVLink1 message then extended fields are not sent
* if received by an implementation that doesn't have the extended
  fields then the fields will not be seen
* if sent by an implementation that doesn't have the extended fields
  then the recipient will see zero values for the extended fields

### Handling message signing

One of the key features of MAVLink2 is support for signing of
messages. To enable signing in your application you will need to add
some additional code. In particular you will need to add:

* code to handle the SETUP_SIGNING message
* code to setup and teardown signing on a link
* code to save and load the secret key and timestamp in persistent storage
* a callback to allow for accepting of certain kinds of unsigned messages

Example code in ArduPilot for each of these pieces if available here:

 https://github.com/tridge/ardupilot/blob/mavlink2-wip/libraries/GCS_MAVLink/GCS_Signing.cpp

### Handling SETUP_SIGNING

The SETUP_SIGNING message is the mechanism for a GCS to setup a
signing key on a MAVLink2 device. It takes a 32 byte secret key and an
initial timestamp. The method of generating the 32 byte secret key is
up to the GCS implementation, although it is suggested that all GCS
implementations should support the use of a sha256 hash of a user
provided passphrase.

### Handling timestamps

The timestamp is a 64 bit number, and is in units of 10 microseconds
since 1st January 2015. For systems where the time since 1/1/1970 is
available (the unix epoch) you can use an offset in seconds of
1420070400.

Storage and handling of the timestamp is critical to the security of
the signing system. The rules are:

* the current timestamp should be stored regularly in persistent
  storage (suggested at least once a minute)
* the timestamp used on startup should be the maximum of the timestamp
  implied by the system clock and the stored timestamp
* if the system does not have a RTC mechanism then it should update
  its timestamp when GPS lock is achieved. The maximum of the
  timestamp from the GPS and the stored timestamp should be used
* the timestamp should be incremented by one on each message send.
  This is done for you by the generated headers.
* when a correctly signed message is decoded the timestamp should be
  replaced by the timestamp of the incoming message if that timestamp
  is greater than the current timestamp. This is done for you by the
  generated headers
* the timestamp on incoming signed messages should be checked against
  the previous timestamp for the incoming
  (linkID,srcSystem,SrcComponent) tuple and the message rejected if it
  is smaller. This is done for you by generated headers.
* if there is no previous message with the given
  (linkID,srcSystem,SrcComponent) then the timestamp should be
  accepted if it not more than 6 million (one minute) behind the
  current timestamp

### Enabling signing on a channel

To enable signing on a channel you need to fill in two pointers in the
status structure for the channel. The two pointed are:

```
   mavlink_signing_t *signing;
   mavlink_signing_streams_t *signing_streams;
```

The signing pointer controls signing for this stream. It is
per-stream, and contains the secret key, the timestamp and a set of
flags, plus an optional callback function for accepting unsigned
packets. Typical setup would be:

```
    memcpy(signing.secret_key, key.secret_key, 32);
    signing.link_id = (uint8_t)chan;
    signing.timestamp = key.timestamp;
    signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    signing.accept_unsigned_callback = accept_unsigned_callback;
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    status.signing = &signing;
```

The signing_streams pointer is a structure used to record the previous
timestamp for a (linkId,srcSystem,SrcComponent) tuple. This must point
to a structure that is common to all channels in order to prevent
inter-channel replay attacks. Typical setup is:

```
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    status.signing_streams = &signing_streams;
```

The maximum number of signing streams supported is given by the
MAVLINK_MAX_SIGNING_STREAMS macro. This defaults to 16, but it may be
worth raising this for GCS implementations. If the C implementation
runs out of signing streams then new streams will be rejected.

### Using the accept_unsigned_callback

In the signing structure there is an optional accept_unsigned_callback
function pointer. The C prototype for this function is:

```
    bool accept_unsigned_callback(const mavlink_status_t *status, uint32_t msgId);
```                                     

If set in the signing structure then this function will be called on
any unsigned packet (including all MAVLink1 packets) or any packet
where the signature is incorrect. The function offers a way for the
implementation to allow unsigned packets to be accepted.

The rules for what unsigned packets should be accepted is
implementation specific, but it is suggested the following rules be
considered:

* have a mechanism for marking a particular communication channel as
  being secure (such as a USB connection) to allow for signing
  setup.
* always accept RADIO_STATUS packets for feedback from 3DR radios
  (which don't do signing)

For example:

```
static const uint32_t unsigned_messages[] = {
	MAVLINK_MSG_ID_RADIO_STATUS
};

static bool accept_unsigned_callback(const mavlink_status_t *status, uint32_t message_id)
{
	// Make the assumption that channel 0 is USB and should always be accessible
	if (status == mavlink_get_channel_status(MAVLINK_COMM_0)) {
		return true;
	}

	for (unsigned i = 0; i < sizeof(unsigned_messages) / sizeof(unsigned_messages[0]); i++) {
		if (unsigned_messages[i] == message_id) {
			return true;
		}
	}

	return false;
}
```

### Handling link IDs

The purpose of the link_id field in the MAVLink2 signing structure is
to prevent cross-channel replay attacks. Without the link_id an
attacker could record a packet (such as a disarm request) on one
channel, then play it back on a different channel.

The intention with the link IDs is that each channel of communication
between an autopilot and a GCS uses a different link ID. There is no
requirement that the same link ID be used in both directions however.

For C implementations the obvious mechanism is to use the MAVLink
channel number as the link ID. That works well for an autopilot, but
runs into an issue for a GCS implementation. The issue is that a user
may launch multiple GCS instances talking to the same autopilot via
different communication links (such as two radios, or USB and a
radio). These multiple GCS instances will not be aware of each other,
and so may choose the same link ID. If that happens then a large
number of correctly signed packets will be rejected by the autopilot
as they will have timestamps that are older than the timestamp
received for the same stream tuple on the other communication link.

The solution that I have adopted for MAVProxy is this:

```
        if (msg.get_signed() and
            self.mav.signing.link_id == 0 and
            msg.get_link_id() != 0 and
            self.target_system == msg.get_srcSystem() and
            self.target_component == msg.get_srcComponent()):
            # change to link_id from incoming packet
            self.mav.signing.link_id = msg.get_link_id()
```

what that says is that if the current link ID in use by MAVProxy is
zero, and it receives a correctly signed packet with a non-zero link
ID then it switches link ID to the one from the incoming packet.

The has the effect of making the GCS slave its link ID to the link ID
of the autopilot.


### Negotiating MAVLink2

It is expected that vehicle and GCS implementations will support both
MAVLink1 and MAVLink2 for quite some time. We would like most users to
receive the benefit of MAVLink2, while still supporting
implementations that don't yet support MAVLink2.

The following is meant to capture best practice for vehicle firmware
and GCS authors:

 * vehicle implementations should have a way to enable/disable the
 sending of MAVLink2 messages. This should preferably be on a per-link
 basis to allow for some peripherals to be MAVLink1 while others are
 MAVLink2. It is acceptable for this option to require a reboot of the flight controller to take effect.

 * if signing is enabled and MAVLink2 is enabled then the vehicle should
 immediately start sending MAVLink2 on startup

 * if signing is not enabled and MAVLink2 is enabled then the vehicle may
 choose to start by sending MAVLink1 and switch to MAVLink2 on a link
 when it first receives a MAVLink2 message on the link

 * vehicles should set the MAV_PROTOCOL_CAPABILITY_MAVLINK2 capability
 flag in the AUTOPILOT_VERSION message if MAVLink2 is available on a
 link. This should be set in the case where the link is currently sending MAVLink1 packets but MAVLink2 packets will be accepted and will cause a switch to MAVLink2
 
 * GCS implementations can choose to either automatically switch to MAVLink2 where available or to have a configuration option for MAVLink2
 
 * if the GCS chooses to use a configuration option then when the option is enabled it should send MAVLink2 on starting the link
 
 * if the GCS chooses to use automatic switching then it should switch to sending MAVLink2 if either it receives a MAVLink2 message on the link or by asking for the AUTOPILOT_VERSION message to be sent and seeing the MAV_PROTOCOL_CAPABILITY_MAVLINK2 flag is set
 
