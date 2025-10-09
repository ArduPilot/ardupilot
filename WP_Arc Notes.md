
# WAYPOINT_ARC Implimentation Notes

The new MAVLink mission command definition is here: https://github.com/MattKear/mavlink/tree/pr_wp_arc_cmd

## Setup Instructions

As it is a new nav command definition you need to tell the GCS about the new message definition otherwise it will not understand that the command has a location associated with it and it will not send the lat and lng as deg⁡〖 ×1e^7 〗.  The easiest way I found to do this was to add the definition to MAVProxy.  The below are the steps I used to do that.

1. Uninstall pymavlink as we are going to re-build it to include new MAVLink definition:

    `python -m pip uninstall pymavlink`

2. Ensure that the new MAVLink definition is available by updating submodules:

    `git submodule update --recursive --init`

3. CD down into the pymavlink directory:

    `cd ./modules/mavlink/pymavlink`

4. Build and install the new version of pymavlink for MAVProxy to use:

    `python setup.py install --user`

    Noting that you must be in the pymavlink directory when calling this function otherwise you get cython errors.

5. Test that the new definition is definitely present in the newly installed pymavlink by running the simple script that I added in the MAVLink branch:

    `python ./modules/mavlink/msg_test.py`

    You should see this print to the command line: 

    “MAV_CMD_NAV_ARC_WAYPOINT exists?: True ; value: 35” 

    Plus some other information about the new command, like the description from the xml.
	That’s it, running SITL normally should now launch MAVProxy that uses this installed version of pymavlink so the new command will be supported by MAVProxy.


## Definition

Currently the MAVLink command is defined as `MAV_CMD_NAV_WAYPOINT_ARC` and has value of 35.

It has location and param 1 is defined as the arc angle in degrees.  We will need to give some thought to how we are going to define direction of travel (if we don't just use the shortest arc length as some arc definitions do (e.g. SVGs)).  Param 1 is a uint16 so we would need to do some casting magic if we want to use a negative number. I am not sure how much others would like this as I don't believe it matches the MAVLink spec. I would need to ask around though.


## Testing 

I have a little test mission: [Link to test mission file](./TestMission.waypoints)

I just checked that the mission still runs and I get the debug prints to the GCS when the WAYPOINT_ARC command is started.








