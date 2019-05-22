## AirSim Setup

This is a temporary page describing the development setup of AirSim and how to use Ardupilot SITL with it (currently under development)

### Linux

This AirSim page describes how to build Unreal Engine, AirSim
<https://microsoft.github.io/AirSim/docs/build_linux/>

Setting up the Blocks Environment - <https://microsoft.github.io/AirSim/docs/unreal_blocks/>

Development Workflow in AirSim -  

- Updating the repo - Normal Git workflow
- Make any changes required
- Run the `build.sh` script in Airsim which will also copy the changes to the plugin directory
- Launch UnrealEngine Editor, choose Blocks environment and when prompted about missing .so files, press Yes to build it again.

- Troubleshooting - <https://microsoft.github.io/AirSim/docs/build_linux/>

### Windows

Build AirSim on Windows - <https://microsoft.github.io/AirSim/docs/build_windows/>

Setup Blocks Environment - <https://microsoft.github.io/AirSim/docs/unreal_blocks/>

Development Workflow - <https://microsoft.github.io/AirSim/docs/dev_workflow/>

### Using ArduCopter vehicle

Use the following branch for ArduCopter vehicle - <https://github.com/rajat2004/AirSim/tree/pr-arducopter>  

Note: Has been tested on a single Linux Machine running Ubuntu 16.04. The branch should compile with Windows as well but hasn't been tested yet.

Before launching for the first time, go to `Edit->Editor Preferences`, in the 'Search' box type `CPU` and ensure that the `Use Less CPU when in Background` is unchecked.

Press Play after Editor loads.

Initially, the editor will hang after pressing the Play button if the Ardupilot SITL hasn't been started. Run `sim_vehicle.py` and it should go back to normal

See <https://github.com/Microsoft/AirSim/blob/master/docs/settings.md> for info about settings

Current `settings.json` file for launching Arducopter

```
{
  "SettingsVersion": 1.2,
  "LocalHostIp": "127.0.0.1",
  "LogMessagesVisible": true,
  "SimMode": "Multirotor",
  "OriginGeopoint": {
    "Latitude": -35.363261,
    "Longitude": 149.165230,
    "Altitude": 583
  },
  "Vehicles": {
      "Copter": {
          "VehicleType": "ArduCopter",
          "UseSerial": false,
          "AllowAPIAlways": false,
          "UdpIp": "127.0.0.1",
          "UdpPort": 9003,
          "SitlPort": 9002 
        }
    }
}
```

Note: Many of the fields have the default values as above only but just specifying them. If you want to select a specific local network adapter so you can reach certain remote machines (e.g. wifi versus ethernet), then you will want to change the `LocalHostIp` accordingly.

First launch AirSim, after that launch the ArduPilot SITL using 

```
sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=libraries/SITL/examples/Airsim/quadX.parm --console --map
```

For closing, first stop the AirSim simulation by pressing the Stop button, then close Ardupilot.  
If Ardupilot is closed first, then UE hangs and you'll need to force close it.  
You can restart by just pressing the Play button and then start the Ardupilot side, no need to close the Editor completely and then start it again


#### Run on different machines
Change `UdpIp` to the IP address of the machine running Ardupilot, use `--sim-address` to specify Airsim's IP address


Using different ports -  
`UdpPort` denotes the port no. which Ardupilot receives the sensor data on (i.e. the port that Airsim sends the data to)  
`SitlPort` assigns the motor control port on which Airsim receives the rotor control message

- `--sim-port-in` should be equal to sensor port i.e. `UdpPort`  
- `--sim-port-out` should be equal to motor control port i.e. `SitlPort`
