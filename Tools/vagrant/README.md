# Vagrant for SITL and ardupilot development

We support a vagrant container for _easily_ running SITL (software in the loop simulator) and compling Ardupilot code.
This README is mean't to be the instructions on how to run this.

## Install vagrant for your platform

Windows, OS-X and Linux are supported.  [Download](https://www.vagrantup.com/downloads.html) and install per their instructions.

If you are on Windows you will want to [install](https://www.virtualbox.org/wiki/Downloads) VirtualBox prior to installing Vagrant.

## Start a vagrant instance

* Change the current directory to be any directory inside this source tree and run "vagrant up".  It will start running the instance
in a VM (the initial time you run this command it will likely need to fetch many things).  All the files in this directory tree will
magically appear inside the running instance at /vagrant.  

## Start running SITL

In your vagrant shell run:
<pre>
vagrant ssh -c "sim_vehicle.sh -j 2"
</pre>
This will build the Ardupilot code if needed and then run the simulator.  The mavlink prompt allows you to monitor vehicle state.  To exit
the simulation just press control-d to exit the mavlink shell.

## Run mission planner or mavproxy in your main OS

You can now connect to the running simulator from your main OS.  Just connect to UDP port 14550, either from mission planner or mavproxy. 
The mavproxy command is "mavproxy.py --master=127.0.0.1:14550"

## Shutting down the simulator

* When you are done with the simulator, just press ctrl-d in the vagrant ssh window to exit the special mavproxy that is gluing everything together.
* Run "vagrant suspend" to stop the running VM.  When you need it again just run "vagrant up" to resume.

