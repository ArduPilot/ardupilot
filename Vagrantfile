# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|

  config.vm.box = "ubuntu/yakkety32"
  # push.app = "geeksville/ardupilot-sitl"

  # The following forwarding is not necessary (or possible), because our sim_vehicle.py is smart enough to send packets
  # out to the containing OS
  # config.vm.network "forwarded_port", guest: 14550, host: 14550, protocol: "udp"

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
      # Don't boot with headless mode
      #   vb.gui = true
      #
      #   # Use VBoxManage to customize the VM. For example to change memory:
      vb.customize ["modifyvm", :id, "--memory", "2048"]
      vb.customize ["modifyvm", :id, "--ioapic", "on"]
      vb.customize ["modifyvm", :id, "--cpus", "2"]
      # Make some effort to avoid clock skew
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", "5000"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-start"]
      vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", "1"]
  end

  # The created VM sets PX4_WINTOOL=y to allow builds to proceed using shared folders with using symlinks.
  # However shared folders are quite slow. If you have rsync installed then this is a faster way of building.
  # In addition there are problems with px4-clean when using shared folders. Using rsync avoids this.
  # config.vm.synced_folder ".", "/vagrant", type: "rsync", rsync__auto: true
  
  # If you are on windows then you must use a version of git >= 1.8.x to update the submodules
  # in order to build. Older versions of git use absolute paths for submodules which confuses things.

  config.vm.provision :shell, path: "Tools/vagrant/initvagrant.sh"  
end

