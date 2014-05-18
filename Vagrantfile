# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
    config.vm.box = "ubuntu-12.04-32bit"
    config.vm.box_url = "http://files.vagrantup.com/precise32.box"
    config.vm.hostname = "apm-dev"
    config.vm.network "public_network"
    config.vm.synced_folder(".", "/home/vagrant/ardupilot" )
    config.vm.synced_folder("../../diydrones/PX4Firmware", "/home/vagrant/PX4Firmware" )
    config.vm.synced_folder("../../diydrones/PX4NuttX", "/home/vagrant/PX4NuttX" )
    config.vm.synced_folder("../../mavlink/mavlink", "/home/vagrant/mavlink" )
    config.vm.synced_folder("C:/Users/fchan/Dropbox/Private/APM", "/home/vagrant/dropbox" )
    

    # Puppet
    config.vm.provision :puppet do |vb_provision|
        vb_provision.manifests_path="puppet/manifests"
        vb_provision.manifest_file="apm_dev.pp"
        vb_provision.module_path="puppet/modules"
    end
  
    # VBox Manage
    config.vm.provider :virtualbox do |vb_config|
        # Allow symlinks
        vb_config.customize ["setextradata", :id, "VBoxInternal2/SharedFoldersEnableSymlinksCreate/v-root", "1"]
        # Otherwise the compile will go into swap, making things slow
        vb_config.customize ["modifyvm", :id, "--memory", "2048"]
        vb_config.customize ["modifyvm", :id, "--vram", "256"]
        vb_config.customize ["modifyvm", :id, "--usb", "on"]        
        vb_config.gui = true
	end
end

