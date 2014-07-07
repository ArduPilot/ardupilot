# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
    config.vm.box = "ubuntu-12.04-32bit"
    config.vm.box_url = "http://files.vagrantup.com/precise32.box"
    config.vm.hostname = "apm-dev"
    config.vm.network "public_network"
    config.vm.synced_folder(".", "/home/vagrant/ardupilot" )
    config.vm.synced_folder("../diydrones/PX4Firmware", "/home/vagrant/PX4Firmware" )
    config.vm.synced_folder("../diydrones/PX4NuttX", "/home/vagrant/PX4NuttX" )
    config.vm.synced_folder("../mavlink/mavlink", "/home/vagrant/mavlink" )
    config.vm.synced_folder("../tridge/jsbsim", "/home/vagrant/jsbsim" )

    
    config.vm.provision "shell", inline: "cp /home/vagrant/ardupilot/puppet/Puppetfile /tmp"
    config.vm.provision "shell", inline: "cp -R /home/vagrant/ardupilot/puppet/modules /tmp"
    config.vm.provision "shell", inline: "cp -R /home/vagrant/ardupilot/puppet/manifests /tmp"

    # Puppet
    config.vm.provision "puppet" do |vb_provision_init|
        vb_provision_init.temp_dir = "/tmp"
        vb_provision_init.options = [ '--modulepath=/tmp/modules' ]
        vb_provision_init.manifests_path = [ "vm", "/tmp/manifests" ]
        vb_provision_init.manifest_file = "init.pp"
    end
    
    config.vm.provision "shell", inline: "gem install librarian-puppet -v 1.0.3"
    config.vm.provision "shell", inline: "cd /tmp && librarian-puppet install --verbose"

    # Puppet
    config.vm.provision "puppet" do |vb_provision|
        vb_provision.temp_dir = "/tmp"
        vb_provision.options = [ '--modulepath=/tmp/modules' ]
        vb_provision.manifests_path = [ "vm", "/tmp/manifests" ]
        vb_provision.manifest_file = "apm_dev.pp"
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

