# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "ubuntu-12.04-32bit"
  config.vm.box_url = "http://files.vagrantup.com/precise32.box"
  config.vm.synced_folder ".", "/home/vagrant/ardupilot"
  
  config.ssh.forward_x11 = true
  config.ssh.forward_agent = true
  config.vm.provider "virtualbox" do |v|
	v.gui = true
	# Allow symlinks
	v.customize ["setextradata", :id, "VBoxInternal2/SharedFoldersEnableSymlinksCreate/cross-compiler", "1"]
	# Otherwise the compile will go into swap, making things slow
	v.customize ["modifyvm", :id, "--memory", "2048"]
  end
end

