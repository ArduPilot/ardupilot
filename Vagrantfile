# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant::Config.run do |config|
  config.vm.box = "ubuntu-12.04-32bit"
  config.vm.box_url = "http://files.vagrantup.com/precise32.box"

  config.vm.share_folder("ardupilot", "/home/vagrant/ardupilot", ".")

  # Allow symlinks
  config.vm.customize ["setextradata", :id, "VBoxInternal2/SharedFoldersEnableSymlinksCreate/cross-compiler", "1"]
  # Otherwise the compile will go into swap, making things slow
  config.vm.customize ["modifyvm", :id, "--memory", "2048"]
end

