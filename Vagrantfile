# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "ubuntu-12.04-32bit"
  config.vm.box_url = "http://files.vagrantup.com/precise32.box"

  config.vm.synced_folder ".", "/home/vagrant/ardupilot"

  config.vm.provision "shell" do |s|
      s.privileged = false
      s.path = "Tools/scripts/install-prereqs-ubuntu.sh"
      s.args = ["-y"]
    end

  config.vm.provider "virtualbox" do |vb|

    # Allow symlinks
    vb.customize ["setextradata", :id, "VBoxInternal2/SharedFoldersEnableSymlinksCreate/cross-compiler", "1"]

    # Otherwise the compile will go into swap, making things slow
    vb.customize ["modifyvm", :id, "--memory", "2048"]
  end

end

