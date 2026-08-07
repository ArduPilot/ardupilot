#!/usr/bin/env bash
echo "---------- $0 start ----------"

# this script is run by the root user in the virtual machine

set -e
set -x

who=$(whoami)
echo "Initial setup of SITL-vagrant instance."
if [ $who != 'root' ]; then
    echo "SORRY, MUST RUN THIS SCRIPT AS ROOT, GIVING UP"
    exit 1
fi

DISTRIBUTION_ID=$(lsb_release -i -s)
if [ ${DISTRIBUTION_ID} == 'Ubuntu' ]; then
  DISTRIBUTION_CODENAME=$(lsb_release -c -s)
fi

VAGRANT_USER=ubuntu
if [ -e /home/vagrant ]; then
    # prefer vagrant user
    VAGRANT_USER=vagrant
fi
echo USING VAGRANT_USER:$VAGRANT_USER

cd /home/$VAGRANT_USER

IS_BENTO=0
if [ -e /etc/update-motd.d/99-bento ]; then
    IS_BENTO=1
fi

# artful rootfs is 2GB without resize.  Do not resize if using Bento:
if [ ! $IS_BENTO ]; then
    sudo resize2fs /dev/sda1
fi

DASHDASHLOGIN=""
if false ||
     [ ${DISTRIBUTION_CODENAME} == 'oracular' ] ||
     [ ${DISTRIBUTION_CODENAME} == 'plucky' ] ||
     [ ${DISTRIBUTION_CODENAME} == 'questing' ] ||
     false; then
    # we run out of space in tmpfs /tmp while compiling wxpython, so
    # do it elsewhere:
cat <<"EOF" | sudo -H -u vagrant bash
    mkdir -p $HOME/tmp
    echo "export TMPDIR=$HOME/tmp" >>$HOME/.bashrc
    echo "export TMPDIR=$HOME/tmp" >>$HOME/.profile
EOF
    export TMPDIR=/home/vagrant/tmp
    DASHDASHLOGIN="--login"
fi

echo "calling pre-reqs script..."
sudo $DASHDASHLOGIN -H -u $VAGRANT_USER /vagrant/Tools/environment_install/install-prereqs-ubuntu.sh -y
echo "...pre-reqs script done... initvagrant.sh continues."

# valgrind support:
apt-get install -y valgrind

# gdb support:
apt-get install -y gdb

sudo -u $VAGRANT_USER ln -fs /vagrant/Tools/vagrant/screenrc /home/$VAGRANT_USER/.screenrc

# enable permissive ptrace:
perl -pe 's/kernel.yama.ptrace_scope = ./kernel.yama.ptrace_scope = 0/' -i /etc/sysctl.d/10-ptrace.conf
echo 0 > /proc/sys/kernel/yama/ptrace_scope

RELEASE_CODENAME=$(lsb_release -c -s)

# build JSB sim
apt-get install -y libtool automake autoconf libexpat1-dev cmake
#  libtool-bin
sudo --login -u $VAGRANT_USER /vagrant/Tools/scripts/build-jsbsim.sh

# adjust environment for every login shell:
DOT_PROFILE=/home/$VAGRANT_USER/.profile
echo "source /vagrant/Tools/vagrant/shellinit.sh" |
    sudo -u $VAGRANT_USER dd conv=notrunc oflag=append of=$DOT_PROFILE

BASHRC="/home/$VAGRANT_USER/.bashrc"
# adjust environment for every login shell:
BASHRC_GIT="/vagrant/Tools/vagrant/bashrc_git"
echo "source $BASHRC_GIT" |
    sudo -u $VAGRANT_USER dd conv=notrunc oflag=append of=$BASHRC

# link a half-way decent .mavinit.scr into place:
sudo --login -u $VAGRANT_USER ln -sf /vagrant/Tools/vagrant/mavinit.scr /home/$VAGRANT_USER/.mavinit.scr

# no multipath available, stop mutlipathd complaining about lack of data:
if [ ${RELEASE_CODENAME} == 'jammy' ]; then
    cat >>/etc/multipath.conf <<EOF
blacklist { devnode "sda" }
blacklist { devnode "sdb" }
EOF
fi


#Plant a marker for sim_vehicle that we're inside a vagrant box
touch /ardupilot.vagrant

# Now you can run
# vagrant ssh -c "screen -d -R"
echo "---------- $0 end ----------"

