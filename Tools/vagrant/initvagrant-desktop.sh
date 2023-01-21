#!/bin/bash
echo "---------- $0 start ----------"

set -e
set -x

/vagrant/Tools/vagrant/initvagrant.sh

VAGRANT_USER=ubuntu
if [ -e /home/vagrant ]; then
    # prefer vagrant user
    VAGRANT_USER=vagrant
fi

apt-get update

RELEASE_CODENAME=$(lsb_release -c -s)

PACKAGES="ubuntu-desktop"
if [ ${RELEASE_CODENAME} == 'jammy' ]; then
    PACKAGES="$PACKAGES dbus-x11"
fi

apt-get install -y $PACKAGES

GDB_CONF="/etc/gdm3/custom.conf"
perl -pe 's/#  AutomaticLoginEnable = true/AutomaticLoginEnable = true/'  -i "$GDB_CONF"
perl -pe 's/#  AutomaticLogin = user1/AutomaticLogin = vagrant/' -i "$GDB_CONF"

cat >>/etc/xdg/autostart/open-gnome-terminal.desktop <<EOF
[Desktop Entry]
Type=Application
Name=Start gnome terminal
TryExec=gnome-terminal
Exec=gnome-terminal

X-GNOME-Autostart-Phase=Application
EOF

# disable the screensaver:
sudo -u "$VAGRANT_USER" dbus-launch gsettings set org.gnome.desktop.session idle-delay 0

# don't show the initial setup crap:
sudo -u "$VAGRANT_USER" mkdir -p /home/"$VAGRANT_USER"/.config
echo "yes" | sudo -u "$VAGRANT_USER" dd of=/home/"$VAGRANT_USER"/.config/gnome-initial-setup-done

# sssd is missing config:
if [ ${RELEASE_CODENAME} == 'jammy' ]; then
    systemctl disable sssd
fi

# start the graphical environment right now:
systemctl isolate graphical.target

echo "---------- $0 end ----------"
