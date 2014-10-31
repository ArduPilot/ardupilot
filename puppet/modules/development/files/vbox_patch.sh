#! /bin/bash
echo "Fixing 4.3.10 guest additions"
if [ -d /opt/VBoxGuestAdditions-4.3.10/lib/VBoxGuestAdditions -a ! -d /usr/lib/VBoxGuestAdditions ]
then
    sudo ln -s /opt/VBoxGuestAdditions-4.3.10/lib/VBoxGuestAdditions /usr/lib/VBoxGuestAdditions
    echo "Patched!"
else
    echo "Already patched!"
fi
