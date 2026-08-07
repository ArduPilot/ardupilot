#!/bin/bash

# after this has run you should be able to:

# vagrant ssh autotest-server
# APM/build_autotest.sh

# this should run through the autobuild steps and leave binaries in
# APM/buildslogs/binaries.  It is worth applying patches to
# build_binaries.py, board_list.py and autotest.py to shorten cycle
# times while testing infrastructure changes.

set -e
set -x

# swiped from initvagrant.sh:

VAGRANT_USER=ubuntu
if [ -e /home/vagrant ]; then
    # prefer vagrant user
    VAGRANT_USER=vagrant
fi
echo USING VAGRANT_USER:$VAGRANT_USER

cd /home/$VAGRANT_USER

echo "calling pre-reqs script..."
sudo -H -u $VAGRANT_USER /vagrant/Tools/environment_install/install-prereqs-ubuntu.sh -y
echo "...pre-reqs script done... initvagrant.sh continues."

# end called from initvagrant.sh

apt install -y timelimit g++-10-aarch64-linux-gnu

# autotest server doesn't use our standard pattern of dumping things in /opt:
mkdir -p arm-gcc
pushd arm-gcc
ln -sf /opt/gcc-arm-none-eabi-10-2020-q4-major g++-10.2.1
popd

cat <<"EOF" | sudo -u $VAGRANT_USER -H bash

# configure git so we can tag
git config --global user.email "you@example.com"
git config --global user.name "Your Name"


mkdir -p APM
pushd APM

# clone ArduPilot and related repos
if test -e APM; then
   pushd APM
   git fetch origin
   git reset --hard origin/master
   popd
else
  git clone https://github.com/ardupilot/ardupilot APM
fi

for i in MAVProxy pymavlink; do
  if test -e $i; then
     pushd APM
       git fetch origin
       git reset --hard origin/master
     popd
  else
    git clone https://github.com/ardupilot/$i
  fi
done

mkdir -p buildlogs
cp APM/Tools/scripts/build_autotest.sh .
popd

EOF

NEW_CRONTAB="/tmp/new.crontab"
cat >"$NEW_CRONTAB" <<"EOF"
#*/5 * * * * /home/vagrant/APM/build_autotest.sh
#*/15 * * * * /home/vagrant/bin/sitl_dl.sh
#*/15 * * * * /home/vagrant/bin/qurt_dl.sh
#*/15 * * * * /home/vagrant/bin/update_webtools.sh
#*/5 * * * * /home/vagrant/APM/generate-all-files-html.sh
#*/5 * * * * /home/vagrant/APM/env-dump.sh
#*/5 * * * * /home/vagrant/APM/update_mp.sh
#*/5 * * * * /home/vagrant/APM/APM/Tools/scripts/unpack_mp.sh /home/vagrant/APM/buildlogs/binaries/Tools/MissionPlanner >> APM/unpack.log
#0 0 * * * /home/vagrant/APM/cleanup.sh
#0 */3 * * * /home/vagrant/bin/fetch_ublox_assist.sh
#*/5 * * * * /home/vagrant/bin/chibios_svn_update.sh
#1 */6 * * * /home/vagrant/bin/chibios_svn_zip.sh
#3 */1 * * * /home/vagrant/bin/gen_manifest.sh
#*/5 * * * * /home/vagrant/cron/gen_build_sizes.sh
## 0 3 * * * /home/vagrant/cron/run-update-features.sh
#10 */4 * * * /home/vagrant/bin/fwstats.sh
#10 7 * * * /home/vagrant/cron/clean_hex.sh
EOF

crontab -u $VAGRANT_USER $NEW_CRONTAB

