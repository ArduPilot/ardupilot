#!/bin/bash
# install dependencies for travis build testing

set -e
set -v

# Disable ccache for the configure phase, it's not worth it
export CCACHE_DISABLE="true"

CCACHE_ROOT="ccache-3.2.5"
CCACHE_TARBALL="$CCACHE_ROOT.tar.bz2"

mkdir -p $HOME/opt
pushd $HOME

# CCache
dir=$CCACHE_ROOT
if [ ! -d "$HOME/opt/$dir" ]; then
  wget https://www.samba.org/ftp/ccache/$CCACHE_TARBALL
  tar -xf $CCACHE_TARBALL
  pushd $CCACHE_ROOT
  ./configure --prefix="/tmp" --bindir="$HOME/opt/$dir"
  make
  make install
  popd
fi

popd

git clone https://github.com/UrusTeam/avr_toolchain_multi_cross.git $HOME/avrbin

export PATH=$HOME/avrbin/bin:$PATH

mkdir -p $HOME/bin

# symlink to compiler versions
ln -s /usr/bin/gcc-4.9 ~/bin/gcc
ln -s /usr/bin/g++-4.9 ~/bin/g++
ln -s /usr/bin/clang-3.7 ~/bin/clang
ln -s /usr/bin/clang++-3.7 ~/bin/clang++
ln -s /usr/bin/llvm-ar-3.7 ~/bin/llvm-ar

mkdir -p $HOME/ccache

# configure ccache
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/g++
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/gcc
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/clang++
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/clang

exportline="export PATH=$HOME/ccache"
exportline="${exportline}:$HOME/bin"
exportline="${exportline}:$HOME/avrbin/bin"
exportline="${exportline}:$HOME/.local/bin"
exportline="${exportline}:$HOME/opt/$CCACHE_ROOT"
exportline="${exportline}:\$PATH"

if grep -Fxq "$exportline" ~/.profile; then
    echo nothing to do;
else
    echo $exportline >> ~/.profile;
fi

. ~/.profile

avr-g++ -v

pip install --user -U argparse empy pyserial pexpect future lxml

echo $PATH
