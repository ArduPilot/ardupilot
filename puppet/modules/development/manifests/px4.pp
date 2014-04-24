class development::px4 {

    require preconditionals

    package { 'python-serial':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'python-argparse':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'openocd':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'flex':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'bison':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'libncurses5-dev':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'autoconf':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'texinfo':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'libftdi-dev':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'libtool':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'zlib1g-dev':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'zip':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'genromfs':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    common::netinstall{ 'gcc-arm-none-eabi-4_8-2013q4':
        url                  =>   'https://launchpad.net/gcc-arm-embedded/4.8/4.8-2013-q4-major/+download/gcc-arm-none-eabi-4_8-2013q4-20131204-linux.tar.bz2',
        extracted_dir        =>   'gcc-arm-none-eabi-4_8-2013q4',
        destination_dir      =>   '/home/vagrant',
        extract_command      =>   'tar xjf',
        postextract_command  =>   ':',
    } ->
    file_line { 'gcc-arm-path':
        line  => 'export PATH=/home/vagrant/gcc-arm-none-eabi-4_8-2013q4/bin:$PATH',
        path  => '/home/vagrant/.profile'
    } ->
    file { '/usr/lib/ccache/arm-none-eabi-g++':
       ensure                => 'link',
       target                => '/usr/bin/ccache',
       require               => Package [ 'ccache' ]
    } ->
    file { '/usr/lib/ccache/arm-none-eabi-gcc':
       ensure                => 'link',
       target                => '/usr/bin/ccache',
       require               => Package [ 'ccache' ]
    }

}