class development::base {
    require preconditionals

    package { 'gawk':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'make':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'cmake':
        ensure          =>  'installed',
        provider        =>  'apt',
    }    
    
    # Need to build gtest after gmock so set an explicit dependency chain
    package { 'google-mock':
        ensure          =>  'installed',
        provider        =>  'apt',
        require         =>  Package[ 'cmake' ]
    } ->
    exec { 'build_gtest':
        cwd             =>  '/usr/src/gtest',
        command         =>  'cmake -E make_directory build && cmake -E chdir build cmake .. && cmake --build build && cp build/libgtest* /usr/local/lib',
        path            =>  '/usr/bin:/usr/sbin:/sbin:/bin',
    }
    
    
    package { 'arduino-core':
        ensure          =>  'installed',
        provider        =>  'apt',
    }    

    package { 'curl':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'emacs23':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'terminator':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
    package { 'ccache':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    exec { 'usermod-dialout':
        command         =>   'usermod -a -G dialout vagrant',
        onlyif          =>   'test `groups | grep -c "dialout"` -eq 0'
    }
    
    # Prevent conflicts with the firmware flash
    package { 'modemmanager':
        ensure          =>  'purged',
        provider        =>  'apt',
    }
    
    package { 'eclipse':
        ensure          =>  'installed',
        provider        =>  'apt',
    } ->
    package { 'eclipse-cdt':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'gpsbabel':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'imagemagick':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

}