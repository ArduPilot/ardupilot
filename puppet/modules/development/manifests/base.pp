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
    
    package { 'git':
        ensure          =>  'installed',
        provider        =>  'apt',
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
}