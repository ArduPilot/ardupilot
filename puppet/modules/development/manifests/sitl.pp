class development::sitl {

    require preconditionals

    package { 'python-wxgtk2.8':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'python-scipy':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'python-opencv':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    package { 'python-pip':
        ensure          =>  'installed',
        provider        =>  'apt',
    }

    exec { 'pip-pymavlink':
        command         =>   'pip install pymavlink',
        onlyif          =>   'test `pip freeze | grep -c "pymavlink"` -eq 0',
        require         =>    Package [ 'python-pip' ]
    }

    exec { 'pip-mavproxy':
        command         =>   'pip install MAVProxy',
        onlyif          =>   'test `pip freeze | grep -c "MAVProxy"` -eq 0',
        require         =>    Package [ 'python-pip' ]
    }

    file_line { 'autotest-path':
        line  => 'export PATH=$PATH:/home/vagrant/ardupilot/Tools/autotest',
        path  => '/home/vagrant/.profile'
    }
    file_line { 'MAVProxy-path':
        line  => 'export PATH=$PATH:/home/vagrant/MAVProxy',
        path  => '/home/vagrant/.profile'
    }
    file_line { 'MAVProxy-path-examples':
        line  => 'export PATH=$PATH:/home/vagrant/mavlink/pymavlink/examples',
        path  => '/home/vagrant/.profile'
    }

    common::netinstall{ 'zeromq-4':
        url                  =>   'http://download.zeromq.org/zeromq-4.0.4.tar.gz',
        extracted_dir        =>   'zeromq-4.0.4',
        destination_dir      =>   '/home/vagrant',
    } ->
    exec { 'pip-libzmq':
        command         =>   'pip install pyzmq',
        onlyif          =>   'test `pip freeze | grep -c "pyzmq"` -eq 0',
        require         =>   Package [ 'python-pip' ]
    }

    package { 'flightgear':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
    
}