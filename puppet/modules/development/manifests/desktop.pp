class development::desktop {
    require preconditionals
    
    file { '/home/vagrant/Desktop':
        ensure      => "directory"
    }
    file { '/home/vagrant/Desktop/Terminator.desktop':
        source      => 'puppet:///modules/development/Terminator.desktop',
        mode        =>  '777',
        require     =>  File [ '/home/vagrant/Desktop' ]
    }
    file { '/home/vagrant/Desktop/Eclipse.desktop':
        source      => 'puppet:///modules/development/Eclipse.desktop',
        mode        =>  '777',
        require     =>  File [ '/home/vagrant/Desktop' ]
    }
    file { [ '/home/vagrant/.gconf',
             '/home/vagrant/.gconf/desktop',
             '/home/vagrant/.gconf/desktop/gnome',
             '/home/vagrant/.gconf/desktop/gnome/interface',
             '/home/vagrant/.gconf/desktop/gnome/background' ]:
        ensure      => "directory",
        mode        =>  '666',
    } -> 
    file { '/home/vagrant/.gconf/desktop/gnome/background/%gconf.xml':
        source      => 'puppet:///modules/development/%gconf-background.xml',
        mode        =>  '666',
    } ->
    file { '/home/vagrant/.gconf/desktop/gnome/interface/%gconf.xml':
        source      => 'puppet:///modules/development/%gconf-interface.xml',
        mode        =>  '666',
    }
}