class development::desktop {
    require preconditionals
    
    file { '/home/vagrant/Desktop/Terminator.desktop':
        source      => 'puppet:///modules/development/Terminator.desktop',
        mode        =>  '666',
    }
    file { '/home/vagrant/Desktop/Eclipse.desktop':
        source      => 'puppet:///modules/development/Eclipse.desktop',
        mode        =>  '666',
    }
}