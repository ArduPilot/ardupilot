Exec {path => [ "/usr/local/bin/", "/bin/", "/usr/bin/", "usr/sbin/", "/sbin/", "/usr/sbin/" ]}

exec { 'apt-update':
    command => 'apt-get update',
}

package { 'linux-headers-3.2.0-23-generic':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         => Exec ['apt-update' ]
}

package { 'linux-headers-3.2.0-23-generic-pae':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         => Package [ 'linux-headers-3.2.0-23-generic' ]
}

package { 'ubuntu-desktop':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         => Exec ['apt-update' ]
}

package { 'gnome-panel':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         =>  Package [ 'ubuntu-desktop' ],
}

package { 'dkms':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         => Exec ['apt-update' ]
}

package { 'build-essential':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         => Exec ['apt-update' ]
}