Exec {path => [ "/usr/local/bin/", "/bin/", "/usr/bin/", "usr/sbin/", "/sbin/", "/usr/sbin/" ]}

 package { 'linux-headers-3.2.0-23-generic':
    ensure          =>  'installed',
    provider        =>  'apt',
}

package { 'linux-headers-3.2.0-23-generic-pae':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         => Package [ 'linux-headers-3.2.0-23-generic' ],
}

package { 'ubuntu-desktop':
    ensure          =>  'installed',
    provider        =>  'apt',
}

package { 'gnome-panel':
    ensure          =>  'installed',
    provider        =>  'apt',
    require         =>  Package [ 'ubuntu-desktop' ],
}

package { 'dkms':
    ensure          =>  'installed',
    provider        =>  'apt',
}

package { 'build-essential':
    ensure          =>  'installed',
    provider        =>  'apt',
}