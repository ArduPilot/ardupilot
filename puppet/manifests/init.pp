Exec {path => [ "/usr/local/bin/", "/bin/", "/usr/bin/", "usr/sbin/", "/sbin/", "/usr/sbin/" ]}


package { 'git':
    ensure          =>  'installed',
    provider        =>  'apt',
}