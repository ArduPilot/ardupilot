class preconditionals {
    file { '/etc/apt/sources.list':
        source      => 'puppet:///modules/preconditionals/sources.list',
        mode        =>  '666',
    } ->
	exec { 'apt-update':
    	command => 'apt-get update',
	} ->
    package { 'build-essential':
        ensure          =>  'installed',
        provider        =>  'apt',
    }
}