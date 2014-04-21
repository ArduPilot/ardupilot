class development::sitl {

    require preconditionals

    package { 'g++':
        ensure          =>  'installed',
        provider        =>  'apt',
    } 
}