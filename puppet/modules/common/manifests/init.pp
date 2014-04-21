include stdlib

class common {

}

define common::netinstall (
  $url,
  $extracted_dir,
  $destination_dir,
  $owner = "root",
  $group = "root",
  $work_dir = "/var/tmp",
  $extract_command = "tar zxf",
  $configure_command = "configure",
  $configure_options = "",
  $preextract_command = "",
  $postextract_command = ""
)
{
    $source_filename = url_parse($url, file)

    if $preextract_command
    {
        exec { "PreExtract $source_filename":
            command => $preextract_command,
            path => '/usr/bin:/usr/sbin:/sbin',
            before  => Exec["Extract $source_filename"],
            refreshonly => true,
        }
    }

    exec {
        "Retrieve $url":
            cwd     => "$work_dir",
            command => "wget $url",
            path => '/usr/bin:/usr/sbin:/sbin',
            creates => "$work_dir/$source_filename",
            timeout => 3600,
    }

    ### EXTRACTION
    exec {
        "Extract $source_filename":
            command => "mkdir -p $destination_dir && cd $destination_dir && $extract_command $work_dir/$source_filename",
            path => '/usr/bin:/usr/sbin:/sbin:/bin',
            unless  => "ls $destination_dir/$extracted_dir",
            creates => "$destination_dir/$extracted_dir",
            require => Exec["Retrieve $url"],
    }

    ### POSTEXTRACT
    if $postextract_command == '' {
        $final_postextract_command = "$destination_dir/$extracted_dir/$configure_command $configure_options && make && make install > $destination_dir/$extracted_dir.install"
    }
    else {
        $final_postextract_command = $postextract_command
    }

    notify { "Trace PostExtract $source_filename":
        name   => "Post command complete $source_filename, $final_postextract_command",
        require => Exec [ "PostExtract $source_filename" ],
    }
    exec {
        "PostExtract $source_filename":
            command => "$final_postextract_command",
            cwd => "$destination_dir/$extracted_dir",
            path => '/usr/bin:/usr/sbin:/sbin:/bin:',
            subscribe => Exec["Extract $source_filename"],
            refreshonly => true,
            timeout => 3600,
            require => Exec["Retrieve $url"],
    }

}