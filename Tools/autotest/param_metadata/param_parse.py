#!/usr/bin/env python

import os, glob, re

from param import *
from wikiemit import WikiEmit
from xmlemit import XmlEmit

# Regular expressions for parsing the parameter metadata

prog_param = re.compile(r"@Param:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*G", re.MULTILINE)

prog_param_fields = re.compile(r"\s*//\s*@(\w+): (.*)")
    
prog_groups = re.compile(r"@Group:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*G", re.MULTILINE)
    
prog_group_param = re.compile(r"@Param:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*AP_", re.MULTILINE)

apm_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')
vehicle_paths = glob.glob(apm_path + "*/Parameters.pde")
vehicle_paths.sort(reverse=True)

vehicles = []
libraries = []

for vehicle_path in vehicle_paths:
    name = os.path.basename(os.path.dirname(vehicle_path))
    path = os.path.normpath(os.path.dirname(vehicle_path))
    vehicles.append(Vehicle(name, path))
    #print 'Added %s at %s' % (name, path)
    
for vehicle in vehicles:
    print "===\n\n\nProcessing %s" % vehicle.name
    
    f = open(vehicle.path+'/Parameters.pde')
    p_text = f.read()
    f.close()
    
    

    param_matches = prog_param.findall(p_text)
    group_matches = prog_groups.findall(p_text)
    
    print group_matches
    for group_match in group_matches:
        l = Library(group_match[0])
        fields = prog_param_fields.findall(group_match[1])
        for field in fields:
            if field[0] in known_group_fields:
                setattr(l, field[0], field[1])
            else:
                print "unknown parameter metadata field %s" % field[0]
        if l not in libraries:
            libraries.append(l)
        
        
        
    for param_match in param_matches:
        p = Parameter(vehicle.name+":"+param_match[0])
        print p.name + ' '
        field_text = param_match[1]
        #print "\n\nParameter: %s\n" % p.name
        fields = prog_param_fields.findall(field_text)
        for field in fields:
            if field[0] in known_param_fields:
                #  #print "    %s: %s" % (field[0], field[1])
                setattr(p, field[0], field[1])
            else:
                print "unknown parameter metadata field %s" % field[0]
                
        vehicle.params.append(p)
        
    ##print vehicle.__dict__
    print "Processed %u params" % len(vehicle.params)
    
print "Found %u documented libraries" % len(libraries)

for library in libraries:
    print "===\n\n\nProcessing library %s" % library.name
    
    if hasattr(library, 'Path'):   
        paths = library.Path.split(',')
        for path in paths:
            path = path.strip()
            print "\n Processing file %s" % path
            libraryfname = os.path.normpath(os.path.join(apm_path + '/libraries/' + path))
            if path and os.path.exists(libraryfname):
                f = open(libraryfname)
                p_text = f.read()
                f.close()
            
            param_matches = prog_group_param.findall(p_text)
            print "Found %u documented parameters" % len(param_matches)
            for param_match in param_matches:
                p = Parameter(library.name+param_match[0])
                print p.name + ' '
                field_text = param_match[1]
                #  #print "\n\nParameter: %s\n" % p.name
                fields = prog_param_fields.findall(field_text)
                for field in fields:
                    if field[0] in known_param_fields:
                        #  #print "    %s: %s" % (field[0], field[1])
                        setattr(p, field[0], field[1])
                    else:
                        print "unknown parameter metadata field %s" % field[0]
                    
                ##print p.__dict__
                library.params.append(p)
    else:
        print "Skipped: no Path found"

    print "Processed %u documented parameters" % len(library.params)

    def do_emit(emit):
        for vehicle in vehicles:
            emit.emit(vehicle, f)
        
        emit.start_libraries()
            
        for library in libraries:
            if library.params:
                emit.emit(library, f)
           
        emit.close()
    
    do_emit(XmlEmit())
    do_emit(WikiEmit())



