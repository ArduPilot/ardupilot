#!/usr/bin/env python

import os, glob, re

from param import *


# Regular expressions for parsing the parameter metadata

prog_param = re.compile(r"@Param:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*G", re.MULTILINE)

prog_param_fields = re.compile(r"\s*//\s*@(\w+): (.*)")
    
prog_groups = re.compile(r"@Group:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*G", re.MULTILINE)
    
prog_group_param = re.compile(r"@Param:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*AP_", re.MULTILINE)

prog_values_field = re.compile(r"\s*(-?\w+:\w+)+,*")

def camelcase_escape(word):
    if re.match(r"([A-Z][a-z]+[A-Z][a-z]*)", word.strip()):
        return "!"+word
    else:
        return word

def wikichars_escape(text):
    for c in "*,{,},[,],_,=,#,^,~,!,@,$,|,<,>,&,|,\,/".split(','):
        text = re.sub("\\"+c, '`'+c+'`', text)
    return text

def wiki_parameters(g, f):    
    
    t = "\n\n== %s Parameters ==\n"  % (camelcase_escape(g.name))
    
    for param in g.params:
        if hasattr(param, 'DisplayName'):   
            t += "\n\n=== %s (%s) ===" % (camelcase_escape(param.DisplayName),camelcase_escape(param.name))
        else:
            t += "\n\n=== %s ===" % camelcase_escape(param.name)
                
        if hasattr(param, 'Description'):   
            t += "\n\n_%s_\n" % wikichars_escape(param.Description)
        else:
            t += "\n\n_TODO: description_\n"
            
        for field in param.__dict__.keys():
            if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                if field == 'Values' and prog_values_field.match(param.__dict__[field]):
                    t+= " * Values \n"
                    values = (param.__dict__[field]).split(',')
                    t+="|| *Value* || *Meaning* ||\n"
                    for value in values:
                        v = value.split(':')
                        t+="|| "+v[0]+" || "+camelcase_escape(v[1])+" ||\n"
                else:
                    t += " * %s: %s\n" % (camelcase_escape(field), wikichars_escape(param.__dict__[field]))
                
    #print t
    f.write(t)


apm_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../')
vehicle_paths = glob.glob(apm_path + "*/Parameters.pde")

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

wiki_fname = 'Parameters.wiki'
f = open(wiki_fname, mode='w')
preamble = '''#summary Dynamically generated list of documented parameters
= Table of Contents = 
<wiki:toc max_depth="4" />
    
= Vehicles =    
'''
f.write(preamble)

for vehicle in vehicles:
    wiki_parameters(vehicle, f)

t = "\n\n=Libraries=\n\n"
f.write(t)
    
for library in libraries:
    if library.params:
        wiki_parameters(library, f)
   
f.close

