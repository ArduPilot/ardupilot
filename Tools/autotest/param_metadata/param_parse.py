import os, glob, re

from param import *

def wiki_parameters(g):    
    wiki_fname = '%s-Parameters.wiki' % g.name
    f = open(wiki_fname, mode='w')
    
    t = '''#summary Dynamically generated list of documented parameters for %s
    
    = %s Parameters =
    
    '''  % (g.name, g.name)
    
    for param in g.params:
        t += "\n\n=== %s (%s) ===" % (param.DisplayName, param.name)
        t += "\n\n_%s_\n" % param.Description
        for field in param.__dict__.keys():
            if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                t += " * %s: %s\n" % (field, param.__dict__[field])
                
    print t
    f.write(t)
    f.close



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
    #print "\n\n===\n\n\nProcessing %s" % vehicle.name
    
    f = open(vehicle.path+'/Parameters.pde')
    p_text = f.read()
    f.close()
    
    prog_param = re.compile(r"@Param:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*GSCALAR", re.MULTILINE)
    prog_param_fields = re.compile(r"\s*//\s*@(\w+): (.*)")

    prog_groups = re.compile(r"@Group:\s*(\w+).*\n.*@Path:\s*(.*)\s*\n\s*GOBJECT", re.MULTILINE)
    
    prog_group_param = re.compile(r"@Param:\s*(\w+).*((?:\n\s*//\s*@(\w+): (.*))+)\s*AP_GROUPINFO", re.MULTILINE)

    param_matches = prog_param.findall(p_text)
    group_matches = prog_groups.findall(p_text)
    
    ##print group_matches
    for group_match in group_matches:
        paths = prog_param_fields.findall(group_match[1])
        l = Library(group_match[0],group_match[1])
        if l not in libraries:
            libraries.append(l)
        
        
        
    for param_match in param_matches:
        p = Parameter(param_match[0])
        #print p.name + ' '
        field_text = param_match[1]
        #  #print "\n\nParameter: %s\n" % p.name
        fields = prog_param_fields.findall(field_text)
        for field in fields:
            if field[0] in known_param_fields:
                #  #print "    %s: %s" % (field[0], field[1])
                setattr(p, field[0], field[1])
            else:
                print "unknown parameter metadata field %s" % field[0]
                
        vehicle.params.append(p)
        
    ##print vehicle.__dict__
    #print "Processed %u params" % len(vehicle.params)
    
#print "Found %u documented libraries" % len(libraries)

for library in libraries:
    #print "\n\n===\n\n\nProcessing library %s" % library.name
    
    paths = library.path.split(',')
    for path in paths:
        path = path.strip()
        #print "\n Processing file %s" % path
        f = open(os.path.normpath(os.path.join(apm_path + '/libraries/' + path)))
        p_text = f.read()
        f.close()
        
        param_matches = prog_group_param.findall(p_text)
        #print "Found %u documented parameters" % len(param_matches)
        for param_match in param_matches:
            p = Parameter(param_match[0])
            #print p.name + ' '
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
            
    #print "Processed %u documented parameters" % len(library.params)


for vehicle in vehicles:
    wiki_parameters(vehicle)
    
for library in libraries:
    wiki_parameters(library)
    
