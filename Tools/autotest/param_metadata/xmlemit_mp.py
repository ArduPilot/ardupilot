from xml.sax.saxutils import escape, quoteattr

from emit import Emit
from param import known_param_fields, known_units
from lxml import etree

# Emit ArduPilot documentation in an machine readable XML format for Mission Planner
class XmlEmitMP(Emit):
    def __init__(self, *args, **kwargs):
        Emit.__init__(self, *args, **kwargs)
        self.mp_fname = 'ParameterMetaData.xml'
        self.f = open(self.mp_fname, mode='w')
        self.preamble = '''<?xml version="1.0" encoding="utf-8"?>\n'''
        self.f.write(self.preamble)
        self.f.write('<Params>\n')
        self.gname = None
        self.skip_name = False

    def close(self):
        self.f.write('  </%s>\n' % self.gname)
        self.f.write('''</Params>\n''')
        self.f.close()
        # sort and reformat XML
        parser = etree.XMLParser(remove_blank_text=True)
        tree = etree.parse(self.mp_fname, parser)
        root = tree.getroot()
        vehicle = tree.find(self.gname)
        sort_xml_node(vehicle)
        sorted_unicode = etree.tostring(root, pretty_print=True, encoding='unicode')
        f = open(self.mp_fname, mode='w')
        f.write(self.preamble)
        f.write(sorted_unicode)
        f.close()

    def emit_comment(self, s):
        self.f.write("<!-- " + s + " -->")

    def start_libraries(self):
        self.skip_name = True

    def emit(self, g):
        t = ""
        if not self.skip_name:
            self.gname = g.reference
            if self.gname == "ArduCopter":
                self.gname = "ArduCopter2"
            if self.gname == "APMrover2" or self.gname == "Rover":
                self.gname = "ArduRover"
            t = '  <%s>\n' % self.gname

        for param in g.params:
            # Begin our parameter node
            # Get param name and and remove key
            name = param.name.split(':')[-1]

            t += '    <%s>\n' % name

            if hasattr(param, 'DisplayName'):
                t += '      <DisplayName>%s</DisplayName>\n' % param.DisplayName

            if hasattr(param, 'Description'):
                t += '      <Description>%s</Description>\n' % escape(param.Description)  # i.e. parameter docs
            if hasattr(param, 'User'):
                t += '      <User>%s</User>\n' % param.User  # i.e. Standard or Advanced

            # not used yet
            # if hasattr(param, 'Calibration'):
            #     t += '  <Calibration>%s</Calibration\n' % quoteattr(param.Calibration)

            # Add values as chidren of this node
            for field in param.__dict__.keys():
                if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                    t += '      <%s>%s</%s>\n' % (field, escape(param.__dict__[field]), field)

            t += '    </%s>\n' % name

        # print t
        self.f.write(t)


def sort_xml_node(node):
    if not isinstance(node.tag, str):
        # not a TAG, it is comment or DATA
        # no need to sort
        return
    # sort child along attr
    node[:] = sorted(node, key=lambda field: field.tag)

    # and recurse
    for child in node:
        child[:] = sorted(child, key=lambda field: field.tag)
