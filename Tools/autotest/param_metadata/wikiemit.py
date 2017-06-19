#!/usr/bin/env python

import re

from emit import Emit
from param import known_param_fields


# Emit docs in a form acceptable to the APM wiki site
class WikiEmit(Emit):
    def __init__(self):
        Emit.__init__(self)
        wiki_fname = 'Parameters.wiki'
        self.f = open(wiki_fname, mode='w')
        preamble = '''#summary Dynamically generated list of documented parameters
        = Table of Contents = 
        <wiki:toc max_depth="4" />
            
        = Vehicles =    
        '''
        self.f.write(preamble)

    def close(self):
        self.f.close()

    def camelcase_escape(self, word):
        if re.match(r"([A-Z][a-z]+[A-Z][a-z]*)", word.strip()):
            return "!" + word
        else:
            return word

    def wikichars_escape(self, text):
        for c in "*,{,},[,],_,=,#,^,~,!,@,$,|,<,>,&,|,\,/".split(','):
            text = re.sub("\\" + c, '`' + c + '`', text)
        return text

    def emit_comment(self, s):
        self.f.write("\n\n=" + s + "=\n\n")

    def start_libraries(self):
        self.emit_comment("Libraries")

    def emit(self, g, f):
        t = "\n\n== %s Parameters ==\n" % (self.camelcase_escape(g.name))

        for param in g.params:
            if hasattr(param, 'DisplayName'):
                t += "\n\n=== %s (%s) ===" % (self.camelcase_escape(param.DisplayName), self.camelcase_escape(param.name))
            else:
                t += "\n\n=== %s ===" % self.camelcase_escape(param.name)

            if hasattr(param, 'Description'):
                t += "\n\n_%s_\n" % self.wikichars_escape(param.Description)
            else:
                t += "\n\n_TODO: description_\n"

            for field in param.__dict__.keys():
                if field not in ['name', 'DisplayName', 'Description', 'User'] and field in known_param_fields:
                    if field == 'Values' and Emit.prog_values_field.match(param.__dict__[field]):
                        t += " * Values \n"
                        values = (param.__dict__[field]).split(',')
                        t += "|| *Value* || *Meaning* ||\n"
                        for value in values:
                            v = value.split(':')
                            t += "|| " + v[0] + " || " + self.camelcase_escape(v[1]) + " ||\n"
                    else:
                        t += " * %s: %s\n" % (self.camelcase_escape(field), self.wikichars_escape(param.__dict__[field]))

        # print t
        self.f.write(t)
