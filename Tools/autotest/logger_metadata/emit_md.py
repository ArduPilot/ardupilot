import os
import time
import emitter

class MDEmitter(emitter.Emitter):
    def preface(self):
        if os.getenv('BRDOC') is not None:
            now = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            now = now[:-2] + ':' + now[-2:]
            return '\n'.join((
                '+++',
                'title = "Onboard Log Messages"',
                'description = "Message listing for DataFlash autopilot logs."',
                f'date = {now}',
                'template = "docs/page.html"',
                'sort_by = "weight"',
                'weight = 30',
                'draft = false',
                '[extra]',
                'toc = true',
                'top = false',
                '+++\n',
                '<!-- Dynamically generated using Tools/autotest/logger_metadata/parse.py',
                'DO NOT EDIT -->',
                'This is a list of log messages which may be present in DataFlash (`.bin`) '
                'logs produced and stored onboard ArduSub vehicles (see [Log Parameters]'
                '(../parameters/#log-parameters) for creation details). '
                'It is possible to [add a new message]'
                '(https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html) '
                'by modifying the firmware.\n',
                'DataFlash logs can be downloaded and analysed '
                '[from a computer](http://www.ardusub.com/reference/data-logging.html#downloading) '
                'or [through BlueOS]'
                '(@/software/onboard/BlueOS-1.1/advanced-usage/index.md#log-browser).\n'
            ))

        return """<!-- Dynamically generated list of Logger Messages
This page was generated using Tools/autotest/logger_metdata/parse.py

DO NOT EDIT
-->


<h3 style="text-align: center">Onboard Message Log Messages</h3>
<hr />

<p>This is a list of log messages which may be present in logs produced and stored onboard ArduPilot vehicles.</p>

<!-- add auto-generated table of contents with "Table of Contents Plus" plugin -->
[toc exclude="Onboard Message Log Messages"]

"""
    def postface(self):
        return ""

    def start(self):
        self.fh = open("LogMessages.md", mode='w')
        print(self.preface(), file=self.fh)

    def emit(self, doccos, enumerations=None):
        self.start()
        for docco in doccos:
            print(f'## {docco.name}', file=self.fh)
            desc = ''
            if docco.description is not None:
                desc += docco.description
            if docco.url is not None:
                desc += f' ([Read more...]({docco.url}))'
            print(desc, file=self.fh)
            print("\n|FieldName|Units/Type|Description|\n|---|---|---|", file=self.fh)
            for f in docco.fields_order:
                if "description" in docco.fields[f]:
                    fdesc = docco.fields[f]["description"]
                else:
                    fdesc = ""
                if "units" in docco.fields[f] and docco.fields[f]["units"]!="":
                    ftypeunits = docco.fields[f]["units"]
                elif "fmt" in docco.fields[f] and "char" in docco.fields[f]["fmt"]:
                    ftypeunits = docco.fields[f]["fmt"]
                elif "bitmaskenum" in docco.fields[f]:
                    ftypeunits = "bitmask"
                elif "valueenum" in docco.fields[f]:
                    ftypeunits = "enum"
                else:
                    ftypeunits = ""
                print(f'|{f}|{ftypeunits}|{fdesc}|', file=self.fh)
            print("", file=self.fh)
        self.stop()

    def stop(self):
        print(self.postface(), file=self.fh)
        self.fh.close()
