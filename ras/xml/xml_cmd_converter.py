import time

import pandas as pd
import xml.etree.ElementTree as et

from itertools import chain

xml_content = 'mavlink_common.xml'

tree = et.parse(xml_content)
root = tree.getroot()

data = []
for enums in root.findall('.//enums'):
    for enum in root.findall('.//enum'):
        name = enum.get('name')
        if name == 'MAV_CMD':
            for entry in enum:
                cmd_dict = {}
                if entry.get('name') == None:
                    pass
                else:
                    cmd_dict['Id'] = entry.get('value')
                    cmd_dict['Name'] = entry.get('name')
                    cmd_dict['HasLocation'] = entry.get('hasLocation')
                    cmd_dict['IsDestination'] = entry.get('isDestination')

                    entry_list = []
                    for child in entry:
                        if child.attrib == {}:
                            if child.tag == 'wip':
                                cmd_dict['WorkInProgress'] = 'True'
                            else:
                                cmd_dict['Description'] = child.text
                                cmd_dict['WorkInProgress'] = 'False'
                        else:
                            child_group = (child.attrib, child.text)
                            entry_list.append(child_group)
                        
                        if child.tag == 'deprecated':
                            del cmd_dict['Id']
                            del cmd_dict['Name']
                            del cmd_dict['HasLocation']
                            del cmd_dict['IsDestination']
                    
                    cmd_dict['Parameters'] = list(chain(*entry_list))


                    # for entry_content in entry:
                    #     if entry_content.tag.startswith('deprecated'):
                    #         del cmd_dict['id']
                    #         del cmd_dict['name']
                    #         del cmd_dict['description']
                    #         del cmd_dict['entry']

                    data.append(cmd_dict)

df = pd.DataFrame(data)
df.dropna(inplace = True)
df.reset_index(drop=True, inplace=True)
print(df)

# df.to_excel('mavlink_commands.xlsx')
