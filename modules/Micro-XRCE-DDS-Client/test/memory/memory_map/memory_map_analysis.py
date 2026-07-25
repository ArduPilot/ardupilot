import sys
import re
import csv

pattern = re.compile('(?P<name>[\w]+)\s(?P<text>[\d]+)\s(?P<data>[\d]+)\s(?P<bss>[\d]+)\s')

# Take data from file.
complete_profile_data = []
core_profile_data = []
profiles_data = [[], [], [], []]
with open(sys.argv[1], 'r') as memory_map_file:
    # Complete profile data
    line = memory_map_file.readline()
    m = pattern.match(line)
    complete_profile_data = [int(m.group('text')), int(m.group('data')), int(m.group('bss'))]

    # Core profile data
    line = memory_map_file.readline()
    m = pattern.match(line)
    core_profile_data = [int(m.group('text')), int(m.group('data')), int(m.group('bss'))]

    line = memory_map_file.readline()
    while line:
        m = pattern.match(line)
        profiles_data[0].append(m.group('name'))
        profiles_data[1].append(int(m.group('text')) - core_profile_data[0])
        profiles_data[2].append(int(m.group('data')) - core_profile_data[1])
        profiles_data[3].append(int(m.group('bss')) - core_profile_data[2])
        line = memory_map_file.readline()

# Close file.
memory_map_file.close()

# Export CSV complete profile.
with open('complete_profile.csv', 'w+') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_ALL)
    csv_writer.writerow(['text', 'data', 'bss']) 
    csv_writer.writerow(complete_profile_data) 
csv_file.close()

# Export CSV complete profile.
with open('core_profile.csv', 'w+') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_ALL)
    csv_writer.writerow(['text', 'data', 'bss']) 
    csv_writer.writerow(core_profile_data) 
csv_file.close()

# Export CSV profiles text.
with open('profiles_text.csv', 'w+') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_ALL)
    csv_writer.writerow(profiles_data[0]) 
    csv_writer.writerow(profiles_data[1]) 
csv_file.close()

# Export CSV profiles data.
with open('profiles_data.csv', 'w+') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_ALL)
    csv_writer.writerow(profiles_data[0]) 
    csv_writer.writerow(profiles_data[2]) 
csv_file.close()

# Export CSV profiles bss.
with open('profiles_bss.csv', 'w+') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=',', quoting=csv.QUOTE_ALL)
    csv_writer.writerow(profiles_data[0]) 
    csv_writer.writerow(profiles_data[3]) 
csv_file.close()

