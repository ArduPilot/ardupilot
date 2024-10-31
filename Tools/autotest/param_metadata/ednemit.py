"""
 Emits parameters as an EDN file, does some small remapping of names
"""

from emit import Emit
import edn_format
import datetime
import pytz
import subprocess


class EDNEmit(Emit):
    def __init__(self, *args, **kwargs):
        Emit.__init__(self, *args, **kwargs)
        self.output = "{:date " + edn_format.dumps(datetime.datetime.now(pytz.utc)) + " "
        git = subprocess.Popen(["git log --pretty=format:'%h'  -n 1"], shell=True, stdout=subprocess.PIPE).communicate()[0]
        self.output += ":git-hash \"" + git.decode("ascii") + "\" "
        self.remove_keys = ["real_path"]
        self.explict_remap = [["displayname", "display-name"]]
        self.vehicle_name = None

    def close(self):
        if self.vehicle_name is not None:
            self.output += ":vehicle \"" + self.vehicle_name + "\" "
        else:
            raise Exception('Vehicle name never found')
        self.output += "}"
        f = open("parameters.edn", mode='w')
        f.write(self.output)
        f.close()

    def start_libraries(self):
        pass

    def emit(self, g):
        for param in g.params:
            output_dict = dict()
            # lowercase all keywords
            for key in param.__dict__.keys():
                output_dict[key.lower()] = param.__dict__[key]

            # strip off any leading sillyness on the param name
            split_name = param.__dict__["name"].split(":")
            if len(split_name) == 2:
                self.vehicle_name = split_name[0]
            name = param.__dict__["name"].split(":")[-1]
            output_dict["name"] = name

            # remove any keys we don't really care to share
            for key in self.remove_keys:
                output_dict.pop(key, None)
            for key in list(output_dict.keys()):
                if not self.should_emit_field(param, key):
                    output_dict.pop(key, None)

            # rearrange bitmasks to be a vector with nil's if the bit doesn't have meaning
            if "bitmask" in output_dict:
                highest_set_bit = 0
                bits = []
                for bit in output_dict["bitmask"].split(","):
                    bit_parts = bit.split(":")
                    bit_number = int(bit_parts[0])
                    bit_parts[0] = bit_number
                    bits.append(bit_parts)
                    if bit_number > highest_set_bit:
                        highest_set_bit = bit_number
                output_bits = (highest_set_bit+1)*[None]
                for bit in bits:
                    output_bits[bit[0]] = bit[1]
                output_dict["bitmask"] = output_bits

            # rearrange values into a float indexed map
            if "values" in output_dict:
                values = dict()
                for value in output_dict["values"].split(","):
                    index, description = value.split(":")
                    values[float(index)] = description
                output_dict["values"] = values

            # remap range to be a map of floats
            if "range" in output_dict:
                low, high = output_dict["range"].split()
                output_dict["range"] = {"low": float(low), "high": float(high)}

            # remap the string to a float
            if "increment" in output_dict:
                output_dict["increment"] = float(output_dict["increment"])

            # do any name changing desired
            for remap in self.explict_remap:
                output_dict[remap[1]] = output_dict.pop(remap[0])

            self.output += "\"" + name + "\" " + edn_format.dumps(output_dict, keyword_keys=True)
