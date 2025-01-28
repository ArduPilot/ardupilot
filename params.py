#!/usr/bin/env python3


from pathlib import Path
import re

PARAM_RE = re.compile(r"\s*\/\/\s*@Param:")
GROUP_RE = re.compile(r"\s*\/\/\s*@Group:")
GPATH_RE = re.compile(r"\s*\/\/\s*@Path:")
AT_STARTS = re.compile(r"\s*\/\/\s*@")

def add_param_info(line, param_name, param_dict):
    """
    """
    elem = re.sub(AT_STARTS, "", line)
    ess = param_dict.get(param_name, {})
    [fullkey, fullvalue] = elem.split(":", maxsplit=1)
    key = fullkey.strip()
    value = fullvalue.strip()
    if key == "Range":
        [start, stop] = value.split(" ", maxsplit=1)
        ess["Range Start"] = start
        ess["Range End"] = stop
        param_dict[param_name] = ess
    else:
        ess[key] = value
        param_dict[param_name] = ess
    return param_dict

def extract_param_name(line, group):
    l0 = re.sub(AT_STARTS, "", line)
    pname = l0.split(":", maxsplit=1)[1]
    return group.strip() + pname.strip()

def get_params(file, group=""):
    """
    """
    param_dict = {}
    with open(file, "rt", buffering=1) as pfile:
        param_name = None
        for line in pfile:
            if re.search(PARAM_RE, line):
                param_name = extract_param_name(line, group)
                param_dict[param_name] = {}
            elif param_name is not None and re.search(AT_STARTS, line):
                param_dict = add_param_info(line, param_name, param_dict)
            elif param_name is not None:
                param_name = None
    return param_dict

def get_groups(file, cgrp=""):
    """
    """
    groups = []
    pmap = {}
    with open(file, "rt", buffering=1) as gfile:
        group_found = False
        gname = None
        for line in gfile:
            if re.search(GPATH_RE, line) and group_found:
                # Get the group path relative to this file.
                relative_dir = file.parents[0]
                [_, gpath] = line.split(":", maxsplit=1)
                pgen0 = (relative_dir.joinpath(p.strip()) for p in gpath.split(","))
                groups = groups + [(gname, f.resolve()) for f in pgen0]
            elif re.search(GROUP_RE, line):
                gname = cgrp + line.split(":", maxsplit=1)[1].strip()
                group_found = True
            else:
                group_found = False
    for (name, path) in groups:
        if path != file:
            pmap = {**pmap, **get_groups(path, name), **get_params(path, name)}
    return pmap

#
# Main Entry Point of script
#
if __name__ == "__main__":
    tags = ["Param", "DisplayName", "Description", "Units", "Range Start", "Range End", "Increment", "Additional"]
    map = get_groups(Path("./ArduSub/Parameters.cpp"))
    separator = "$$"
    print(separator.join(tags))
    for param, info in map.items():
        print(separator.join(
            [param] +
            [info.pop(tag, "") for tag in tags[1:-1]] +
            [separator.join(key + ":" + value for key, value in info.items() if key not in tags)]
        ))
