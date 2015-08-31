#!/usr/bin/env python


import datetime
from copy import deepcopy
import logging
import xml.etree.ElementTree as et
import os
import pystache
import re
import string
import sys
import subprocess
import urllib2

LIBARCOMMANDS_GIT_OWNER = "Parrot-Developers"
LIBARCOMMANDS_GIT_HASH = "7e2f55fafcd45ba2380ca2574a08b7359c005f47"

# From XML types to ROS primitive types
ROS_TYPE_MAP = {
    "bool": "bool",
    "u8": "uint8",
    "i8": "int8",
    "u16": "uint16",
    "i16": "int16",
    "u32": "uint32",
    "i32": "int32",
    "u64": "uint64",
    "i64": "int64",
    "float": "float32",
    "double": "float64",
    "string": "string",
    "enum": "enum"
    }

# From XML types to BebopSDK union defined in ARCONTROLLER_Dictionary.h
BEBOP_TYPE_MAP = {
    "bool": "U8",
    "u8": "U8",
    "i8": "I8",
    "u16": "U16",
    "i16": "I16",
    "u32": "U32",
    "i32": "I32",
    "u64": "U64",
    "i64": "I64",
    "float": "Float",
    "double": "Double",
    "string": "String",
    "enum": "U8"
}

rend = pystache.Renderer()

def get_xml_url(filename):
    return rend.render_path("url.mustache", 
        {"repo_owner": LIBARCOMMANDS_GIT_OWNER, "hash": LIBARCOMMANDS_GIT_HASH, "filename": filename})

def load_from_url(url):
    f = urllib2.urlopen(url)
    data = f.read()
    f.close()
    return data

def is_state_tag(name):
    return (not name.find("State") == -1)# and (name.find("Settings") == -1) 

def strip_text(text):
    return re.sub("\s\s+", " ", text.strip().replace('\n', '').replace('\r', ''))

def cap_word(text):
    return text.lower().title()

def main():
    # Setup stuff
    logging.basicConfig(level="INFO")

    xml_url = get_xml_url("common_commands")
    project = "common"

    logging.info("Fetching source XML file for project %s " % (project, ))
    logging.info("URL: %s" % (xml_url, ))
    xml = load_from_url(xml_url)
    xml_root = et.fromstring(xml)

    # iterate all <class> tags
    logging.info("Iterating all State <class> tags ...")

    now = datetime.datetime.now()
    generator = os.path.basename(__file__)
    generator_git_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip()

    d_cpp = dict({
            "url": xml_url,
            "date": now,
            "generator": generator,
            "generator_git_hash": generator_git_hash,
            "queue_size": 10,  # 5Hz
            "frame_id": '"base_link"',
            "cpp_class": list()
        })
    d_msg = dict()

    for cl in xml_root.iter("class"):
        if not is_state_tag(cl.attrib["name"]):
            continue 

        # Iterate all cmds
        # Generate one .msg and one C++ class for each of them
        for cmd in cl.iter("cmd"):
            # .msg
            msg_name = cap_word(project) + cl.attrib["name"] + cmd.attrib["name"]
    
            d = dict({
                "url": xml_url,
                "msg_filename": msg_name,
                "date": now,
                "generator": generator,
                "generator_git_hash": generator_git_hash,
                "msg_file_comment": strip_text(cmd.text),
                "msg_field": list()
            })

            # C++ class
            cpp_class_dict_key = rend.render_path("dictionary_key.mustache",
                {"project": project.upper(), "class": cl.attrib["name"].upper(), "cmd": cmd.attrib["name"].upper()})
            # cmd.attrib["name"] and cl.attrib["name"] are already in CamelCase
            cpp_class_name = msg_name


            arg_list = []
            for arg in cmd.iter("arg"):
                # .msg
                f_name = cmd.attrib["name"] + "_" + arg.attrib["name"]
                f_type = ROS_TYPE_MAP[arg.attrib.get("type", "bool")]
                f_comment = strip_text(arg.text)
                f_enum_list = list()
                if (f_type == "enum"):
                    f_type = "uint8"
                    counter = 0
                    for enum in arg.iter("enum"):
                        f_enum_list.append({
                            "constant_name": f_name + "_" + enum.attrib["name"],
                            "constant_value": counter})
                        counter += 1
                
                d["msg_field"].append({
                    "msg_field_type": f_type,
                    "msg_field_name": f_name,
                    "msg_field_comment": f_comment,
                    "msg_field_enum": deepcopy(f_enum_list)
                })

                # C++ class
                arg_list.append({
                    "cpp_class_arg_name": cpp_class_dict_key + "_" + arg.attrib["name"].upper(),
                    "cpp_class_arg_sdk_type": BEBOP_TYPE_MAP[arg.attrib.get("type", "bool")]
                    })

            d_msg[msg_name] = deepcopy(d)

            # C++ class
            d_cpp["cpp_class"].append({
                "cpp_class_name": cpp_class_name,
                "cpp_class_msg_type": msg_name,
                "key": cpp_class_dict_key,
                "cpp_class_arg": deepcopy(arg_list)
                })
    
    logging.info("... Done iterating, writing results to file")      
    # .msg write
    for k, d in d_msg.items():
        msg_filename = k + ".msg"
        logging.info("Writing %s" % (msg_filename, ))
        with open(msg_filename, "w") as msg_file:
            msg_file.write(rend.render_path("msg.mustache", d))
    
    logging.info("Writing bebop_callbacks.h")
    with open("bebop_callbacks.h", "w") as header_file:
        header_file.write(rend.render_path("bebop_callbacks.h.mustache", d_cpp))

if __name__ == "__main__":
    main()

