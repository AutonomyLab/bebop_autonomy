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

# SDK 3.12.6: https://github.com/Parrot-Developers/arsdk_manifests/blob/d7640c80ed7147971995222d9f4655932a904aa8/release.xml
LIBARCOMMANDS_GIT_OWNER = "Parrot-Developers"
LIBARCOMMANDS_GIT_HASH = "ab28dab91845cd36c4d7002b55f70805deaff3c8"

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
    "enum": "I32"
}

# From XML types to Dynamic Reconfigure Types
DYN_TYPE_MAP = {
    "bool": "bool_t",
    "u8": "int_t",
    "i8": "int_t",
    "u16": "int_t",
    "i16": "int_t",
    "u32": "int_t",
    "i32": "int_t",
    "u64": "int_t",
    "i64": "int_t",
    "float": "double_t",
    "double": "double_t",
    "string": "str_t",
    "enum": "enum"
}

C_TYPE_MAP = {
    "bool": "bool",
    "u8": "int32_t",
    "i8": "int32_t",
    "u16": "int32_t",
    "i16": "int32_t",
    "u32": "int32_t",
    "i32": "int32_t",
    "u64": "int32_t",
    "i64": "int32_t",
    "float": "double",  # for rosparam
    "double": "double",
    "string": "std::string",
    "enum": "int32_t"
}

blacklist_settings_keys = set(["wifiSecurity"])

min_max_regex = re.compile('\[([0-9\.\-]+)\:([0-9\.\-]+)\]')
rend = pystache.Renderer()

def get_xml_url(filename):
    return rend.render_path("templates/url.mustache",
        {"repo_owner": LIBARCOMMANDS_GIT_OWNER, "hash": LIBARCOMMANDS_GIT_HASH, "filename": filename})

def load_from_url(url):
    f = urllib2.urlopen(url)
    data = f.read()
    f.close()
    return data

def is_state_tag(name):
    return (not name.find("State") == -1) and (name.find("Settings") == -1)

def is_settings_tag(name):
    return (not name.find("Settings") == -1) and (name.find("State") == -1)

def strip_text(text):
    return re.sub("\s\s+", " ", text.strip().replace('\n', '').replace('\r', '')).replace('"', '').replace("'", "")

def cap_word(text):
    return text.lower().title()

def guess_min_max(arg_comment):
    m = min_max_regex.search(arg_comment)
    if m:
        logging.info("  ... [min:max]")
        return [float(m.group(1)), float(m.group(2))]
    elif (arg_comment.lower().find("m/s2") != -1):
        logging.info("  ... acc (m/s2)")
        return [0.0, 5.0]
    elif (arg_comment.lower().find("m/s") != -1):
        logging.info("  ... speed (m/s)")
        return [0.0, 10.0]
    elif (arg_comment.lower().find("in meters") != -1) or (arg_comment.lower().find("in m") != -1):
        logging.info("  ... meters")
        return [0, 160]
    elif (arg_comment.lower().find("in degree/s") != -1):
        logging.info("  ... rotations speed degrees/s")
        return [0, 900.0]
    elif (arg_comment.lower().find("in degree") != -1):
        logging.info("  ... degrees")
        return [-180.0, 180.0]
    elif (arg_comment.lower().find("1") != -1) and (arg_comment.lower().find("0") != -1):
        logging.info("  ... bool")
        return [0, 1]
    elif (arg_comment.lower().find("latitude") != -1):
        logging.info("  ... latitude")
        return [-90.0, 90.0]
    elif (arg_comment.lower().find("longitude") != -1):
        logging.info("  ... longitude")
        return [-180.0, 180.0]
    elif (arg_comment.lower().find("[rad/s]") != -1):
        logging.info("  ... angular speed (rad/s)")
        return [0.0, 5.0]
    elif (arg_comment.lower().find("channel") != -1):
        logging.info("  ... unknown int")
        return [0, 50]
    elif (arg_comment.lower().find("second") != -1):
        logging.info("  ... time (s)")
        return [0, 120]

    return []

def today():
    return datetime.datetime.now().strftime("%Y-%m-%d")

def generate_states(xml_filename):
    xml_url = get_xml_url(xml_filename)
    project = xml_filename.split(".")[0]

    logging.info("XML Filename: %s" % (xml_filename, ))
    logging.info("Fetching source XML file for project %s " % (project, ))
    logging.info("URL: %s" % (xml_url, ))
    xml = load_from_url(xml_url)
    xml_root = et.fromstring(xml)

    # iterate all <class> tags
    logging.info("Iterating all State <class> tags ...")

    generator = os.path.basename(__file__)
    generator_git_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip()

    d_cpp = dict({
            "url": xml_url,
            "project": project,
            "date": today(),
            "generator": generator,
            "generator_git_hash": generator_git_hash,
            "queue_size": 10,  # 5Hz
            "frame_id": "base_link",
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

            comment_el = cmd.find("comment")
            msg_file_comment = ""
            if not comment_el is None:
                msg_file_comment = comment_el.attrib["desc"]

            d = dict({
                "url": xml_url,
                "msg_filename": msg_name,
                "date": today(),
                "generator": generator,
                "generator_git_hash": generator_git_hash,
                "msg_file_comment": strip_text(msg_file_comment),
                "msg_field": list()
            })

            # C++ class
            cpp_class_dict_key = rend.render_path("templates/dictionary_key.mustache",
                {"project": project.upper(), "class": cl.attrib["name"].upper(), "cmd": cmd.attrib["name"].upper()})
            # cmd.attrib["name"] and cl.attrib["name"] are already in CamelCase
            cpp_class_name = msg_name
            cpp_class_instance_name = project.lower() + "_" + cl.attrib["name"].lower() + "_" + cmd.attrib["name"].lower() + "_ptr";
            cpp_class_param_name = "states/enable_" + cl.attrib["name"].lower() + "_" + cmd.attrib["name"].lower()
            topic_name = "states/" + project + "/" + cl.attrib["name"] + "/" + cmd.attrib["name"]

            arg_list = []
            for arg in cmd.iter("arg"):
                # .msg
                f_name = arg.attrib["name"]
                f_type = ROS_TYPE_MAP[arg.attrib.get("type", "bool")]
                f_comment = strip_text(arg.text)
                f_enum_list = list()
                if (f_type == "enum"):
                    f_type = "uint8"
                    counter = 0
                    for enum in arg.iter("enum"):
                        f_enum_list.append({
                            "constant_name": f_name + "_" + enum.attrib["name"],
                            "constant_value": counter,
                            "constant_comment": strip_text(enum.text)
                            })
                        counter += 1

                d["msg_field"].append({
                    "msg_field_type": f_type,
                    "msg_field_name": f_name,
                    "msg_field_comment": f_comment,
                    "msg_field_enum": deepcopy(f_enum_list)
                })

                # C++ class
                arg_list.append({
                    "cpp_class_arg_key": cpp_class_dict_key + "_" + arg.attrib["name"].upper(),
                    "cpp_class_arg_name": f_name,
                    "cpp_class_arg_sdk_type": BEBOP_TYPE_MAP[arg.attrib.get("type", "bool")]
                    })

            d_msg[msg_name] = deepcopy(d)

            # C++ class
            d_cpp["cpp_class"].append({
                "cpp_class_name": cpp_class_name,
                "cpp_class_comment": strip_text(msg_file_comment),
                "cpp_class_instance_name": cpp_class_instance_name,
                "cpp_class_param_name": cpp_class_param_name,
                "topic_name": topic_name,
                "latched": "true",
                "cpp_class_msg_type": msg_name,
                "key": cpp_class_dict_key,
                "cpp_class_arg": deepcopy(arg_list)
                })

    logging.info("... Done iterating, writing results to file")
    # .msg write
    for k, d in d_msg.items():
        msg_filename = "%s.msg" % k
        logging.info("Writing %s" % (msg_filename, ))
        with open(msg_filename, "w") as msg_file:
            msg_file.write(rend.render_path("templates/msg.mustache", d))

    header_file_name = "%s_state_callbacks.h" % (project.lower(), )
    logging.info("Writing %s" % (header_file_name, ))
    with open(header_file_name, "w") as header_file:
        header_file.write(rend.render_path("templates/state_callbacks.h.mustache", d_cpp))

    include_file_name = "%s_state_callback_includes.h" % (project.lower(), )
    logging.info("Writing %s" % (include_file_name, ))
    with open(include_file_name, "w") as include_file:
        include_file.write(rend.render_path("templates/state_callback_includes.h.mustache", d_cpp))

    with open("callbacks_common.h", "w") as header_file:
        header_file.write(rend.render_path("templates/callbacks_common.h.mustache", d_cpp))

    rst_file_name = "%s_states_param_topic.rst" % (project.lower(), )
    logging.info("Writing %s" % (rst_file_name, ))
    with open(rst_file_name, "w") as rst_file:
        rst_file.write(rend.render_path("templates/states_param_topic.rst.mustache", d_cpp))

def generate_settings(xml_filename):
    xml_url = get_xml_url(xml_filename)
    project = xml_filename.split(".")[0]

    logging.info("Fetching source XML file for project %s " % (project, ))
    logging.info("URL: %s" % (xml_url, ))
    xml = load_from_url(xml_url)
    xml_root = et.fromstring(xml)

    generator = os.path.basename(__file__)
    generator_git_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip()

    # make sure that the name of the config file matches the third argument
    # of gen.generate()
    d_cfg = dict({
            "cfg_filename": "Bebop%s.cfg" % (project.title(), ),
            "url": xml_url,
            "project": project.title(),
            "date": today(),
            "generator": generator,
            "generator_git_hash": generator_git_hash,
            "cfg_class": list(),
            "cpp_class": list()
        })

    for cl in xml_root.iter("class"):
        if not is_settings_tag(cl.attrib["name"]):
            continue

        # At the moment the XML file is not 100% consistent between Settings and SettingsChanged and inner Commands
        # 1. Check if `class["name"]State` exists
        if not xml_root.findall(".//class[@name='%s']" % (cl.attrib["name"] + "State", )):
            logging.warning("No State Class for %s " % (cl.attrib["name"], ))
            continue

        # Iterate all cmds
        # generate one C++ class for each command
        cfg_class_d = {
            "cfg_class_name": cl.attrib["name"].lower(),
            "cfg_class_comment": strip_text(cl.text),
            "cfg_cmd": list()
        }
        for cmd in cl.iter("cmd"):
            # 2. Check if `cmd["name"]Changed` exists
            if not xml_root.findall(".//cmd[@name='%s']" % (cmd.attrib["name"] + "Changed", )):
                logging.warning("No Changed CMD for %s " % (cmd.attrib["name"], ))
                continue

            # blacklist
            if strip_text(cmd.attrib["name"]) in blacklist_settings_keys:
                logging.warning("Key %s is blacklisted!" % (cmd.attrib["name"], ))
                continue

            comment_el = cmd.find("comment")
            cmd_comment = ""
            if not comment_el is None:
                cmd_comment = comment_el.attrib["desc"]


            # .cfg
            cfg_cmd_d = {
                "cfg_cmd_comment": strip_text(cmd_comment),
                "cfg_arg": list()
            }

            # C++
            # We are iterating classes with names ending in "Setting". For each of these classes
            # there exists a corresponding class with the same name + "State" (e.g PilotingSetting and PilottingSettingState)
            # The inner commands of the corresponding class are also follow a similar conention, they end in "CHANGED".
            # We create cfg files based on Settings, and ROS param updates based on SettingsChanged
            cpp_class_dict_key = rend.render_path("templates/dictionary_key.mustache",
                {"project": project.upper(), "class": cl.attrib["name"].upper() + "STATE", "cmd": cmd.attrib["name"].upper() + "CHANGED"} )
            # cmd.attrib["name"] and cl.attrib["name"] are already in CamelCase
            cpp_class_name = cl.attrib["name"] + cmd.attrib["name"]
            cpp_class_comment = strip_text(cmd_comment)
            cpp_class_instance_name = project.lower() + "_" + cl.attrib["name"].lower() + "_" + cmd.attrib["name"].lower() + "_ptr";
            cpp_class_params = list()

            counter = 0
            # generate one dyamic reconfigure variable per arg
            for arg in cmd.iter("arg"):
                # .cfg
                arg_name = cl.attrib["name"] + cmd.attrib["name"] + cap_word(arg.attrib["name"])
                arg_type = DYN_TYPE_MAP[arg.attrib.get("type", "bool")]
                arg_comment = strip_text(arg.text)

                arg_enum_list = list()
                minmax_list = list()
                arg_default = 0
                arg_min = 0.0
                arg_max = 0.0
                counter = 0
                need_enum_cast = False
                if (arg_type == "enum"):
                    need_enum_cast = True
                    arg_type = "int_t"
                    for enum in arg.iter("enum"):
                        arg_enum_list.append({
                            "constant_name": arg_name + "_" + enum.attrib["name"],
                            "constant_value": counter,
                            "constant_comment": strip_text(enum.text)
                            })
                        counter += 1
                elif not arg_type == "str_t":
                    # No min/max values defined in XML, guessing the type and propose a value:
                    logging.info("Guessing type of \"%s\"" % (arg_name))
                    logging.info("  from: %s" % (arg_comment))
                    minmax_list = guess_min_max(arg_comment)
                    if (len(minmax_list) == 2):
                        [arg_min, arg_max] = minmax_list
                        logging.info("  min: %s max: %s" % (arg_min, arg_max))
                    else:
                        logging.warning("  Can not guess [min:max] values for this arg, skipping it")

                    # We create a fake enum for int_t types that only accept bool values
                    # The source XML should have defined them as bool_t
                    # Since these are fake enums (no defines in SDK), we don't need int->enum casting
                    if arg_type == "int_t" and arg_min == 0 and arg_max == 1:
                        arg_enum_list.append({
                            "constant_name": arg_name + "_OFF",
                            "constant_value": 0,
                            "constant_comment": "Disabled"
                            })
                        arg_enum_list.append({
                            "constant_name": arg_name + "_ON",
                            "constant_value": 1,
                            "constant_comment": "Enabled"
                            })
                        counter = 2

                # either we found minmax or the arg is of type enum
                if len(minmax_list) or need_enum_cast or arg_type == "str_t":
                    # hack
                    if arg_type == "str_t":
                        arg_min = "''"
                        arg_max = "''"
                        arg_default = "''"

                    cfg_cmd_d["cfg_arg"].append({
                        "cfg_arg_type": arg_type,
                        "cfg_arg_name": arg_name,
                        "cfg_arg_comment": arg_comment,
                        "cfg_arg_default": arg_default,
                        "cfg_arg_min": arg_min,
                        "cfg_arg_max": arg_max,
                        # Render once trick: http://stackoverflow.com/a/10118092
                        "cfg_arg_enum": {'items' : deepcopy(arg_enum_list)} if len(arg_enum_list) else [],
                        "enum_max": counter - 1
                        })

                    # generate c enum type
                    if (need_enum_cast):
                        enum_cast = "static_cast<eARCOMMANDS_%s_%s_%s_%s>" % (project.upper(), cl.attrib["name"].upper(), cmd.attrib["name"].upper(), arg.attrib["name"].upper())
                    else:
                        enum_cast = ""

                    cpp_class_params.append({
                        "cpp_class_arg_key": cpp_class_dict_key + "_" + arg.attrib["name"].upper(),
                        "cpp_class_param_name": arg_name,
                        "cpp_class_comment": cpp_class_comment,
                        "cpp_class_param_enum_cast": enum_cast,
                        "cpp_class_param_type": C_TYPE_MAP[arg.attrib.get("type", "bool")],
                        "cpp_class_param_sdk_type": BEBOP_TYPE_MAP[arg.attrib.get("type", "bool")]
                        })

            # Skip cmds with no arguments
            if len(cfg_cmd_d["cfg_arg"]):
                cfg_class_d["cfg_cmd"].append(deepcopy(cfg_cmd_d))
                d_cfg["cpp_class"].append({
                    "cpp_class_dict_key": cpp_class_dict_key,
                    "cpp_class_name": cpp_class_name,
                    "cpp_class_instance_name": cpp_class_instance_name,
                    "cpp_class_params": deepcopy(cpp_class_params)
                    })

        d_cfg["cfg_class"].append(deepcopy(cfg_class_d))


    logging.info("... Done iterating, writing results to file")

    # .cfg write
    cfg_file_name = d_cfg["cfg_filename"]
    logging.info("Writing %s" % (cfg_file_name, ))
    with open(cfg_file_name, "w") as cfg_file:
        cfg_file.write(rend.render_path("templates/cfg.mustache", d_cfg))

    header_file_name = "%s_setting_callbacks.h" % (project.lower(), )
    logging.info("Writing %s" % (header_file_name, ))
    with open(header_file_name, "w") as header_file:
        header_file.write(rend.render_path("templates/setting_callbacks.h.mustache", d_cfg))

    include_file_name = "%s_setting_callback_includes.h" % (project.lower(), )
    logging.info("Writing %s" % (include_file_name, ))
    with open(include_file_name, "w") as include_file:
        include_file.write(rend.render_path("templates/setting_callback_includes.h.mustache", d_cfg))

    rst_file_name = "%s_settings_param.rst" % (project.lower(), )
    logging.info("Writing %s" % (rst_file_name, ))
    with open(rst_file_name, "w") as rst_file:
        rst_file.write(rend.render_path("templates/settings_param.rst.mustache", d_cfg))

def main():
    # Setup stuff
    logging.basicConfig(level="INFO")

    generate_states("common.xml")
    generate_states("ardrone3.xml")
    #generate_settings("common_commands.xml")
    generate_settings("ardrone3.xml")

    generator = os.path.basename(__file__)
    generator_git_hash = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).strip()
    with open("last_build_info", "w") as last_build_file:
        last_build_file.write(rend.render_path(
            "templates/last_build_info.mustache",
            {
                "source_hash": LIBARCOMMANDS_GIT_HASH,
                "date": datetime.datetime.now(),
                "generator": generator,
                "generator_git_hash": generator_git_hash
            }))

if __name__ == "__main__":
    main()

