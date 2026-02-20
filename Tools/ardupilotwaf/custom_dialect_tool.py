import os

# All Waf tools must have a 'configure' function that Waf can call.
def configure(conf):
    """
    This function is automatically run by Waf when 'cfg.load("custom_dialect_tool")'
    is called in the main wscript.
    """
    # Use 'conf' which is the configuration context passed into this tool.
    conf.to_log("Running custom_dialect_tool to manage MAVLink dialect...")
    
    # Dynamically get the absolute path to the ardupilot root directory
    ardupilot_root = conf.srcnode.abspath()
    
    # Construct the full path to your custom dialect file
    custom_dialect_path = os.path.join(ardupilot_root, 'libraries/AP_CustomMavlinkHandler/custom_mavlink.xml')
    
    # The path to the all.xml file within the ArduPilot source tree
    all_xml_path = os.path.join(ardupilot_root, 'modules/mavlink/message_definitions/v1.0/all.xml')

    try:
        with open(all_xml_path, 'r') as f:
            original_lines = f.readlines()
    except FileNotFoundError:
        conf.fatal(f"Could not find {all_xml_path}. Make sure submodules are updated.")
    except Exception as e:
        conf.fatal(f"An error occurred while reading {all_xml_path}: {e}")

    # Check if the custom line currently exists
    line_exists = any(custom_dialect_path in line for line in original_lines)
    needs_write = False
    
    if conf.options.enable_custom_storage:
        # Flag is ON: We want the line to EXIST
        if not line_exists:
            insert_index = -1
            for i, line in enumerate(original_lines):
                if '<messages/>' in line:
                    insert_index = i
                    break
            
            if insert_index == -1:
                conf.fatal(f"Could not find insertion point (<messages/>) in {all_xml_path}")
            
            include_line_to_add = f'  <include>{custom_dialect_path}</include>\n'
            original_lines.insert(insert_index, include_line_to_add)
            needs_write = True
            conf.to_log(f"-> Adding custom dialect to {all_xml_path}")
        print('Custom Storage Setting                         : Enabled')
    
    else:
        # Flag is OFF: We want the line to be REMOVED
        if line_exists:
            original_lines = [line for line in original_lines if custom_dialect_path not in line]
            needs_write = True
            conf.to_log(f"-> Removing custom dialect from {all_xml_path}")
        print('Custom Storage Setting                         : Disabled')

    # If a change was made, write the new content back to the file
    if needs_write:
        try:
            with open(all_xml_path, 'w') as f:
                f.writelines(original_lines)
        except Exception as e:
            conf.fatal(f"An error occurred while writing to {all_xml_path}: {e}")
    else:
        conf.to_log(f"-> Custom dialect in {all_xml_path} is already in the correct state.")