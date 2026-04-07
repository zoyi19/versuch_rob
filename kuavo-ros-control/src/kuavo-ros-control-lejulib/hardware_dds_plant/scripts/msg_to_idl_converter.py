#!/usr/bin/env python3
"""
ROS Message to Pure DDS IDL Converter
Converts ROS .msg files to clean DDS IDL without any ROS dependencies
"""

import os
from typing import List, Tuple

class MsgToIdlConverter:
    def __init__(self):
        # Mapping from ROS types to DDS IDL types
        self.type_mapping = {
            # Basic types
            'bool': 'boolean',
            'int8': 'int8',
            'uint8': 'uint8', 
            'int16': 'int16',
            'uint16': 'uint16',
            'int32': 'int32',
            'uint32': 'uint32',
            'int64': 'int64',
            'uint64': 'uint64',
            'float32': 'float',
            'float64': 'double',
            'string': 'string',
            'time': 'int64',  # Convert to timestamp
            'duration': 'int64',  # Convert to nanoseconds
            # Arrays
            'byte': 'uint8',
            'char': 'char',
        }
        
        self.generated_structs = set()
    
    def parse_msg_file(self, filepath: str) -> Tuple[str, List[Tuple[str, str, str, str]]]:
        """Parse a .msg file and return module name and fields"""
        with open(filepath, 'r') as f:
            content = f.read()
        
        # Get module and struct name from filepath
        filename = os.path.basename(filepath)
        struct_name = filename.replace('.msg', '')
        
        fields = []
        for line in content.split('\n'):
            line = line.strip()
            if not line or line.startswith('#'):
                continue
                
            # Parse field: type field_name [= default_value]
            parts = line.split()
            if len(parts) >= 2:
                field_type = parts[0]
                field_name = parts[1]
                original_type = field_type  # Store original type
                
                # Handle arrays
                array_size = ""
                if '[' in field_type and ']' in field_type:
                    field_type, array_part = field_type.split('[', 1)
                    array_size = '[' + array_part
                    original_type = field_type  # Update original type without array
                
                idl_type = self.convert_type(field_type)
                if idl_type is not None:  # Skip fields that return None (like Header)
                    fields.append((idl_type, field_name, array_size, original_type))
        
        return struct_name, fields
    
    def convert_type(self, ros_type: str) -> str:
        """Convert ROS type to DDS IDL type"""
        # Remove any package prefix (e.g., geometry_msgs/Point -> Point)
        if '/' in ros_type:
            ros_type = ros_type.split('/')[-1]
        
        # Check if it's a basic type
        if ros_type in self.type_mapping:
            return self.type_mapping[ros_type]
        
        # Handle common ROS types that should be converted to basic types
        if ros_type == 'Header':
            return 'Header'  # Handle Header specially with stamp fields
        elif ros_type == 'Time':
            return 'int64'  # Convert Time to timestamp in nanoseconds
        elif ros_type == 'Duration':
            return 'int64'  # Convert Duration to nanoseconds
        elif ros_type == 'Empty':
            return None  # Skip empty messages
        # Handle geometry_msgs types
        elif ros_type == 'Point':
            return None  # Will be expanded inline if needed
        elif ros_type == 'Vector3':
            return 'double'  # Convert Vector3 to double with [3] suffix
        elif ros_type == 'Quaternion':
            return 'double'  # Convert Quaternion to double with [4] suffix
        elif ros_type == 'Pose':
            return None  # Will be expanded inline if needed
        elif ros_type == 'Twist':
            return None  # Will be expanded inline if needed
        
        # For custom types, assume they're structs
        return ros_type
    
    def generate_idl(self, struct_name: str, fields: List[Tuple[str, str, str, str]], 
                     module_name: str = None, output_dir: str = None) -> str:
        """Generate IDL content for a struct"""
        if not module_name:
            module_name = "leju"
        
        idl_content = []
        
        # Find custom types that need includes
        custom_types = set()
        for field_type, _, _, _ in fields:
            if (field_type not in self.type_mapping.values() and 
                field_type not in ['boolean', 'int8', 'uint8', 'int16', 'uint16', 
                                 'int32', 'uint32', 'int64', 'uint64', 'float', 
                                 'double', 'string', 'char', 'Header']):
                custom_types.add(field_type)
        
        # Add includes only for types that exist as IDL files in the same directory
        existing_includes = []
        if output_dir:
            for custom_type in sorted(custom_types):
                idl_file_path = os.path.join(output_dir, f"{custom_type}.idl")
                if os.path.exists(idl_file_path):
                    existing_includes.append(custom_type)
        
        for custom_type in existing_includes:
            idl_content.append(f"#include \"{custom_type}.idl\"")
        
        if existing_includes:
            idl_content.append("")
        
        # Add nested module declaration
        idl_content.append(f"module {module_name} {{")
        idl_content.append("  module msgs {")
        
        # Add struct definition
        idl_content.append(f"    struct {struct_name} {{")
        
        for field_type, field_name, array_size, original_type in fields:
            if array_size:
                # Handle arrays
                if array_size == '[]':
                    # Dynamic array
                    idl_content.append(f"      sequence<{field_type}> {field_name};")
                else:
                    # Fixed array
                    size = array_size.strip('[]')
                    if size.isdigit():
                        idl_content.append(f"      {field_type} {field_name}[{size}];")
                    else:
                        # Dynamic array with max size
                        idl_content.append(f"      sequence<{field_type}, {size}> {field_name};")
            else:
                # Check if this is a special geometry type that needs array suffix
                orig_type_no_package = original_type.split('/')[-1] if '/' in original_type else original_type
                if orig_type_no_package == 'Vector3' and field_type == 'double':
                    idl_content.append(f"      {field_type} {field_name}[3];")
                elif orig_type_no_package == 'Quaternion' and field_type == 'double':
                    idl_content.append(f"      {field_type} {field_name}[4];")
                elif orig_type_no_package == 'Header' and field_type == 'Header':
                    # Expand Header to stamp fields only
                    idl_content.append(f"      int32 {field_name}_sec;")
                    idl_content.append(f"      uint32 {field_name}_nanosec;")
                else:
                    idl_content.append(f"      {field_type} {field_name};")
        
        idl_content.append("    };")
        idl_content.append("  };")
        idl_content.append("};")
        idl_content.append("")  # Add blank line at the end
        
        return '\n'.join(idl_content)
    
    def convert_file(self, msg_filepath: str, output_dir: str = None, module_name: str = None) -> str:
        """Convert a single .msg file to .idl"""
        if not os.path.exists(msg_filepath):
            raise FileNotFoundError(f"Message file not found: {msg_filepath}")
        
        struct_name, fields = self.parse_msg_file(msg_filepath)
        idl_content = self.generate_idl(struct_name, fields, module_name, output_dir)
        
        # Determine output path
        if output_dir is None:
            output_dir = os.path.dirname(msg_filepath)
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        output_filepath = os.path.join(output_dir, f"{struct_name}.idl")
        
        # Write IDL file
        with open(output_filepath, 'w') as f:
            f.write(idl_content)
        
        print(f"Generated: {output_filepath}")
        return output_filepath
    
    def convert_multiple(self, msg_files: List[str], output_dir: str = None, module_name: str = None):
        """Convert multiple .msg files"""
        for msg_file in msg_files:
            try:
                self.convert_file(msg_file, output_dir, module_name)
            except Exception as e:
                print(f"Error converting {msg_file}: {e}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Convert ROS .msg files to DDS IDL format')
    
    # Input options
    parser.add_argument('msg_files', nargs='*', help='Individual .msg files to convert (or use --msg-dir)')
    parser.add_argument('--msg-dir', type=str, help='Directory containing .msg files')
    
    # Output options
    parser.add_argument('--output-dir', type=str, help='Output directory for .idl files')
    parser.add_argument('--module', type=str, default='leju', help='Module name (default: leju)')
    
    args = parser.parse_args()
    
    converter = MsgToIdlConverter()
    
    # Determine input files
    msg_files = []
    if args.msg_dir:
        # Process all .msg files in directory
        if not os.path.exists(args.msg_dir):
            print(f"Error: Directory {args.msg_dir} does not exist!")
            return
        
        for filename in os.listdir(args.msg_dir):
            if filename.endswith('.msg'):
                msg_files.append(os.path.join(args.msg_dir, filename))
        
        if not msg_files:
            print(f"No .msg files found in directory: {args.msg_dir}")
            return
    else:
        # Use individual files
        msg_files = args.msg_files
        if not msg_files:
            print("Error: No .msg files specified! Use individual files or --msg-dir")
            return
    
    # Convert files
    converter.convert_multiple(msg_files, args.output_dir, args.module)

if __name__ == "__main__":
    main()