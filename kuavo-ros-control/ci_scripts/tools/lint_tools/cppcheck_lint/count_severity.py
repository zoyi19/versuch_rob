import xml.etree.ElementTree as ET
from collections import Counter
import argparse
import sys
import os

def _build_severity_counter(severities_list):
    """Helper function to create a Counter from a list of severities."""
    # 统计每种 severity 的数量
    return Counter(severities_list)

def extract_severity_counts_from_xml(xml_path):
    """
    Parses a Cppcheck XML report and counts the occurrences of each severity type.

    Args:
        xml_path (str): The path to the Cppcheck XML report file.

    Returns:
        collections.Counter: A Counter object mapping severity strings to their counts.
                             Returns an empty Counter if the file cannot be parsed.
        bool: True if parsing was successful, False otherwise.
    """
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        severities = [error.get('severity') for error in root.findall('.//error')]
        severity_counter = _build_severity_counter(severities)
        for severity, count in severity_counter.items():
            print(f"{severity}: {count}")
        return severity_counter, True
    except ET.ParseError as e:
        print(f"Error parsing XML file '{xml_path}': {e}")
        return Counter(), False # Return an empty counter and False on error
    except FileNotFoundError:
        print(f"Error: XML file not found at '{xml_path}'")
        return Counter(), False # Return an empty counter and False on error
    except Exception as e:
        print(f"An unexpected error occurred while processing XML file '{xml_path}': {e}")
        return Counter(), False # Return an empty counter and False on other errors

# 设置命令行参数解析
parser = argparse.ArgumentParser(description='Count severity types in cppcheck XML report.')
parser.add_argument('xml_file_path', type=str, help='Path to the cppcheck XML report file')
parser.add_argument('baseline_file_path', type=str, help='Path to the baseline file (e.g., baseline.txt)')
args = parser.parse_args()

# 使用命令行参数中的路径
xml_file_path = args.xml_file_path

# 提取 severity 数量
severity_count, xml_parse_success = extract_severity_counts_from_xml(xml_file_path)

if not xml_parse_success:
    abs_xml_file_path = os.path.abspath(xml_file_path)
    print(f"\nOverall Script Status: FAILED due to XML processing error for file: {abs_xml_file_path}")
    sys.exit(1)

def compare_with_baseline(baseline_file_path_arg, current_severity_counts):
    script_should_fail = False
    RED = '\033[91m'
    RESET = '\033[0m'
    # Parse the baseline file
    baseline_data = {}
    try:
        with open(baseline_file_path_arg, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'): # Skip empty lines and comments in baseline file
                    continue
                if ':' in line:
                    parts = line.split(':', 1)
                    severity = parts[0].strip()
                    try:
                        count = int(parts[1].strip())
                        baseline_data[severity] = count
                    except ValueError:
                        print(f"Warning: Could not parse count for severity '{severity}' in baseline file. Line: '{line}'")
                else:
                    print(f"Warning: Malformed line in baseline file (missing ':'): '{line}'")
    except FileNotFoundError:
        print(f"Error: Baseline file not found at '{baseline_file_path_arg}'")
        baseline_data = None # Indicate baseline loading failed
        script_should_fail = True
    except Exception as e:
        print(f"Error reading or parsing baseline file '{baseline_file_path_arg}': {e}")
        baseline_data = None
        script_should_fail = True

    if baseline_data is not None:
        print("\n--- Baseline Comparison ---")
        # Iterate through the severities defined in the baseline
        for severity_key, baseline_count_val in baseline_data.items():
            # Get the current count for this severity from the report.
            # If the severity (from baseline) is not in the current report (current_severity_counts),
            # its count is considered 100000 for comparison purposes as per instruction.
            current_report_count_val = current_severity_counts.get(severity_key, 100000)
            
            print(f"Severity: {severity_key}")
            print(f"  Baseline: {baseline_count_val}")
            print(f"  Current (effective for comparison): {current_report_count_val}")

            if current_report_count_val > baseline_count_val:
                print(f"{RED}  STATUS: WARNING - Current count ({current_report_count_val}) EXCEEDS baseline ({baseline_count_val}) for '{severity_key}'!{RESET}")
                script_should_fail = True
            elif current_report_count_val < baseline_count_val:
                print(f"  STATUS: OK - Current count ({current_report_count_val}) is BELOW baseline ({baseline_count_val}) for '{severity_key}'.")
            else: # current_report_count_val == baseline_count_val
                print(f"  STATUS: OK - Current count ({current_report_count_val}) MATCHES baseline ({baseline_count_val}) for '{severity_key}'.")

        # Check for severities in current report but not in baseline
        print("\n--- Severities in Current Report Not Covered by Baseline ---")
        found_new_severities_not_in_baseline = False
        for report_severity_key, report_severity_count in current_severity_counts.items():
            if report_severity_key not in baseline_data:
                print(f"New/Unbaselined Severity: {report_severity_key}: {report_severity_count}")
                found_new_severities_not_in_baseline = True
        
        if not found_new_severities_not_in_baseline:
            print("All severities in the current report are covered by the baseline.")
    else:
        print("\nSkipping baseline comparison due to issues loading or parsing the baseline file.")
    
    return script_should_fail

script_failed_baseline_check = compare_with_baseline(args.baseline_file_path, severity_count)

if script_failed_baseline_check:
    # Detailed messages are already printed by compare_with_baseline
    print("\nOverall Script Status: FAILED.")
    sys.exit(1)
else:
    # Detailed messages are already printed by compare_with_baseline
    print("\nOverall Script Status: PASSED.")
    # sys.exit(0) is implicit if this point is reached
