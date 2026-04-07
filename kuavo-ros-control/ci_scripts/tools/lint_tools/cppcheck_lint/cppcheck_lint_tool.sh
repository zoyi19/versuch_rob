#!/bin/bash

# 读取白名单中的 package 名字，路径 和 文件（带上相对路径保证唯一性，绝对路径也可以)
whitelist_file="isolated_file_list.txt"   # 与脚本同路径
declare -a processed_whitelist_entries
script_root=$(pwd)

if [ -f "$whitelist_file" ]; then
    echo "Reading whitelist from: $script_root/$whitelist_file"
    mapfile -t raw_whitelist_entries < "$whitelist_file"

    for entry in "${raw_whitelist_entries[@]}"; do
        if [[ -z "$entry" || "$entry" == \#* ]]; then
            continue
        fi
        processed_whitelist_entries+=("$entry")
    done
    if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
        echo "Processed ${#processed_whitelist_entries[@]} whitelist keywords:"
        printf "  - %s\n" "${processed_whitelist_entries[@]}"
    else
        echo "Whitelist file '$whitelist_file' is empty or contains only comments."
    fi
else
    echo "Whitelist file not found at '$script_root/$whitelist_file'. Proceeding without whitelist."
fi
echo "" 


# 获取参与到的编译的 packages, 可以选择部分 packages
package_names_output=$(catkin build --dry-run 2>&1)

readarray -t package_names < <(echo "$package_names_output" | \
    awk '/Packages to be built:/ {p=1; next} p && /^- / {sub(/^- /, ""); sub(/\s*\[.*\]\s*$/, ""); sub(/\s*\(catkin\)\s*$/, ""); gsub(/[ \t]+$/, ""); print}' | \
    grep -v '^$')

if [ ${#package_names[@]} -eq 0 ]; then
    echo "Error: No packages found in the dry-run output. Please check the command and its output."
    exit 1
fi


# 获取 package 的源码的绝对路径列表（其 package.xml 的路径）
declare -a package_paths

echo -e "\nLocating package paths (${#package_names[@]}):"
for pkg_name in "${package_names[@]}"; do
    pkg_path=$(catkin locate -s "$pkg_name" 2>/dev/null)
    if [ -n "$pkg_path" ]; then
        package_paths+=("$pkg_path")
        echo "  - $pkg_name: $pkg_path"
    else
        echo "  - Warning: Could not locate path for package: $pkg_name"
    fi
done


# 创建输出存储的文件夹
results_dir="cppcheck_results"
mkdir -p "$results_dir"
echo -e "\nCreated directory for Cppcheck results: $(pwd)/$results_dir"


# 经过白名单的作用后的输出的检测结果
cppcheck_report_file="$results_dir/cppcheck_report.txt"
echo "Cppcheck Report - $(date)" > "$cppcheck_report_file"

for i in "${!package_names[@]}"; do
    pkg_name="${package_names[$i]}"
    pkg_path="${package_paths[$i]}"

    echo "  Processing package: $pkg_name (Source: $pkg_path)"

    # 检查 package name 是否在 白名单 
    if [[ " ${processed_whitelist_entries[@]} " =~ " ${pkg_name} " ]]; then
        echo "    Skipping package $pkg_name as it is in the processed whitelist."
        continue
    fi

    # 搜集路径下的 C++ 源码，cppcheck 不支持检查头文件
    package_all_source_files=()
    while IFS= read -r line; do
        if [ -n "$line" ]; then
            package_all_source_files+=("$(realpath -m "$line")")
        fi
    done < <(find "$pkg_path" -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.cc" \) )

    # 若白名单中的参数可以匹配到源码绝对路径中的内容就过滤
    source_files_list=()
    if [ ${#package_all_source_files[@]} -gt 0 ]; then
        echo "    Found ${#package_all_source_files[@]} potential source file(s) for $pkg_name. Filtering based on whitelist keywords..."
        for file_to_check in "${package_all_source_files[@]}"; do
            is_whitelisted=false
            if [ ${#processed_whitelist_entries[@]} -gt 0 ]; then
                for wl_entry in "${processed_whitelist_entries[@]}"; do
                    if [[ "$file_to_check" == *"$wl_entry"* ]]; then
                        is_whitelisted=true
                        echo "      Skipping (whitelisted): $file_to_check (path contains keyword: '$wl_entry')"
                        break
                    fi
                done
            fi

            if [ "$is_whitelisted" = false ]; then
                source_files_list+=("$file_to_check")
            fi
        done
        echo "    After whitelist filtering, ${#source_files_list[@]} source file(s) will be processed for $pkg_name."
    fi

    if [ ${#source_files_list[@]} -eq 0 ]; then
        echo "    No source files (.c, .cpp, .cc) found for $pkg_name in $pkg_path after applying whitelist (or none found initially). Skipping cppcheck."
        continue
    fi

    echo "    Found ${#source_files_list[@]} source file(s) for $pkg_name."

    echo -e "\n\n################################################################################" >> "$cppcheck_report_file"
    echo -e "# Checking package: $pkg_name" >> "$cppcheck_report_file"
    echo -e "# Source: $pkg_path" >> "$cppcheck_report_file"
    echo -e "################################################################################\n" >> "$cppcheck_report_file"

    if cppcheck --enable=all --quiet --suppressions-list=suppressions.txt "${source_files_list[@]}" >> "$cppcheck_report_file" 2>&1; then
        echo "    Successfully ran cppcheck for $pkg_name. Output appended to $cppcheck_report_file."
        echo -e "\n--- cppcheck for $pkg_name completed successfully (Exit Code: $?) ---\n" >> "$cppcheck_report_file"
    else
        echo "    Error or issues running cppcheck for $pkg_name. Exit code: $?"
        echo "    Output and errors (if any) for $pkg_name have been appended to $cppcheck_report_file"
        echo -e "\n--- ERROR running cppcheck for $pkg_name (Exit Code: $?) ---\n" >> "$cppcheck_report_file"
    fi
    echo ""
done

echo -e "\nCppcheck checks finished."
echo "All results have been combined into: $cppcheck_report_file"
echo ""


# 获取要选择文件的绝对路径列表
selected_file_list="selected_file_list.txt"
declare -a selected_files

if [ -f "$selected_file_list" ]; then
    echo "Reading selected file list from: $selected_file_list"
    while IFS= read -r line || [ -n "$line" ]; do
        if [[ -z "$line" || "$line" == \#* ]]; then
            continue
        fi

        pkg_name=$(echo "$line" | awk '{print $1}')
        rel_path=$(echo "$line" | awk '{print $2}')

        echo "  Processing package: $pkg_name (Source: $rel_path)"  

        pkg_root=$(catkin locate -s "$pkg_name" 2>/dev/null)
        if [ -z "$pkg_root" ]; then
            echo "  - Warning: Could not locate package: $pkg_name"
            continue
        else
            echo "  - Located package root: $pkg_root"
        fi

        # 拼接功能包路径和文件的相对路径（相对于功能包的路径）
        target_path="$pkg_root/$rel_path"

        # 检查路径是文件还是文件夹
        if [ -d "$target_path" ]; then
            echo "  - Target path is a directory. Searching for C/C++ files..."
            while IFS= read -r file; do
                selected_files+=("$file")
            done < <(find "$target_path" -type f \( -name "*.c" -o -name "*.cpp" -o -name "*.cc" \))
        elif [ -f "$target_path" ]; then
            echo "  - Target path is a file."
            if [[ "$target_path" == *.c || "$target_path" == *.cpp || "$target_path" == *.cc ]]; then
                selected_files+=("$target_path")
            else
                echo "  - Warning: $target_path is not a C/C++ source file. Skipping."
            fi
        else
            echo "  - Warning: $target_path does not exist. Skipping."
        fi
    done < "$selected_file_list"

    if [ ${#selected_files[@]} -gt 0 ]; then
        echo "Selected files for cppcheck (${#selected_files[@]}):"
        printf "  - %s\n" "${selected_files[@]}"
    else
        echo "Selected file list '$selected_file_list' is empty or contains only comments."
    fi
else
    echo "Selected file list not found at '$selected_file_list'. Proceeding with package discovery."
fi


# 检测选中的文件
selected_report_file="$results_dir/cppcheck_selected_file_report.txt"

if [ ${#selected_files[@]} -gt 0 ]; then
    echo -e "\nRunning cppcheck for selected files..."
    if cppcheck --enable=all --quiet --suppressions-list=suppressions.txt "${selected_files[@]}" > "$selected_report_file" 2>&1; then
        echo "    Successfully ran cppcheck for selected files. Output appended to $selected_report_file."
        echo -e "\n--- cppcheck for selected files completed successfully (Exit Code: $?) ---\n" >> "$selected_report_file"
    else
        echo "    Error or issues running cppcheck for selected files. Exit code: $?"
        echo "    Output and errors (if any) for selected files have been appended to $selected_report_file"
        echo -e "\n--- ERROR running cppcheck for selected files (Exit Code: $?) ---\n" >> "$selected_report_file"
    fi
else
    echo "No selected files to run cppcheck on. Skipping selected file check."
fi
