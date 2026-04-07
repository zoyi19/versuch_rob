import os
import re
import shutil
import sys


def copy_file(src, dest):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    shutil.copy2(src, dest)
    # print(f"Copied {src} to {dest}")


def find_local_images(md_content, md_dir):
    md_image_links = re.findall(r"!\[.*?\]\((.*?)\)", md_content)
    
    html_image_links = re.findall(r'<img[^>]*?src=[\'"](.*?)[\'"][^>]*?>', md_content)
    
    md_links = re.findall(r"\[.*?\]\((.*?(?:\.png|\.jpg|\.jpeg|\.gif|\.bmp|\.svg))\)", md_content)
    
    image_links = md_image_links + html_image_links + md_links
    
    local_images = []
    for link in image_links:
        link = link.split('#')[0].split('?')[0]
        if not link.startswith(("http://", "https://", "data:", "//")):
            full_path = os.path.abspath(os.path.join(md_dir, link))
            if os.path.exists(full_path):
                local_images.append(full_path)
            else:
                print(f"Warning: Image not found: {full_path}")
    
    return local_images


def should_exclude(file_path, exclude_list):
    file_path = os.path.abspath(file_path)
    file_dir, file_name = os.path.split(file_path)

    for exclude_item in exclude_list:
        if exclude_item.startswith("./"):
            exclude_abs_path = os.path.abspath(exclude_item)
        else:
            exclude_abs_path = os.path.abspath(exclude_item)

        if os.path.isfile(exclude_abs_path):
            if file_path == exclude_abs_path or file_name == os.path.basename(
                exclude_abs_path
            ):
                return True

        elif os.path.isdir(exclude_abs_path):
            if os.path.commonprefix([file_path, exclude_abs_path]) == exclude_abs_path:
                return True

    return False


def should_include(file_path, include_list):
    if not include_list:
        return True

    file_path = os.path.abspath(file_path)
    
    for include_item in include_list:
        if include_item.startswith("./"):
            include_abs_path = os.path.abspath(include_item)
            if file_path == include_abs_path:
                return True
        elif os.path.isdir(os.path.abspath(include_item)):
            include_abs_path = os.path.abspath(include_item)
            if os.path.commonprefix([file_path, include_abs_path]) == include_abs_path:
                return True
        else:
            if os.path.basename(file_path) == include_item:
                return True

    return False


def check_invalid_md_filenames(src_dir, exclude_list, include_list):
    invalid_md_files = []

    for root, _, files in os.walk(src_dir):
        for file in files:
            if not file.endswith(".md"):
                continue
                
            src_file_path = os.path.join(root, file)
            
            if should_exclude(src_file_path, exclude_list) or not should_include(src_file_path, include_list):
                continue
                
            filename = os.path.splitext(file)[0]
            
            if "." in filename:
                invalid_md_files.append(src_file_path)
    
    return invalid_md_files


def find_all_md_files(src_dir, exclude_list, include_list):
    normal_md_files = []
    en_md_files = []
    
    for root, _, files in os.walk(src_dir):
        for file in files:
            if not file.endswith(".md"):
                continue
                
            src_file_path = os.path.join(root, file)
            
            if should_exclude(src_file_path, exclude_list) or not should_include(src_file_path, include_list):
                continue
            
            if file.endswith("_en.md"):
                en_md_files.append(src_file_path)
            else:
                normal_md_files.append(src_file_path)
    
    return normal_md_files, en_md_files


def remove_en_suffix(filename):
    if filename.endswith("_en.md"):
        name_without_ext = filename[:-3]
        if name_without_ext.endswith("_en"):
            return name_without_ext[:-3] + ".md"
    return filename


def copy_md_files_and_images(src_dir, dest_dir, target_en_docs_path, exclude_list, include_list):
    current_dir_name = os.path.basename(os.path.abspath(src_dir))
    dest_dir = os.path.join(dest_dir, current_dir_name)

    normal_md_files, en_md_files = find_all_md_files(src_dir, exclude_list, include_list)
    
    en_docs_dest = target_en_docs_path
    
    print(f"英文文档目录路径: {en_docs_dest}")
    print(f"找到 {len(normal_md_files)} 个普通Markdown文件")
    print(f"找到 {len(en_md_files)} 个英文版本文件")
    
    processed_images = set()
    
    for src_file_path in normal_md_files:
        rel_path = os.path.relpath(src_file_path, src_dir)
        
        dest_file_path = os.path.join(dest_dir, rel_path)
        copy_file(src_file_path, dest_file_path)
        
        file_name = os.path.basename(src_file_path)
        dir_name = os.path.dirname(src_file_path)
        en_file_name = os.path.splitext(file_name)[0] + "_en.md"
        en_file_path = os.path.join(dir_name, en_file_name)
        
        if en_file_path not in en_md_files:
            en_dest_file_path = os.path.join(en_docs_dest, rel_path)
            copy_file(src_file_path, en_dest_file_path)
            # print(f"No English version found for {src_file_path}, copied to {en_dest_file_path}")
        
        with open(src_file_path, "r", encoding="utf-8") as md_file:
            md_content = md_file.read()
        
        local_images = find_local_images(md_content, os.path.dirname(src_file_path))
        for img_path in local_images:
            if img_path in processed_images:
                continue
                
            processed_images.add(img_path)
            
            if not should_exclude(img_path, exclude_list) and should_include(img_path, include_list):
                rel_img_path = os.path.relpath(img_path, src_dir)
                
                dest_img_path = os.path.join(dest_dir, rel_img_path)
                copy_file(img_path, dest_img_path)
                
                en_img_path = os.path.join(en_docs_dest, rel_img_path)
                copy_file(img_path, en_img_path)
            else:
                print(f"Skipping excluded or not included image: {img_path}")
    
    for en_file_path in en_md_files:
        rel_path = os.path.relpath(en_file_path, src_dir)
        
        file_name = os.path.basename(en_file_path)
        dir_path = os.path.dirname(rel_path)
        
        base_file_name = remove_en_suffix(file_name)
        en_dest_path = os.path.join(dir_path, base_file_name)
        en_dest_file_path = os.path.join(en_docs_dest, en_dest_path)
        
        copy_file(en_file_path, en_dest_file_path)
        # print(f"Copied English version {en_file_path} to {en_dest_file_path}")
        
        with open(en_file_path, "r", encoding="utf-8") as md_file:
            md_content = md_file.read()
        
        local_images = find_local_images(md_content, os.path.dirname(en_file_path))
        for img_path in local_images:
            if img_path in processed_images:
                continue
                
            processed_images.add(img_path)
            
            if not should_exclude(img_path, exclude_list) and should_include(img_path, include_list):
                rel_img_path = os.path.relpath(img_path, src_dir)
                
                dest_img_path = os.path.join(dest_dir, rel_img_path)
                copy_file(img_path, dest_img_path)
                
                en_img_path = os.path.join(en_docs_dest, rel_img_path)
                copy_file(img_path, en_img_path)
            else:
                print(f"Skipping excluded or not included image: {img_path}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python sync_docs.py <source_directory> <target_repo_path> <target_en_docs_path>")
        sys.exit(1)

    source_directory = sys.argv[1]
    target_repo_path = sys.argv[2]
    target_en_docs_path = sys.argv[3]

    exclude_list = [
        "./src/elevation_mapping/"
    ]
    include_list = [
      "docs",
      "src",
      "images",
      "./readme_opensource.md",
      "./readme.md",
      "tools",
      "./CHANGELOG.md"
    ]
    
    invalid_md_files = check_invalid_md_filenames(source_directory, exclude_list, include_list)
    if invalid_md_files:
        print("错误: 以下Markdown文件名无效 (文件名中包含'.'字符):")
        for invalid_file in invalid_md_files:
            print(f"  - {invalid_file}")
        print("请重命名这些文件，确保文件名中不包含'.'符号 (除了文件扩展名)。")
        sys.exit(1)

    copy_md_files_and_images(source_directory, target_repo_path, target_en_docs_path, exclude_list, include_list)
