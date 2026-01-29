import os
import glob
import xml.etree.ElementTree as ET

# ================= 用户配置区域 =================
KEIL_SUB_DIR = "MDK-ARM"
DEFAULT_PROJECT_NAME = "REACTOR-46H.uvprojx"
SEARCH_DIRS = ['ReactorLibs']
EXCLUDE_DIRS = ['Examples', 'Test', '.git', 'build', '__pycache__']

# [新增] 支持的源文件后缀与 Keil FileType 的映射
# 1: C source, 8: C++ source, 2: ASM source
SOURCE_EXT_MAP = {
    '.c': 1,
    '.cpp': 8,
    '.cc': 8,
    '.s': 2,
    '.asm': 2
}
# ===============================================

def find_keil_project(root_dir):
    """(保持原逻辑不变) 自动寻找 .uvprojx 文件"""
    mdk_path = os.path.join(root_dir, KEIL_SUB_DIR)
    if not os.path.exists(mdk_path):
        print(f"错误: 找不到 Keil 子目录: {mdk_path}")
        return None
    root_name = os.path.basename(root_dir)
    candidate = os.path.join(mdk_path, f"{root_name}.uvprojx")
    if os.path.exists(candidate):
        return candidate
    files = glob.glob(os.path.join(mdk_path, "*.uvprojx"))
    if files:
        return files[0]
    return os.path.join(mdk_path, DEFAULT_PROJECT_NAME)

def scan_files(root_dir, target_base_dir):
    """
    [重构] 同时扫描头文件路径和源文件
    返回:
    1. include_paths: set, 头文件搜索路径
    2. source_groups: dict, 结构为 { "Group名称": [file_info_dict, ...] }
    """
    include_paths = set()
    source_groups = {} # Key: 文件夹名(作为Group名), Value: List of file objects

    for search_dir in SEARCH_DIRS:
        start_path = os.path.join(root_dir, search_dir)
        if not os.path.exists(start_path):
            print(f"警告: 找不到搜索目录 {search_dir}，跳过。")
            continue

        for root, dirs, files in os.walk(start_path):
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            
            # 计算相对于 MDK-ARM 的路径
            rel_path_from_proj = os.path.relpath(root, target_base_dir)
            
            # --- 逻辑1: 收集头文件路径 ---
            if any(f.endswith(('.h', '.hpp')) for f in files):
                include_paths.add(rel_path_from_proj)

            # --- 逻辑2: 收集源文件并按文件夹分组 ---
            # Group 名称生成策略：使用相对于 SEARCH_DIRS 的路径，例如 "ReactorLibs/Utils"
            # 如果想更扁平，可以使用 os.path.basename(root)
            rel_path_from_root = os.path.relpath(root, root_dir)
            group_name = rel_path_from_root.replace('\\', '/') # 统一使用 / 作为 Group 名展示

            group_files = []
            for file in files:
                ext = os.path.splitext(file)[1].lower()
                if ext in SOURCE_EXT_MAP:
                    file_path = os.path.join(rel_path_from_proj, file)
                    group_files.append({
                        'name': file,
                        'path': file_path,
                        'type': str(SOURCE_EXT_MAP[ext])
                    })
            
            if group_files:
                source_groups[group_name] = group_files
                
    return include_paths, source_groups

def indent(elem, level=0):
    """辅助函数：用于美化 XML 输出 (增加缩进)"""
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def update_xml_structure(proj_path, new_inc_paths, new_src_groups):
    """[核心更新逻辑] 更新 IncludePath 和 Groups"""
    if not os.path.exists(proj_path):
        return

    try:
        tree = ET.parse(proj_path)
        root = tree.getroot()
    except ET.ParseError:
        print("错误: 解析 XML 失败。")
        return

    # 1. 更新 IncludePath (保持原有逻辑)
    # -------------------------------------------------
    updated_inc_count = 0
    for node in root.iter('IncludePath'):
        current_text = node.text if node.text else ""
        existing_paths = [p.strip() for p in current_text.split(';') if p.strip()]
        final_paths = list(existing_paths)
        norm_final_paths = [p.replace('\\', '/').lower() for p in final_paths]

        for path in new_inc_paths:
            path_norm = path.replace('\\', '/').lower()
            if path_norm not in norm_final_paths:
                final_paths.append(path)
                norm_final_paths.append(path_norm)
        
        node.text = ";".join(final_paths)
        updated_inc_count += 1
    
    print(f"  - 已更新 {updated_inc_count} 处 IncludePath 配置")

    # 2. 更新 Groups (新增逻辑)
    # -------------------------------------------------
    # Keil 结构通常是: <Targets> -> <Target> -> <Groups>
    # 我们需要遍历所有 Target 下的 Groups 节点
    targets = root.findall('Targets/Target')
    
    total_added_files = 0
    
    for target in targets:
        groups_node = target.find('Groups')
        if groups_node is None:
            # 如果不存在 Groups 节点，创建它
            groups_node = ET.SubElement(target, 'Groups')
        
        # 获取现有的 Group 列表，用于去重检查
        existing_groups = {g.find('GroupName').text: g for g in groups_node.findall('Group')}
        
        for grp_name, file_list in new_src_groups.items():
            # 2.1 检查 Group 是否存在，不存在则创建
            if grp_name in existing_groups:
                current_group = existing_groups[grp_name]
            else:
                current_group = ET.SubElement(groups_node, 'Group')
                name_node = ET.SubElement(current_group, 'GroupName')
                name_node.text = grp_name
                ET.SubElement(current_group, 'Files') # 创建空的 Files 容器
                existing_groups[grp_name] = current_group # 更新缓存
                print(f"    [新建 Group]: {grp_name}")

            # 2.2 获取该 Group 下的 Files 节点
            files_node = current_group.find('Files')
            if files_node is None:
                files_node = ET.SubElement(current_group, 'Files')

            # 2.3 检查文件是否存在，防止重复添加
            # 获取当前 Group 下所有已存在的文件路径进行比对
            existing_file_paths = []
            for f in files_node.findall('File'):
                fp = f.find('FilePath')
                if fp is not None and fp.text:
                    existing_file_paths.append(fp.text.replace('\\', '/').lower())

            for file_info in file_list:
                norm_path = file_info['path'].replace('\\', '/').lower()
                
                if norm_path not in existing_file_paths:
                    # 添加新文件节点
                    f_node = ET.SubElement(files_node, 'File')
                    ET.SubElement(f_node, 'FileName').text = file_info['name']
                    ET.SubElement(f_node, 'FileType').text = file_info['type']
                    ET.SubElement(f_node, 'FilePath').text = file_info['path']
                    
                    existing_file_paths.append(norm_path)
                    total_added_files += 1

    print(f"  - 已向工程添加 {total_added_files} 个新源文件")

    # 3. 保存并美化
    indent(root)
    tree.write(proj_path, encoding='UTF-8', xml_declaration=True)
    print("  - XML 文件保存完成")

if __name__ == "__main__":
    current_root = os.getcwd()
    print(f"当前工作根目录: {current_root}")

    project_file_path = find_keil_project(current_root)
    
    if project_file_path:
        project_dir = os.path.dirname(project_file_path)
        print(f"目标工程: {os.path.basename(project_file_path)}")

        print("正在扫描代码...")
        inc_paths, src_groups = scan_files(current_root, project_dir)
        print(f"扫描结果: {len(inc_paths)} 个头文件路径, {sum(len(v) for v in src_groups.values())} 个源文件。")
        
        print("正在同步到工程文件...")
        update_xml_structure(project_file_path, inc_paths, src_groups)
        print("完成。请在 Keil 中重新加载工程 (通常 Keil 会提示检测到外部修改)。")
    else:
        print("未找到工程文件，操作终止。")