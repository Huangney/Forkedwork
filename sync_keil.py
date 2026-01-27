import os
import glob
import xml.etree.ElementTree as ET

# ================= 用户配置区域 =================
# 1. Keil 工程所在的子文件夹名称 (通常是 MDK-ARM)
KEIL_SUB_DIR = "MDK-ARM"

# 2. 默认工程名称 (仅当自动识别失败时使用)
DEFAULT_PROJECT_NAME = "REACTOR-46H.uvprojx"

# 3. 需要递归扫描头文件的根目录列表 (相对于脚本所在目录)
SEARCH_DIRS = ['ReactorLibs']

# 4. 排除的文件夹名称
EXCLUDE_DIRS = ['Examples', 'Test', '.git', 'build']
# ===============================================

def find_keil_project(root_dir):
    """
    自动寻找 .uvprojx 文件
    策略1: 寻找 MDK-ARM/根目录名.uvprojx
    策略2: 寻找 MDK-ARM/*.uvprojx (任取一个)
    策略3: 使用默认名称
    """
    mdk_path = os.path.join(root_dir, KEIL_SUB_DIR)
    
    if not os.path.exists(mdk_path):
        print(f"错误: 找不到 Keil 子目录: {mdk_path}")
        return None

    # 策略1: 尝试匹配文件夹名称
    root_name = os.path.basename(root_dir)
    candidate = os.path.join(mdk_path, f"{root_name}.uvprojx")
    if os.path.exists(candidate):
        print(f"自动识别到工程文件: {os.path.basename(candidate)}")
        return candidate

    # 策略2: 搜索目录下的任意 .uvprojx
    files = glob.glob(os.path.join(mdk_path, "*.uvprojx"))
    if files:
        print(f"自动搜索到工程文件: {os.path.basename(files[0])}")
        return files[0]

    # 策略3: 兜底
    print(f"警告: 无法自动识别，尝试使用默认名称: {DEFAULT_PROJECT_NAME}")
    return os.path.join(mdk_path, DEFAULT_PROJECT_NAME)

def get_include_paths(root_dir, target_base_dir):
    """
    扫描包含 .h 的目录，并计算相对于 .uvprojx 的路径
    :param root_dir: 脚本所在的根目录
    :param target_base_dir: .uvprojx 所在的目录 (用于计算相对路径)
    """
    include_paths = set()
    
    for search_dir in SEARCH_DIRS:
        start_path = os.path.join(root_dir, search_dir)
        if not os.path.exists(start_path):
            print(f"警告: 找不到搜索目录 {search_dir}，跳过。")
            continue

        for root, dirs, files in os.walk(start_path):
            # 过滤排除目录
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            
            # 检查是否有 .h 文件
            if any(file.endswith('.h') or file.endswith('.hpp') for file in files):
                # 核心修正：计算相对于 MDK-ARM 文件夹的路径
                # 这里的 target_base_dir 就是 MDK-ARM 的绝对路径
                rel_path = os.path.relpath(root, target_base_dir)
                
                # 统一路径分隔符 (Keil 对 \ 和 / 兼容性尚可，但建议统一)
                # 使用 replace 确保 Windows 风格，或者保持 os.sep
                include_paths.add(rel_path)
                
    return include_paths

def update_uvprojx(proj_path, new_paths):
    """更新 uvprojx 文件中的 IncludePath 标签"""
    if not os.path.exists(proj_path):
        print(f"错误: 工程文件不存在 {proj_path}")
        return

    try:
        tree = ET.parse(proj_path)
        root = tree.getroot()
    except ET.ParseError:
        print("错误: 解析 XML 失败，文件可能损坏。")
        return

    # 查找所有的 IncludePath 节点
    nodes = list(root.iter('IncludePath'))
    
    if not nodes:
        print("错误: XML 中未找到 <IncludePath> 标签，请先在 Keil 中手动添加一个路径以生成结构。")
        return

    count = 0
    for node in nodes:
        current_text = node.text if node.text else ""
        existing_paths = [p.strip() for p in current_text.split(';') if p.strip()]
        
        final_paths = list(existing_paths)
        added_count = 0
        
        # 路径去重检查
        # 简单的字符串比对，防止重复添加
        norm_final_paths = [p.replace('\\', '/').lower() for p in final_paths]

        for path in new_paths:
            path_norm = path.replace('\\', '/').lower()
            if path_norm not in norm_final_paths:
                final_paths.append(path)
                norm_final_paths.append(path_norm) # 更新比对列表
                added_count += 1
        
        node.text = ";".join(final_paths)
        count += 1
    
    if count > 0:
        tree.write(proj_path, encoding='UTF-8', xml_declaration=True)
        print(f"成功: 更新了 {count} 个 Target 配置，工程文件已保存。")
    else:
        print("未做任何修改。")

if __name__ == "__main__":
    current_root = os.getcwd()
    print(f"当前工作根目录: {current_root}")

    # 1. 寻找工程文件
    project_file_path = find_keil_project(current_root)
    
    if project_file_path:
        # 2. 获取工程文件所在的目录 (用于计算相对路径 ../)
        project_dir = os.path.dirname(project_file_path)

        # 3. 扫描并计算路径
        print("正在扫描头文件...")
        # 注意这里传入了 project_dir 作为计算相对路径的基准
        detected_paths = get_include_paths(current_root, project_dir)
        print(f"扫描到 {len(detected_paths)} 个含头文件的目录。")
        
        # 4. 写入 XML
        print(f"正在写入文件: {os.path.basename(project_file_path)} ...")
        update_uvprojx(project_file_path, detected_paths)
        print("完成。")