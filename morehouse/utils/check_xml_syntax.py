import xml.etree.ElementTree as ET

try:
    # 尝试解析XML文件
    tree = ET.parse('three_bedroom_apartment.xml')
    root = tree.getroot()
    print("XML文件语法正确！")
    print(f"根元素: {root.tag}")
    print(f"模型名称: {root.get('model')}")
    
    # 检查主要元素是否存在
    required_elements = ['compiler', 'option', 'asset', 'worldbody']
    for elem in required_elements:
        if root.find(elem) is not None:
            print(f"✓ 找到元素: {elem}")
        else:
            print(f"✗ 缺少元素: {elem}")
    
    # 检查材质和纹理
    asset = root.find('asset')
    if asset:
        textures = asset.findall('texture')
        materials = asset.findall('material')
        print(f"找到 {len(textures)} 个纹理定义")
        print(f"找到 {len(materials)} 个材质定义")
    
    # 检查worldbody中的几何形状
    worldbody = root.find('worldbody')
    if worldbody:
        geoms = worldbody.findall('.//geom')
        bodies = worldbody.findall('body')
        print(f"找到 {len(bodies)} 个body定义")
        print(f"找到 {len(geoms)} 个几何形状定义")
        
    # 检查执行器
    actuator = root.find('actuator')
    if actuator:
        motors = actuator.findall('motor')
        print(f"找到 {len(motors)} 个电机定义")
        
except ET.ParseError as e:
    print(f"XML语法错误: {e}")
except Exception as e:
    print(f"其他错误: {e}")
    import traceback
    traceback.print_exc()