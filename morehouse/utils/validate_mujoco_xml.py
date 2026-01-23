import xml.etree.ElementTree as ET

# MuJoCo支持的属性列表（根据MuJoCo官方文档）
SUPPORTED_PROPERTIES = {
    'mujoco': ['model', 'version'],
    'compiler': ['angle', 'coordinate', 'inertiafromgeom', 'meshdir', 'texturedir', 'basedir'],
    'option': ['timestep', 'gravity', 'integrator', 'implicitthreshold', 'jittermargin', 
               'solver', 'iterations', 'ls_iterations', 'tolerance', 'constraintsolref', 
               'constraintsolimp', 'enableflags', 'disableflags', 'viscosity'],
    'contact': ['solref', 'solimp', 'frictionloss', 'maxtriction', 'slip1', 'slip2', 
                'density', 'damping', 'stiffness', 'armratio', 'condim', 'contactheight', 
                'contactmargin', 'mincontactforce', 'maxcontactforce'],
    'asset': [],
    'mesh': ['name', 'file', 'scale'],
    'texture': ['name', 'type', 'builtin', 'file', 'rgb1', 'rgb2', 'width', 'height', 
                'mark', 'markrgb'],
    'material': ['name', 'texture', 'texrepeat', 'texuniform', 'texcoord', 'reflectance', 
                 'rgba', 'specular', 'emission'],
    'worldbody': [],
    'body': ['name', 'pos', 'quat', 'axisangle', 'xyaxes', 'zaxis', 'euler'],
    'geom': ['name', 'type', 'pos', 'quat', 'axisangle', 'xyaxes', 'zaxis', 'euler', 
             'size', 'rgba', 'material', 'mass', 'inertia', 'density', 'friction', 
             'condim', 'solref', 'solimp', 'margin', 'gap', 'contype', 'conaffinity', 
             'priority', 'group', 'exclude', 'mesh'],
    'joint': ['name', 'type', 'pos', 'axis', 'range', 'damping', 'stiffness', 'armature', 
              'springref', 'springstiffness', 'limitforce', 'forcerange', 'gear', 'actuatorfrcrange'],
    'light': ['name', 'pos', 'dir', 'dirrel', 'attenuation', 'diffuse', 'specular', 
              'ambient', 'castshadow', 'mode', 'active'],
    'camera': ['name', 'pos', 'quat', 'axisangle', 'xyaxes', 'zaxis', 'euler', 'fovy', 
               'focal', 'near', 'far', 'mode', 'trackbodyid', 'lookat'],
    'actuator': [],
    'motor': ['name', 'joint', 'gear', 'ctrlrange', 'gain', 'bias', 'forcerange', 
              'ctrlratio', 'actuatorfrcrange'],
    'freejoint': []
}

def check_element(element, element_path):
    """检查元素的属性是否都被支持"""
    errors = []
    tag = element.tag
    
    if tag in SUPPORTED_PROPERTIES:
        supported_attrs = SUPPORTED_PROPERTIES[tag]
        for attr in element.attrib:
            if attr not in supported_attrs:
                errors.append(f"Unsupported attribute '{attr}' in element '{tag}' at path '{element_path}'")
    else:
        # 对于未知元素，检查是否有属性
        if element.attrib:
            errors.append(f"Unknown element '{tag}' at path '{element_path}' with attributes: {list(element.attrib.keys())}")
    
    return errors

def validate_xml(file_path):
    """验证整个XML文件"""
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        all_errors = []
        
        # 递归检查所有元素
        def traverse(element, path):
            current_path = f"{path}/{element.tag}" if path else element.tag
            errors = check_element(element, current_path)
            all_errors.extend(errors)
            
            for child in element:
                traverse(child, current_path)
        
        traverse(root, "")
        
        if all_errors:
            print("发现以下错误:")
            for error in all_errors:
                print(f"  - {error}")
            return False
        else:
            print("XML文件验证通过！所有属性都符合MuJoCo规范。")
            return True
            
    except ET.ParseError as e:
        print(f"XML解析错误: {e}")
        return False
    except Exception as e:
        print(f"其他错误: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    validate_xml('three_bedroom_apartment.xml')