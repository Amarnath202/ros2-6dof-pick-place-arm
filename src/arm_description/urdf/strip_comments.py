import os
import re

def strip_xml_comments(filepath):
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Remove all comments <!-- ... -->
    cleaned_content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)
    
    with open(filepath, 'w') as f:
        f.write(cleaned_content)

dir_path = '/home/amarnath/Projects/arm/arm_ws/src/arm_description/urdf/'
for filename in os.listdir(dir_path):
    if filename.endswith('.xacro'):
        strip_xml_comments(os.path.join(dir_path, filename))
