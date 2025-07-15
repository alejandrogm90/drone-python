import sys

import src.utils.common_functions as cf

if __name__ == '__main__':
    # Show script info
    info = {
        "name": str(cf.get_file_name(sys.argv[0], True)),
        "location": sys.argv[0],
        "description": "Basic Python3 template",
        "Autor": "Alejandro Gómez",
        "calling": f"{sys.argv[0]} parameters"
    }
    cf.show_script_info(info)
    
    # info message
    cf.info_msg("info message")
