import os
import shutil
import sys

# WMX3 DLL/모듈이 있는 폴더 (스크립트 위치 기준)
def _script_dir():
    return os.path.dirname(os.path.abspath(__file__))

def ensure_dll_exists():
    # 스크립트가 있는 폴더를 PATH 맨 앞에 추가 → IDE/다른 cwd에서 실행해도 DLL을 찾을 수 있음
    current_dir = _script_dir()
    path_env = os.environ.get("PATH", "")
    if current_dir not in path_env.split(os.pathsep):
        os.environ["PATH"] = current_dir + os.pathsep + path_env

    # WMX3 환경변수가 있으면 해당 경로에서 프로젝트 폴더로 복사
    wmx_root = os.environ.get("WMX3", "").strip()
    if not wmx_root or not os.path.isdir(wmx_root):
        return

    python_version = f"Python{sys.version_info.major}{sys.version_info.minor}"
    files_to_check = {
        "IMDll.dll":          os.path.join(wmx_root, "Lib",              "IMDll.dll"),
        "_WMX3ApiPython.pyd": os.path.join(wmx_root, "Lib", "PythonApi", python_version, "_WMX3ApiPython.pyd"),
        "WMX3ApiPython.py":   os.path.join(wmx_root, "Lib", "PythonApi", python_version, "WMX3ApiPython.py")
    }

    for file_name, source_path in files_to_check.items():
        if not os.path.isfile(source_path):
            continue
        destination_path = os.path.join(current_dir, file_name)
        try:
            shutil.copy(source_path, destination_path)
        except Exception:
            pass
