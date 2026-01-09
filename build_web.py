"""
Build script for ESP32 Robot Arm Web UI

Builds the React app and copies to data folder for SPIFFS upload.
Run before platformio run --target uploadfs
"""

import os
import subprocess
import shutil
import sys

def build_web_ui():
    """Build React app and copy to data folder"""
    
    project_root = os.path.dirname(os.path.abspath(__file__))
    web_ui_dir = os.path.join(project_root, 'web-ui')
    data_dir = os.path.join(project_root, 'data')
    
    # Check if web-ui exists
    if not os.path.exists(web_ui_dir):
        print("[ERROR] web-ui directory not found")
        return False
    
    # Check for node_modules
    if not os.path.exists(os.path.join(web_ui_dir, 'node_modules')):
        print("[INFO] Installing npm dependencies...")
        result = subprocess.run(['npm', 'install'], cwd=web_ui_dir, shell=True)
        if result.returncode != 0:
            print("[ERROR] npm install failed")
            return False
    
    # Build React app
    print("[INFO] Building React app...")
    result = subprocess.run(['npm', 'run', 'build'], cwd=web_ui_dir, shell=True)
    if result.returncode != 0:
        print("[ERROR] Build failed")
        return False
    
    # Check bundle size
    dist_dir = os.path.join(web_ui_dir, 'dist')
    if os.path.exists(dist_dir):
        total_size = 0
        for root, dirs, files in os.walk(dist_dir):
            for f in files:
                total_size += os.path.getsize(os.path.join(root, f))
        
        size_kb = total_size / 1024
        print(f"[INFO] Bundle size: {size_kb:.1f} KB")
        
        if size_kb > 1500:
            print("[WARNING] Bundle exceeds 1.5MB SPIFFS limit!")
            print("[INFO] Consider code splitting or removing unused dependencies")
    
    # Copy to data folder (Vite outputs to ../data via config)
    print("[INFO] Build complete. Files in data/ folder.")
    
    return True

def before_build(source, target, env):
    """PlatformIO pre-build hook"""
    build_web_ui()

if __name__ == "__main__":
    success = build_web_ui()
    sys.exit(0 if success else 1)
