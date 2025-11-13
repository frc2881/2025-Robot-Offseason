import os
import subprocess

try:
  result = subprocess.run("git submodule add https://github.com/frc2881/Robot-Lib.git lib", check = True, capture_output = True, text = True)
  print(result.stdout)
except subprocess.CalledProcessError as e:
  print(e.stderr)

os.system("git submodule update --init")
