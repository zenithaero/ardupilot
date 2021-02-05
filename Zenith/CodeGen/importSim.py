#!/usr/bin/env python
"""
@brief Generated code setup script
@date Created Feb 17, 2020
@author Bertrand Bevillard <bertrand@zenithaero.com>
"""

import os
import zipfile
from pathlib import Path
import shutil
import itertools

LOCAL_DIR = Path(__file__).parent
SIM_DIR = LOCAL_DIR.joinpath("../../libraries/Zenith/Simulator")
SIM_MODEL_DIR = os.getenv("SIM_MODEL_DIR")
SIM_ZIP = "Simulator.zip"
DEFINES_HEADER = """
// Autogenerated by CodeGen.py
#pragma once\n
"""
HEADER = """
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"\n
#include "defines.h"\n
"""
FOOTER = """
#pragma GCC diagnostic pop
"""
CHECK = """
#ifdef {0}
# error {0} already defined
#endif
"""
STATE_DECL = "DW_Simulator_T Simulator_DW;"
PUBLIC_DECL = "public:"

if __name__ == "__main__":
    # Control variables
    if not os.path.isdir(SIM_DIR):
        raise Exception("Sim dir '{}' not found".format(SIM_DIR))
    if not SIM_MODEL_DIR:
        raise Exception("The env variable 'SIM_MODEL_DIR' is not defined")
    SIM_ZIP_PATH = Path(SIM_MODEL_DIR).joinpath(SIM_ZIP)
    if not os.path.isfile(SIM_ZIP_PATH):
        raise Exception("The sim zip file could not be found at '{}'".format(SIM_ZIP_PATH))
    # Clear directory
    shutil.rmtree(SIM_DIR)
    os.mkdir(SIM_DIR)
    # Extract zip file
    with zipfile.ZipFile(SIM_ZIP_PATH, "r") as zip_ref:
        zip_ref.extractall(SIM_DIR)
    # Traverse all files
    gen = [SIM_DIR.rglob(ext) for ext in ["*.cpp", "*.h", "*.txt"]]
    sources = []
    for file in itertools.chain(*gen):
        dest = SIM_DIR.joinpath(file.name)
        shutil.copy(str(file), str(dest))
        sources.append(str(dest))
    # Clear directories
    for f in SIM_DIR.glob("*"):
        if f.is_dir():
            shutil.rmtree(f)
    # Create environment header
    DEFINES_TXT_PATH = SIM_DIR.joinpath("defines.txt")
    DEFINES_HEADER_PATH = SIM_DIR.joinpath("defines.h")
    defines = DEFINES_HEADER
    with open(DEFINES_TXT_PATH, "r") as f:
        for line in f:
            statement = line.strip("\n").strip(" ").split("=")
            defines += CHECK.format(statement[0])
            defines += "#define {}\n".format(" ".join(statement))
    with open(DEFINES_HEADER_PATH, "w") as f:
        f.write(defines)
    # Include header
    for file in sources:
        with open(file, "r+") as f:
            s = f.read()
            f.seek(0)
            f.write(HEADER + s + FOOTER)
    # Make state structure public
    SIM_HEADER_PATH = SIM_DIR.joinpath("Simulator.h")
    with open(SIM_HEADER_PATH, "r+") as f:
        s = f.read()
        # Comment out state declaration
        s = "//{}".format(STATE_DECL).join(s.split(STATE_DECL))
        s = "{}\n{}\n".format(PUBLIC_DECL, STATE_DECL).join(s.split(PUBLIC_DECL))
        f.seek(0)
        f.write(s)