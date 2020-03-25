#!/usr/bin/env python
"""
@brief Gains header import script
@date Created March 23, 2020
@author Bertrand Bevillard <bertrand@zenithaero.com>
"""

import os
from pathlib import Path
import shutil

LOCAL_DIR = Path(__file__).parent
ZENITH_MODULE_DIR = LOCAL_DIR.joinpath("../../libraries/Zenith")
SIM_MODEL_DIR = os.getenv("SIM_MODEL_DIR")
SIM_MODEL_SUBDIR = "Gen"
GAINS_HEADER = "ZenithGains.h"

if __name__ == "__main__":
    # Control variables
    if not os.path.isdir(ZENITH_MODULE_DIR):
        raise Exception("Zenith module dir '{}' not found".format(ZENITH_MODULE_DIR))
    if not SIM_MODEL_DIR:
        raise Exception("The env variable 'SIM_MODEL_DIR' is not defined")
    HEADER_SRC = Path(SIM_MODEL_DIR).joinpath(SIM_MODEL_SUBDIR).joinpath(GAINS_HEADER)
    HEADER_DST = ZENITH_MODULE_DIR.joinpath(GAINS_HEADER)
    shutil.copy(str(HEADER_SRC), str(HEADER_DST))
