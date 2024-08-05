## config.py
## This module contains global configuration variables, including file paths.

import os
import sys

# Add the parent directory to sys.path
CURR_DIR: str = os.path.dirname(os.path.abspath(__file__))
HOME_DIR: str = os.path.dirname(CURR_DIR)
sys.path.insert(0, HOME_DIR)

# Classes
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class BaseConfig:
    """Class for base configuration."""

    project_name: str = "vehicle-routing"
    project_config: str = f"{HOME_DIR}/.env"


BASE_CONFIG = BaseConfig()


@dataclass(frozen=True)
class DataPath:
    """Class for data paths."""

    # General data paths
    root: str = f"{HOME_DIR}/data/"
    base: str = root + "base/"
    interim: str = root + "interim/"
    vector_db: str = root + "vector_db/"

    # Base data paths
    depots: str = base + "woolworths_depots_nsw.csv"
    stores: str = base + "woolworths_stores_nsw.csv"


DATA_PATH = DataPath()
