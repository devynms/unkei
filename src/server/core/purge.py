import shutil
import os

def purge(folder):
    shutil.rmtree(folder)
    os.makedirs(folder)
