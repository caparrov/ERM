import subprocess
import os.path
import itertools
import math
import json
import sys


llvm_path = "/home/caparrov/ERM"


files_to_modify = [llvm_path+"/lib/Support/DynamicAnalysis.cpp",
                    llvm_path+"/lib/Support/TBV.cpp",
                    llvm_path+"/include/llvm/Support/DynamicAnalysis.h"]








for file in files_to_modify:
    with open(file) as f:
        newText=f.read()
        if "//#define INTERPRETER" in newText:
            print "Replacing newText"
            newText=newText.replace('//#define INTERPRETER', '#define INTERPRETER')
        else:
            if "#define INTERPRETER" in newText:
                print "Interpreter already enabled"
            else:
                sys.exit("#define INTERPRETER not found")
    
    with open(file, "w") as f:
        f.write(newText)
        f.close()
