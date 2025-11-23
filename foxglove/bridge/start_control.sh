export DEP_LIB_PATH=$(pip show pndsdk2py | grep Location | awk '{print $2}')/pndsdk2py/lib
export LD_LIBRARY_PATH=$DEP_LIB_PATH:$LD_LIBRARY_PATH
python3 example.py --cfg cfg/control_module_cfg.yaml