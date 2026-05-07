# robosar_khepera_lib
Khepera Library for RoboSAR Project. If library source code is modified, then it will need to be recompiled and then put onto the Khepera again.
For agent with IP address 192.168.0.109, do the following:
1. Modify source code as desired in src directory
2. In top directory of repo, run make clean, then make all
3. Transfer new compiled library to agent: scp build-khepera-3.18.18-custom/lib/libkhepera.so.2.1 root@192.168.0.108:/usr/lib/

