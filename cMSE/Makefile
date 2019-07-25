_core.so: core.cpp core.h core_wrap.cxx
		g++ -shared -fPIC -I/usr/include/python3.5m -lpython3.5m -o _core.so core.cpp core_wrap.cxx
core_wrap.cxx: core.i
		swig -c++ -python -py3 core.i
clean: 
		rm _core.so core_wrap.cxx core.py