# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.8
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_core', [dirname(__file__)])
        except ImportError:
            import _core
            return _core
        if fp is not None:
            try:
                _mod = imp.load_module('_core', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _core = swig_import_helper()
    del swig_import_helper
else:
    import _core
del version_info
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr_nondynamic(self, class_type, name, static=1):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    if (not static):
        return object.__getattr__(self, name)
    else:
        raise AttributeError(name)

def _swig_getattr(self, class_type, name):
    return _swig_getattr_nondynamic(self, class_type, name, 0)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0


class SwigPyIterator(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, SwigPyIterator, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, SwigPyIterator, name)

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _core.delete_SwigPyIterator
    __del__ = lambda self: None

    def value(self) -> "PyObject *":
        return _core.SwigPyIterator_value(self)

    def incr(self, n: 'size_t'=1) -> "swig::SwigPyIterator *":
        return _core.SwigPyIterator_incr(self, n)

    def decr(self, n: 'size_t'=1) -> "swig::SwigPyIterator *":
        return _core.SwigPyIterator_decr(self, n)

    def distance(self, x: 'SwigPyIterator') -> "ptrdiff_t":
        return _core.SwigPyIterator_distance(self, x)

    def equal(self, x: 'SwigPyIterator') -> "bool":
        return _core.SwigPyIterator_equal(self, x)

    def copy(self) -> "swig::SwigPyIterator *":
        return _core.SwigPyIterator_copy(self)

    def next(self) -> "PyObject *":
        return _core.SwigPyIterator_next(self)

    def __next__(self) -> "PyObject *":
        return _core.SwigPyIterator___next__(self)

    def previous(self) -> "PyObject *":
        return _core.SwigPyIterator_previous(self)

    def advance(self, n: 'ptrdiff_t') -> "swig::SwigPyIterator *":
        return _core.SwigPyIterator_advance(self, n)

    def __eq__(self, x: 'SwigPyIterator') -> "bool":
        return _core.SwigPyIterator___eq__(self, x)

    def __ne__(self, x: 'SwigPyIterator') -> "bool":
        return _core.SwigPyIterator___ne__(self, x)

    def __iadd__(self, n: 'ptrdiff_t') -> "swig::SwigPyIterator &":
        return _core.SwigPyIterator___iadd__(self, n)

    def __isub__(self, n: 'ptrdiff_t') -> "swig::SwigPyIterator &":
        return _core.SwigPyIterator___isub__(self, n)

    def __add__(self, n: 'ptrdiff_t') -> "swig::SwigPyIterator *":
        return _core.SwigPyIterator___add__(self, n)

    def __sub__(self, *args) -> "ptrdiff_t":
        return _core.SwigPyIterator___sub__(self, *args)
    def __iter__(self):
        return self
SwigPyIterator_swigregister = _core.SwigPyIterator_swigregister
SwigPyIterator_swigregister(SwigPyIterator)

class FloatVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, FloatVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, FloatVector, name)
    __repr__ = _swig_repr

    def iterator(self) -> "swig::SwigPyIterator *":
        return _core.FloatVector_iterator(self)
    def __iter__(self):
        return self.iterator()

    def __nonzero__(self) -> "bool":
        return _core.FloatVector___nonzero__(self)

    def __bool__(self) -> "bool":
        return _core.FloatVector___bool__(self)

    def __len__(self) -> "std::vector< float >::size_type":
        return _core.FloatVector___len__(self)

    def __getslice__(self, i: 'std::vector< float >::difference_type', j: 'std::vector< float >::difference_type') -> "std::vector< float,std::allocator< float > > *":
        return _core.FloatVector___getslice__(self, i, j)

    def __setslice__(self, *args) -> "void":
        return _core.FloatVector___setslice__(self, *args)

    def __delslice__(self, i: 'std::vector< float >::difference_type', j: 'std::vector< float >::difference_type') -> "void":
        return _core.FloatVector___delslice__(self, i, j)

    def __delitem__(self, *args) -> "void":
        return _core.FloatVector___delitem__(self, *args)

    def __getitem__(self, *args) -> "std::vector< float >::value_type const &":
        return _core.FloatVector___getitem__(self, *args)

    def __setitem__(self, *args) -> "void":
        return _core.FloatVector___setitem__(self, *args)

    def pop(self) -> "std::vector< float >::value_type":
        return _core.FloatVector_pop(self)

    def append(self, x: 'std::vector< float >::value_type const &') -> "void":
        return _core.FloatVector_append(self, x)

    def empty(self) -> "bool":
        return _core.FloatVector_empty(self)

    def size(self) -> "std::vector< float >::size_type":
        return _core.FloatVector_size(self)

    def swap(self, v: 'FloatVector') -> "void":
        return _core.FloatVector_swap(self, v)

    def begin(self) -> "std::vector< float >::iterator":
        return _core.FloatVector_begin(self)

    def end(self) -> "std::vector< float >::iterator":
        return _core.FloatVector_end(self)

    def rbegin(self) -> "std::vector< float >::reverse_iterator":
        return _core.FloatVector_rbegin(self)

    def rend(self) -> "std::vector< float >::reverse_iterator":
        return _core.FloatVector_rend(self)

    def clear(self) -> "void":
        return _core.FloatVector_clear(self)

    def get_allocator(self) -> "std::vector< float >::allocator_type":
        return _core.FloatVector_get_allocator(self)

    def pop_back(self) -> "void":
        return _core.FloatVector_pop_back(self)

    def erase(self, *args) -> "std::vector< float >::iterator":
        return _core.FloatVector_erase(self, *args)

    def __init__(self, *args):
        this = _core.new_FloatVector(*args)
        try:
            self.this.append(this)
        except Exception:
            self.this = this

    def push_back(self, x: 'std::vector< float >::value_type const &') -> "void":
        return _core.FloatVector_push_back(self, x)

    def front(self) -> "std::vector< float >::value_type const &":
        return _core.FloatVector_front(self)

    def back(self) -> "std::vector< float >::value_type const &":
        return _core.FloatVector_back(self)

    def assign(self, n: 'std::vector< float >::size_type', x: 'std::vector< float >::value_type const &') -> "void":
        return _core.FloatVector_assign(self, n, x)

    def resize(self, *args) -> "void":
        return _core.FloatVector_resize(self, *args)

    def insert(self, *args) -> "void":
        return _core.FloatVector_insert(self, *args)

    def reserve(self, n: 'std::vector< float >::size_type') -> "void":
        return _core.FloatVector_reserve(self, n)

    def capacity(self) -> "std::vector< float >::size_type":
        return _core.FloatVector_capacity(self)
    __swig_destroy__ = _core.delete_FloatVector
    __del__ = lambda self: None
FloatVector_swigregister = _core.FloatVector_swigregister
FloatVector_swigregister(FloatVector)

class AgentState(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, AgentState, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, AgentState, name)
    __repr__ = _swig_repr
    __swig_setmethods__["x"] = _core.AgentState_x_set
    __swig_getmethods__["x"] = _core.AgentState_x_get
    if _newclass:
        x = _swig_property(_core.AgentState_x_get, _core.AgentState_x_set)
    __swig_setmethods__["y"] = _core.AgentState_y_set
    __swig_getmethods__["y"] = _core.AgentState_y_get
    if _newclass:
        y = _swig_property(_core.AgentState_y_get, _core.AgentState_y_set)
    __swig_setmethods__["vel_b"] = _core.AgentState_vel_b_set
    __swig_getmethods__["vel_b"] = _core.AgentState_vel_b_get
    if _newclass:
        vel_b = _swig_property(_core.AgentState_vel_b_get, _core.AgentState_vel_b_set)
    __swig_setmethods__["theta"] = _core.AgentState_theta_set
    __swig_getmethods__["theta"] = _core.AgentState_theta_get
    if _newclass:
        theta = _swig_property(_core.AgentState_theta_get, _core.AgentState_theta_set)
    __swig_setmethods__["phi"] = _core.AgentState_phi_set
    __swig_getmethods__["phi"] = _core.AgentState_phi_get
    if _newclass:
        phi = _swig_property(_core.AgentState_phi_get, _core.AgentState_phi_set)
    __swig_setmethods__["movable"] = _core.AgentState_movable_set
    __swig_getmethods__["movable"] = _core.AgentState_movable_get
    if _newclass:
        movable = _swig_property(_core.AgentState_movable_get, _core.AgentState_movable_set)
    __swig_setmethods__["crash"] = _core.AgentState_crash_set
    __swig_getmethods__["crash"] = _core.AgentState_crash_get
    if _newclass:
        crash = _swig_property(_core.AgentState_crash_get, _core.AgentState_crash_set)
    __swig_setmethods__["reach"] = _core.AgentState_reach_set
    __swig_getmethods__["reach"] = _core.AgentState_reach_get
    if _newclass:
        reach = _swig_property(_core.AgentState_reach_get, _core.AgentState_reach_set)
    __swig_setmethods__["target_x"] = _core.AgentState_target_x_set
    __swig_getmethods__["target_x"] = _core.AgentState_target_x_get
    if _newclass:
        target_x = _swig_property(_core.AgentState_target_x_get, _core.AgentState_target_x_set)
    __swig_setmethods__["target_y"] = _core.AgentState_target_y_set
    __swig_getmethods__["target_y"] = _core.AgentState_target_y_get
    if _newclass:
        target_y = _swig_property(_core.AgentState_target_y_get, _core.AgentState_target_y_set)

    def __init__(self):
        this = _core.new_AgentState()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _core.delete_AgentState
    __del__ = lambda self: None
AgentState_swigregister = _core.AgentState_swigregister
AgentState_swigregister(AgentState)

class Action(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, Action, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, Action, name)
    __repr__ = _swig_repr
    __swig_setmethods__["ctrl_vel"] = _core.Action_ctrl_vel_set
    __swig_getmethods__["ctrl_vel"] = _core.Action_ctrl_vel_get
    if _newclass:
        ctrl_vel = _swig_property(_core.Action_ctrl_vel_get, _core.Action_ctrl_vel_set)
    __swig_setmethods__["ctrl_phi"] = _core.Action_ctrl_phi_set
    __swig_getmethods__["ctrl_phi"] = _core.Action_ctrl_phi_get
    if _newclass:
        ctrl_phi = _swig_property(_core.Action_ctrl_phi_get, _core.Action_ctrl_phi_set)

    def __init__(self):
        this = _core.new_Action()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _core.delete_Action
    __del__ = lambda self: None
Action_swigregister = _core.Action_swigregister
Action_swigregister(Action)

class Observation(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, Observation, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, Observation, name)
    __repr__ = _swig_repr
    __swig_setmethods__["pos_x"] = _core.Observation_pos_x_set
    __swig_getmethods__["pos_x"] = _core.Observation_pos_x_get
    if _newclass:
        pos_x = _swig_property(_core.Observation_pos_x_get, _core.Observation_pos_x_set)
    __swig_setmethods__["pos_y"] = _core.Observation_pos_y_set
    __swig_getmethods__["pos_y"] = _core.Observation_pos_y_get
    if _newclass:
        pos_y = _swig_property(_core.Observation_pos_y_get, _core.Observation_pos_y_set)
    __swig_setmethods__["pos_theta"] = _core.Observation_pos_theta_set
    __swig_getmethods__["pos_theta"] = _core.Observation_pos_theta_get
    if _newclass:
        pos_theta = _swig_property(_core.Observation_pos_theta_get, _core.Observation_pos_theta_set)
    __swig_setmethods__["pos_target_x"] = _core.Observation_pos_target_x_set
    __swig_getmethods__["pos_target_x"] = _core.Observation_pos_target_x_get
    if _newclass:
        pos_target_x = _swig_property(_core.Observation_pos_target_x_get, _core.Observation_pos_target_x_set)
    __swig_setmethods__["pos_target_y"] = _core.Observation_pos_target_y_set
    __swig_getmethods__["pos_target_y"] = _core.Observation_pos_target_y_get
    if _newclass:
        pos_target_y = _swig_property(_core.Observation_pos_target_y_get, _core.Observation_pos_target_y_set)
    __swig_setmethods__["laser_data"] = _core.Observation_laser_data_set
    __swig_getmethods__["laser_data"] = _core.Observation_laser_data_get
    if _newclass:
        laser_data = _swig_property(_core.Observation_laser_data_get, _core.Observation_laser_data_set)

    def __init__(self):
        this = _core.new_Observation()
        try:
            self.this.append(this)
        except Exception:
            self.this = this
    __swig_destroy__ = _core.delete_Observation
    __del__ = lambda self: None
Observation_swigregister = _core.Observation_swigregister
Observation_swigregister(Observation)

class Agent(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, Agent, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, Agent, name)
    __repr__ = _swig_repr

    def __init__(self, *args):
        this = _core.new_Agent(*args)
        try:
            self.this.append(this)
        except Exception:
            self.this = this

    def reset(self, *args) -> "void":
        return _core.Agent_reset(self, *args)

    def check_AA_collisions(self, arg2: 'Agent') -> "bool":
        return _core.Agent_check_AA_collisions(self, arg2)

    def check_reach(self) -> "bool":
        return _core.Agent_check_reach(self)

    def laser_agent_agent(self, arg2: 'Agent') -> "std::vector< float >":
        return _core.Agent_laser_agent_agent(self, arg2)
    __swig_setmethods__["R_safe"] = _core.Agent_R_safe_set
    __swig_getmethods__["R_safe"] = _core.Agent_R_safe_get
    if _newclass:
        R_safe = _swig_property(_core.Agent_R_safe_get, _core.Agent_R_safe_set)
    __swig_setmethods__["R_reach"] = _core.Agent_R_reach_set
    __swig_getmethods__["R_reach"] = _core.Agent_R_reach_get
    if _newclass:
        R_reach = _swig_property(_core.Agent_R_reach_get, _core.Agent_R_reach_set)
    __swig_setmethods__["L_car"] = _core.Agent_L_car_set
    __swig_getmethods__["L_car"] = _core.Agent_L_car_get
    if _newclass:
        L_car = _swig_property(_core.Agent_L_car_get, _core.Agent_L_car_set)
    __swig_setmethods__["W_car"] = _core.Agent_W_car_set
    __swig_getmethods__["W_car"] = _core.Agent_W_car_get
    if _newclass:
        W_car = _swig_property(_core.Agent_W_car_get, _core.Agent_W_car_set)
    __swig_setmethods__["L_axis"] = _core.Agent_L_axis_set
    __swig_getmethods__["L_axis"] = _core.Agent_L_axis_get
    if _newclass:
        L_axis = _swig_property(_core.Agent_L_axis_get, _core.Agent_L_axis_set)
    __swig_setmethods__["R_laser"] = _core.Agent_R_laser_set
    __swig_getmethods__["R_laser"] = _core.Agent_R_laser_get
    if _newclass:
        R_laser = _swig_property(_core.Agent_R_laser_get, _core.Agent_R_laser_set)
    __swig_setmethods__["N_laser"] = _core.Agent_N_laser_set
    __swig_getmethods__["N_laser"] = _core.Agent_N_laser_get
    if _newclass:
        N_laser = _swig_property(_core.Agent_N_laser_get, _core.Agent_N_laser_set)
    __swig_setmethods__["K_vel"] = _core.Agent_K_vel_set
    __swig_getmethods__["K_vel"] = _core.Agent_K_vel_get
    if _newclass:
        K_vel = _swig_property(_core.Agent_K_vel_get, _core.Agent_K_vel_set)
    __swig_setmethods__["K_phi"] = _core.Agent_K_phi_set
    __swig_getmethods__["K_phi"] = _core.Agent_K_phi_get
    if _newclass:
        K_phi = _swig_property(_core.Agent_K_phi_get, _core.Agent_K_phi_set)
    __swig_setmethods__["init_x"] = _core.Agent_init_x_set
    __swig_getmethods__["init_x"] = _core.Agent_init_x_get
    if _newclass:
        init_x = _swig_property(_core.Agent_init_x_get, _core.Agent_init_x_set)
    __swig_setmethods__["init_y"] = _core.Agent_init_y_set
    __swig_getmethods__["init_y"] = _core.Agent_init_y_get
    if _newclass:
        init_y = _swig_property(_core.Agent_init_y_get, _core.Agent_init_y_set)
    __swig_setmethods__["init_theta"] = _core.Agent_init_theta_set
    __swig_getmethods__["init_theta"] = _core.Agent_init_theta_get
    if _newclass:
        init_theta = _swig_property(_core.Agent_init_theta_get, _core.Agent_init_theta_set)
    __swig_setmethods__["init_vel_b"] = _core.Agent_init_vel_b_set
    __swig_getmethods__["init_vel_b"] = _core.Agent_init_vel_b_get
    if _newclass:
        init_vel_b = _swig_property(_core.Agent_init_vel_b_get, _core.Agent_init_vel_b_set)
    __swig_setmethods__["init_phi"] = _core.Agent_init_phi_set
    __swig_getmethods__["init_phi"] = _core.Agent_init_phi_get
    if _newclass:
        init_phi = _swig_property(_core.Agent_init_phi_get, _core.Agent_init_phi_set)
    __swig_setmethods__["init_movable"] = _core.Agent_init_movable_set
    __swig_getmethods__["init_movable"] = _core.Agent_init_movable_get
    if _newclass:
        init_movable = _swig_property(_core.Agent_init_movable_get, _core.Agent_init_movable_set)
    __swig_setmethods__["init_target_x"] = _core.Agent_init_target_x_set
    __swig_getmethods__["init_target_x"] = _core.Agent_init_target_x_get
    if _newclass:
        init_target_x = _swig_property(_core.Agent_init_target_x_get, _core.Agent_init_target_x_set)
    __swig_setmethods__["init_target_y"] = _core.Agent_init_target_y_set
    __swig_getmethods__["init_target_y"] = _core.Agent_init_target_y_get
    if _newclass:
        init_target_y = _swig_property(_core.Agent_init_target_y_get, _core.Agent_init_target_y_set)
    __swig_setmethods__["state"] = _core.Agent_state_set
    __swig_getmethods__["state"] = _core.Agent_state_get
    if _newclass:
        state = _swig_property(_core.Agent_state_get, _core.Agent_state_set)
    __swig_setmethods__["action"] = _core.Agent_action_set
    __swig_getmethods__["action"] = _core.Agent_action_get
    if _newclass:
        action = _swig_property(_core.Agent_action_get, _core.Agent_action_set)
    __swig_setmethods__["laser_state"] = _core.Agent_laser_state_set
    __swig_getmethods__["laser_state"] = _core.Agent_laser_state_get
    if _newclass:
        laser_state = _swig_property(_core.Agent_laser_state_get, _core.Agent_laser_state_set)
    __swig_destroy__ = _core.delete_Agent
    __del__ = lambda self: None
Agent_swigregister = _core.Agent_swigregister
Agent_swigregister(Agent)

class World(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, World, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, World, name)
    __repr__ = _swig_repr

    def __init__(self, arg2: 'int', arg3: 'float'):
        this = _core.new_World(arg2, arg3)
        try:
            self.this.append(this)
        except Exception:
            self.this = this

    def SetWorld(self, arg2: 'int', arg3: 'float', arg4: 'float', arg5: 'float', arg6: 'float', arg7: 'float', arg8: 'float', arg9: 'int', arg10: 'float', arg11: 'float', arg12: 'float', arg13: 'float', arg14: 'float', arg15: 'float', arg16: 'float', arg17: 'bool', arg18: 'float', arg19: 'float') -> "void":
        return _core.World_SetWorld(self, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13, arg14, arg15, arg16, arg17, arg18, arg19)

    def set_action(self, arg2: 'int', arg3: 'bool', arg4: 'float', arg5: 'float') -> "void":
        return _core.World_set_action(self, arg2, arg3, arg4, arg5)

    def set_state(self, arg2: 'int', arg3: 'bool', arg4: 'float', arg5: 'float', arg6: 'float', arg7: 'float', arg8: 'float', arg9: 'bool', arg10: 'bool', arg11: 'bool', arg12: 'float', arg13: 'float') -> "void":
        return _core.World_set_state(self, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11, arg12, arg13)

    def get_state(self, arg2: 'int') -> "AgentState":
        return _core.World_get_state(self, arg2)

    def get_obs(self, arg2: 'int') -> "Observation":
        return _core.World_get_obs(self, arg2)

    def get_agent(self, arg2: 'int') -> "Agent":
        return _core.World_get_agent(self, arg2)

    def step(self) -> "void":
        return _core.World_step(self)

    def apply_action(self) -> "void":
        return _core.World_apply_action(self)

    def update_laser_state(self) -> "void":
        return _core.World_update_laser_state(self)

    def integrate_state(self) -> "void":
        return _core.World_integrate_state(self)

    def check_collisions(self) -> "void":
        return _core.World_check_collisions(self)

    def check_reach(self) -> "void":
        return _core.World_check_reach(self)
    __swig_setmethods__["total_time"] = _core.World_total_time_set
    __swig_getmethods__["total_time"] = _core.World_total_time_get
    if _newclass:
        total_time = _swig_property(_core.World_total_time_get, _core.World_total_time_set)
    __swig_setmethods__["dt"] = _core.World_dt_set
    __swig_getmethods__["dt"] = _core.World_dt_get
    if _newclass:
        dt = _swig_property(_core.World_dt_get, _core.World_dt_set)
    __swig_destroy__ = _core.delete_World
    __del__ = lambda self: None
World_swigregister = _core.World_swigregister
World_swigregister(World)

# This file is compatible with both classic and new-style classes.

