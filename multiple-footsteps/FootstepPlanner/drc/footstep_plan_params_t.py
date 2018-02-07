"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class footstep_plan_params_t(object):
    __slots__ = ["utime", "max_num_steps", "min_num_steps", "min_step_width", "nom_step_width", "max_step_width", "nom_forward_step", "max_forward_step", "nom_upward_step", "nom_downward_step", "planning_mode", "behavior", "map_mode", "leading_foot"]

    MODE_AUTO = 0
    MODE_SPLINE = 1
    MODE_NO_SPLINE = 2
    BEHAVIOR_WALKING = 0
    BEHAVIOR_CRAWLING = 1
    BEHAVIOR_BDI_WALKING = 2
    BEHAVIOR_BDI_STEPPING = 3
    TERRAIN_HEIGHTS_AND_NORMALS = 0
    TERRAIN_HEIGHTS_Z_NORMALS = 1
    FOOT_PLANE = 2
    HORIZONTAL_PLANE = 3
    LEAD_RIGHT = 1
    LEAD_LEFT = 0
    LEAD_AUTO = -1

    def __init__(self):
        self.utime = 0
        self.max_num_steps = 0
        self.min_num_steps = 0
        self.min_step_width = 0.0
        self.nom_step_width = 0.0
        self.max_step_width = 0.0
        self.nom_forward_step = 0.0
        self.max_forward_step = 0.0
        self.nom_upward_step = 0.0
        self.nom_downward_step = 0.0
        self.planning_mode = 0
        self.behavior = 0
        self.map_mode = 0
        self.leading_foot = 0

    def encode(self):
        buf = BytesIO()
        buf.write(footstep_plan_params_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qiifffffffbbbb", self.utime, self.max_num_steps, self.min_num_steps, self.min_step_width, self.nom_step_width, self.max_step_width, self.nom_forward_step, self.max_forward_step, self.nom_upward_step, self.nom_downward_step, self.planning_mode, self.behavior, self.map_mode, self.leading_foot))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != footstep_plan_params_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return footstep_plan_params_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = footstep_plan_params_t()
        self.utime, self.max_num_steps, self.min_num_steps, self.min_step_width, self.nom_step_width, self.max_step_width, self.nom_forward_step, self.max_forward_step, self.nom_upward_step, self.nom_downward_step, self.planning_mode, self.behavior, self.map_mode, self.leading_foot = struct.unpack(">qiifffffffbbbb", buf.read(48))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if footstep_plan_params_t in parents: return 0
        tmphash = (0xd25f9f60bea19ca5) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if footstep_plan_params_t._packed_fingerprint is None:
            footstep_plan_params_t._packed_fingerprint = struct.pack(">Q", footstep_plan_params_t._get_hash_recursive([]))
        return footstep_plan_params_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
