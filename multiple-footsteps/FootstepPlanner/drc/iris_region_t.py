"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import drc.lin_con_t

import bot_core.position_3d_t

class iris_region_t(object):
    __slots__ = ["utime", "lin_con", "seed_pose"]

    def __init__(self):
        self.utime = 0
        self.lin_con = drc.lin_con_t()
        self.seed_pose = bot_core.position_3d_t()

    def encode(self):
        buf = BytesIO()
        buf.write(iris_region_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.utime))
        assert self.lin_con._get_packed_fingerprint() == drc.lin_con_t._get_packed_fingerprint()
        self.lin_con._encode_one(buf)
        assert self.seed_pose._get_packed_fingerprint() == bot_core.position_3d_t._get_packed_fingerprint()
        self.seed_pose._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != iris_region_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return iris_region_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = iris_region_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.lin_con = drc.lin_con_t._decode_one(buf)
        self.seed_pose = bot_core.position_3d_t._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if iris_region_t in parents: return 0
        newparents = parents + [iris_region_t]
        tmphash = (0x891f24f23b1c1f7a+ drc.lin_con_t._get_hash_recursive(newparents)+ bot_core.position_3d_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if iris_region_t._packed_fingerprint is None:
            iris_region_t._packed_fingerprint = struct.pack(">Q", iris_region_t._get_hash_recursive([]))
        return iris_region_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

