"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class lin_con_t(object):
    __slots__ = ["utime", "m", "n", "m_times_n", "A", "b"]

    def __init__(self):
        self.utime = 0
        self.m = 0
        self.n = 0
        self.m_times_n = 0
        self.A = []
        self.b = []

    def encode(self):
        buf = BytesIO()
        buf.write(lin_con_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qiii", self.utime, self.m, self.n, self.m_times_n))
        buf.write(struct.pack('>%dd' % self.m_times_n, *self.A[:self.m_times_n]))
        buf.write(struct.pack('>%dd' % self.m, *self.b[:self.m]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != lin_con_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return lin_con_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = lin_con_t()
        self.utime, self.m, self.n, self.m_times_n = struct.unpack(">qiii", buf.read(20))
        self.A = struct.unpack('>%dd' % self.m_times_n, buf.read(self.m_times_n * 8))
        self.b = struct.unpack('>%dd' % self.m, buf.read(self.m * 8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if lin_con_t in parents: return 0
        tmphash = (0xf6b473cc75899cab) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if lin_con_t._packed_fingerprint is None:
            lin_con_t._packed_fingerprint = struct.pack(">Q", lin_con_t._get_hash_recursive([]))
        return lin_con_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
