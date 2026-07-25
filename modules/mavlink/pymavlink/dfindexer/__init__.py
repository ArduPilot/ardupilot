try:
    from .dfindexer_cy import build_offsets
    available = True
except ImportError:
    build_offsets = None
    available = False
