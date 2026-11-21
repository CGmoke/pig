"""Test contact against the authored visible triangles, not PhysX hulls."""
import numpy as np


def meshes_touch(first, second):
    """Return whether two meshes share a point (including coplanar contact)."""
    eps = 1e-10  # floating-point roundoff, not a contact gap allowance in mm
    if not len(first) or not len(second):
        return False
    lo, hi = first.min(axis=1), first.max(axis=1)
    other_lo, other_hi = second.min(axis=1), second.max(axis=1)
    mask = ((other_lo <= hi.max(axis=0) + eps) & (other_hi >= lo.min(axis=0) - eps)).all(axis=1)
    second, other_lo, other_hi = second[mask], other_lo[mask], other_hi[mask]
    if not len(second):
        return False
    near = ((lo <= other_hi.max(axis=0) + eps) & (hi >= other_lo.min(axis=0) - eps)).all(axis=1)
    for i in np.flatnonzero(near):
        paired = ((lo[i] <= other_hi + eps) & (hi[i] >= other_lo - eps)).all(axis=1)
        if np.any(paired) and _triangle_overlap(first[i], second[paired], eps):
            return True
    return False


def _triangle_overlap(triangle, others, eps):
    # Separating axes: both normals, all edge crosses, and edge normals in
    # each plane (needed for coplanar triangles). Work in local coordinates.
    a, b = triangle - triangle[0], others - triangle[0]
    ea = np.roll(a, -1, axis=0) - a
    eb = np.roll(b, -1, axis=1) - b
    n = np.cross(ea[0], ea[1])
    m = np.cross(eb[:, 0], eb[:, 1])
    if np.linalg.norm(n) <= eps**2:
        return False
    axes = np.concatenate((np.broadcast_to(n, (len(b), 1, 3)), m[:, None],
                           np.cross(ea[:, None], eb[:, None]).reshape(-1, 9, 3),
                           np.broadcast_to(np.cross(n, ea), b.shape),
                           np.cross(m[:, None], eb)), axis=1)
    axis_length = np.linalg.norm(axes, axis=-1)
    axes = np.divide(axes, axis_length[..., None], out=np.zeros_like(axes),
                     where=axis_length[..., None] > eps**2)
    p, q = axes @ a.T, axes @ b.transpose(0, 2, 1)
    overlap = ((p.max(axis=-1) + eps >= q.min(axis=-1)) &
               (q.max(axis=-1) + eps >= p.min(axis=-1))).all(axis=-1)
    return bool(np.any(overlap & (np.linalg.norm(m, axis=-1) > eps**2)))


class VisibleMesh:
    """Cache authored face topology; put triangles in the current world pose on demand."""
    def __init__(self, prim, Usd, UsdGeom):
        self.meshes = []
        for part in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if not part.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(part)
            if mesh.ComputeVisibility() == 'invisible' or mesh.ComputePurpose() not in ('default', 'render'):
                continue
            points = np.array(mesh.GetPointsAttr().Get(), dtype=float)
            indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get(), dtype=int)
            counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get(), dtype=int)
            holes = set(mesh.GetHoleIndicesAttr().Get() or ())
            triangles = []
            offset = 0
            for face, size in enumerate(counts):
                if face not in holes:
                    triangles.extend((indices[offset], indices[offset+k], indices[offset+k+1])
                                     for k in range(1, size-1))
                offset += size
            if triangles:
                self.meshes.append((part, points[np.asarray(triangles)]))
        if not self.meshes:
            raise ValueError(f'No visible triangle mesh: {prim.GetPath()}')

    def triangles(self, cache):
        arrays = []
        for part, triangles in self.meshes:
            pose = np.asarray(cache.GetLocalToWorldTransform(part)).T
            arrays.append(triangles @ pose[:3, :3].T + pose[:3, 3])
        return np.concatenate(arrays)
