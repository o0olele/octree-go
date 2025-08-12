package geometry

import "github.com/o0olele/octree-go/math32"

// RayTriangle checks if the ray intersects with the triangle (based on Möller–Trumbore)
func RayTriangle(origin, dir math32.Vector3, tri *Triangle) (bool, float32) {
	const eps = 1e-6
	v0 := tri.A
	v1 := tri.B
	v2 := tri.C

	e1 := v1.Sub(v0)
	e2 := v2.Sub(v0)
	pvec := dir.Cross(e2)
	det := e1.Dot(pvec)
	if det > -eps && det < eps {
		return false, 0
	}
	invDet := 1.0 / det
	tvec := origin.Sub(v0)
	u := tvec.Dot(pvec) * invDet
	if u < 0 || u > 1 {
		return false, 0
	}
	qvec := tvec.Cross(e1)
	v := dir.Dot(qvec) * invDet
	if v < 0 || u+v > 1 {
		return false, 0
	}
	t := e2.Dot(qvec) * invDet
	if t <= eps {
		return false, 0
	}
	return true, t
}

// RayAABB checks if the ray intersects with the AABB (slab method), returns [tmin, tmax] and whether it intersects
func RayAABB(origin, dir math32.Vector3, aabb AABB) (float32, float32, bool) {
	const eps = 1e-6
	tmin := float32(-math32.MaxFloat32)
	tmax := float32(math32.MaxFloat32)

	// X axis
	if math32.Abs(dir.X) < eps {
		if origin.X < aabb.Min.X || origin.X > aabb.Max.X {
			return 0, 0, false
		}
	} else {
		tx1 := (aabb.Min.X - origin.X) / dir.X
		tx2 := (aabb.Max.X - origin.X) / dir.X
		if tx1 > tx2 {
			tx1, tx2 = tx2, tx1
		}
		if tx1 > tmin {
			tmin = tx1
		}
		if tx2 < tmax {
			tmax = tx2
		}
		if tmin > tmax {
			return 0, 0, false
		}
	}

	// Y axis
	if math32.Abs(dir.Y) < eps {
		if origin.Y < aabb.Min.Y || origin.Y > aabb.Max.Y {
			return 0, 0, false
		}
	} else {
		ty1 := (aabb.Min.Y - origin.Y) / dir.Y
		ty2 := (aabb.Max.Y - origin.Y) / dir.Y
		if ty1 > ty2 {
			ty1, ty2 = ty2, ty1
		}
		if ty1 > tmin {
			tmin = ty1
		}
		if ty2 < tmax {
			tmax = ty2
		}
		if tmin > tmax {
			return 0, 0, false
		}
	}

	// Z axis
	if math32.Abs(dir.Z) < eps {
		if origin.Z < aabb.Min.Z || origin.Z > aabb.Max.Z {
			return 0, 0, false
		}
	} else {
		tz1 := (aabb.Min.Z - origin.Z) / dir.Z
		tz2 := (aabb.Max.Z - origin.Z) / dir.Z
		if tz1 > tz2 {
			tz1, tz2 = tz2, tz1
		}
		if tz1 > tmin {
			tmin = tz1
		}
		if tz2 < tmax {
			tmax = tz2
		}
		if tmin > tmax {
			return 0, 0, false
		}
	}

	if tmax < 0 {
		return 0, 0, false
	}
	if tmin < 0 {
		tmin = 0
	}
	return tmin, tmax, true
}
