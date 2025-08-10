package math32

type Vector3i struct {
	X int32
	Y int32
	Z int32
}

func (v Vector3i) Add(other Vector3i) Vector3i {
	return Vector3i{v.X + other.X, v.Y + other.Y, v.Z + other.Z}
}

func (v Vector3i) Sub(other Vector3i) Vector3i {
	return Vector3i{v.X - other.X, v.Y - other.Y, v.Z - other.Z}
}

func (v Vector3i) Max(other Vector3i) Vector3i {
	return Vector3i{Max(v.X, other.X), Max(v.Y, other.Y), Max(v.Z, other.Z)}
}

func (v Vector3i) Min(other Vector3i) Vector3i {
	return Vector3i{Min(v.X, other.X), Min(v.Y, other.Y), Min(v.Z, other.Z)}
}
