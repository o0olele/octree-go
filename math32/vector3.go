package math32

import (
	"fmt"
	"math"
)

// Vector3 表示3D向量
type Vector3 struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
	Z float32 `json:"z"`
}

func (v Vector3) Add(other Vector3) Vector3 {
	return Vector3{v.X + other.X, v.Y + other.Y, v.Z + other.Z}
}

func (v Vector3) Sub(other Vector3) Vector3 {
	return Vector3{v.X - other.X, v.Y - other.Y, v.Z - other.Z}
}

func (v Vector3) Mul(s float32) Vector3 {
	return Vector3{v.X * s, v.Y * s, v.Z * s}
}

func (v Vector3) Scale(s float32) Vector3 {
	return Vector3{v.X * s, v.Y * s, v.Z * s}
}

func (v Vector3) Distance(other Vector3) float32 {
	diff := v.Sub(other)
	return float32(math.Sqrt(float64(diff.X*diff.X + diff.Y*diff.Y + diff.Z*diff.Z)))
}

func (v Vector3) Length() float32 {
	return float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
}

func (v Vector3) Dot(other Vector3) float32 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

func (v Vector3) Cross(other Vector3) Vector3 {
	return Vector3{
		v.Y*other.Z - v.Z*other.Y,
		v.Z*other.X - v.X*other.Z,
		v.X*other.Y - v.Y*other.X,
	}
}

func (v Vector3) Normalize() Vector3 {
	len := v.Length()
	if len == 0 {
		return Vector3{0, 0, 0}
	}
	return v.Mul(1.0 / len)
}

func (v Vector3) String() string {
	return fmt.Sprintf("[%2f,%2f,%2f]", v.X, v.Y, v.Z)
}

func (v Vector3) Hash() uint64 {
	return uint64(v.X*100)<<32 | uint64(v.Y*100)<<16 | uint64(v.Z*100)
}
