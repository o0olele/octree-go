package math32

import (
	"fmt"
	"math"
)

// Vector3 represents a 3D vector.
type Vector3 struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
	Z float32 `json:"z"`
}

// Add adds two vectors.
func (v Vector3) Add(other Vector3) Vector3 {
	return Vector3{v.X + other.X, v.Y + other.Y, v.Z + other.Z}
}

// Sub subtracts two vectors.
func (v Vector3) Sub(other Vector3) Vector3 {
	return Vector3{v.X - other.X, v.Y - other.Y, v.Z - other.Z}
}

// Mul multiplies a vector by a scalar.
func (v Vector3) Mul(s float32) Vector3 {
	return Vector3{v.X * s, v.Y * s, v.Z * s}
}

// Scale scales a vector by a scalar.
func (v Vector3) Scale(s float32) Vector3 {
	return Vector3{v.X * s, v.Y * s, v.Z * s}
}

// Distance calculates the distance between two vectors.
func (v Vector3) Distance(other Vector3) float32 {
	diff := v.Sub(other)
	return float32(math.Sqrt(float64(diff.X*diff.X + diff.Y*diff.Y + diff.Z*diff.Z)))
}

// DistanceSquared calculates the squared distance between two vectors.
func (v Vector3) DistanceSquared(other Vector3) float32 {
	diff := v.Sub(other)
	return diff.X*diff.X + diff.Y*diff.Y + diff.Z*diff.Z
}

// LengthSquared calculates the squared length of a vector.
func (v Vector3) LengthSquared() float32 {
	return v.X*v.X + v.Y*v.Y + v.Z*v.Z
}

// Length calculates the length of a vector.
func (v Vector3) Length() float32 {
	return float32(math.Sqrt(float64(v.X*v.X + v.Y*v.Y + v.Z*v.Z)))
}

// Dot calculates the dot product of two vectors.
func (v Vector3) Dot(other Vector3) float32 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

// Cross calculates the cross product of two vectors.
func (v Vector3) Cross(other Vector3) Vector3 {
	return Vector3{
		v.Y*other.Z - v.Z*other.Y,
		v.Z*other.X - v.X*other.Z,
		v.X*other.Y - v.Y*other.X,
	}
}

// Normalize normalizes a vector.
func (v Vector3) Normalize() Vector3 {
	len := v.Length()
	if len == 0 {
		return Vector3{0, 0, 0}
	}
	return v.Mul(1.0 / len)
}

// String returns a string representation of the vector.
func (v Vector3) String() string {
	return fmt.Sprintf("[%2f,%2f,%2f]", v.X, v.Y, v.Z)
}

// Hash returns a hash of the vector.
func (v Vector3) Hash() uint64 {
	return uint64(v.X*100)<<32 | uint64(v.Y*100)<<16 | uint64(v.Z*100)
}

// Get returns the value of the vector at the given index.
func (v Vector3) Get(i int) float32 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	case 2:
		return v.Z
	}
	return 0
}
