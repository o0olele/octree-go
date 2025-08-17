package math32

import "math"

// Min returns the minimum of two values.
func Min[T float32 | int32](a, b T) T {
	if a < b {
		return a
	}
	return b
}

// Max returns the maximum of two values.
func Max[T float32 | int32](a, b T) T {
	if a > b {
		return a
	}
	return b
}

// Abs returns the absolute value of a float32.
func Abs(a float32) float32 {
	if a < 0 {
		return -a
	}
	return a
}

// CeilToInt returns the ceiling of a float32 as an integer.
func CeilToInt(a float32) int {
	return int(math.Ceil(float64(a)))
}

// RoundToInt returns the round of a float32 as an integer.
func RoundToInt(a float32) int {
	return int(math.Round(float64(a)))
}

// Sqrt returns the square root of a float32.
func Sqrt(a float32) float32 {
	return float32(math.Sqrt(float64(a)))
}
