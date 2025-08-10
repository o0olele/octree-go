package math32

import "math"

func Min[T float32 | int32](a, b T) T {
	if a < b {
		return a
	}
	return b
}

func Max[T float32 | int32](a, b T) T {
	if a > b {
		return a
	}
	return b
}

func Abs(a float32) float32 {
	if a < 0 {
		return -a
	}
	return a
}

func CeilToInt(a float32) int {
	return int(math.Ceil(float64(a)))
}

func RoundToInt(a float32) int {
	return int(math.Round(float64(a)))
}

func Sqrt(a float32) float32 {
	return float32(math.Sqrt(float64(a)))
}
