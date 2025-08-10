package math32

type Bitmap []uint64

// Set sets the bit x in the bitmap and grows it if necessary.
func (dst *Bitmap) Set(x uint32) {
	blkAt := int(x >> 6)
	bitAt := int(x % 64)
	if size := len(*dst); blkAt >= size {
		dst.grow(blkAt)
	}

	(*dst)[blkAt] |= (1 << bitAt)
}

// Remove removes the bit x from the bitmap, but does not shrink it.
func (dst *Bitmap) Remove(x uint32) {
	if blkAt := int(x >> 6); blkAt < len(*dst) {
		bitAt := int(x % 64)
		(*dst)[blkAt] &^= (1 << bitAt)
	}
}

// Contains checks whether a value is contained in the bitmap or not.
func (dst Bitmap) Contains(x uint32) bool {
	blkAt := int(x >> 6)
	if size := len(dst); blkAt >= size {
		return false
	}

	bitAt := int(x % 64)
	return (dst[blkAt] & (1 << bitAt)) > 0
}

// Grow grows the bitmap size until we reach the desired bit.
func (dst *Bitmap) Grow(desiredBit uint32) {
	dst.grow(int(desiredBit >> 6))
}

// grow grows the size of the bitmap until we reach the desired block offset
func (dst *Bitmap) grow(blkAt int) {
	if len(*dst) > blkAt {
		return
	}

	// If there's space, resize the slice without copying.
	if cap(*dst) > blkAt {
		*dst = (*dst)[:blkAt+1]
		return
	}

	old := *dst
	*dst = make(Bitmap, blkAt+1, resize(cap(old), blkAt+1))
	copy(*dst, old)
}

// resize calculates the new required capacity and a new index
func resize(capacity, v int) int {
	const threshold = 256
	if v < threshold {
		v |= v >> 1
		v |= v >> 2
		v |= v >> 4
		v |= v >> 8
		v |= v >> 16
		v++
		return int(v)
	}

	if capacity < threshold {
		capacity = threshold
	}

	for 0 < capacity && capacity < (v+1) {
		capacity += (capacity + 3*threshold) / 4
	}
	return capacity
}
