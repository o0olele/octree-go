package math32

import (
	"container/list"
	"sync"
)

// Cache is a generic LRU cache structure.
type Cache[K comparable, V any] struct {
	capacity int
	ll       *list.List          // Doubly linked list to store elements
	cache    map[K]*list.Element // Hash table for fast lookup
	mu       sync.RWMutex        // Mutex to ensure thread safety
	hits     int64               // Cache hit count
	misses   int64               // Cache miss count
}

// entry is the entry structure in the cache.
type entry[K comparable, V any] struct {
	key   K
	value V
}

// NewCache creates a new LRU cache, capacity must be greater than 0.
func NewCache[K comparable, V any](capacity int) *Cache[K, V] {
	if capacity <= 0 {
		panic("lru: capacity must be greater than 0")
	}
	return &Cache[K, V]{
		capacity: capacity,
		ll:       list.New(),
		cache:    make(map[K]*list.Element),
		hits:     0,
		misses:   0,
	}
}

// Get gets the value from the cache, if it exists, returns the value and true, otherwise returns zero value and false.
func (c *Cache[K, V]) Get(key K) (value V, ok bool) {
	c.mu.RLock()
	defer c.mu.RUnlock()

	el, ok := c.cache[key]
	if !ok {
		c.misses++
		return
	}

	// Move the accessed element to the head of the list (most recently used).
	c.ll.MoveToFront(el)
	c.hits++
	return el.Value.(*entry[K, V]).value, true
}

// Put puts the value into the cache.
func (c *Cache[K, V]) Put(key K, value V) {
	c.mu.Lock()
	defer c.mu.Unlock()

	// If the key exists, update the value and move it to the head of the list.
	if el, ok := c.cache[key]; ok {
		el.Value.(*entry[K, V]).value = value
		c.ll.MoveToFront(el)
		return
	}

	// Create a new entry and add it to the head of the list.
	newEntry := &entry[K, V]{key, value}
	element := c.ll.PushFront(newEntry)
	c.cache[key] = element

	// If the capacity is exceeded, remove the element at the tail of the list (least recently used).
	if c.ll.Len() > c.capacity {
		c.removeOldest()
	}
}

// removeOldest removes the least recently used element.
func (c *Cache[K, V]) removeOldest() {
	el := c.ll.Back()
	if el == nil {
		return
	}

	c.ll.Remove(el)
	kv := el.Value.(*entry[K, V])
	delete(c.cache, kv.key)
}

// Len returns the number of elements in the cache.
func (c *Cache[K, V]) Len() int {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.ll.Len()
}

// Capacity returns the capacity of the cache.
func (c *Cache[K, V]) Capacity() int {
	return c.capacity
}

// Clear clears the cache.
func (c *Cache[K, V]) Clear() {
	c.mu.Lock()
	defer c.mu.Unlock()

	c.ll.Init()
	for k := range c.cache {
		delete(c.cache, k)
	}

	c.hits = 0
	c.misses = 0
}

// Hits 返回缓存命中次数
func (c *Cache[K, V]) Hits() int64 {
	return c.hits
}

// Misses 返回缓存未命中次数
func (c *Cache[K, V]) Misses() int64 {
	return c.misses
}

// HitRate 获取缓存命中率
func (c *Cache[K, V]) HitRate() float64 {
	total := c.hits + c.misses
	if total == 0 {
		return 0.0
	}
	return float64(c.hits) / float64(total)
}

type CacheStats struct {
	Hits     int64
	Misses   int64
	HitRate  float64
	Capacity int
	Size     int
}

// GetStats 获取缓存统计信息
func (c *Cache[K, V]) GetStats() CacheStats {
	return CacheStats{
		Hits:     c.hits,
		Misses:   c.misses,
		HitRate:  c.HitRate(),
		Capacity: c.capacity,
		Size:     c.ll.Len(),
	}
}
