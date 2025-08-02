package math32

import (
	"container/list"
	"sync"
)

// Cache 是一个泛型 LRU 缓存结构
type Cache[K comparable, V any] struct {
	capacity int
	ll       *list.List          // 双向链表存储元素
	cache    map[K]*list.Element // 哈希表用于快速查找
	mu       sync.RWMutex        // 互斥锁保证线程安全
	hits     int64               // 缓存命中次数
	misses   int64               // 缓存未命中次数
}

// entry 是缓存中的条目结构
type entry[K comparable, V any] struct {
	key   K
	value V
}

// NewCache 创建一个新的 LRU 缓存，capacity 必须大于 0
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

// Get 从缓存中获取值，如果存在则返回值和 true，否则返回零值和 false
func (c *Cache[K, V]) Get(key K) (value V, ok bool) {
	c.mu.RLock()
	defer c.mu.RUnlock()

	el, ok := c.cache[key]
	if !ok {
		c.misses++
		return
	}

	// 将访问的元素移到链表头部（最近使用）
	c.ll.MoveToFront(el)
	c.hits++
	return el.Value.(*entry[K, V]).value, true
}

// Put 将值放入缓存
func (c *Cache[K, V]) Put(key K, value V) {
	c.mu.Lock()
	defer c.mu.Unlock()

	// 如果键已存在，更新值并移到链表头部
	if el, ok := c.cache[key]; ok {
		el.Value.(*entry[K, V]).value = value
		c.ll.MoveToFront(el)
		return
	}

	// 创建新条目并添加到链表头部
	newEntry := &entry[K, V]{key, value}
	element := c.ll.PushFront(newEntry)
	c.cache[key] = element

	// 如果超过容量，移除链表尾部（最久未使用）的元素
	if c.ll.Len() > c.capacity {
		c.removeOldest()
	}
}

// removeOldest 移除最久未使用的元素
func (c *Cache[K, V]) removeOldest() {
	el := c.ll.Back()
	if el == nil {
		return
	}

	c.ll.Remove(el)
	kv := el.Value.(*entry[K, V])
	delete(c.cache, kv.key)
}

// Len 返回当前缓存中的元素数量
func (c *Cache[K, V]) Len() int {
	c.mu.RLock()
	defer c.mu.RUnlock()
	return c.ll.Len()
}

// Capacity 返回缓存的容量
func (c *Cache[K, V]) Capacity() int {
	return c.capacity
}

// Clear 清空缓存
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
