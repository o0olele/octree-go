package builder

import (
	"bytes"
	"compress/gzip"
	"encoding/binary"
	"encoding/gob"
	"fmt"
	"io"
	"os"
	"time"

	"github.com/o0olele/octree-go/geometry"
)

var useGzip = true

func UseGzip(use bool) {
	useGzip = use
}

func Load(filename string) (*NavigationData, error) {

	content, err := os.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to read file: %v", err)
	}

	if useGzip {
		content = Decompress(content)
	}

	buf := bytes.NewBuffer(content)
	// 读取文件头
	var header FileHeader
	if err := binary.Read(buf, binary.LittleEndian, &header); err != nil {
		return nil, fmt.Errorf("failed to read header: %v", err)
	}

	// 验证文件格式
	if header.Magic != NAVIGATION_FILE_MAGIC {
		return nil, fmt.Errorf("invalid file format: magic number mismatch")
	}

	if header.Version != NAVIGATION_FILE_VERSION {
		return nil, fmt.Errorf("unsupported file version: %d", header.Version)
	}

	navData := &NavigationData{}

	if err := binary.Read(buf, binary.LittleEndian, &navData.Bounds); err != nil {
		return nil, fmt.Errorf("failed to read bounds: %v", err)
	}

	if err := binary.Read(buf, binary.LittleEndian, &navData.MaxDepth); err != nil {
		return nil, fmt.Errorf("failed to read max depth: %v", err)
	}

	if err := binary.Read(buf, binary.LittleEndian, &navData.MinSize); err != nil {
		return nil, fmt.Errorf("failed to read min size: %v", err)
	}

	if err := binary.Read(buf, binary.LittleEndian, &navData.StepSize); err != nil {
		return nil, fmt.Errorf("failed to read step size: %v", err)
	}

	var nodeCount uint32
	if err := binary.Read(buf, binary.LittleEndian, &nodeCount); err != nil {
		return nil, fmt.Errorf("failed to read node count: %v", err)
	}

	navData.Nodes = make([]CompactNode, nodeCount)

	for i := range navData.Nodes {
		if err := binary.Read(buf, binary.LittleEndian, &navData.Nodes[i]); err != nil {
			return nil, fmt.Errorf("failed to read node: %v", err)
		}
	}

	var edgeCount uint32
	if err := binary.Read(buf, binary.LittleEndian, &edgeCount); err != nil {
		return nil, fmt.Errorf("failed to read edge count: %v", err)
	}

	navData.Edges = make([]CompactEdge, edgeCount)

	for i := range navData.Edges {
		if err := binary.Read(buf, binary.LittleEndian, &navData.Edges[i]); err != nil {
			return nil, fmt.Errorf("failed to read edge: %v", err)
		}
	}

	var mortonIndexCount uint32
	if err := binary.Read(buf, binary.LittleEndian, &mortonIndexCount); err != nil {
		return nil, fmt.Errorf("failed to read morton index count: %v", err)
	}

	navData.MortonIndex = make([]int32, mortonIndexCount)

	for i := range navData.MortonIndex {
		if err := binary.Read(buf, binary.LittleEndian, &navData.MortonIndex[i]); err != nil {
			return nil, fmt.Errorf("failed to read morton index: %v", err)
		}
	}

	if err := binary.Read(buf, binary.LittleEndian, &navData.MortonResolution); err != nil {
		return nil, fmt.Errorf("failed to read morton resolution: %v", err)
	}

	var geometryDataSize uint32
	if err := binary.Read(buf, binary.LittleEndian, &geometryDataSize); err != nil {
		return nil, fmt.Errorf("failed to read geometry data size: %v", err)
	}

	navData.GeometryData = make([]byte, geometryDataSize)

	if err := binary.Read(buf, binary.LittleEndian, navData.GeometryData); err != nil {
		return nil, fmt.Errorf("failed to read geometry data: %v", err)
	}

	return navData, nil
}

func Save(navData *NavigationData, filename string) error {
	// 验证数据完整性
	if err := navData.Validate(); err != nil {
		return fmt.Errorf("invalid navigation data: %v", err)
	}

	buf := bytes.NewBuffer(nil)
	// 写入文件头
	header := FileHeader{
		Magic:   NAVIGATION_FILE_MAGIC,
		Version: NAVIGATION_FILE_VERSION,
	}

	if err := binary.Write(buf, binary.LittleEndian, header); err != nil {
		return fmt.Errorf("failed to write header: %v", err)
	}

	// write bounds
	if err := binary.Write(buf, binary.LittleEndian, navData.Bounds); err != nil {
		return fmt.Errorf("failed to write base info: %v", err)
	}

	// write max depth
	if err := binary.Write(buf, binary.LittleEndian, navData.MaxDepth); err != nil {
		return fmt.Errorf("failed to write max depth: %v", err)
	}

	// write min size
	if err := binary.Write(buf, binary.LittleEndian, navData.MinSize); err != nil {
		return fmt.Errorf("failed to write min size: %v", err)
	}

	// write step size
	if err := binary.Write(buf, binary.LittleEndian, navData.StepSize); err != nil {
		return fmt.Errorf("failed to write step size: %v", err)
	}

	// write node count
	if err := binary.Write(buf, binary.LittleEndian, uint32(len(navData.Nodes))); err != nil {
		return fmt.Errorf("failed to write node count: %v", err)
	}

	for _, node := range navData.Nodes {
		// write node id
		if err := binary.Write(buf, binary.LittleEndian, node); err != nil {
			return fmt.Errorf("failed to write node id: %v", err)
		}
	}

	// write edge count
	if err := binary.Write(buf, binary.LittleEndian, uint32(len(navData.Edges))); err != nil {
		return fmt.Errorf("failed to write edge count: %v", err)
	}

	for _, edge := range navData.Edges {
		// write edge id
		if err := binary.Write(buf, binary.LittleEndian, edge); err != nil {
			return fmt.Errorf("failed to write edge id: %v", err)
		}
	}

	// write morton index count
	if err := binary.Write(buf, binary.LittleEndian, uint32(len(navData.MortonIndex))); err != nil {
		return fmt.Errorf("failed to write morton index count: %v", err)
	}

	// write morton index
	for _, idx := range navData.MortonIndex {
		if err := binary.Write(buf, binary.LittleEndian, uint32(idx)); err != nil {
			return fmt.Errorf("failed to write morton index: %v", err)
		}
	}

	// write morton resolution
	if err := binary.Write(buf, binary.LittleEndian, navData.MortonResolution); err != nil {
		return fmt.Errorf("failed to write morton resolution: %v", err)
	}

	// write geometry data size
	if err := binary.Write(buf, binary.LittleEndian, uint32(len(navData.GeometryData))); err != nil {
		return fmt.Errorf("failed to write geometry data size: %v", err)
	}

	// write geometry data
	if err := binary.Write(buf, binary.LittleEndian, navData.GeometryData); err != nil {
		return fmt.Errorf("failed to write geometry data: %v", err)
	}

	content := buf.Bytes()

	if useGzip {
		content = Compress(content)
	}

	err := os.WriteFile(filename, content, 0644)
	if err != nil {
		return fmt.Errorf("failed to write file: %v", err)
	}

	return nil
}

func Decompress(content []byte) []byte {

	buf := bytes.NewBuffer(content)
	gzipReader, err := gzip.NewReader(buf)
	if err != nil {
		return nil
	}

	decompressed, err := io.ReadAll(gzipReader)
	if err != nil {
		return nil
	}

	return decompressed
}

func Compress(content []byte) []byte {
	buf := bytes.NewBuffer(nil)
	gzipWriter := gzip.NewWriter(buf)
	gzipWriter.Write(content)
	gzipWriter.Close()
	return buf.Bytes()
}

// BuildAndSave 构建并保存导航数据（一步到位）
func BuildAndSave(bounds geometry.AABB, maxDepth uint8, minSize float32, stepSize float32, triangles []geometry.Triangle, filename string) error {
	// 创建构建器
	builder := NewBuilder(bounds, maxDepth, minSize, stepSize)

	// 添加几何体
	builder.AddTriangles(triangles)

	// 构建导航数据
	navData, err := builder.Build(nil)
	if err != nil {
		return fmt.Errorf("failed to build navigation data: %v", err)
	}

	// 保存到文件
	if err := Save(navData, filename); err != nil {
		return fmt.Errorf("failed to save navigation data: %v", err)
	}

	return nil
}

// GetFileInfo 获取导航文件信息
func GetFileInfo(filename string) (*NavigationFileInfo, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to open file: %v", err)
	}
	defer file.Close()

	// 获取文件大小
	fileInfo, err := file.Stat()
	if err != nil {
		return nil, fmt.Errorf("failed to get file info: %v", err)
	}

	// 读取文件头
	var header FileHeader
	if err := binary.Read(file, binary.LittleEndian, &header); err != nil {
		return nil, fmt.Errorf("failed to read header: %v", err)
	}

	// 验证文件格式
	if header.Magic != NAVIGATION_FILE_MAGIC {
		return nil, fmt.Errorf("invalid file format")
	}

	// 读取导航数据（只读取基本信息）
	decoder := gob.NewDecoder(file)
	var navData NavigationData
	if err := decoder.Decode(&navData); err != nil {
		return nil, fmt.Errorf("failed to decode navigation data: %v", err)
	}

	return &NavigationFileInfo{
		Filename:  filename,
		FileSize:  fileInfo.Size(),
		Version:   header.Version,
		Bounds:    navData.Bounds,
		MaxDepth:  navData.MaxDepth,
		MinSize:   navData.MinSize,
		StepSize:  navData.StepSize,
		NodeCount: len(navData.Nodes),
		EdgeCount: len(navData.Edges),
		DataSize:  navData.GetDataSize(),
		ModTime:   fileInfo.ModTime(),
	}, nil
}

// NavigationFileInfo 导航文件信息
type NavigationFileInfo struct {
	Filename  string        `json:"filename"`
	FileSize  int64         `json:"file_size"`
	Version   uint32        `json:"version"`
	Bounds    geometry.AABB `json:"bounds"`
	MaxDepth  uint8         `json:"max_depth"`
	MinSize   float32       `json:"min_size"`
	StepSize  float32       `json:"step_size"`
	NodeCount int           `json:"node_count"`
	EdgeCount int           `json:"edge_count"`
	DataSize  int           `json:"data_size"`
	ModTime   time.Time     `json:"mod_time"`
}

// ValidateNavigationFile 验证导航文件的完整性
func ValidateNavigationFile(filename string) error {
	navData, err := Load(filename)
	if err != nil {
		return err
	}

	return navData.Validate()
}

// BatchBuild 批量构建多个导航文件
func BatchBuild(configs []BuildConfig) error {
	for i, config := range configs {
		fmt.Printf("Building navigation file %d/%d: %s\n", i+1, len(configs), config.OutputFile)

		err := BuildAndSave(
			config.Bounds,
			config.MaxDepth,
			config.MinSize,
			config.StepSize,
			config.Geometries,
			config.OutputFile,
		)

		if err != nil {
			return fmt.Errorf("failed to build %s: %v", config.OutputFile, err)
		}
	}

	return nil
}

// BuildConfig 构建配置
type BuildConfig struct {
	Bounds     geometry.AABB       `json:"bounds"`
	MaxDepth   uint8               `json:"max_depth"`
	MinSize    float32             `json:"min_size"`
	StepSize   float32             `json:"step_size"`
	Geometries []geometry.Triangle `json:"geometries"`
	OutputFile string              `json:"output_file"`
}
