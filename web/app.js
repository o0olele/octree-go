class OctreeVisualizer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.octreeGroup = new THREE.Group();
        this.geometryGroup = new THREE.Group();
        this.pathGroup = new THREE.Group();
        this.markersGroup = new THREE.Group();
        this.waypointGroup = new THREE.Group();
        this.gltfGroup = new THREE.Group(); // For GLTF models

        this.apiBase = window.location.origin + '/api';
        this.showOctree = false;
        this.octreeData = null;

        // GLTF Loader
        this.gltfLoader = new THREE.GLTFLoader();
        this.loadedModels = []; // Store loaded GLTF models
        
        // Dynamic scene bounds tracking
        this.sceneBounds = {
            min: new THREE.Vector3(Infinity, Infinity, Infinity),
            max: new THREE.Vector3(-Infinity, -Infinity, -Infinity),
            hasGeometry: false
        };

        // Animation related properties
        this.animatingAgent = null;
        this.currentPath = null;
        this.animationProgress = 0;
        this.animationDuration = 5000; // 5 seconds

        this.init();
        this.setupEventListeners();
        this.animate();
    }

    init() {
        // Scene setup
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1a1a2e);
        this.scene.fog = new THREE.Fog(0x1a1a2e, 50, 200);

        // Camera setup
        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.camera.position.set(20, 20, 20);
        this.camera.lookAt(0, 0, 0);

        // Renderer setup
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        document.getElementById('container').appendChild(this.renderer.domElement);

        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(20, 20, 10);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        this.scene.add(directionalLight);

        // Controls
        this.setupControls();

        // Add groups to scene
        this.scene.add(this.octreeGroup);
        this.scene.add(this.geometryGroup);
        this.scene.add(this.pathGroup);
        this.scene.add(this.markersGroup);
        this.scene.add(this.waypointGroup);
        this.scene.add(this.gltfGroup);

        // Add coordinate axes
        const axesHelper = new THREE.AxesHelper(15);
        this.scene.add(axesHelper);

        // Add grid
        const gridHelper = new THREE.GridHelper(20, 20, 0x888888, 0x444444);
        this.scene.add(gridHelper);
    }

    setupControls() {
        // Simple orbit controls implementation
        let isMouseDown = false;
        let mouseX = 0, mouseY = 0;
        let targetRotationX = 0, targetRotationY = 0;
        let rotationX = 0, rotationY = 0;

        this.renderer.domElement.addEventListener('mousedown', (event) => {
            isMouseDown = true;
            mouseX = event.clientX;
            mouseY = event.clientY;
        });

        this.renderer.domElement.addEventListener('mouseup', () => {
            isMouseDown = false;
        });

        this.renderer.domElement.addEventListener('mousemove', (event) => {
            if (!isMouseDown) return;

            const deltaX = event.clientX - mouseX;
            const deltaY = event.clientY - mouseY;

            targetRotationY += deltaX * 0.01;
            targetRotationX += deltaY * 0.01;

            mouseX = event.clientX;
            mouseY = event.clientY;
        });

        this.renderer.domElement.addEventListener('wheel', (event) => {
            const distance = this.camera.position.length();
            const factor = event.deltaY > 0 ? 1.1 : 0.9;
            this.camera.position.multiplyScalar(factor);
        });

        // Update camera rotation
        const updateCamera = () => {
            rotationX += (targetRotationX - rotationX) * 0.1;
            rotationY += (targetRotationY - rotationY) * 0.1;

            const distance = this.camera.position.length();
            this.camera.position.x = distance * Math.sin(rotationY) * Math.cos(rotationX);
            this.camera.position.y = distance * Math.sin(rotationX);
            this.camera.position.z = distance * Math.cos(rotationY) * Math.cos(rotationX);
            this.camera.lookAt(0, 0, 0);

            requestAnimationFrame(updateCamera);
        };
        updateCamera();
    }

    setupEventListeners() {
        document.getElementById('initBtn').addEventListener('click', () => this.initializeOctree());
        document.getElementById('addGeometryBtn').addEventListener('click', () => this.addRandomGeometry());
        document.getElementById('buildBtn').addEventListener('click', () => this.buildOctree());
        document.getElementById('pathfindBtn').addEventListener('click', () => this.findPath());
        document.getElementById('toggleOctreeBtn').addEventListener('click', () => this.toggleOctreeVisualization());
        document.getElementById('clearBtn').addEventListener('click', () => this.clearScene());

        // GLTF related events
        document.getElementById('geometryType').addEventListener('change', (e) => {
            const gltfSection = document.getElementById('gltfUploadSection');
            gltfSection.style.display = e.target.value === 'gltf' ? 'block' : 'none';
        });
        document.getElementById('loadGltfBtn').addEventListener('click', () => this.loadGltfModel());

        // Agent checkbox toggle
        document.getElementById('enableAgent').addEventListener('change', (e) => {
            const agentControls = document.getElementById('agentControls');
            agentControls.style.display = e.target.checked ? 'block' : 'none';
            this.updateStartEndMarkers(); // Update visualization
        });

        // Real-time position updates
        ['startX', 'startY', 'startZ', 'endX', 'endY', 'endZ'].forEach(id => {
            document.getElementById(id).addEventListener('input', () => this.updateStartEndMarkers());
        });

        // Real-time agent parameter updates
        ['agentRadius', 'agentHeight'].forEach(id => {
            document.getElementById(id).addEventListener('input', () => this.updateStartEndMarkers());
        });

        // Animation speed control
        document.getElementById('animationSpeed').addEventListener('input', (e) => {
            const speed = parseInt(e.target.value);
            this.animationDuration = 10000 / speed; // Inverse relationship: higher value = faster
        });

        // Stop animation button
        document.getElementById('stopAnimationBtn').addEventListener('click', () => {
            this.stopAgentAnimation();
            this.updateStatus('Animation stopped by user');
        });

        window.addEventListener('resize', () => this.onWindowResize());

        // Initialize start/end markers
        this.updateStartEndMarkers();
    }

    async initializeOctree() {
        this.updateStatus('Calculating scene bounds...');
        
        // Calculate bounds based on all loaded geometries
        const bounds = this.calculateSceneBounds();
        
        // Calculate scene size and suggest optimal parameters
        const sceneSize = {
            x: bounds.max.x - bounds.min.x,
            y: bounds.max.y - bounds.min.y,
            z: bounds.max.z - bounds.min.z
        };
        const maxDimension = Math.max(sceneSize.x, sceneSize.y, sceneSize.z);
        
        // Automatically adjust parameters for large scenes
        let optimalMinSize = 1.0;
        let suggestedStepSize = 0.5;
        let suggestedMaxDepth = 5;
        
        if (maxDimension > 50) {
            // Large scene adjustments
            optimalMinSize = Math.max(1.0, maxDimension / 100);
            suggestedStepSize = Math.max(0.2, maxDimension / 200);
            suggestedMaxDepth = Math.min(10, Math.ceil(Math.log2(maxDimension / optimalMinSize)));
            
            // Update UI with suggestions
            document.getElementById('stepSize').value = suggestedStepSize.toFixed(1);
            this.updateStatus(`Large scene detected (${maxDimension.toFixed(1)} units). Suggested stepSize: ${suggestedStepSize.toFixed(1)}, maxDepth: ${suggestedMaxDepth}`);
        } else {
            // For complex corridor scenes (like Recast dungeons), suggest smaller stepSize
            if (maxDimension > 10) {
                suggestedStepSize = Math.max(0.1, maxDimension / 100);
                suggestedMaxDepth = Math.min(8, 6 + Math.ceil(maxDimension / 20));
                optimalMinSize = Math.max(0.5, maxDimension / 50);
                
                document.getElementById('stepSize').value = suggestedStepSize.toFixed(1);
                this.updateStatus(`Complex scene detected (${maxDimension.toFixed(1)} units). For corridor/dungeon scenes, suggested stepSize: ${suggestedStepSize.toFixed(1)}, maxDepth: ${suggestedMaxDepth}. Use smaller stepSize (0.1-0.3) for better precision in narrow corridors.`);
            } else {
                this.updateStatus('Initializing octree...');
            }
        }
        
        try {
            const response = await fetch(`${this.apiBase}/init`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    bounds: bounds,
                    max_depth: suggestedMaxDepth,
                    min_size: optimalMinSize
                })
            });

            if (response.ok) {
                document.getElementById('buildBtn').disabled = false;
                this.updateStatus(`Octree initialized with bounds: [${bounds.min.x.toFixed(1)}, ${bounds.min.y.toFixed(1)}, ${bounds.min.z.toFixed(1)}] to [${bounds.max.x.toFixed(1)}, ${bounds.max.y.toFixed(1)}, ${bounds.max.z.toFixed(1)}]. Scene size: ${maxDimension.toFixed(1)} units. MinSize: ${optimalMinSize.toFixed(1)}, MaxDepth: ${suggestedMaxDepth}`);
                
                // Reset button state
                document.getElementById('initBtn').style.backgroundColor = '';
                document.getElementById('initBtn').textContent = 'Initialize Octree';
            } else {
                throw new Error('Failed to initialize octree');
            }
        } catch (error) {
            this.updateStatus(`Error: ${error.message}`);
        }
    }
    
    calculateSceneBounds() {
        let minX = Infinity, minY = Infinity, minZ = Infinity;
        let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
        let hasGeometry = false;
        
        // Check GLTF models
        this.gltfGroup.traverse((child) => {
            if (child.isMesh && child.geometry) {
                const box = new THREE.Box3().setFromObject(child);
                if (!box.isEmpty()) {
                    minX = Math.min(minX, box.min.x);
                    minY = Math.min(minY, box.min.y);
                    minZ = Math.min(minZ, box.min.z);
                    maxX = Math.max(maxX, box.max.x);
                    maxY = Math.max(maxY, box.max.y);
                    maxZ = Math.max(maxZ, box.max.z);
                    hasGeometry = true;
                }
            }
        });
        
        // Check other geometries (boxes, capsules, etc.)
        this.geometryGroup.traverse((child) => {
            if (child.isMesh) {
                const box = new THREE.Box3().setFromObject(child);
                if (!box.isEmpty()) {
                    minX = Math.min(minX, box.min.x);
                    minY = Math.min(minY, box.min.y);
                    minZ = Math.min(minZ, box.min.z);
                    maxX = Math.max(maxX, box.max.x);
                    maxY = Math.max(maxY, box.max.y);
                    maxZ = Math.max(maxZ, box.max.z);
                    hasGeometry = true;
                }
            }
        });
        
        // If no geometry found, use default bounds
        if (!hasGeometry) {
            const size = 50;
            return {
                min: { x: -size, y: -size, z: -size },
                max: { x: size, y: size, z: size }
            };
        }
        
        // Add some padding around the geometry (20% of the largest dimension)
        const sizeX = maxX - minX;
        const sizeY = maxY - minY;
        const sizeZ = maxZ - minZ;
        const maxSize = Math.max(sizeX, sizeY, sizeZ);
        const padding = Math.max(maxSize * 0.2, 2.0); // At least 2 units padding
        
        return {
            min: { 
                x: minX - padding, 
                y: minY - padding, 
                z: minZ - padding 
            },
            max: { 
                x: maxX + padding, 
                y: maxY + padding, 
                z: maxZ + padding 
            }
        };
    }

    async addRandomGeometry() {
        const type = document.getElementById('geometryType').value;
        this.updateStatus(`Adding ${type}...`);

        try {
            let geometryData;
            let visualGeometry;

            if (type === 'box') {
                geometryData = {
                    center: {
                        x: (Math.random() - 0.5) * 16,
                        y: (Math.random() - 0.5) * 16,
                        z: (Math.random() - 0.5) * 16
                    },
                    size: {
                        x: Math.random() * 3 + 1,
                        y: Math.random() * 3 + 1,
                        z: Math.random() * 3 + 1
                    }
                };

                // Create visual representation
                const geometry = new THREE.BoxGeometry(
                    geometryData.size.x,
                    geometryData.size.y,
                    geometryData.size.z
                );
                const material = new THREE.MeshLambertMaterial({
                    color: new THREE.Color().setHSL(Math.random(), 0.7, 0.5),
                    transparent: true,
                    opacity: 0.8
                });
                visualGeometry = new THREE.Mesh(geometry, material);
                visualGeometry.position.copy(geometryData.center);
                visualGeometry.castShadow = true;
                visualGeometry.receiveShadow = true;

            } else if (type === 'capsule') {
                const start = {
                    x: (Math.random() - 0.5) * 16,
                    y: (Math.random() - 0.5) * 16,
                    z: (Math.random() - 0.5) * 16
                };
                const end = {
                    x: start.x + (Math.random() - 0.5) * 6,
                    y: start.y + (Math.random() - 0.5) * 6,
                    z: start.z + (Math.random() - 0.5) * 6
                };

                geometryData = {
                    start: start,
                    end: end,
                    radius: Math.random() * 1 + 0.5
                };

                // Create visual representation (simplified as cylinder)
                const height = Math.sqrt(
                    Math.pow(end.x - start.x, 2) +
                    Math.pow(end.y - start.y, 2) +
                    Math.pow(end.z - start.z, 2)
                );
                const geometry = new THREE.CylinderGeometry(
                    geometryData.radius,
                    geometryData.radius,
                    height,
                    8
                );
                const material = new THREE.MeshLambertMaterial({
                    color: new THREE.Color().setHSL(Math.random(), 0.7, 0.5),
                    transparent: true,
                    opacity: 0.8
                });
                visualGeometry = new THREE.Mesh(geometry, material);

                const center = {
                    x: (start.x + end.x) / 2,
                    y: (start.y + end.y) / 2,
                    z: (start.z + end.z) / 2
                };
                visualGeometry.position.copy(center);
                visualGeometry.castShadow = true;
                visualGeometry.receiveShadow = true;

            } else if (type === 'mesh') {
                // Create a simple triangle mesh (tetrahedron)
                const center = {
                    x: (Math.random() - 0.5) * 16,
                    y: (Math.random() - 0.5) * 16,
                    z: (Math.random() - 0.5) * 16
                };
                const size = Math.random() * 2 + 1;

                const triangles = [
                    {
                        a: { x: center.x, y: center.y + size, z: center.z },
                        b: { x: center.x - size, y: center.y - size, z: center.z + size },
                        c: { x: center.x + size, y: center.y - size, z: center.z + size }
                    },
                    {
                        a: { x: center.x, y: center.y + size, z: center.z },
                        b: { x: center.x + size, y: center.y - size, z: center.z + size },
                        c: { x: center.x, y: center.y - size, z: center.z - size }
                    }
                ];

                const response = await fetch(`${this.apiBase}/mesh`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(triangles)
                });

                if (response.ok) {
                    // Create visual representation
                    const geometry = new THREE.TetrahedronGeometry(size);
                    const material = new THREE.MeshLambertMaterial({
                        color: new THREE.Color().setHSL(Math.random(), 0.7, 0.5),
                        transparent: true,
                        opacity: 0.8
                    });
                    visualGeometry = new THREE.Mesh(geometry, material);
                    visualGeometry.position.copy(center);
                    visualGeometry.castShadow = true;
                    visualGeometry.receiveShadow = true;

                    this.geometryGroup.add(visualGeometry);
                    this.updateStatus(`Added triangle mesh with ${triangles.length} triangles`);
                    this.updateBoundsDisplay();
                    return;
                }
            }

            // Send geometry to server
            const response = await fetch(`${this.apiBase}/geometry`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    type: type,
                    data: geometryData
                })
            });

            if (response.ok) {
                this.geometryGroup.add(visualGeometry);
                this.updateStatus(`Added ${type} successfully`);
                this.updateBoundsDisplay();
            } else {
                throw new Error(`Failed to add ${type}`);
            }

        } catch (error) {
            this.updateStatus(`Error: ${error.message}`);
        }
    }

    async buildOctree() {
        this.updateStatus('Building octree...');

        try {
            const response = await fetch(`${this.apiBase}/build`, {
                method: 'POST'
            });

            if (response.ok) {
                document.getElementById('pathfindBtn').disabled = false;
                await this.loadOctreeData();
                this.updateStatus('Octree built successfully');
            } else {
                throw new Error('Failed to build octree');
            }
        } catch (error) {
            this.updateStatus(`Error: ${error.message}`);
        }
    }

    async loadOctreeData() {
        try {
            const response = await fetch(`${this.apiBase}/octree`);
            if (response.ok) {
                this.octreeData = await response.json();
                if (this.showOctree) {
                    this.visualizeOctree();
                }
            }
        } catch (error) {
            console.error('Failed to load octree data:', error);
        }
    }

    visualizeOctree() {
        this.octreeGroup.clear();

        if (!this.octreeData || !this.showOctree) return;

        const traverseNode = (node) => {
            if (!node) return;

            // Only visualize occupied leaf nodes
            if (node.is_leaf && node.is_occupied) {
                const size = {
                    x: node.bounds.max.x - node.bounds.min.x,
                    y: node.bounds.max.y - node.bounds.min.y,
                    z: node.bounds.max.z - node.bounds.min.z
                };

                const center = {
                    x: (node.bounds.min.x + node.bounds.max.x) / 2,
                    y: (node.bounds.min.y + node.bounds.max.y) / 2,
                    z: (node.bounds.min.z + node.bounds.max.z) / 2
                };

                const geometry = new THREE.BoxGeometry(size.x, size.y, size.z);
                const material = new THREE.MeshBasicMaterial({
                    color: 0xff6b6b,
                    wireframe: true,
                    transparent: true,
                    opacity: 0.3
                });

                const cube = new THREE.Mesh(geometry, material);
                cube.position.copy(center);
                this.octreeGroup.add(cube);
            }

            // Traverse children
            if (node.children) {
                node.children.forEach(child => traverseNode(child));
            }
        };

        traverseNode(this.octreeData.root);
    }

    async findPath() {
        if (this.octreeData === null) {
            this.updateStatus('Please build octree first');
            return;
        }

        const startX = parseFloat(document.getElementById('startX').value);
        const startY = parseFloat(document.getElementById('startY').value);
        const startZ = parseFloat(document.getElementById('startZ').value);
        const endX = parseFloat(document.getElementById('endX').value);
        const endY = parseFloat(document.getElementById('endY').value);
        const endZ = parseFloat(document.getElementById('endZ').value);
        const stepSize = parseFloat(document.getElementById('stepSize').value);

        const enableAgent = document.getElementById('enableAgent').checked;
        let agentRadius = 0;
        let agentHeight = 0;

        if (enableAgent) {
            agentRadius = parseFloat(document.getElementById('agentRadius').value) || 0.5;
            agentHeight = parseFloat(document.getElementById('agentHeight').value) || 1.8;
        }

        // Get selected algorithm
        const algorithm = document.getElementById('pathfindingAlgorithm').value;

        this.updateStatus(`Finding path using ${algorithm.toUpperCase()}...`);

        try {
            const requestBody = {
                start: { x: startX, y: startY, z: startZ },
                end: { x: endX, y: endY, z: endZ },
                step_size: stepSize,
                algorithm: algorithm
            };

            if (enableAgent) {
                requestBody.agent_radius = agentRadius;
                requestBody.agent_height = agentHeight;
            }

            const response = await fetch(`${this.apiBase}/pathfind`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(requestBody)
            });

            if (response.ok) {
                const result = await response.json();
                
                if (result.found && result.path && result.path.length > 0) {
                    this.visualizePath(result.path, 
                        { x: startX, y: startY, z: startZ }, 
                        { x: endX, y: endY, z: endZ });
                    
                    // 显示调试信息
                    let debugMsg = `Path found with ${result.length} points`;
                    if (result.debug) {
                        debugMsg += `. Algorithm: ${result.debug.algorithm || algorithm}`;
                        debugMsg += `, stepSize=${result.debug.stepSize}`;
                        if (result.debug.agentRadius > 0) {
                            debugMsg += `, agent=${result.debug.agentRadius}x${result.debug.agentHeight}`;
                            debugMsg += `, startValidAgent=${result.debug.startValidAgent}`;
                            debugMsg += `, endValidAgent=${result.debug.endValidAgent}`;
                        } else {
                            debugMsg += `, startValid=${result.debug.startValid}`;
                            debugMsg += `, endValid=${result.debug.endValid}`;
                        }
                    }
                    this.updateStatus(debugMsg);
                } else {
                    let debugMsg = 'No path found';
                    if (result.debug) {
                        debugMsg += `. Algorithm: ${result.debug.algorithm || algorithm}`;
                        debugMsg += `, stepSize=${result.debug.stepSize}`;
                        if (result.debug.agentRadius > 0) {
                            debugMsg += `, agent=${result.debug.agentRadius}x${result.debug.agentHeight}`;
                            debugMsg += `, startValidAgent=${result.debug.startValidAgent}`;
                            debugMsg += `, endValidAgent=${result.debug.endValidAgent}`;
                        } else {
                            debugMsg += `, startValid=${result.debug.startValid}`;
                            debugMsg += `, endValid=${result.debug.endValid}`;
                        }
                    }
                    this.updateStatus(debugMsg);
                }
            } else {
                throw new Error('Pathfinding request failed');
            }
        } catch (error) {
            this.updateStatus(`Error: ${error.message}`);
        }
    }

    updateStartEndMarkers() {
        // Clear existing start/end markers but keep waypoints
        this.clearStartEndMarkers();

        // Get current positions
        const start = {
            x: parseFloat(document.getElementById('startX').value) || 0,
            y: parseFloat(document.getElementById('startY').value) || 0,
            z: parseFloat(document.getElementById('startZ').value) || 0
        };

        const end = {
            x: parseFloat(document.getElementById('endX').value) || 0,
            y: parseFloat(document.getElementById('endY').value) || 0,
            z: parseFloat(document.getElementById('endZ').value) || 0
        };

        // Check if agent is enabled
        const enableAgent = document.getElementById('enableAgent').checked;
        const agentRadius = parseFloat(document.getElementById('agentRadius').value) || 0.5;
        const agentHeight = parseFloat(document.getElementById('agentHeight').value) || 1.8;

        // Always create new markers
        if (enableAgent && agentRadius > 0 && agentHeight > 0) {
            // Create capsule representations for start and end
            this.createAgentMarker(start, 0x00ff00, agentRadius, agentHeight, 'start');
            this.createAgentMarker(end, 0xff0000, agentRadius, agentHeight, 'end');
        } else {
            // Create simple sphere markers
            this.createSimpleMarker(start, 0x00ff00, 0.3, 'start');
            this.createSimpleMarker(end, 0xff0000, 0.3, 'end');
        }
    }

    clearStartEndMarkers() {
        // Remove only start/end markers, keep waypoints
        const toRemove = [];
        this.markersGroup.children.forEach(child => {
            if (child.userData && (child.userData.type === 'start' || child.userData.type === 'end')) {
                toRemove.push(child);
            } else if (!child.userData) {
                // If no userData, assume it's a start/end marker (old markers without userData)
                toRemove.push(child);
            }
        });
        toRemove.forEach(marker => this.markersGroup.remove(marker));
    }

    createAgentMarker(position, color, radius, height, label) {
        // Create capsule representation (cylinder + 2 hemispheres)
        const group = new THREE.Group();

        // Main cylinder
        const cylinderGeometry = new THREE.CylinderGeometry(radius, radius, height, 8);
        const cylinderMaterial = new THREE.MeshBasicMaterial({
            color: color,
            transparent: true,
            opacity: 0.6,
            wireframe: true
        });
        const cylinder = new THREE.Mesh(cylinderGeometry, cylinderMaterial);
        group.add(cylinder);

        // Top hemisphere
        const topSphereGeometry = new THREE.SphereGeometry(radius, 8, 8, 0, Math.PI * 2, 0, Math.PI / 2);
        const topSphere = new THREE.Mesh(topSphereGeometry, cylinderMaterial);
        topSphere.position.y = height / 2;
        group.add(topSphere);

        // Bottom hemisphere
        const bottomSphereGeometry = new THREE.SphereGeometry(radius, 8, 8, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2);
        const bottomSphere = new THREE.Mesh(bottomSphereGeometry, cylinderMaterial);
        bottomSphere.position.y = -height / 2;
        group.add(bottomSphere);

        // Center point marker
        const centerMarker = new THREE.Mesh(
            new THREE.SphereGeometry(0.1, 8, 8),
            new THREE.MeshBasicMaterial({ color: color })
        );
        group.add(centerMarker);

        group.position.set(position.x, position.y, position.z);
        group.userData = { type: label };
        this.markersGroup.add(group);
    }

    createSimpleMarker(position, color, size, label) {
        const marker = new THREE.Mesh(
            new THREE.SphereGeometry(size, 16, 16),
            new THREE.MeshBasicMaterial({ color: color })
        );
        marker.position.set(position.x, position.y, position.z);
        marker.userData = { type: label };
        this.markersGroup.add(marker);
    }

    visualizePath(path, start, end) {
        this.pathGroup.clear();
        this.waypointGroup.clear();

        if (!path || path.length === 0) return;

        // Store current path for animation
        this.currentPath = path;

        // Create path line
        const points = path.map(p => new THREE.Vector3(p.x, p.y, p.z));
        const geometry = new THREE.BufferGeometry().setFromPoints(points);
        const material = new THREE.LineBasicMaterial({
            color: 0x00ff00,
            linewidth: 5
        });
        const line = new THREE.Line(geometry, material);
        this.pathGroup.add(line);

        // Add waypoint markers to separate group
        path.forEach((point, index) => {
            const sphereGeometry = new THREE.SphereGeometry(0.08, 8, 8);
            const sphereMaterial = new THREE.MeshBasicMaterial({
                color: index === 0 ? 0x00ff00 : index === path.length - 1 ? 0xff0000 : 0xffff00
            });
            const sphere = new THREE.Mesh(sphereGeometry, sphereMaterial);
            sphere.position.set(point.x, point.y, point.z);
            sphere.userData = { type: 'waypoint' };
            this.waypointGroup.add(sphere);
        });

        // Update start and end markers with current agent settings
        this.updateStartEndMarkers();

        // Start agent animation if enabled
        this.startAgentAnimation();
    }

    toggleOctreeVisualization() {
        this.showOctree = !this.showOctree;
        if (this.showOctree) {
            this.visualizeOctree();
        } else {
            this.octreeGroup.clear();
        }

        document.getElementById('toggleOctreeBtn').textContent =
            this.showOctree ? 'Hide Octree' : 'Show Octree';
    }

    startAgentAnimation() {
        const enableAgent = document.getElementById('enableAgent').checked;
        if (!enableAgent || !this.currentPath || this.currentPath.length < 2) {
            return;
        }

        // Stop existing animation
        this.stopAgentAnimation();

        // Create animated agent
        const agentRadius = parseFloat(document.getElementById('agentRadius').value) || 0.5;
        const agentHeight = parseFloat(document.getElementById('agentHeight').value) || 1.8;

        this.animatingAgent = this.createAnimatedAgent(agentRadius, agentHeight);
        this.scene.add(this.animatingAgent);

        // Start animation
        this.animationProgress = 0;
        this.animationStartTime = Date.now();

        // Show stop animation button
        document.getElementById('stopAnimationBtn').style.display = 'block';

        this.updateStatus('Agent animation started...');
    }

    stopAgentAnimation() {
        if (this.animatingAgent) {
            this.scene.remove(this.animatingAgent);
            this.animatingAgent = null;
        }
        this.animationProgress = 0;

        // Hide stop animation button
        document.getElementById('stopAnimationBtn').style.display = 'none';
    }

    createAnimatedAgent(radius, height) {
        const group = new THREE.Group();

        // Main cylinder
        const cylinderGeometry = new THREE.CylinderGeometry(radius, radius, height, 8);
        const cylinderMaterial = new THREE.MeshBasicMaterial({
            color: 0x0088ff,
            transparent: true,
            opacity: 0.8
        });
        const cylinder = new THREE.Mesh(cylinderGeometry, cylinderMaterial);
        group.add(cylinder);

        // Top hemisphere
        const topSphereGeometry = new THREE.SphereGeometry(radius, 8, 8, 0, Math.PI * 2, 0, Math.PI / 2);
        const topSphere = new THREE.Mesh(topSphereGeometry, cylinderMaterial);
        topSphere.position.y = height / 2;
        group.add(topSphere);

        // Bottom hemisphere
        const bottomSphereGeometry = new THREE.SphereGeometry(radius, 8, 8, 0, Math.PI * 2, Math.PI / 2, Math.PI / 2);
        const bottomSphere = new THREE.Mesh(bottomSphereGeometry, cylinderMaterial);
        bottomSphere.position.y = -height / 2;
        group.add(bottomSphere);

        return group;
    }

    updateAgentAnimation() {
        if (!this.animatingAgent || !this.currentPath || this.currentPath.length < 2) {
            return;
        }

        const elapsed = Date.now() - this.animationStartTime;
        this.animationProgress = Math.min(elapsed / this.animationDuration, 1);

        // Calculate position along path
        const position = this.getPositionAlongPath(this.animationProgress);
        this.animatingAgent.position.copy(position);

        // Update status
        const percentage = Math.round(this.animationProgress * 100);
        this.updateStatus(`Agent moving... ${percentage}%`);

        // Animation complete
        if (this.animationProgress >= 1) {
            this.updateStatus('Agent reached destination!');
            setTimeout(() => {
                this.stopAgentAnimation();
                this.updateStatus('Animation complete. Agent ready for next path.');
            }, 1000);
        }
    }

    getPositionAlongPath(progress) {
        if (!this.currentPath || this.currentPath.length < 2) {
            return new THREE.Vector3(0, 0, 0);
        }

        // Calculate total path length
        let totalLength = 0;
        const segmentLengths = [];
        for (let i = 0; i < this.currentPath.length - 1; i++) {
            const start = new THREE.Vector3(this.currentPath[i].x, this.currentPath[i].y, this.currentPath[i].z);
            const end = new THREE.Vector3(this.currentPath[i + 1].x, this.currentPath[i + 1].y, this.currentPath[i + 1].z);
            const length = start.distanceTo(end);
            segmentLengths.push(length);
            totalLength += length;
        }

        // Find which segment we're in
        const targetDistance = progress * totalLength;
        let currentDistance = 0;

        for (let i = 0; i < segmentLengths.length; i++) {
            const segmentEnd = currentDistance + segmentLengths[i];

            if (targetDistance <= segmentEnd) {
                // We're in this segment
                const segmentProgress = (targetDistance - currentDistance) / segmentLengths[i];
                const start = new THREE.Vector3(this.currentPath[i].x, this.currentPath[i].y, this.currentPath[i].z);
                const end = new THREE.Vector3(this.currentPath[i + 1].x, this.currentPath[i + 1].y, this.currentPath[i + 1].z);

                return start.lerp(end, segmentProgress);
            }

            currentDistance = segmentEnd;
        }

        // Return last point if something goes wrong
        const lastPoint = this.currentPath[this.currentPath.length - 1];
        return new THREE.Vector3(lastPoint.x, lastPoint.y, lastPoint.z);
    }

    clearScene() {
        this.octreeGroup.clear();
        this.geometryGroup.clear();
        this.pathGroup.clear();
        this.markersGroup.clear();
        this.waypointGroup.clear();
        this.gltfGroup.clear(); // Clear GLTF models
        this.stopAgentAnimation();

        this.currentPath = null;
        this.loadedModels = []; // Clear loaded models array

        document.getElementById('buildBtn').disabled = true;
        document.getElementById('pathfindBtn').disabled = true;
        document.getElementById('toggleOctreeBtn').textContent = 'Show Octree';
        this.showOctree = false;
        
        // Reset button states
        document.getElementById('initBtn').style.backgroundColor = '';
        document.getElementById('initBtn').textContent = 'Initialize Octree';

        this.updateStatus('Scene cleared');

        // Restore start/end markers
        setTimeout(() => this.updateStartEndMarkers(), 100);
        
        // Update bounds display (will hide it since no geometry)
        this.updateBoundsDisplay();
    }

    updateStatus(message) {
        document.getElementById('status').textContent = message;
        console.log(message);
    }

    onWindowResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }

    animate() {
        requestAnimationFrame(() => this.animate());

        // Update agent animation
        this.updateAgentAnimation();

        // Add some rotation animation to geometries
        //this.geometryGroup.children.forEach((child, index) => {
        //    child.rotation.y += 0.005 * (index % 2 === 0 ? 1 : -1);
        //});

        this.renderer.render(this.scene, this.camera);
    }

    async loadGltfModel() {
        const fileInput = document.getElementById('gltfFileInput');
        const scaleInput = document.getElementById('gltfScale');

        if (!fileInput.files || fileInput.files.length === 0) {
            this.updateStatus('Please select a GLTF file');
            return;
        }

        const file = fileInput.files[0];
        const scale = parseFloat(scaleInput.value) || 1.0;

        this.updateStatus('Loading GLTF model...');

        try {
            // Create URL for the file
            const url = URL.createObjectURL(file);

            // Load GLTF model
            const gltf = await new Promise((resolve, reject) => {
                this.gltfLoader.load(url, resolve, undefined, reject);
            });

            // Clean up URL
            URL.revokeObjectURL(url);

            // Scale the model
            gltf.scene.scale.setScalar(scale);

            // Add to scene for visualization
            this.gltfGroup.add(gltf.scene);
            this.loadedModels.push(gltf.scene);

            // Extract triangles from the model
            const triangles = this.extractTrianglesFromGltf(gltf.scene, scale);

            this.updateStatus(`Extracted ${triangles.length} triangles from GLTF model`);

            // Send triangles to backend
            if (triangles.length > 0) {
                await this.sendTrianglesToBackend(triangles);
                
                // Calculate bounds of the loaded model
                const modelBounds = this.calculateModelBounds(gltf.scene);
                
                this.updateStatus(`GLTF model loaded: ${triangles.length} triangles. Bounds: [${modelBounds.min.x.toFixed(1)}, ${modelBounds.min.y.toFixed(1)}, ${modelBounds.min.z.toFixed(1)}] to [${modelBounds.max.x.toFixed(1)}, ${modelBounds.max.y.toFixed(1)}, ${modelBounds.max.z.toFixed(1)}]. Re-initialize octree to update bounds.`);
                
                // Enable build button but suggest re-initializing
                document.getElementById('pathfindBtn').disabled = false;
                document.getElementById('initBtn').style.backgroundColor = '#ff6b6b'; // Highlight init button
                document.getElementById('initBtn').textContent = 'Re-initialize Octree';
                
                // Update bounds display
                this.updateBoundsDisplay();
            } else {
                this.updateStatus('Warning: No triangles extracted from GLTF model');
            }

        } catch (error) {
            this.updateStatus(`Error loading GLTF: ${error.message}`);
            console.error('GLTF loading error:', error);
        }
    }

    extractTrianglesFromGltf(scene, scale = 1.0) {
        const triangles = [];

        scene.traverse((child) => {
            if (child.isMesh && child.geometry) {
                const geometry = child.geometry;
                const matrix = child.matrixWorld;

                // Get position attribute
                const positions = geometry.attributes.position;
                if (!positions) return;

                // Get indices if available
                const indices = geometry.index;

                if (indices) {
                    // Indexed geometry
                    for (let i = 0; i < indices.count; i += 3) {
                        const a = indices.getX(i);
                        const b = indices.getX(i + 1);
                        const c = indices.getX(i + 2);

                        const triangle = this.createTriangleFromIndices(positions, matrix, a, b, c, scale);
                        if (triangle) triangles.push(triangle);
                    }
                } else {
                    // Non-indexed geometry
                    for (let i = 0; i < positions.count; i += 3) {
                        const triangle = this.createTriangleFromIndices(positions, matrix, i, i + 1, i + 2, scale);
                        if (triangle) triangles.push(triangle);
                    }
                }
            }
        });

        return triangles;
    }

    createTriangleFromIndices(positions, matrix, indexA, indexB, indexC, scale) {
        try {
            const vectorA = new THREE.Vector3();
            const vectorB = new THREE.Vector3();
            const vectorC = new THREE.Vector3();

            vectorA.fromBufferAttribute(positions, indexA).applyMatrix4(matrix);
            vectorB.fromBufferAttribute(positions, indexB).applyMatrix4(matrix);
            vectorC.fromBufferAttribute(positions, indexC).applyMatrix4(matrix);

            // Apply additional scaling if needed
            vectorA.multiplyScalar(scale);
            vectorB.multiplyScalar(scale);
            vectorC.multiplyScalar(scale);

            return {
                a: { x: vectorA.x, y: vectorA.y, z: vectorA.z },
                b: { x: vectorB.x, y: vectorB.y, z: vectorB.z },
                c: { x: vectorC.x, y: vectorC.y, z: vectorC.z }
            };
        } catch (error) {
            console.warn('Error creating triangle:', error);
            return null;
        }
    }

    async sendTrianglesToBackend(triangles) {
        if (!triangles || triangles.length === 0) {
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/mesh`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(triangles)
            });

            if (!response.ok) {
                throw new Error(`Failed to send triangles: ${response.status}`);
            }

            const result = await response.json();
            console.log('Triangles sent to backend:', result);

        } catch (error) {
            this.updateStatus(`Error sending triangles: ${error.message}`);
            throw error;
        }
    }

    calculateModelBounds(scene) {
        let minX = Infinity, minY = Infinity, minZ = Infinity;
        let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;

        scene.traverse((child) => {
            if (child.isMesh && child.geometry) {
                const box = new THREE.Box3().setFromObject(child);
                if (!box.isEmpty()) {
                    minX = Math.min(minX, box.min.x);
                    minY = Math.min(minY, box.min.y);
                    minZ = Math.min(minZ, box.min.z);
                    maxX = Math.max(maxX, box.max.x);
                    maxY = Math.max(maxY, box.max.y);
                    maxZ = Math.max(maxZ, box.max.z);
                }
            }
        });

        return {
            min: new THREE.Vector3(minX, minY, minZ),
            max: new THREE.Vector3(maxX, maxY, maxZ)
        };
    }

    updateBoundsDisplay() {
        const bounds = this.calculateSceneBounds();
        const boundsInfo = document.getElementById('boundsInfo');
        const boundsText = document.getElementById('boundsText');
        
        // Check if we have any geometry
        let hasGeometry = false;
        this.gltfGroup.traverse((child) => {
            if (child.isMesh && child.geometry) hasGeometry = true;
        });
        this.geometryGroup.traverse((child) => {
            if (child.isMesh) hasGeometry = true;
        });
        
        if (hasGeometry) {
            boundsInfo.style.display = 'block';
            const minStr = `Min: [${bounds.min.x.toFixed(1)}, ${bounds.min.y.toFixed(1)}, ${bounds.min.z.toFixed(1)}]`;
            const maxStr = `Max: [${bounds.max.x.toFixed(1)}, ${bounds.max.y.toFixed(1)}, ${bounds.max.z.toFixed(1)}]`;
            const sizeX = bounds.max.x - bounds.min.x;
            const sizeY = bounds.max.y - bounds.min.y;
            const sizeZ = bounds.max.z - bounds.min.z;
            const sizeStr = `Size: [${sizeX.toFixed(1)}, ${sizeY.toFixed(1)}, ${sizeZ.toFixed(1)}]`;
            boundsText.innerHTML = `${minStr}<br>${maxStr}<br>${sizeStr}`;
        } else {
            boundsInfo.style.display = 'none';
        }
    }
}

// Initialize the application
const app = new OctreeVisualizer();