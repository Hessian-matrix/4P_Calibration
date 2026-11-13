const THREE_MODULE = './vendor/three-0.160.0/three.module.js';
const ORBIT_CONTROLS_MODULE = './vendor/three-0.160.0/OrbitControls.js';

// Exported poses are camera-from-reference: p_camera = R * p_reference + t.
// The displayed camera pose in the common reference frame is its inverse:
// R_world_camera = Rᵀ and p_world_camera = -Rᵀt.
function cameraPose(item, index) {
  const quaternion = item?.pose?.quaternion_wxyz;
  const translation = item?.pose?.translation_m;
  if (!Array.isArray(quaternion) || quaternion.length !== 4 || !quaternion.every(Number.isFinite)
    || !Array.isArray(translation) || translation.length !== 3 || !translation.every(Number.isFinite)) return null;

  const norm = Math.hypot(...quaternion);
  if (norm < 1e-12) return null;
  const [w, x, y, z] = quaternion.map((value) => value / norm);
  const cameraFromReference = [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
  const rotation = cameraFromReference[0].map((_, column) => cameraFromReference.map((row) => row[column]));
  const position = rotation.map((row) => -row.reduce((sum, value, axis) => sum + value * translation[axis], 0));
  const suppliedName = item.direction ?? item.camera_id ?? item.id;
  return { name: suppliedName == null || suppliedName === '' ? `相机 ${index + 1}` : String(suppliedName), rotation, position };
}

function setStyles(element, styles) {
  Object.assign(element.style, styles);
  return element;
}

function makeLabel(THREE, text, worldHeight) {
  const canvas = document.createElement('canvas');
  const context = canvas.getContext('2d');
  const ratio = Math.min(window.devicePixelRatio || 1, 2);
  const fontSize = Math.round(28 * ratio);
  context.font = `600 ${fontSize}px system-ui, sans-serif`;
  const horizontalPadding = Math.round(16 * ratio);
  canvas.width = Math.ceil(context.measureText(text).width + horizontalPadding * 2);
  canvas.height = Math.round(48 * ratio);
  context.font = `600 ${fontSize}px system-ui, sans-serif`;
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.fillStyle = 'rgba(246, 250, 247, 0.92)';
  context.fillRect(0, 0, canvas.width, canvas.height);
  context.strokeStyle = '#b7cdbd';
  context.lineWidth = Math.max(1, ratio);
  context.strokeRect(0.5, 0.5, canvas.width - 1, canvas.height - 1);
  context.fillStyle = '#173d29';
  context.fillText(text, canvas.width / 2, canvas.height / 2);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.minFilter = THREE.LinearFilter;
  const material = new THREE.SpriteMaterial({ map: texture, transparent: true, depthTest: false });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set(worldHeight * canvas.width / canvas.height, worldHeight, 1);
  sprite.renderOrder = 10;
  return sprite;
}

function addCameraGlyph(THREE, scene, pose, size) {
  const group = new THREE.Group();
  group.name = pose.name;
  group.position.fromArray(pose.position);
  const matrix = new THREE.Matrix4().set(
    pose.rotation[0][0], pose.rotation[0][1], pose.rotation[0][2], 0,
    pose.rotation[1][0], pose.rotation[1][1], pose.rotation[1][2], 0,
    pose.rotation[2][0], pose.rotation[2][1], pose.rotation[2][2], 0,
    0, 0, 0, 1,
  );
  group.setRotationFromMatrix(matrix);

  const depth = size;
  const halfWidth = size * 0.58;
  const halfHeight = size * 0.4;
  const corners = [
    [-halfWidth, -halfHeight, depth], [halfWidth, -halfHeight, depth],
    [halfWidth, halfHeight, depth], [-halfWidth, halfHeight, depth],
  ];
  const segments = [];
  for (const corner of corners) segments.push(0, 0, 0, ...corner);
  for (let index = 0; index < corners.length; index += 1) {
    segments.push(...corners[index], ...corners[(index + 1) % corners.length]);
  }
  // A small roof mark makes the camera's image-up direction unambiguous.
  segments.push(...corners[2], 0, halfHeight + size * 0.28, depth, 0, halfHeight + size * 0.28, depth, ...corners[3]);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(segments, 3));
  group.add(new THREE.LineSegments(geometry, new THREE.LineBasicMaterial({ color: 0x155f35 })));
  group.add(new THREE.AxesHelper(size * 0.82));

  const label = makeLabel(THREE, pose.name, size * 0.68);
  label.position.set(0, halfHeight + size * 0.58, 0);
  group.add(label);
  scene.add(group);
}

export function extrinsicsView(result) {
  const inertialItems = result.inertial?.camera_from_imu;
  const usesInertialReference = Array.isArray(inertialItems);
  const items = inertialItems ?? result.camera_from_reference;
  const poses = Array.isArray(items) ? items.map(cameraPose).filter(Boolean) : [];
  if (!poses.length) return null;

  const root = document.createElement('div');
  root.className = 'extrinsics-view';
  root.setAttribute('aria-label', '相机外参三维视图');

  const header = setStyles(document.createElement('div'), {
    display: 'flex', alignItems: 'center', justifyContent: 'space-between', gap: '10px', flexWrap: 'wrap',
  });
  const copy = document.createElement('div');
  const description = document.createElement('p');
  description.className = 'small';
  description.textContent = '共同参考系中的相机位姿 · 米 · 已对 camera-from-reference 求逆';
  setStyles(description, { margin: '0 0 3px' });
  const help = document.createElement('p');
  help.className = 'small';
  help.textContent = '左键旋转 · 右键平移 · 滚轮缩放 · 单指旋转 / 双指平移缩放 · 红绿蓝轴 = X / Y / Z';
  setStyles(help, { margin: '0', color: 'var(--muted, #587061)' });
  copy.append(description, help);

  const reset = document.createElement('button');
  reset.type = 'button';
  reset.textContent = '重置视角';
  reset.disabled = true;
  header.append(copy, reset);

  const viewport = setStyles(document.createElement('div'), {
    position: 'relative', width: '100%', height: 'clamp(280px, 45vw, 380px)', overflow: 'hidden',
    border: '1px solid var(--border, #cbd9ce)', borderRadius: '8px', background: '#f6faf7',
  });
  const status = document.createElement('p');
  status.className = 'small';
  status.setAttribute('role', 'status');
  status.textContent = '正在准备三维视图…';
  setStyles(status, {
    position: 'absolute', inset: '0', display: 'grid', placeItems: 'center', margin: '0', padding: '20px', textAlign: 'center',
  });
  viewport.append(status);
  root.append(header, viewport);

  let disposed = false;
  let cleanup = () => {};
  root.dispose = () => {
    if (disposed) return;
    disposed = true;
    cleanup();
    cleanup = () => {};
  };

  const showError = (message) => {
    if (disposed) return;
    reset.disabled = true;
    status.textContent = message;
    status.setAttribute('role', 'alert');
    setStyles(status, { color: '#8b2f24', background: '#fff8f6' });
    viewport.replaceChildren(status);
  };

  void (async () => {
    let THREE;
    let OrbitControls;
    try {
      [THREE, { OrbitControls }] = await Promise.all([
        import(THREE_MODULE),
        import(ORBIT_CONTROLS_MODULE),
      ]);
    } catch (error) {
      showError(`三维组件加载失败：${error instanceof Error ? error.message : String(error)}`);
      return;
    }
    if (disposed) return;

    let renderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
    } catch {
      showError('无法显示三维外参：此浏览器或设备未提供可用的 WebGL。');
      return;
    }
    if (disposed) {
      renderer.dispose();
      renderer.forceContextLoss();
      return;
    }

    cleanup = () => {
      renderer.dispose();
      renderer.forceContextLoss();
    };

    renderer.setClearColor(0xf6faf7, 1);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.domElement.setAttribute('role', 'img');
    renderer.domElement.setAttribute('aria-label', '可旋转、平移和缩放的相机外参三维视图');
    setStyles(renderer.domElement, { display: 'block', width: '100%', height: '100%', outline: 'none' });

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(42, 1, 0.001, 1000);
    const positions = [new THREE.Vector3(0, 0, 0), ...poses.map((pose) => new THREE.Vector3(...pose.position))];
    const bounds = new THREE.Box3().setFromPoints(positions);
    const center = bounds.getCenter(new THREE.Vector3());
    const rawRadius = Math.max(...positions.map((position) => position.distanceTo(center)));
    const sceneScale = Math.max(rawRadius, 0.01);
    const glyphSize = sceneScale * 0.18;

    const gridSize = sceneScale * 4;
    const grid = new THREE.GridHelper(gridSize, 12, 0x8daf97, 0xd7e4da);
    grid.material.transparent = true;
    grid.material.opacity = 0.72;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(glyphSize * 1.8));
    const reference = usesInertialReference ? 'IMU' : String(result.reference_camera ?? '参考系');
    const referenceLabel = makeLabel(THREE, reference, glyphSize * 0.68);
    referenceLabel.position.set(0, glyphSize * 0.45, 0);
    scene.add(referenceLabel);

    const connectionGeometry = new THREE.BufferGeometry().setFromPoints(positions.flatMap((position, index) => index === 0 ? [] : [positions[0], position]));
    const connections = new THREE.LineSegments(connectionGeometry, new THREE.LineDashedMaterial({ color: 0xa9bdae, dashSize: glyphSize * 0.12, gapSize: glyphSize * 0.09, transparent: true, opacity: 0.75 }));
    connections.computeLineDistances();
    scene.add(connections);
    for (const pose of poses) addCameraGlyph(THREE, scene, pose, glyphSize);

    const framingRadius = Math.max(rawRadius + glyphSize * 2.5, glyphSize * 4);
    const distance = framingRadius / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2)) * 1.15;
    camera.near = Math.max(distance / 2000, 1e-6);
    camera.far = Math.max(distance + framingRadius * 20, camera.near * 1000);
    camera.updateProjectionMatrix();
    camera.position.copy(center).add(new THREE.Vector3(1.2, 0.85, 1.35).normalize().multiplyScalar(distance));

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = false;
    controls.screenSpacePanning = true;
    controls.minDistance = sceneScale * 0.08;
    controls.maxDistance = sceneScale * 80;
    controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
    controls.mouseButtons.RIGHT = THREE.MOUSE.PAN;
    controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
    controls.touches.ONE = THREE.TOUCH.ROTATE;
    controls.touches.TWO = THREE.TOUCH.DOLLY_PAN;
    controls.target.copy(center);
    controls.update();
    controls.saveState();

    const render = () => renderer.render(scene, camera);
    controls.addEventListener('change', render);
    const resetView = () => {
      controls.reset();
      render();
    };
    reset.addEventListener('click', resetView);

    const resize = () => {
      if (disposed) return;
      const width = Math.max(1, viewport.clientWidth || 760);
      const height = Math.max(1, viewport.clientHeight || 340);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      render();
    };
    let resizeObserver = null;
    if ('ResizeObserver' in window) {
      resizeObserver = new ResizeObserver(resize);
      resizeObserver.observe(viewport);
    } else {
      window.addEventListener('resize', resize);
    }

    cleanup = () => {
      resizeObserver?.disconnect();
      if (!resizeObserver) window.removeEventListener('resize', resize);
      reset.removeEventListener('click', resetView);
      controls.removeEventListener('change', render);
      controls.dispose();
      const geometries = new Set();
      const materials = new Set();
      const textures = new Set();
      scene.traverse((object) => {
        if (object.geometry) geometries.add(object.geometry);
        const objectMaterials = Array.isArray(object.material) ? object.material : [object.material];
        for (const material of objectMaterials) {
          if (!material) continue;
          materials.add(material);
          if (material.map) textures.add(material.map);
        }
      });
      for (const texture of textures) texture.dispose();
      for (const material of materials) material.dispose();
      for (const geometry of geometries) geometry.dispose();
      renderer.renderLists.dispose();
      renderer.dispose();
      renderer.forceContextLoss();
    };

    viewport.replaceChildren(renderer.domElement);
    reset.disabled = false;
    resize();
  })().catch((error) => {
    cleanup();
    cleanup = () => {};
    showError(`无法初始化三维外参：${error instanceof Error ? error.message : String(error)}`);
  });

  return root;
}
