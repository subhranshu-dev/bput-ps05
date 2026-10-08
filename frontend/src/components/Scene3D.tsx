import { Line, OrbitControls } from '@react-three/drei'
import { Canvas } from '@react-three/fiber'
import { useMemo, useState } from 'react'
import * as THREE from 'three'
import type { Clouds, Obstacle, Scene, Snapshot, Vec3 } from '../types'

THREE.Object3D.DEFAULT_UP.set(0, 0, 1)   // map frame is ENU: z up

const LAYERS = ['world', 'occupancy', 'fused', 'corridor', 'planned', 'local', 'actual', 'target'] as const
type Layer = (typeof LAYERS)[number]
const LAYER_LABEL: Record<Layer, string> = {
  world: 'Obstacles (sim truth)', occupancy: 'Occupancy map', fused: 'Fused cloud', corridor: 'Safety corridor',
  planned: 'Planned path', local: 'Local trajectory', actual: 'Actual path', target: 'Landing target',
}
const LAYER_COLOR: Record<Layer, string> = {
  world: '#64748b', occupancy: '#a78bfa', fused: '#f472b6', corridor: '#34d399', planned: '#fbbf24',
  local: '#38bdf8', actual: '#f8fafc', target: '#22c55e',
}

function ObstacleMesh({ o, pos, wire }: { o: Obstacle; pos?: Vec3; wire?: boolean }) {
  const p = pos ?? o.pos
  const color = new THREE.Color(...o.color)
  return (
    <mesh position={p as any} rotation={o.type === 'cylinder' ? [Math.PI / 2, 0, 0] : [0, 0, 0]}>
      {o.type === 'box' && <boxGeometry args={o.size as any} />}
      {o.type === 'cylinder' && <cylinderGeometry args={[o.radius!, o.radius!, o.length!, 16]} />}
      {o.type === 'sphere' && <sphereGeometry args={[o.radius!, 16, 12]} />}
      <meshStandardMaterial color={color} transparent opacity={wire ? 0.25 : 0.55} wireframe={wire} />
    </mesh>
  )
}

function Points({ xyz, rgb, size }: { xyz: number[]; rgb?: number[]; size: number }) {
  const geom = useMemo(() => {
    const g = new THREE.BufferGeometry()
    g.setAttribute('position', new THREE.Float32BufferAttribute(xyz, 3))
    if (rgb) {
      const c = new Float32Array(rgb.length * 3)
      rgb.forEach((v, i) => { c[3 * i] = ((v >> 16) & 255) / 255; c[3 * i + 1] = ((v >> 8) & 255) / 255; c[3 * i + 2] = (v & 255) / 255 })
      g.setAttribute('color', new THREE.BufferAttribute(c, 3))
    }
    return g
  }, [xyz, rgb])
  return (
    <points geometry={geom}>
      <pointsMaterial size={size} vertexColors={!!rgb} color={rgb ? undefined : '#a78bfa'} sizeAttenuation />
    </points>
  )
}

function Voxels({ xyz, rgb }: { xyz: number[]; rgb: number[] }) {
  const n = xyz.length / 3
  const mesh = useMemo(() => {
    const geo = new THREE.BoxGeometry(0.25, 0.25, 0.25)
    const mat = new THREE.MeshStandardMaterial({ transparent: true, opacity: 0.85 })
    const m = new THREE.InstancedMesh(geo, mat, Math.max(n, 1))
    const tmp = new THREE.Object3D()
    const col = new THREE.Color()
    for (let i = 0; i < n; i++) {
      tmp.position.set(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2])
      tmp.updateMatrix()
      m.setMatrixAt(i, tmp.matrix)
      const v = rgb[i]
      col.setRGB(((v >> 16) & 255) / 255, ((v >> 8) & 255) / 255, (v & 255) / 255)
      m.setColorAt(i, col)
    }
    m.count = n
    return m
  }, [xyz, rgb, n])
  return <primitive object={mesh} />
}

function Uav({ p, yaw }: { p: Vec3; yaw: number }) {
  return (
    <group position={p as any} rotation={[0, 0, yaw]}>
      <mesh><boxGeometry args={[0.5, 0.5, 0.12]} /><meshStandardMaterial color="#38bdf8" emissive="#0ea5e9" emissiveIntensity={0.6} /></mesh>
      {[[0.35, 0.35], [-0.35, 0.35], [0.35, -0.35], [-0.35, -0.35]].map(([x, y], i) => (
        <mesh key={i} position={[x, y, 0.06]}><cylinderGeometry args={[0.18, 0.18, 0.02, 16]} /><meshStandardMaterial color="#e2e8f0" transparent opacity={0.5} /></mesh>
      ))}
      <mesh position={[0.45, 0, 0]}><coneGeometry args={[0.08, 0.25, 8]} /><meshStandardMaterial color="#fbbf24" /></mesh>
    </group>
  )
}

export default function Scene3D({ scene, snap, clouds, dyn }: { scene: Scene | null; snap: Snapshot; clouds: Clouds; dyn: Record<string, Vec3> }) {
  const [on, setOn] = useState<Record<Layer, boolean>>({ world: true, occupancy: true, fused: true, corridor: true, planned: true, local: true, actual: true, target: true })
  const m = snap.mission
  const paths = snap.paths ?? {}
  const corridor = scene?.mission.corridor
  const ms = scene?.mission.mission_space
  const pad = scene?.world.landing_pad
  return (
    <div className="relative h-full w-full">
      <Canvas camera={{ position: [-12, -18, 38], fov: 45, up: [0, 0, 1] }} dpr={[1, 1.5]} gl={{ antialias: true }}>
        <color attach="background" args={['#060b12']} />
        <ambientLight intensity={0.55} />
        <directionalLight position={[30, 10, 40]} intensity={1.1} />
        <OrbitControls target={[25, 25, 2]} makeDefault maxPolarAngle={Math.PI * 0.49} />
        <mesh position={[25, 25, -0.01]}><planeGeometry args={[60, 60]} /><meshStandardMaterial color="#0b1622" /></mesh>
        <gridHelper args={[50, 25, '#1e3a52', '#12263a']} position={[25, 25, 0]} rotation={[Math.PI / 2, 0, 0]} />
        {scene && on.world && scene.world.obstacles.map((o) => <ObstacleMesh key={o.name} o={o} />)}
        {scene && on.world && scene.world.dynamic.filter((o) => dyn[o.name]).map((o) => <ObstacleMesh key={o.name} o={o} pos={dyn[o.name]} />)}
        {pad && (
          <group position={[pad.center[0], pad.center[1], 0.05]}>
            <mesh><boxGeometry args={[pad.size[0], pad.size[1], 0.1]} /><meshStandardMaterial color="#334155" /></mesh>
            <mesh position={[0, 0, 0.06]}><planeGeometry args={[pad.tag_plate, pad.tag_plate]} /><meshStandardMaterial color="#f8fafc" /></mesh>
          </group>
        )}
        {on.corridor && ms && (
          <mesh position={[(ms.x[0] + ms.x[1]) / 2, (ms.y[0] + ms.y[1]) / 2, (ms.z[0] + ms.z[1]) / 2]}>
            <boxGeometry args={[ms.x[1] - ms.x[0], ms.y[1] - ms.y[0], ms.z[1] - ms.z[0]]} />
            <meshBasicMaterial color="#34d399" wireframe transparent opacity={0.12} />
          </mesh>
        )}
        {on.corridor && corridor && (
          <>
            <Line points={corridor.centerline as any} color="#34d399" lineWidth={1.5} dashed dashSize={0.8} gapSize={0.5} />
            {corridor.centerline.slice(0, -1).map((a, i) => {
              const b = corridor.centerline[i + 1]
              const va = new THREE.Vector3(...a), vb = new THREE.Vector3(...b)
              const len = va.distanceTo(vb)
              const mid = va.clone().add(vb).multiplyScalar(0.5)
              const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), vb.clone().sub(va).normalize())
              const r = corridor.max_deviation - corridor.safety_margin
              return (
                <mesh key={i} position={mid} quaternion={q}>
                  <cylinderGeometry args={[r, r, len, 24, 1, true]} />
                  <meshBasicMaterial color="#34d399" transparent opacity={0.05} side={THREE.DoubleSide} depthWrite={false} />
                </mesh>
              )
            })}
          </>
        )}
        {on.occupancy && clouds.map && clouds.map.xyz.length > 0 && <Voxels xyz={clouds.map.xyz} rgb={clouds.map.rgb} />}
        {on.fused && clouds.fused && clouds.fused.xyz.length > 0 && <Points xyz={clouds.fused.xyz} rgb={clouds.fused.rgb} size={0.12} />}
        {on.planned && paths.planned && paths.planned.length > 1 && <Line points={paths.planned as any} color="#fbbf24" lineWidth={2.5} />}
        {on.local && paths.local && paths.local.length > 1 && <Line points={paths.local as any} color="#38bdf8" lineWidth={2} />}
        {on.actual && paths.actual && paths.actual.length > 1 && <Line points={paths.actual as any} color="#f8fafc" lineWidth={1.5} />}
        {on.target && m?.target && (
          <mesh position={m.target.pos as any}><torusGeometry args={[0.45, 0.06, 8, 32]} /><meshBasicMaterial color="#22c55e" /></mesh>
        )}
        {m?.goal && <mesh position={m.goal as any}><octahedronGeometry args={[0.35]} /><meshBasicMaterial color="#fbbf24" wireframe /></mesh>}
        {m?.reference && m.state === 'NAVIGATE' && <mesh position={m.reference as any}><sphereGeometry args={[0.18]} /><meshBasicMaterial color="#fb923c" /></mesh>}
        {m?.pose && <Uav p={m.pose} yaw={m.yaw ?? 0} />}
      </Canvas>
      <div className="absolute left-3 top-3 flex flex-col gap-1 rounded-lg bg-black/40 p-2 backdrop-blur">
        {LAYERS.map((l) => (
          <label key={l} className="flex cursor-pointer items-center gap-2 text-[11px] text-dim hover:text-ink">
            <input type="checkbox" className="accent-sky-400" checked={on[l]} onChange={() => setOn((s) => ({ ...s, [l]: !s[l] }))} />
            <span className="inline-block h-2 w-3 rounded-sm" style={{ background: LAYER_COLOR[l] }} />{LAYER_LABEL[l]}
          </label>
        ))}
      </div>
      <div className="absolute bottom-2 right-3 text-[10px] text-dim">
        map voxels {clouds.map?.total ?? 0} · fused pts {clouds.fused ? clouds.fused.xyz.length / 3 : 0} · drag to orbit
      </div>
    </div>
  )
}
