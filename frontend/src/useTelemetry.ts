import { useEffect, useRef, useState } from 'react'
import type { Clouds, Scene, Snapshot, Vec3 } from './types'

export interface HistPoint { t: number; E: number | null; loc: number | null; clr: number | null; z: number | null }

/** WebSocket telemetry with automatic reconnect (exponential back-off). */
export function useTelemetry() {
  const [snap, setSnap] = useState<Snapshot>({})
  const [images, setImages] = useState<Record<string, string>>({})
  const [clouds, setClouds] = useState<Clouds>({})
  const [scene, setScene] = useState<Scene | null>(null)
  const [connected, setConnected] = useState(false)
  const [lastMsg, setLastMsg] = useState(0)
  const [history, setHistory] = useState<HistPoint[]>([])
  const [dynObjects, setDynObjects] = useState<Record<string, Vec3>>({})
  const retry = useRef(500)

  useEffect(() => {
    let ws: WebSocket | null = null
    let timer: number | undefined
    let ping: number | undefined
    let closed = false
    const connect = () => {
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`)
      ws.onopen = () => {
        setConnected(true); retry.current = 500
        ping = window.setInterval(() => ws?.readyState === 1 && ws.send('ping'), 10000)
      }
      ws.onmessage = (e) => {
        const msg = JSON.parse(e.data)
        setLastMsg(Date.now())
        if (msg.type === 'hello') {
          if (msg.data.snapshot) setSnap(msg.data.snapshot)
          if (msg.data.images) setImages(msg.data.images)
          if (msg.data.clouds) setClouds(msg.data.clouds)
          if (msg.data.scene) setScene(msg.data.scene)
        } else if (msg.type === 'snapshot') {
          const s: Snapshot = msg.data
          setSnap(s)
          const m = s.mission
          const loc = s.metrics?.localization_error?.mean ?? null
          setHistory((h) => {
            const p: HistPoint = { t: s.ros_time ?? 0, E: m && m.state === 'NAVIGATE' ? m.spatial_error : null, loc,
              clr: s.avoidance?.min_clearance_lookahead ?? null, z: m?.pose ? m.pose[2] : null }
            const n = h.length && p.t < h[h.length - 1].t ? [p] : [...h, p]   // sim restarted -> reset
            return n.slice(-600)
          })
          const ev = s.events ?? []
          const moved: Record<string, Vec3> = {}
          for (const e of ev) if ((e as any).type === 'object_pose') moved[(e as any).object] = (e as any).pos
          if (Object.keys(moved).length) setDynObjects((d) => ({ ...d, ...moved }))
        } else if (msg.type === 'images') setImages((i) => ({ ...i, ...msg.data }))
        else if (msg.type === 'clouds') setClouds((c) => ({ ...c, ...msg.data }))
      }
      ws.onclose = () => {
        setConnected(false); window.clearInterval(ping)
        if (!closed) { timer = window.setTimeout(connect, retry.current); retry.current = Math.min(retry.current * 2, 8000) }
      }
      ws.onerror = () => ws?.close()
    }
    connect()
    fetch('/api/scene').then((r) => r.json()).then(setScene).catch(() => {})
    return () => { closed = true; window.clearTimeout(timer); window.clearInterval(ping); ws?.close() }
  }, [])

  const live = connected && Date.now() - lastMsg < 4000
  return { snap, images, clouds, scene, connected, live, history, dynObjects }
}

export const fmt = (v: unknown, d = 2, unit = '') =>
  typeof v === 'number' && Number.isFinite(v) ? `${v.toFixed(d)}${unit}` : '—'
