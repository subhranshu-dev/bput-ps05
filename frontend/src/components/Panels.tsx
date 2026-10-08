import { useEffect, useState } from 'react'
import { CartesianGrid, Line, LineChart, ReferenceLine, ResponsiveContainer, Tooltip, XAxis, YAxis } from 'recharts'
import { fmt, type HistPoint } from '../useTelemetry'
import type { Snapshot } from '../types'

export const NOMINAL = ['NAVIGATE', 'LANDING_ZONE_SEARCH', 'TARGET_LOCK', 'ALIGN', 'DESCEND', 'FINAL_CHECK', 'LAND']
const PRE = ['INIT', 'ARMING', 'TAKEOFF', 'PLANNING']
const FAIL_LABEL: Record<string, string> = {
  OBSTACLE_STOP: 'OBSTACLE DETECTED', REPLANNING: 'REPLANNING', NO_SAFE_PATH: 'NO SAFE PATH', TARGET_LOST: 'TARGET LOST',
  UNSAFE_PAD: 'UNSAFE LANDING AREA', ABORT_CLIMB: 'ABORT · CLIMB', SENSOR_FAILURE: 'SENSOR FAILURE',
  CORRIDOR_BREACH: 'CORRIDOR BREACH', ABORTED: 'MISSION ABORTED',
}
const pretty = (s?: string) => (s ? s.replace(/_/g, ' ') : '—')

export function Panel({ title, right, children, className = '' }: { title: string; right?: React.ReactNode; children: React.ReactNode; className?: string }) {
  return (
    <section className={`panel flex min-h-0 flex-col p-3 ${className}`}>
      <div className="mb-2 flex items-center justify-between"><h2 className="panel-title">{title}</h2>{right}</div>
      <div className="min-h-0 flex-1">{children}</div>
    </section>
  )
}

function Chip({ tone, children, pulse }: { tone: 'ok' | 'warn' | 'bad' | 'info' | 'dim'; children: React.ReactNode; pulse?: boolean }) {
  const cls = { ok: 'border-ok/40 text-ok bg-ok/10', warn: 'border-warn/40 text-warn bg-warn/10', bad: 'border-bad/50 text-bad bg-bad/10',
    info: 'border-accent/40 text-accent bg-accent/10', dim: 'border-edge text-dim bg-panel2' }[tone]
  const dot = { ok: 'bg-ok', warn: 'bg-warn', bad: 'bg-bad', info: 'bg-accent', dim: 'bg-dim' }[tone]
  return <span className={`chip ${cls}`}><span className={`h-1.5 w-1.5 rounded-full ${dot} ${pulse ? 'pulse-dot' : ''}`} />{children}</span>
}

export function TopBar({ snap, live, connected }: { snap: Snapshot; live: boolean; connected: boolean }) {
  const m = snap.mission
  const failure = m?.failure
  const st = m?.state
  return (
    <header className="panel flex flex-wrap items-center gap-x-5 gap-y-2 px-4 py-2.5">
      <div className="flex items-center gap-2.5">
        <svg width="26" height="26" viewBox="0 0 32 32"><path d="M6 16h20M16 6v20" stroke="#38bdf8" strokeWidth="2.5" /><circle cx="6" cy="16" r="3" fill="#38bdf8" /><circle cx="26" cy="16" r="3" fill="#38bdf8" /><circle cx="16" cy="6" r="3" fill="#38bdf8" /><circle cx="16" cy="26" r="3" fill="#38bdf8" /></svg>
        <div>
          <div className="text-sm font-bold tracking-wide text-white">PS05 GROUND CONTROL</div>
          <div className="text-[10px] tracking-widest text-dim">BPUT HACKATHON 2026 · GPS-DENIED AUTONOMY</div>
        </div>
      </div>
      <Chip tone="bad">GPS: OFF / DISABLED</Chip>
      <Chip tone="info">AUTONOMOUS</Chip>
      <Chip tone={live ? 'ok' : connected ? 'warn' : 'bad'} pulse={live}>SIMULATION: {live ? 'RUNNING' : connected ? 'IDLE' : 'OFFLINE'}</Chip>
      <Chip tone={snap.px4?.connected ? 'ok' : 'dim'}>PX4 {snap.px4?.connected ? (snap.px4?.armed ? 'ARMED' : 'CONNECTED') : '—'}</Chip>
      <div className="ml-auto flex items-center gap-3">
        <span className="panel-title">Mission</span>
        <span key={st} className={`state-enter rounded-lg px-3 py-1 font-mono text-sm font-bold tracking-wider ${failure ? 'bg-bad/15 text-bad ring-1 ring-bad/50' : st === 'LANDED' ? 'bg-ok/15 text-ok ring-1 ring-ok/40' : 'bg-accent/10 text-accent ring-1 ring-accent/40'}`}>
          {failure ? FAIL_LABEL[st!] ?? pretty(st) : pretty(st)}
        </span>
        <span className="kv text-xs text-dim">t={fmt(snap.ros_time, 1, ' s')}</span>
      </div>
    </header>
  )
}

export function CameraFeeds({ images, snap }: { images: Record<string, string>; snap: Snapshot }) {
  const det = snap.detection
  return (
    <div className="flex h-full flex-col gap-2">
      {(['camera', 'down_camera'] as const).map((k) => (
        <div key={k} className="relative min-h-0 flex-1 overflow-hidden rounded-lg border border-edge bg-black">
          {images[k] ? <img src={`data:image/jpeg;base64,${images[k]}`} className="h-full w-full object-contain" alt={k} />
            : <div className="flex h-full items-center justify-center text-xs text-dim">no {k.replace('_', ' ')} stream</div>}
          <div className="absolute left-2 top-2 flex gap-1.5">
            <span className="chip border-edge bg-black/60 text-ink">{k === 'camera' ? 'FORWARD RGB' : 'DOWNWARD · LANDING'}</span>
            {k === 'down_camera' && det && <span className={`chip ${det.detected ? 'border-ok/50 bg-ok/15 text-ok' : 'border-edge bg-black/60 text-dim'}`}>{det.detected ? `TAG 36h11 · ${fmt(det.range_m, 2, ' m')}` : 'NO TAG'}</span>}
          </div>
          {k === 'camera' && <div className="absolute bottom-1.5 right-2 font-mono text-[10px] text-dim">sync Δ {fmt((snap.fusion?.sync_delta_s ?? NaN) * 1000, 1, ' ms')} · fov {fmt((snap.fusion?.fov_fraction ?? NaN) * 100, 0, '%')}</div>}
        </div>
      ))}
    </div>
  )
}

function KV({ k, v, tone }: { k: string; v: string; tone?: string }) {
  return <div className="flex items-baseline justify-between border-b border-edge/50 py-1"><span className="text-[11px] text-dim">{k}</span><span className={`kv ${tone ?? 'text-ink'}`}>{v}</span></div>
}

export function VehicleStatus({ snap }: { snap: Snapshot }) {
  const m = snap.mission
  const p = m?.pose, v = m?.velocity
  const E = m?.spatial_error ?? 0, Emax = m?.e_max ?? 2, Ewarn = m?.e_warn ?? 1
  const nav = m?.state === 'NAVIGATE' || m?.state === 'CORRIDOR_BREACH'
  const tone = E > Emax ? 'bad' : E > Ewarn ? 'warn' : 'ok'
  const speed = v ? Math.hypot(v[0], v[1], v[2]) : null
  return (
    <div className="flex h-full flex-col gap-3 overflow-y-auto pr-1">
      <div className="grid grid-cols-3 gap-2">
        {(['X', 'Y', 'Z'] as const).map((a, i) => (
          <div key={a} className="rounded-lg border border-edge bg-panel2 p-2 text-center">
            <div className="text-[10px] text-dim">{a}</div><div className="kv text-lg text-white">{fmt(p?.[i], 2)}</div><div className="text-[9px] text-dim">m · map</div>
          </div>
        ))}
      </div>
      <div className="grid grid-cols-3 gap-2">
        {(['VX', 'VY', 'VZ'] as const).map((a, i) => (
          <div key={a} className="rounded-lg border border-edge bg-panel2 p-1.5 text-center">
            <div className="text-[10px] text-dim">{a}</div><div className="kv">{fmt(v?.[i], 2)}</div>
          </div>
        ))}
      </div>
      <div>
        <KV k="Altitude (est.)" v={fmt(p?.[2], 2, ' m')} />
        <KV k="Ground speed" v={fmt(speed, 2, ' m/s')} />
        <KV k="Heading (ENU)" v={fmt(m?.yaw != null ? (m.yaw * 180) / Math.PI : null, 0, '°')} />
        <KV k="PX4 nav state" v={String(snap.px4?.nav_state ?? '—')} />
        <KV k="Offboard" v={snap.px4?.offboard ? 'ACTIVE' : 'no'} tone={snap.px4?.offboard ? 'text-ok' : undefined} />
        <KV k="Localisation" v="tracker r,θ,φ + EKF2" tone="text-accent" />
      </div>
      <div className="rounded-lg border border-edge bg-panel2 p-2.5">
        <div className="mb-1 flex items-center justify-between">
          <span className="panel-title">PS05 path enforcement</span>
          <Chip tone={!nav ? 'dim' : tone === 'bad' ? 'bad' : 'ok'}>{!nav ? 'STANDBY' : tone === 'bad' ? 'BREACH' : 'NORMAL'}</Chip>
        </div>
        <div className="flex items-baseline gap-2"><span className="kv text-2xl text-white">{nav ? fmt(E, 2) : '—'}</span><span className="text-xs text-dim">m spatial error E · Emax {fmt(Emax, 1)} m</span></div>
        <div className="mt-2 h-2 overflow-hidden rounded bg-black/50">
          <div className={`h-full transition-all ${tone === 'bad' ? 'bg-bad' : tone === 'warn' ? 'bg-warn' : 'bg-ok'}`} style={{ width: `${Math.min(100, (E / Emax) * 100)}%` }} />
        </div>
        <div className="mt-1 flex justify-between text-[9px] text-dim"><span>0</span><span>warn {Ewarn}</span><span>Emax {Emax}</span></div>
      </div>
    </div>
  )
}

export function MissionTimeline({ snap }: { snap: Snapshot }) {
  const st = snap.mission?.state ?? 'INIT'
  const hist = snap.mission?.stats?.state_history ?? []
  const visited = new Set(hist.map((h) => h.to))
  const idx = NOMINAL.indexOf(st)
  const failure = snap.mission?.failure
  return (
    <div className="flex flex-col gap-2">
      <div className="flex items-center gap-1 overflow-x-auto">
        {[...PRE, ...NOMINAL, 'LANDED'].map((s, i, all) => {
          const active = s === st
          const done = visited.has(s) && !active && (idx < 0 || all.indexOf(s) < all.indexOf(st))
          return (
            <div key={s} className="flex items-center gap-1">
              <div className={`whitespace-nowrap rounded-md border px-2 py-1 text-[10px] font-semibold tracking-wider transition ${active ? (failure ? 'border-bad text-bad' : 'border-accent bg-accent/15 text-accent shadow-[0_0_12px_rgba(56,189,248,.35)]') : done ? 'border-ok/40 text-ok/90' : 'border-edge text-dim'}`}>{pretty(s)}</div>
              {i < all.length - 1 && <div className={`h-px w-3 ${done ? 'bg-ok/50' : 'bg-edge'}`} />}
            </div>
          )
        })}
      </div>
      <div className="flex flex-wrap gap-1.5">
        {Object.entries(FAIL_LABEL).map(([k, label]) => (
          <span key={k} className={`chip ${st === k ? 'border-bad bg-bad/20 text-bad' : visited.has(k) ? 'border-warn/40 text-warn/80' : 'border-edge/60 text-dim/50'}`}>{label}</span>
        ))}
      </div>
    </div>
  )
}

function Metric({ label, value, sub, tone }: { label: string; value: string; sub?: string; tone?: 'ok' | 'warn' | 'bad' }) {
  const c = tone === 'bad' ? 'text-bad' : tone === 'warn' ? 'text-warn' : tone === 'ok' ? 'text-ok' : 'text-white'
  return (
    <div className="rounded-lg border border-edge bg-panel2 p-2.5">
      <div className="text-[10px] uppercase tracking-wider text-dim">{label}</div>
      <div className={`kv mt-0.5 text-lg ${c}`}>{value}</div>
      {sub && <div className="text-[10px] text-dim">{sub}</div>}
    </div>
  )
}

export function MetricsGrid({ snap }: { snap: Snapshot }) {
  const met = snap.metrics ?? {}
  const st = snap.mission?.stats
  const lat = (a?: (number | null)[]) => { const v = (a ?? []).filter((x): x is number => x != null); return v.length ? v[v.length - 1] : null }
  const loc = met.localization_error ?? {}
  return (
    <div className="grid grid-cols-2 gap-2 md:grid-cols-3 xl:grid-cols-5">
      <Metric label="Localisation err (eval)" value={fmt(loc.mean, 2, ' m')} sub={`p95 ${fmt(loc.p95, 2)} m · vs sim truth`} />
      <Metric label="Landing error" value={fmt(met.landing_error_m != null ? met.landing_error_m * 100 : null, 1, ' cm')} sub="touchdown vs pad centre" tone={met.landing_error_m != null ? (met.landing_error_m < 0.3 ? 'ok' : 'warn') : undefined} />
      <Metric label="Min clearance" value={fmt(met.min_clearance_m, 2, ' m')} sub={met.min_clearance_object ?? ''} tone={met.min_clearance_m != null ? (met.min_clearance_m < 0.4 ? 'bad' : met.min_clearance_m < 0.65 ? 'warn' : 'ok') : undefined} />
      <Metric label="Collisions" value={String(met.collision_count ?? '—')} sub={`near misses ${met.near_misses ?? '—'}`} tone={met.collision_count ? 'bad' : met.collision_count === 0 ? 'ok' : undefined} />
      <Metric label="Planner latency" value={fmt(lat(st?.plan_latencies_ms), 0, ' ms')} sub={`A* ${fmt(snap.planner?.astar_ms, 0)} ms · ${snap.planner?.expansions ?? '—'} exp`} />
      <Metric label="Replan latency" value={fmt(lat(st?.replan_latencies_ms), 0, ' ms')} sub={`${st?.replans ?? 0} replans`} />
      <Metric label="Path length" value={fmt(snap.planner?.length_m, 1, ' m')} sub={`flown ${fmt(met.executed_path_length_m, 1)} m`} />
      <Metric label="Target detection" value={fmt(met.target_detection_rate != null ? met.target_detection_rate * 100 : null, 0, '%')} sub={`${st?.target_losses ?? 0} losses`} />
      <Metric label="Mission time" value={fmt(met.mission_time_s ?? (snap.ros_time && st?.mission_start_t ? snap.ros_time - st.mission_start_t : null), 1, ' s')} sub={met.outcome ?? 'in progress'} />
      <Metric label="Fusion projection" value={fmt(snap.fusion?.edge_chamfer_px_mean, 1, ' px')} sub={`edge chamfer · ${fmt(snap.fusion?.process_ms, 0)} ms`} />
    </div>
  )
}

export function Charts({ history }: { history: HistPoint[] }) {
  const t0 = history[0]?.t ?? 0
  const data = history.map((h) => ({ ...h, t: +(h.t - t0).toFixed(1) }))
  return (
    <div className="grid h-full grid-cols-1 gap-2 md:grid-cols-2">
      {[{ k: 'E', label: 'Spatial error E (m)', color: '#38bdf8', ref: 2 }, { k: 'z', label: 'Altitude est. (m)', color: '#a78bfa' }].map((c) => (
        <div key={c.k} className="min-h-[140px] rounded-lg border border-edge bg-panel2 p-2">
          <div className="text-[10px] text-dim">{c.label}</div>
          <ResponsiveContainer width="100%" height="88%">
            <LineChart data={data} margin={{ top: 6, right: 8, bottom: 0, left: -20 }}>
              <CartesianGrid stroke="#1e2d40" strokeDasharray="3 3" />
              <XAxis dataKey="t" tick={{ fill: '#7e93ab', fontSize: 9 }} />
              <YAxis tick={{ fill: '#7e93ab', fontSize: 9 }} />
              <Tooltip contentStyle={{ background: '#0d1520', border: '1px solid #1e2d40', fontSize: 11 }} />
              {c.ref && <ReferenceLine y={c.ref} stroke="#f87171" strokeDasharray="4 3" />}
              <Line type="monotone" dataKey={c.k} stroke={c.color} dot={false} strokeWidth={1.6} isAnimationActive={false} connectNulls={false} />
            </LineChart>
          </ResponsiveContainer>
        </div>
      ))}
    </div>
  )
}

export function SensorHealth({ snap }: { snap: Snapshot }) {
  const s = snap.sensors ?? {}
  const names = ['camera', 'lidar', 'down_camera', 'down_lidar', 'tracker', 'odom']
  return (
    <div className="grid grid-cols-2 gap-1.5 lg:grid-cols-3">
      {names.map((n) => {
        const h = s[n]
        const ok = h?.ok
        return (
          <div key={n} className={`rounded-lg border p-2 ${ok ? 'border-ok/30 bg-ok/5' : h ? 'border-bad/50 bg-bad/10' : 'border-edge bg-panel2'}`}>
            <div className="flex items-center justify-between"><span className="text-[11px] font-semibold uppercase tracking-wide">{n.replace('_', ' ')}</span><span className={`h-2 w-2 rounded-full ${ok ? 'bg-ok' : h ? 'bg-bad pulse-dot' : 'bg-dim'}`} /></div>
            <div className="kv text-[11px] text-dim">{h?.rate_hz != null ? `${fmt(h.rate_hz, 1)} Hz` : h?.rate_count != null ? `${h.rate_count} msgs` : '—'}{h?.blackout ? ' · BLACKOUT' : ''}</div>
          </div>
        )
      })}
      <div className="col-span-full mt-1 grid grid-cols-3 gap-1.5 text-[11px]">
        <div className="rounded border border-edge bg-panel2 p-1.5"><span className="text-dim">Map </span><span className="kv">{snap.mapping?.occupied ?? '—'}</span><span className="text-dim"> occ · {snap.mapping?.degraded ? 'LiDAR-only' : 'fused'}</span></div>
        <div className="rounded border border-edge bg-panel2 p-1.5"><span className="text-dim">Sync </span><span className="kv">{snap.fusion?.accepted ?? 0}/{(snap.fusion?.accepted ?? 0) + (snap.fusion?.rejected ?? 0)}</span><span className="text-dim"> pairs</span></div>
        <div className="rounded border border-edge bg-panel2 p-1.5"><span className="text-dim">Avoid </span><span className={`kv ${snap.avoidance?.blocked ? 'text-bad' : 'text-ok'}`}>{snap.avoidance ? (snap.avoidance.blocked ? 'BLOCKED' : 'CLEAR') : '—'}</span></div>
      </div>
    </div>
  )
}

export function LandingPanel({ snap }: { snap: Snapshot }) {
  const m = snap.mission
  const ls = m?.landing_safety
  const det = snap.detection
  const checks: [string, boolean | undefined][] = [
    ['Target detected', det?.detected],
    ['Ground plane', ls ? !ls.reasons.includes('NO_GROUND_PLANE') : undefined],
    ['Surface level/flat', ls ? !ls.reasons.some((r) => r === 'SURFACE_TILTED' || r === 'SURFACE_ROUGH') : undefined],
    ['Clearance', ls ? !ls.reasons.includes('OBSTRUCTION') : undefined],
    ['Alignment', ls ? !ls.reasons.includes('MISALIGNED') : undefined],
  ]
  const met = snap.metrics ?? {}
  return (
    <div className="flex h-full flex-col gap-2 overflow-y-auto pr-1">
      <div className="grid grid-cols-1 gap-1">
        {checks.map(([k, v]) => (
          <div key={k} className="flex items-center justify-between rounded border border-edge/60 bg-panel2 px-2 py-1 text-[11px]">
            <span>{k}</span><span className={`font-semibold ${v === undefined ? 'text-dim' : v ? 'text-ok' : 'text-bad'}`}>{v === undefined ? '—' : v ? 'PASS' : 'FAIL'}</span>
          </div>
        ))}
      </div>
      <div className="grid grid-cols-2 gap-x-3">
        <KV k="Height AGL (LiDAR)" v={fmt(ls?.height_above_ground, 2, ' m')} />
        <KV k="Align error" v={fmt(ls?.alignment_error, 2, ' m')} />
        <KV k="Tilt" v={fmt(ls?.tilt_deg, 1, '°')} />
        <KV k="Plane RMS" v={fmt(ls?.plane_rms != null ? ls.plane_rms * 100 : null, 1, ' cm')} />
        <KV k="PID u_x / u_y" v={`${fmt(m?.pid?.x?.u, 2)} / ${fmt(m?.pid?.y?.u, 2)}`} />
        <KV k="Pixel err" v={det?.pixel_error ? `${fmt(det.pixel_error[0], 0)}, ${fmt(det.pixel_error[1], 0)}` : '—'} />
      </div>
      {met.landing_error_m != null && (
        <div className="rounded-lg border border-ok/40 bg-ok/10 p-2.5 text-center">
          <div className="panel-title text-ok">Touchdown result (measured vs sim truth)</div>
          <div className="kv text-2xl text-white">{fmt(met.landing_error_m * 100, 1)} cm</div>
          <div className="text-[10px] text-dim">horizontal error from pad centre · outcome {met.outcome}</div>
        </div>
      )}
    </div>
  )
}

export function EventLog({ snap }: { snap: Snapshot }) {
  const ev = [...(snap.events ?? [])].reverse()
  return (
    <div className="h-full overflow-y-auto pr-1 font-mono text-[11px]">
      {ev.length === 0 && <div className="text-dim">no events yet</div>}
      {ev.map((e, i) => {
        const tone = e.level === 'ERROR' ? 'text-bad' : e.level === 'WARN' ? 'text-warn' : e.source === 'scenario' ? 'text-fuchsia-300' : 'text-ink/90'
        return (
          <div key={i} className="flex gap-2 border-b border-edge/40 py-0.5">
            <span className="w-14 shrink-0 text-dim">{fmt(e.t ?? e.stamp, 1)}</span>
            <span className="w-16 shrink-0 text-dim">{e.source}</span>
            <span className={tone}>{e.text ?? `${e.event ?? e.note ?? ''} ${e.object ?? ''}`}</span>
          </div>
        )
      })}
    </div>
  )
}

export function Experiments() {
  const [scen, setScen] = useState<string[]>([])
  const [sel, setSel] = useState('demo')
  const [trials, setTrials] = useState(1)
  const [data, setData] = useState<any>(null)
  const [msg, setMsg] = useState('')
  const refresh = () => fetch('/api/experiments').then((r) => r.json()).then(setData).catch(() => {})
  useEffect(() => {
    fetch('/api/scenarios').then((r) => r.json()).then((d) => setScen(d.scenarios)).catch(() => {})
    refresh()
    const t = setInterval(refresh, 5000)
    return () => clearInterval(t)
  }, [])
  const post = async (url: string, body?: object) => {
    const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body ?? {}) })
    const j = await r.json().catch(() => ({}))
    setMsg(r.ok ? `ok: ${j.tag ?? j.selected ?? 'done'}` : `error: ${j.detail ?? r.status}`)
    refresh()
  }
  const running = data?.current?.running
  const summary = data?.summary?.by_scenario ?? {}
  return (
    <div className="flex h-full flex-col gap-2 overflow-hidden">
      <div className="flex flex-wrap items-center gap-2">
        <select value={sel} onChange={(e) => setSel(e.target.value)} className="rounded-lg border border-edge bg-panel2 px-2 py-1.5 text-xs">
          {scen.map((s) => <option key={s}>{s}</option>)}
        </select>
        <input type="number" min={1} max={50} value={trials} onChange={(e) => setTrials(+e.target.value)} className="w-16 rounded-lg border border-edge bg-panel2 px-2 py-1.5 text-xs" />
        <button className="btn" disabled={running} onClick={() => post('/api/experiments/start', { scenario: sel, trials })}>▶ Start</button>
        <button className="btn" disabled={!running} onClick={() => post('/api/experiments/stop')}>■ Stop</button>
        <span className="text-[11px] text-dim">{running ? `running ${data.current.tag}` : msg}</span>
      </div>
      {Object.keys(summary).length > 0 && (
        <div className="overflow-x-auto">
          <table className="w-full text-left text-[11px]">
            <thead className="text-dim"><tr><th>scenario</th><th>n</th><th>success</th><th>RMSE</th><th>P95</th><th>min clr</th><th>coll.</th></tr></thead>
            <tbody className="font-mono">
              {Object.entries<any>(summary).map(([k, v]) => (
                <tr key={k} className="border-t border-edge/40"><td className="font-sans">{k}</td><td>{v.trials}</td><td>{fmt(v.success_rate != null ? v.success_rate * 100 : null, 0, '%')}</td>
                  <td>{fmt(v.landing_error_m?.rmse != null ? v.landing_error_m.rmse * 100 : null, 1, 'cm')}</td><td>{fmt(v.landing_error_m?.p95 != null ? v.landing_error_m.p95 * 100 : null, 1, 'cm')}</td>
                  <td>{fmt(v.min_clearance_m?.min, 2)}</td><td className={v.collisions_total ? 'text-bad' : ''}>{v.collisions_total}</td></tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
      <div className="min-h-0 flex-1 overflow-y-auto">
        <table className="w-full text-left text-[11px]">
          <thead className="sticky top-0 bg-panel text-dim"><tr><th>run</th><th>outcome</th><th>landing err</th><th>clearance</th><th>replans</th></tr></thead>
          <tbody className="font-mono">
            {(data?.runs ?? []).slice(0, 60).map((r: any) => (
              <tr key={r.run_id} className="border-t border-edge/40">
                <td className="max-w-[160px] truncate font-sans" title={r.run_id}>{r.run_id}</td>
                <td className={r.outcome === 'LANDED' ? 'text-ok' : 'text-warn'}>{r.outcome ?? r.note ?? '—'}</td>
                <td>{fmt(r.landing_error_m != null ? r.landing_error_m * 100 : null, 1, ' cm')}</td><td>{fmt(r.min_clearance_m, 2)}</td><td>{r.replans ?? '—'}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  )
}
