import { lazy, Suspense } from 'react'
import { CameraFeeds, Charts, EventLog, Experiments, LandingPanel, MetricsGrid, MissionTimeline, Panel, SensorHealth, TopBar, VehicleStatus } from './components/Panels'
import { useTelemetry } from './useTelemetry'

const Scene3D = lazy(() => import('./components/Scene3D'))

export default function App() {
  const t = useTelemetry()
  return (
    <div className="mx-auto flex min-h-full max-w-[1920px] flex-col gap-2.5 p-2.5">
      <TopBar snap={t.snap} live={t.live} connected={t.connected} />
      <Panel title="Mission state machine"><MissionTimeline snap={t.snap} /></Panel>
      <div className="grid grid-cols-1 gap-2.5 lg:grid-cols-[minmax(260px,1fr)_minmax(0,2.4fr)_minmax(260px,1fr)]">
        <Panel title="Camera feeds" className="h-[520px]"><CameraFeeds images={t.images} snap={t.snap} /></Panel>
        <Panel title="3D map · paths · corridor · fused LiDAR" className="h-[520px]"
          right={<span className="text-[10px] text-dim">ENU map frame · occupancy from LiDAR+camera fusion</span>}>
          <div className="h-full overflow-hidden rounded-lg border border-edge">
            <Suspense fallback={<div className="p-4 text-xs text-dim">loading 3D view…</div>}>
              <Scene3D scene={t.scene} snap={t.snap} clouds={t.clouds} dyn={t.dynObjects} />
            </Suspense>
          </div>
        </Panel>
        <Panel title="Vehicle status" className="h-[520px]"><VehicleStatus snap={t.snap} /></Panel>
      </div>
      <Panel title="Live metrics"><MetricsGrid snap={t.snap} /></Panel>
      <div className="grid grid-cols-1 gap-2.5 lg:grid-cols-3">
        <Panel title="Telemetry" className="h-[260px]"><Charts history={t.history} /></Panel>
        <Panel title="Sensor health" className="h-[260px]"><SensorHealth snap={t.snap} /></Panel>
        <Panel title="Precision landing" className="h-[260px]"><LandingPanel snap={t.snap} /></Panel>
      </div>
      <div className="grid grid-cols-1 gap-2.5 lg:grid-cols-2">
        <Panel title="Event timeline" className="h-[300px]"><EventLog snap={t.snap} /></Panel>
        <Panel title="Experiments" className="h-[300px]"><Experiments /></Panel>
      </div>
      <footer className="pb-1 text-center text-[10px] text-dim">
        All values are live ROS 2 telemetry or recorded experiment results. GPS is not used anywhere in the autonomy chain; simulator ground truth is shown only where labelled (evaluation).
      </footer>
    </div>
  )
}
