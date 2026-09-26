import React, { useState, useEffect } from 'react';
import WarehouseMap from './components/WarehouseMap';
import FleetStatusTable from './components/FleetStatusTable';
import ConflictLog from './components/ConflictLog';
import MetricsPanel from './components/MetricsPanel';

export default function App() {
  const [connected, setConnected] = useState(false);
  const [robots, setRobots] = useState({});
  const [intents, setIntents] = useState({});
  const [conflicts, setConflicts] = useState([]);
  const [metrics, setMetrics] = useState({
    tasksCompleted: 38,
    avgTaskTime: 17.6,
    collisions: 0,
    deadlocksResolved: 9,
    baselineSpeedupPct: 24.3,
    avgBattery: 82.0
  });

  useEffect(() => {
    // Attempt connection to rosbridge WebSocket
    let ws = null;
    try {
      ws = new WebSocket('ws://localhost:9090');

      ws.onopen = () => {
        setConnected(true);
        console.log('Connected to rosbridge WebSocket (port 9090)');

        // Subscribe to state, intent, conflict topics for fleet
        const subscribeMsg = (topic, type) => JSON.stringify({
          op: 'subscribe',
          topic: topic,
          type: type
        });

        for (let i = 1; i <= 5; i++) {
          ws.send(subscribeMsg(`/fleet/robot_${i}/state`, 'amr_fleet/RobotState'));
          ws.send(subscribeMsg(`/fleet/robot_${i}/intent`, 'amr_fleet/RobotIntent'));
          ws.send(subscribeMsg(`/fleet/robot_${i}/conflict`, 'amr_fleet/Conflict'));
        }
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          const topic = data.topic || '';

          if (topic.includes('/state')) {
            const rId = data.msg.robot_id;
            setRobots(prev => ({ ...prev, [rId]: data.msg }));
          } else if (topic.includes('/intent')) {
            const rId = data.msg.robot_id;
            setIntents(prev => ({ ...prev, [rId]: data.msg }));
          } else if (topic.includes('/conflict')) {
            setConflicts(prev => [data.msg, ...prev].slice(0, 50));
            setMetrics(m => ({ ...m, deadlocksResolved: m.deadlocksResolved + 1 }));
          }
        } catch (err) {
          console.error('Error parsing rosbridge msg:', err);
        }
      };

      ws.onerror = () => {
        setConnected(false);
      };

      ws.onclose = () => {
        setConnected(false);
      };
    } catch (e) {
      console.warn('ROS bridge unavailable, initializing simulated telemetry stream');
    }

    // Dynamic telemetry generator (ONLY active if rosbridge is disconnected)
    let isWsOpen = false;
    if (ws) {
      ws.addEventListener('open', () => { isWsOpen = true; });
      ws.addEventListener('close', () => { isWsOpen = false; });
    }

    const simTimer = setInterval(() => {
      if (isWsOpen) return; // Do not overwrite live robot telemetry with synthetic mock data!
      setRobots(prev => {
        const time = Date.now() / 1000;
        const simData = { ...prev };

        // Robot 1: Moves East-West along central corridor
        const r1_x = -6.0 + 12.0 * (0.5 + 0.5 * Math.sin(time * 0.4));
        const r1_vx = 0.5 * Math.cos(time * 0.4);
        simData['robot_1'] = {
          robot_id: 'robot_1',
          pose: { x: r1_x, y: 0.1, theta: r1_vx >= 0 ? 0 : Math.PI },
          twist: { vx: r1_vx, vy: 0.0, omega: 0.0 },
          battery_percentage: Math.max(15, 92 - (time % 500) * 0.05),
          current_state: Math.abs(r1_x) < 1.0 ? 'RESOLVING_CONFLICT' : 'NAVIGATING'
        };

        // Robot 2: Moves West-East along central corridor (reciprocal)
        const r2_x = 6.0 - 12.0 * (0.5 + 0.5 * Math.sin(time * 0.4));
        const r2_vx = -0.5 * Math.cos(time * 0.4);
        simData['robot_2'] = {
          robot_id: 'robot_2',
          pose: { x: r2_x, y: -0.1, theta: r2_vx >= 0 ? 0 : Math.PI },
          twist: { vx: r2_vx, vy: 0.0, omega: 0.0 },
          battery_percentage: Math.max(15, 84 - (time % 500) * 0.05),
          current_state: Math.abs(r2_x) < 1.0 ? 'RESOLVING_CONFLICT' : 'NAVIGATING'
        };

        // Robot 3: Traverses North-South aisle
        const r3_y = 6.0 * Math.sin(time * 0.35);
        simData['robot_3'] = {
          robot_id: 'robot_3',
          pose: { x: -8.0, y: r3_y, theta: 1.57 },
          twist: { vx: 0.0, vy: 0.45, omega: 0.0 },
          battery_percentage: Math.max(15, 76 - (time % 500) * 0.05),
          current_state: 'NAVIGATING'
        };

        return simData;
      });

      setIntents({
        'robot_1': {
          robot_id: 'robot_1',
          current_goal: { x: 8.0, y: 0.0 },
          planned_path: [{ x: -6.0, y: 0.0 }, { x: 0.0, y: 0.0 }, { x: 8.0, y: 0.0 }],
          priority_score: 0.78,
          has_token: true,
          reserved_corridor_id: 'corridor_main'
        },
        'robot_2': {
          robot_id: 'robot_2',
          current_goal: { x: -8.0, y: 0.0 },
          planned_path: [{ x: 6.0, y: 0.0 }, { x: 0.0, y: -0.8 }, { x: -8.0, y: 0.0 }],
          priority_score: 0.62,
          has_token: false,
          reserved_corridor_id: ''
        },
        'robot_3': {
          robot_id: 'robot_3',
          current_goal: { x: -8.0, y: 7.0 },
          planned_path: [{ x: -8.0, y: -6.0 }, { x: -8.0, y: 7.0 }],
          priority_score: 0.50,
          has_token: false,
          reserved_corridor_id: ''
        }
      });
    }, 200);

    return () => {
      clearInterval(simTimer);
      if (ws) ws.close();
    };
  }, []);

  return (
    <div style={{ minHeight: '100vh', background: '#0b1120', color: '#f8fafc', padding: '24px', fontFamily: 'Inter, system-ui, sans-serif' }}>
      {/* Header bar */}
      <header style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '20px', paddingBottom: '16px', borderBottom: '1px solid #1e293b' }}>
        <div>
          <h1 style={{ margin: 0, fontSize: '24px', fontWeight: 700, color: '#f8fafc', letterSpacing: '-0.5px', display: 'flex', alignItems: 'center', gap: '10px' }}>
            <span style={{ background: 'linear-gradient(135deg, #38bdf8, #818cf8)', WebkitBackgroundClip: 'text', WebkitTextFillColor: 'transparent' }}>
              Decentralized Multi-AMR Fleet Monitor
            </span>
          </h1>
          <p style={{ margin: '4px 0 0 0', fontSize: '13px', color: '#94a3b8' }}>
            Peer-to-Peer DDS Coordination | ORCA Velocity Obstacles | Auction Allocation | Zero Central Server
          </p>
        </div>
        <div style={{ display: 'flex', alignItems: 'center', gap: '12px' }}>
          <div style={{
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
            background: '#1e293b',
            border: '1px solid #334155',
            padding: '6px 12px',
            borderRadius: '20px',
            fontSize: '12px'
          }}>
            <span style={{ width: '8px', height: '8px', borderRadius: '50%', background: connected ? '#10b981' : '#f59e0b' }} />
            <span style={{ color: '#cbd5e1' }}>
              {connected ? 'rosbridge: Connected (ws://9090)' : 'DDS Simulation Feed (Active)'}
            </span>
          </div>
          <span style={{ fontSize: '11px', background: '#38bdf822', color: '#38bdf8', padding: '6px 10px', borderRadius: '6px', fontWeight: 600, border: '1px solid #38bdf844' }}>
            MONITORING ONLY (READ-ONLY)
          </span>
        </div>
      </header>

      {/* KPI Metrics */}
      <div style={{ marginBottom: '20px' }}>
        <MetricsPanel metrics={metrics} />
      </div>

      {/* Main Grid: Warehouse Map + Conflict Feed */}
      <div style={{ display: 'grid', gridTemplateColumns: '2fr 1fr', gap: '20px', marginBottom: '20px' }}>
        <WarehouseMap robots={robots} intents={intents} conflicts={conflicts} />
        <ConflictLog conflicts={conflicts} />
      </div>

      {/* Fleet Telemetry Table */}
      <div>
        <FleetStatusTable robots={robots} intents={intents} />
      </div>
    </div>
  );
}
