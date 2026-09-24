import React from 'react';

export default function FleetStatusTable({ robots, intents }) {
  const robotList = Object.entries(robots || {}).map(([id, state]) => ({
    id,
    ...state,
    intent: (intents && intents[id]) || {}
  }));

  return (
    <div style={{ background: '#1e293b', borderRadius: '12px', padding: '16px', border: '1px solid #334155', color: '#f8fafc' }}>
      <h3 style={{ margin: '0 0 14px 0', fontSize: '16px', fontWeight: 600, color: '#e2e8f0', display: 'flex', alignItems: 'center', gap: '8px' }}>
        <span style={{ display: 'inline-block', width: '8px', height: '8px', borderRadius: '50%', background: '#10b981' }} />
        Active Edge Peer Telemetry
      </h3>
      <div style={{ overflowX: 'auto' }}>
        <table style={{ width: '100%', borderCollapse: 'collapse', textAlign: 'left', fontSize: '13px' }}>
          <thead>
            <tr style={{ borderBottom: '1px solid #334155', color: '#94a3b8' }}>
              <th style={{ padding: '8px 12px' }}>Robot ID</th>
              <th style={{ padding: '8px 12px' }}>Operational State</th>
              <th style={{ padding: '8px 12px' }}>Battery %</th>
              <th style={{ padding: '8px 12px' }}>Pose (x, y)</th>
              <th style={{ padding: '8px 12px' }}>Speed</th>
              <th style={{ padding: '8px 12px' }}>Priority</th>
              <th style={{ padding: '8px 12px' }}>Aisle Token</th>
            </tr>
          </thead>
          <tbody>
            {robotList.length === 0 ? (
              <tr>
                <td colSpan="7" style={{ textAlign: 'center', padding: '24px', color: '#64748b' }}>
                  Awaiting DDS peer discovery packets...
                </td>
              </tr>
            ) : (
              robotList.map(r => {
                const speed = Math.hypot(r.twist?.vx || 0, r.twist?.vy || 0).toFixed(2);
                const bat = r.battery_percentage ?? 100;
                const batColor = bat < 15 ? '#ef4444' : bat < 20 ? '#f59e0b' : '#10b981';

                return (
                  <tr key={r.id} style={{ borderBottom: '1px solid #33415555' }}>
                    <td style={{ padding: '10px 12px', fontWeight: 600, color: '#38bdf8' }}>{r.id}</td>
                    <td style={{ padding: '10px 12px' }}>
                      <span style={{
                        padding: '4px 8px',
                        borderRadius: '4px',
                        fontSize: '11px',
                        fontWeight: 600,
                        background: getBadgeBackground(r.current_state),
                        color: getBadgeColor(r.current_state)
                      }}>
                        {r.current_state || 'IDLE'}
                      </span>
                    </td>
                    <td style={{ padding: '10px 12px', minWidth: '120px' }}>
                      <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                        <div style={{ flex: 1, background: '#0f172a', borderRadius: '4px', height: '6px', overflow: 'hidden' }}>
                          <div style={{ width: `${bat}%`, height: '100%', background: batColor, transition: 'width 0.3s ease' }} />
                        </div>
                        <span style={{ color: batColor, fontWeight: 600, fontSize: '12px' }}>{bat}%</span>
                      </div>
                    </td>
                    <td style={{ padding: '10px 12px', color: '#cbd5e1', fontFamily: 'monospace' }}>
                      ({r.pose?.x?.toFixed(2) ?? 0}, {r.pose?.y?.toFixed(2) ?? 0})
                    </td>
                    <td style={{ padding: '10px 12px', color: '#cbd5e1' }}>{speed} m/s</td>
                    <td style={{ padding: '10px 12px', color: '#a855f7', fontWeight: 600 }}>
                      {r.intent?.priority_score?.toFixed(2) || '0.50'}
                    </td>
                    <td style={{ padding: '10px 12px' }}>
                      {r.intent?.has_token ? (
                        <span style={{ color: '#10b981', fontWeight: 600 }}>HELD ({r.intent.reserved_corridor_id})</span>
                      ) : (
                        <span style={{ color: '#64748b' }}>None</span>
                      )}
                    </td>
                  </tr>
                );
              })
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
}

function getBadgeBackground(state) {
  switch (state) {
    case 'NAVIGATING': return '#1d4ed833';
    case 'RESOLVING_CONFLICT': return '#b4530933';
    case 'CHARGING': return '#04785733';
    case 'BLOCKED': return '#b91c1c33';
    default: return '#33415533';
  }
}

function getBadgeColor(state) {
  switch (state) {
    case 'NAVIGATING': return '#60a5fa';
    case 'RESOLVING_CONFLICT': return '#fbbf24';
    case 'CHARGING': return '#34d399';
    case 'BLOCKED': return '#f87171';
    default: return '#94a3b8';
  }
}
