import React from 'react';

export default function ConflictLog({ conflicts }) {
  const events = [...(conflicts || [])].reverse();

  return (
    <div style={{ background: '#1e293b', borderRadius: '12px', padding: '16px', border: '1px solid #334155', color: '#f8fafc', height: '280px', display: 'flex', flexDirection: 'column' }}>
      <h3 style={{ margin: '0 0 12px 0', fontSize: '15px', fontWeight: 600, color: '#e2e8f0', display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
        <span>Decentralized Conflict & Deadlock Feed</span>
        <span style={{ fontSize: '12px', color: '#94a3b8', fontWeight: 400 }}>{events.length} events logged</span>
      </h3>
      <div style={{ flex: 1, overflowY: 'auto', display: 'flex', flexDirection: 'column', gap: '8px', paddingRight: '4px' }}>
        {events.length === 0 ? (
          <div style={{ textAlign: 'center', margin: 'auto', color: '#64748b', fontSize: '13px' }}>
            No active conflicts detected. Peer ORCA velocity constraints nominal.
          </div>
        ) : (
          events.map((c, idx) => (
            <div key={idx} style={{
              background: '#0f172a',
              borderRadius: '8px',
              padding: '10px 12px',
              borderLeft: `4px solid ${getConflictColor(c.conflict_type)}`,
              display: 'flex',
              flexDirection: 'column',
              gap: '4px',
              fontSize: '12px'
            }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <span style={{ fontWeight: 600, color: getConflictColor(c.conflict_type) }}>
                  [{c.conflict_type}] {c.initiating_robot_id} ↔ {c.conflicting_robot_id}
                </span>
                <span style={{ color: '#64748b', fontSize: '11px' }}>
                  {new Date(c.timestamp * 1000).toLocaleTimeString()}
                </span>
              </div>
              <div style={{ display: 'flex', justifyContent: 'space-between', color: '#94a3b8' }}>
                <span>
                  Priorities: {c.initiating_robot_id} ({c.initiating_priority.toFixed(2)}) vs {c.conflicting_robot_id} ({c.conflicting_priority.toFixed(2)})
                </span>
                <span style={{
                  color: c.resolved ? '#10b981' : '#f59e0b',
                  fontWeight: 600
                }}>
                  {c.resolution_action} {c.resolved ? '✓ RESOLVED' : '⏳ NEGOTIATING'}
                </span>
              </div>
            </div>
          ))
        )}
      </div>
    </div>
  );
}

function getConflictColor(type) {
  switch (type) {
    case 'DEADLOCK': return '#ef4444'; // Red
    case 'EDGE': return '#f59e0b';     // Amber
    case 'VERTEX': return '#ec4899';   // Pink
    case 'CORRIDOR': return '#3b82f6'; // Blue
    default: return '#8b5cf6';
  }
}
