import React, { useEffect, useRef } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

/**
 * Interactive warehouse floor plan using Leaflet.
 * Uses a Cartesian CRS (Simple) mapped to warehouse coordinates [-15 to +15] meters.
 */
export default function WarehouseMap({ robots, intents, conflicts, chargingDocks, obstacles }) {
  const mapContainerRef = useRef(null);
  const mapInstanceRef = useRef(null);
  const layersRef = useRef({
    robots: L.layerGroup(),
    paths: L.layerGroup(),
    conflicts: L.layerGroup(),
    staticLayers: L.layerGroup()
  });

  // Transform warehouse meter coords (-15 to 15, -15 to 15) to Leaflet CRS.Simple [y, x]
  const toLeafletLatLng = (x, y) => {
    const scale = 25; // pixels per meter
    const originPx = 400; // center of 800x800 canvas
    return [originPx + y * scale, originPx + x * scale];
  };

  useEffect(() => {
    if (!mapContainerRef.current || mapInstanceRef.current) return;

    const map = L.map(mapContainerRef.current, {
      crs: L.CRS.Simple,
      minZoom: -1,
      maxZoom: 3,
      zoomControl: true,
      attributionControl: false
    });

    const bounds = [[0, 0], [800, 800]];
    map.fitBounds(bounds);
    map.setView([400, 400], 0);

    // Add layer groups
    Object.values(layersRef.current).forEach(layer => layer.addTo(map));

    // Render static warehouse layout (Walls, Racks, Aisles, Charging Docks)
    const staticGroup = layersRef.current.staticLayers;
    staticGroup.clearLayers();

    // Floor boundary
    L.rectangle([[50, 50], [750, 750]], {
      color: '#334155',
      weight: 2,
      fillColor: '#0f172a',
      fillOpacity: 0.95
    }).addTo(staticGroup);

    // Storage racks
    const racks = [
      { x: -8, y: 6, w: 8, h: 1.2 }, { x: -8, y: 2, w: 8, h: 1.2 },
      { x: -8, y: -2, w: 8, h: 1.2 }, { x: -8, y: -6, w: 8, h: 1.2 },
      { x: 8, y: 6, w: 8, h: 1.2 }, { x: 8, y: 2, w: 8, h: 1.2 },
      { x: 8, y: -2, w: 8, h: 1.2 }, { x: 8, y: -6, w: 8, h: 1.2 }
    ];

    racks.forEach(r => {
      const p1 = toLeafletLatLng(r.x - r.w / 2, r.y - r.h / 2);
      const p2 = toLeafletLatLng(r.x + r.w / 2, r.y + r.h / 2);
      L.rectangle([p1, p2], {
        color: '#3b82f6',
        weight: 1,
        fillColor: '#1e3a8a',
        fillOpacity: 0.7
      }).bindTooltip('Storage Rack', { sticky: true }).addTo(staticGroup);
    });

    // Central narrow choke barriers
    const chokeNorth1 = toLeafletLatLng(-0.5, 1.0);
    const chokeNorth2 = toLeafletLatLng(0.5, 5.0);
    L.rectangle([chokeNorth1, chokeNorth2], {
      color: '#eab308',
      fillColor: '#713f12',
      fillOpacity: 0.6
    }).bindTooltip('Single-Lane Choke Barrier').addTo(staticGroup);

    const chokeSouth1 = toLeafletLatLng(-0.5, -5.0);
    const chokeSouth2 = toLeafletLatLng(0.5, -1.0);
    L.rectangle([chokeSouth1, chokeSouth2], {
      color: '#eab308',
      fillColor: '#713f12',
      fillOpacity: 0.6
    }).bindTooltip('Single-Lane Choke Barrier').addTo(staticGroup);

    // Charging docks
    const docks = [
      { id: 'NW Dock', x: -12, y: 8 },
      { id: 'SW Dock', x: -12, y: -8 },
      { id: 'E Dock', x: 12, y: 0 }
    ];
    docks.forEach(d => {
      const p1 = toLeafletLatLng(d.x - 0.7, d.y - 0.7);
      const p2 = toLeafletLatLng(d.x + 0.7, d.y + 0.7);
      L.rectangle([p1, p2], {
        color: '#10b981',
        weight: 2,
        fillColor: '#064e3b',
        fillOpacity: 0.6
      }).bindTooltip(`Charging: ${d.id}`).addTo(staticGroup);
    });

    mapInstanceRef.current = map;
  }, []);

  // Update dynamic layers: robots, paths, conflict markers
  useEffect(() => {
    if (!mapInstanceRef.current) return;

    // 1. Render Planned Intent Paths
    const pathsGroup = layersRef.current.paths;
    pathsGroup.clearLayers();

    Object.entries(intents || {}).forEach(([robotId, intent]) => {
      if (intent.planned_path && intent.planned_path.length > 1) {
        const latLngs = intent.planned_path.map(pt => toLeafletLatLng(pt.x, pt.y));
        const color = getRobotColor(robotId);
        L.polyline(latLngs, {
          color: color,
          weight: 3,
          opacity: 0.6,
          dashArray: '5, 8'
        }).addTo(pathsGroup);

        // Goal marker
        if (intent.current_goal) {
          const goalPos = toLeafletLatLng(intent.current_goal.x, intent.current_goal.y);
          L.circleMarker(goalPos, {
            radius: 5,
            color: color,
            fillColor: color,
            fillOpacity: 0.9
          }).bindTooltip(`Goal (${robotId})`).addTo(pathsGroup);
        }
      }
    });

    // 2. Render Active Robots
    const robotsGroup = layersRef.current.robots;
    robotsGroup.clearLayers();

    Object.entries(robots || {}).forEach(([robotId, data]) => {
      const pos = toLeafletLatLng(data.pose.x, data.pose.y);
      const color = getRobotColor(robotId);
      const statusColor = getStatusColor(data.current_state);

      const html = `
        <div style="position: relative; width: 36px; height: 36px; display: flex; align-items: center; justify-content: center;">
          <div style="width: 28px; height: 28px; border-radius: 50%; background: ${statusColor}; border: 2px solid ${color}; display: flex; align-items: center; justify-content: center; box-shadow: 0 0 12px ${color}88; color: #fff; font-weight: bold; font-size: 11px;">
            ${robotId.replace('robot_', 'R')}
          </div>
          <div style="position: absolute; bottom: -8px; width: 32px; background: rgba(0,0,0,0.8); border-radius: 3px; height: 4px; overflow: hidden;">
            <div style="width: ${data.battery_percentage}%; height: 100%; background: ${data.battery_percentage < 20 ? '#ef4444' : '#10b981'};"></div>
          </div>
        </div>
      `;

      const icon = L.divIcon({
        className: 'custom-amr-marker',
        html: html,
        iconSize: [36, 36],
        iconAnchor: [18, 18]
      });

      L.marker(pos, { icon })
        .bindTooltip(`<b>${robotId}</b><br/>State: ${data.current_state}<br/>Battery: ${data.battery_percentage}%<br/>Speed: ${Math.hypot(data.twist.vx, data.twist.vy).toFixed(2)} m/s`)
        .addTo(robotsGroup);
    });

    // 3. Render Conflict Markers
    const conflictGroup = layersRef.current.conflicts;
    conflictGroup.clearLayers();

    (conflicts || []).forEach(c => {
      if (!c.resolved && c.conflict_location) {
        const pos = toLeafletLatLng(c.conflict_location.x, c.conflict_location.y);
        const iconHtml = `
          <div class="pulse-conflict-ring" style="width: 32px; height: 32px; border-radius: 50%; border: 3px solid #ef4444; background: rgba(239, 68, 68, 0.3); animation: pulse-ring 1.5s infinite;"></div>
        `;
        const icon = L.divIcon({
          className: 'conflict-marker',
          html: iconHtml,
          iconSize: [32, 32],
          iconAnchor: [16, 16]
        });
        L.marker(pos, { icon })
          .bindTooltip(`<b>CONFLICT DETECTED</b><br/>Type: ${c.conflict_type}<br/>Peers: ${c.initiating_robot_id} vs ${c.conflicting_robot_id}<br/>Resolution: ${c.resolution_action}`)
          .addTo(conflictGroup);
      }
    });

  }, [robots, intents, conflicts]);

  return (
    <div style={{ width: '100%', height: '560px', position: 'relative', borderRadius: '12px', overflow: 'hidden', border: '1px solid #334155' }}>
      <div ref={mapContainerRef} style={{ width: '100%', height: '100%', background: '#0b1120' }} />
      <div style={{ position: 'absolute', top: 12, right: 12, background: 'rgba(15, 23, 42, 0.85)', backdropFilter: 'blur(8px)', padding: '8px 14px', borderRadius: '8px', border: '1px solid #334155', fontSize: '12px', color: '#94a3b8', zIndex: 1000 }}>
        <span style={{ color: '#38bdf8', fontWeight: 600 }}>Decentralized DDS Peer View</span> | No Central Coordinator
      </div>
    </div>
  );
}

function getRobotColor(id) {
  const map = {
    'robot_1': '#38bdf8', // Sky blue
    'robot_2': '#f43f5e', // Rose
    'robot_3': '#a855f7', // Purple
    'robot_4': '#f59e0b', // Amber
    'robot_5': '#10b981'  // Emerald
  };
  return map[id] || '#06b6d4';
}

function getStatusColor(state) {
  switch (state) {
    case 'NAVIGATING': return '#2563eb';
    case 'RESOLVING_CONFLICT': return '#d97706';
    case 'CHARGING': return '#059669';
    case 'BLOCKED': return '#dc2626';
    default: return '#475569';
  }
}
