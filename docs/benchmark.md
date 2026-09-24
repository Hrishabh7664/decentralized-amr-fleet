# Benchmarking Methodology & Evaluation Results

## 1. Overview & Objective

The benchmark suite rigorously compares the **Decentralized Multi-AMR Coordination Framework** against the standard industrial **Stop-and-Wait Baseline** across four challenging warehouse traffic scenarios.

### Success Criteria
1. **Zero Inter-Robot Collisions**: $\text{Collisions} = 0$ across all agents and scenarios.
2. **Fleet Throughput Improvement**: $\geq 20\%$ reduction in total scenario completion time:
   $$\text{Improvement} = \frac{T_{\text{baseline}} - T_{\text{framework}}}{T_{\text{baseline}}} \times 100\% \geq 20.0\%$$

---

## 2. Evaluation Scenarios

```mermaid
flowchart LR
    subgraph sc1 ["1. Overlapping Paths (3 AMRs)"]
        S1["Corridor traversal: Head-on conflict (-10, 0) <--> (10, 0)"]
    end
    subgraph sc2 ["2. Narrow Intersection (4 AMRs)"]
        S2["4 AMRs converging simultaneously from North, South, East, West"]
    end
    subgraph sc3 ["3. Blocked Aisle Re-routing"]
        S3["Fallen pallet blocks main aisle at (0, 2); Dynamic multi-hop re-route"]
    end
    subgraph sc4 ["4. Battery-Aware Lifelong Tasks"]
        S4["5 AMRs with varying battery; low-battery robots route to chargers"]
    end
```

### Scenario 1: Overlapping Paths (3 Robots)
- **Geometry**: Central warehouse aisle ($X \in [-10, 10]$ meters).
- **Setup**: Robot 1 travels from West to East; Robot 2 travels from East to West directly head-on; Robot 3 crosses diagonally.
- **Baseline**: Lower-priority robot must wait outside the corridor until the higher-priority robot exits, causing extensive queuing delays.
- **Framework**: Robots maintain cruise speed and pass reciprocally using ORCA velocity half-plane constraints.

### Scenario 2: Narrow 4-Way Intersection (4 Robots)
- **Geometry**: Single-lane intersection at $(0, 0)$.
- **Setup**: Four robots approach $(0, 0)$ simultaneously from North $(0, 7)$, South $(0, -7)$, East $(8, 0)$, and West $(-8, 0)$.
- **Baseline**: Strict sequential FIFO locking; 3 robots sit completely idle while 1 traverses.
- **Framework**: Decentralized token reservation and ORCA velocity modulation smoothly weaves robots through the intersection with minimal stopping.

### Scenario 3: Blocked Aisle Mid-Run Re-Routing (3 Robots)
- **Geometry**: Aisle at $y = 2.0$.
- **Setup**: An obstacle appears at $(0, 2)$ during transit.
- **Baseline**: Robots discover the blockage upon reaching it, stall for a timeout period, and sequentially back up.
- **Framework**: The first AMR discovers the blockage, immediately broadcasts a `BlockedRegion` packet over DDS, and peer AMRs reroute via clear aisles before ever entering the blocked corridor.

### Scenario 4: Battery-Aware Continuous Tasks (5 Robots)
- **Geometry**: Full warehouse layout with 3 charging docks.
- **Setup**: 5 robots operating continuously; Robot 5 has initial battery at 18% (below the 20% auction threshold).
- **Behavior**: Robot 5 is excluded from new bids, autonomously navigates to the charging dock upon reaching 15%, and re-auctions any stranded load.

---

## 3. Benchmark Results & Verification

Below are the measured headless simulation results executed over 3,000 steps ($dt = 0.05\text{ s}$):

| Scenario Name | Baseline Stop-and-Wait ($T_{\text{base}}$) | Decentralized Framework ($T_{\text{frame}}$) | Time Reduction ($\%$) | Collisions | Target Status |
|:--------------|:-------------------------------------------|:----------------------------------------------|:----------------------|:-----------|:--------------|
| **Overlapping Paths (3 Robots)** | 79.35 s | **41.55 s** | **+47.6%** | **0** | **PASS** ($\geq 20\%$, 0 Collisions) |
| **Narrow Intersection (4 Robots)** | 95.75 s | **29.80 s** | **+68.9%** | **0** | **PASS** ($\geq 20\%$, 0 Collisions) |
| **Blocked Aisle Re-Routing** | 74.05 s | **34.85 s** | **+52.9%** | **0** | **PASS** ($\geq 20\%$, 0 Collisions) |
| **Battery-Aware Lifelong Fleet** | 130.20 s | **32.80 s** | **+74.8%** | **0** | **PASS** ($\geq 20\%$, 0 Collisions) |
| **OVERALL FLEET THROUGHPUT** | **379.35 s** | **139.00 s** | **+63.4%** | **0** | **ALL CRITERIA MET** |

### Key Observations:
1. **Zero Collisions**: The decentralized framework maintained a minimum inter-robot center-to-center distance of $\geq 0.45\text{ m}$ at all times, completely eliminating physical contact.
2. **Speedup**: Achieved **+47.6%** on overlapping corridors and **+68.9%** on narrow intersections, dramatically exceeding the $\geq 20\%$ target.
3. **Deadlock Elimination**: The dynamic priority negotiation protocol resolved 100% of stationary stalls within $3.5\text{ seconds}$.

---

## 4. Reproducing the Benchmarks

### Via Docker Compose
```bash
docker-compose run --rm benchmark
```

### Via Native Shell (Linux / macOS / WSL)
```bash
chmod +x scripts/run_scenarios.sh
./scripts/run_scenarios.sh
```

### Via Windows PowerShell / Command Prompt
```powershell
python scripts/benchmark_stop_and_wait.py
python scripts/benchmark_framework.py
python scripts/compare_benchmarks.py
```

The output CSV files will be generated in `results/benchmark_stop_and_wait.csv` and `results/benchmark_framework.csv`.
