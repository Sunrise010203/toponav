# Dijkstra path planning design

Approved approach: handwritten heap-based Dijkstra in the existing Python 3.10 project virtualenv. No ROS integration, new dependencies, map rewriting or system configuration changes.

## Interface and data
- Read configs/graph.json (or a caller-selected JSON map).
- Edge length_m is the nonnegative path cost; edges, not the physical neighbors list, define traversability.
- Respect enabled=false; a temporary set of disabled channel IDs further excludes edges without modifying the input map.
- Respect edge bidirectional; this scene has six bidirectional channels.
- A callable plan(graph, start, goal, disabled_channels) returns route nodes, channel IDs, total length in metres and a reachable/unreachable status. The default start/goal come from the map.
- Replanning is a new call from the current node with the latest disabled channel set, not reuse of a cached route.
- Provide a small CLI with map/start/goal/disable arguments; output JSON. Successful route exits 0; unreachable exits 1; invalid input exits 2. No infinity/NaN in JSON.
- Same start and goal returns a single node, no channels and length 0. Unknown nodes/channels and invalid weights must not silently appear unreachable.

## Algorithm
Build adjacency only from enabled, non-disabled edges. Use a min-heap and distance relaxation; store predecessor node and channel to reconstruct the route. Skip stale heap entries. Equal-length routes use deterministic node-ID heap order and keep the first predecessor. No NetworkX use in production planning.

## Verification
1. Normal N01 to N06: N01-N02-N03-N04-N06, C01/C02/C03/C06, 8m.
2. Disable C02: N01-N02-N05-N04-N06, C01/C05/C04/C06, 8m.
3. Read configs/graph_blocked_C02.json: same alternate route.
4. Disable C06 (or C02+C05): unreachable, empty route, null length.
5. Replan from N02 after C02 closes: N02-N05-N04-N06, 6m.
6. Weighted, reverse-direction, start=goal, unknown-ID and malformed-weight tests.
7. For all 64 channel-disable subsets and all 36 source/target pairs in the real map, compare reachability and minimal distance with an independent oracle; verify each reported channel is enabled and connects its reported nodes.

## Artifacts
A standalone planner module and pytest tests, usage instructions and VM test/acceptance output. Preserve existing source and configs. Do not commit or push Git changes.
