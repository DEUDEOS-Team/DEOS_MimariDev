import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


@dataclass(frozen=True)
class Node:
    id: int
    lat: float
    lon: float


@dataclass(frozen=True)
class Edge:
    u: int
    v: int
    cost: float
    props: dict[str, Any]


@dataclass
class RouteGraph:
    nodes: list[Node]
    adj: dict[int, list[Edge]]


EARTH_RADIUS_M = 6_371_000.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2.0 * EARTH_RADIUS_M * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _round_key(lat: float, lon: float, *, decimals: int) -> tuple[float, float]:
    return (round(lat, decimals), round(lon, decimals))


def build_graph_from_centerlines_geojson(
    geojson: dict[str, Any],
    *,
    coord_round_decimals: int = 7,
) -> RouteGraph:
    """
    QGIS ile çizilmiş LineString centerline'larından basit bir graph üretir.

    Beklenti:
    - Feature.geometry.type == "LineString"
    - coordinates: [[lon,lat], [lon,lat], ...]
    - Kavşak birleşimleri için çizgilerin uç noktaları aynı koordinata getirilmeli.

    Graph modeli:
    - Node: unique (lat,lon) (yuvarlama ile)
    - Edge: her LineString içindeki ardışık nokta çiftleri arası çift yönlü (oneway=true ise tek yönlü)
    - cost: haversine mesafe (metre)
    """
    if geojson.get("type") != "FeatureCollection":
        raise ValueError("centerlines geojson FeatureCollection olmalı")

    key_to_id: dict[tuple[float, float], int] = {}
    nodes: list[Node] = []
    adj: dict[int, list[Edge]] = {}

    def get_node_id(lat: float, lon: float) -> int:
        k = _round_key(lat, lon, decimals=coord_round_decimals)
        if k in key_to_id:
            return key_to_id[k]
        nid = len(nodes)
        key_to_id[k] = nid
        nodes.append(Node(id=nid, lat=float(k[0]), lon=float(k[1])))
        adj[nid] = []
        return nid

    feats = geojson.get("features") or []
    for feat in feats:
        if (feat or {}).get("type") != "Feature":
            continue
        geom = (feat or {}).get("geometry") or {}
        if geom.get("type") != "LineString":
            continue
        coords = geom.get("coordinates") or []
        if len(coords) < 2:
            continue

        props = (feat or {}).get("properties") or {}
        oneway = bool(props.get("oneway", False))

        # coords are [lon,lat]
        pts: list[tuple[float, float]] = [(float(c[1]), float(c[0])) for c in coords]
        for (lat1, lon1), (lat2, lon2) in zip(pts, pts[1:], strict=False):
            u = get_node_id(lat1, lon1)
            v = get_node_id(lat2, lon2)
            if u == v:
                continue
            cost = float(haversine_m(lat1, lon1, lat2, lon2))
            e = Edge(u=u, v=v, cost=cost, props=dict(props))
            adj[u].append(e)
            if not oneway:
                adj[v].append(Edge(u=v, v=u, cost=cost, props=dict(props)))

    return RouteGraph(nodes=nodes, adj=adj)


def load_centerlines_geojson(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    return json.loads(p.read_text(encoding="utf-8"))


def nearest_node_id(g: RouteGraph, *, lat: float, lon: float) -> int:
    if not g.nodes:
        raise ValueError("graph boş")
    best_id = 0
    best_d = float("inf")
    for n in g.nodes:
        d = haversine_m(lat, lon, n.lat, n.lon)
        if d < best_d:
            best_d = d
            best_id = n.id
    return best_id


def dijkstra(g: RouteGraph, *, start: int, goal: int) -> list[int]:
    """
    Basit Dijkstra (pozitif edge cost varsayılır). Çıktı: node id path (start..goal).
    """
    if start == goal:
        return [start]

    import heapq

    dist: dict[int, float] = {start: 0.0}
    prev: dict[int, int] = {}
    pq: list[tuple[float, int]] = [(0.0, start)]
    seen: set[int] = set()

    while pq:
        d, u = heapq.heappop(pq)
        if u in seen:
            continue
        seen.add(u)
        if u == goal:
            break
        for e in g.adj.get(u, []):
            nd = d + float(e.cost)
            if nd < dist.get(e.v, float("inf")):
                dist[e.v] = nd
                prev[e.v] = u
                heapq.heappush(pq, (nd, e.v))

    if goal not in dist:
        return []

    path: list[int] = [goal]
    cur = goal
    while cur != start:
        cur = prev[cur]
        path.append(cur)
    path.reverse()
    return path


def path_coords(g: RouteGraph, node_path: Iterable[int]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    by_id = {n.id: n for n in g.nodes}
    for nid in node_path:
        n = by_id.get(int(nid))
        if n is not None:
            out.append((n.lat, n.lon))
    return out

