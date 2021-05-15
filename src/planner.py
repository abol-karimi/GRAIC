import pyvoronoi
import graph_tool.all as gt
import math


def distance(p, q):
    d2 = (p[0]-q[0])**2 + (p[1]-q[1])**2
    return math.sqrt(d2)


def point_line_distance(p, p1, p2):
    x0, y0 = p[0], p[1]
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    num = abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))
    denum = math.sqrt((x2-x1)**2+(y2-y1)**2)
    return num/denum


def point_seg_distance(p, s, e):
    dps, dpe, dse = distance(p, s), distance(p, e), distance(s, e)
    sp = (p[0]-s[0], p[1]-s[1])
    se = (e[0]-s[0], e[1]-s[1])
    inner = sp[0]*se[0] + sp[1]*se[1]
    proj = inner/dse
    if inner <= 0:
        return dps
    elif proj <= dse:
        return point_line_distance(p, s, e)
    else:
        return dpe


class Planner():
    def __init__(self):
        self.plan = None
        self.roadmap = None
        self.allowed_obs_dist = 1.0
        self.max_discretization_error = 2.0

    def get_plan(self, car_location, milestone, walls, allowed_obs_dist):
        self.make_roadmap(car_location, walls, allowed_obs_dist)

        for path in gt.all_paths(self.roadmap, self.start_vertex, self.get_closest_vertex(milestone)):
            print(path)

        dist, pred = gt.dijkstra_search(self.roadmap, self.eweight,
                                        source=self.start_vertex)
        vertex = self.get_closest_vertex(milestone)
        plan = [self.roadmap.vp.coords[vertex]]
        while vertex != pred[vertex]:
            vertex = pred[vertex]
            plan.append(self.roadmap.vp.coords[vertex])
        plan.reverse()

        return plan

    def make_roadmap(self, car_location, walls, allowed_obs_dist):
        self.allowed_obs_dist = allowed_obs_dist

        component = [0 for w in walls]
        for i in range(1, len(walls)):
            ve = walls[i-1][1]
            ws = walls[i][0]
            d2 = (ve[0]-ws[0])**2 + (ve[1]-ws[1])**2
            if d2 < 0.000001:  # math.sqrt(d2) < 0.001
                component[i] = component[i-1]
            else:
                component[i] = component[i-1]+1

        edge_coords = []

        import pyvoronoi
        pv = pyvoronoi.Pyvoronoi(1000.0)
        for w in walls:
            pv.AddSegment(w)
        pv.Construct()
        edges = pv.GetEdges()
        vertices = pv.GetVertices()
        cells = pv.GetCells()
        for i, e in enumerate(edges):
            if not e.is_primary:
                continue
            cell = cells[e.cell]
            cell_twin = cells[edges[e.twin].cell]
            if component[cell.site] == component[cell_twin.site]:
                continue
            if e.start == -1 or e.end == -1:
                # e is infinite
                continue
            if e.is_linear:
                start, end = vertices[e.start], vertices[e.end]
                edge_coords.append(((start.X, start.Y),
                                    (end.X, end.Y)))
            else:
                ps_map = pv.DiscretizeCurvedEdge(
                    i, self.max_discretization_error)
                ps = [(p[0], p[1]) for p in list(ps_map)]
                edge_coords += [(ps[j], ps[j+1]) for j in range(len(ps)-1)]

        # Make the roadmap graph
        self.roadmap = gt.Graph(directed=False)

        # Add roadmap vertices
        vertices_coords = list(set(
            [e[0] for e in edge_coords] +
            [e[1] for e in edge_coords]))
        self.roadmap.add_vertex(len(vertices_coords))

        # Add coords vertex property
        self.roadmap.vp.coords = self.roadmap.new_vertex_property(
            "vector<double>")
        for i, v in enumerate(self.roadmap.vertices()):
            self.roadmap.vp.coords[v] = vertices_coords[i]

        # Add roadmap edges
        coords_to_vertex = {tuple(self.roadmap.vp.coords[v]): v
                            for v in self.roadmap.vertices()}
        edge_weights = [distance(c[0], c[1]) for c in edge_coords]
        edge_list = [(coords_to_vertex[c[0]], coords_to_vertex[c[1]], w)
                     for c, w in zip(edge_coords, edge_weights)
                     ] + [(coords_to_vertex[c[1]], coords_to_vertex[c[0]], w)
                          for c, w in zip(edge_coords, edge_weights)]

        self.eweight = self.roadmap.new_ep("double")
        self.roadmap.add_edge_list(edge_list, eprops=[self.eweight])

        self.add_start_vertex(car_location)

        return self.get_roadmap_segments()

    def add_start_vertex(self, car_location):
        edge = self.get_closest_edge(car_location)
        vertex0 = edge.source()
        vertex1 = edge.target()
        point0 = self.roadmap.vp.coords[vertex0]
        point1 = self.roadmap.vp.coords[vertex1]
        v0 = (point0[0]-car_location[0], point0[1]-car_location[1])
        v1 = (point1[0]-car_location[0], point1[1]-car_location[1])
        v01 = (v1[0]-v0[0], v1[1]-v0[1])
        inner = v0[0]*v01[0]+v0[1]*v01[1]
        d2 = v01[0]**2 + v01[1]**2
        v_proj = (-v01[0]*inner/d2, -v01[1]*inner/d2)
        p_proj = (point0[0] + v_proj[0], point0[1]+v_proj[1])

        # add p_proj
        self.start_vertex = self.roadmap.add_vertex()
        self.roadmap.vp.coords[self.start_vertex] = p_proj

        # edges from p_proj to vertex0 and vertex1
        e0 = self.roadmap.add_edge(self.start_vertex, vertex0)
        self.eweight[e0] = distance(p_proj, point0)
        e1 = self.roadmap.add_edge(self.start_vertex, vertex1)
        self.eweight[e1] = distance(p_proj, point1)

    def get_closest_edge(self, p):
        min_distance = 10**9  # ~ infinity
        e_closest = None
        for e in self.roadmap.edges():
            p0 = self.roadmap.vp.coords[e.source()]
            p1 = self.roadmap.vp.coords[e.target()]
            d = point_seg_distance(p, p0, p1)
            if d < min_distance:
                e_closest = e
                min_distance = d

        return e_closest

    def get_closest_vertex(self, p):
        min_distance = 10**9  # ~ infinity
        v_closest = None
        for v in self.roadmap.vertices():
            current_p = self.roadmap.vp.coords[v]
            current_d = distance(current_p, p)
            if current_d < min_distance:
                v_closest = v
                min_distance = current_d
        return v_closest

    def get_roadmap_segments(self):
        segments = []
        for e in self.roadmap.edges():
            p0 = self.roadmap.vp.coords[e.source()]
            p1 = self.roadmap.vp.coords[e.target()]
            segments.append((tuple(p0), tuple(p1)))
        return segments
