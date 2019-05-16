package com.graphhopper.resources;

import com.bedatadriven.jackson.datatype.jts.JtsModule3D;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.GraphHopper;
import com.graphhopper.isochrone.algorithm.DelaunayTriangulationIsolineBuilder;
import com.graphhopper.isochrone.algorithm.GatherSupplies;
import com.graphhopper.isochrone.algorithm.Isochrone;
import com.graphhopper.isochrone.model.IsoLabel;
import com.graphhopper.isochrone.model.Warehouse;
import com.graphhopper.json.geo.JsonFeature;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.util.*;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.index.LocationIndex;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.Helper;
import com.graphhopper.util.StopWatch;
import com.graphhopper.util.shapes.GHPoint;
import java.io.File;
import java.io.IOException;
import org.locationtech.jts.geom.GeometryFactory;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.inject.Inject;
import javax.servlet.http.HttpServletRequest;
import javax.ws.rs.*;
import javax.ws.rs.core.Context;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.UriInfo;
import java.util.*;
import java.util.stream.Collectors;
import javax.ws.rs.core.MultivaluedMap;
import jersey.repackaged.com.google.common.collect.ImmutableMap;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.triangulate.ConformingDelaunayTriangulator;
import org.locationtech.jts.triangulate.ConstraintVertex;
import org.locationtech.jts.triangulate.quadedge.QuadEdgeSubdivision;

@Path("isochrone")
public class IsochroneResource {

    private static final Logger logger = LoggerFactory.getLogger(RouteResource.class);

    private final GraphHopper graphHopper;
    private final EncodingManager encodingManager;
    private final DelaunayTriangulationIsolineBuilder delaunayTriangulationIsolineBuilder;
    private final GeometryFactory geometryFactory = new GeometryFactory();

    @Inject
    public IsochroneResource(GraphHopper graphHopper, EncodingManager encodingManager, DelaunayTriangulationIsolineBuilder delaunayTriangulationIsolineBuilder) {
        this.graphHopper = graphHopper;
        this.encodingManager = encodingManager;
        this.delaunayTriangulationIsolineBuilder = delaunayTriangulationIsolineBuilder;
    }

    @GET
    @Produces({MediaType.APPLICATION_JSON})
    public Response doGet(
            @Context HttpServletRequest httpReq,
            @Context UriInfo uriInfo,
            @QueryParam("vehicle") @DefaultValue("car") String vehicle,
            @QueryParam("buckets") @DefaultValue("1") int nBuckets,
            @QueryParam("reverse_flow") @DefaultValue("false") boolean reverseFlow,
            @QueryParam("point") GHPoint point,
            @QueryParam("result") @DefaultValue("polygon") String resultStr,
            @QueryParam("pointlist_ext_header") String extendedHeader,
            @QueryParam("time_limit") @DefaultValue("600") long timeLimitInSeconds,
            @QueryParam("distance_limit") @DefaultValue("-1") double distanceInMeter
    ) {
        if (nBuckets < 1)
            throw new IllegalArgumentException("Number of buckets has to be positive");
        if (point == null)
            throw new IllegalArgumentException("point parameter cannot be null");
        StopWatch sw = new StopWatch().start();
        List<IsoLabel> resultList = getIsolabels(vehicle, reverseFlow, point, timeLimitInSeconds, distanceInMeter, uriInfo.getQueryParameters());
        ObjectNode json = prepareResponse(resultList, resultStr, extendedHeader);
        sw.stop();
        logger.info("Request took {} seconds", sw.getSeconds());
        return Response.ok(json)
                .header("X-GH-Took", sw.getMillis())
                .build();
    }
    
    @POST
    @Path("neighbor")
    @Produces({MediaType.APPLICATION_JSON})
    @Consumes({MediaType.APPLICATION_JSON})
    public Response getNeighbor(
            @Context HttpServletRequest httpReq,
            @Context UriInfo uriInfo,
            @QueryParam("vehicle") @DefaultValue("car") String vehicle,
            @QueryParam("reverse_flow") @DefaultValue("false") boolean reverseFlow,
            @QueryParam("result") @DefaultValue("polygon") String resultStr,
            @QueryParam("pointlist_ext_header") String extendedHeader,
            @QueryParam("time_limit") @DefaultValue("600") long timeLimitInSeconds,
            @QueryParam("distance_limit") @DefaultValue("-1") double distanceInMeter,
            Double[][] points
    ) {
        if (points.length < 3) {
            throw new IllegalArgumentException("Too few points. At least three points must be specified.");
        }
        if (!encodingManager.hasEncoder(vehicle))
            throw new IllegalArgumentException("vehicle not supported:" + vehicle);
        StopWatch sw = new StopWatch().start();
        FlagEncoder encoder = encodingManager.getEncoder(vehicle);
        EdgeFilter edgeFilter = DefaultEdgeFilter.allEdges(encoder);
        LocationIndex locationIndex = graphHopper.getLocationIndex();
        List<QueryResult> queryResults = Arrays.stream(points)
                .map(p -> locationIndex.findClosest(p[0], p[1], edgeFilter))
                .filter(qr -> qr.isValid())
                .collect(Collectors.toList());
        Graph graph = graphHopper.getGraphHopperStorage();
        QueryGraph queryGraph = new QueryGraph(graph);
        queryGraph.lookup(queryResults);
        HintsMap hintsMap = new HintsMap();
        RouteResource.initHints(hintsMap, uriInfo.getQueryParameters());
        Weighting weighting = graphHopper.createWeighting(hintsMap, encoder, graph);
        Map<Coordinate, Map<Integer, IsoLabel>> isochrones = queryResults.parallelStream()
            .collect(Collectors.toMap(
                qr -> new Coordinate(qr.getQueryPoint().lon, qr.getQueryPoint().lat),
                qr -> {
                    Isochrone isochrone = new Isochrone(queryGraph, weighting, reverseFlow);
                    if (distanceInMeter > 0) {
                        isochrone.setDistanceLimit(distanceInMeter);
                    } else {
                        isochrone.setTimeLimit(timeLimitInSeconds);
                    }
                    return isochrone.search(qr.getClosestNode()).stream()
                            .collect(Collectors.toMap(l -> l.adjNode, l -> l));
                },
                (a, b) -> a));
        Map<Integer, Coordinate> nodeIndex = isochrones.values().stream()
                .flatMap(e -> e.entrySet().stream())
                .collect(Collectors.toMap(e -> e.getKey(), e -> e.getValue().adjCoordinate, (a, b) -> a));
        logger.info("Unique edges: {}", nodeIndex.size());
        Set<ConstraintVertex> sites = isochrones.keySet().stream()
                .map(p -> new ConstraintVertex(new Coordinate(p)))
                .collect(Collectors.toSet());
        ConformingDelaunayTriangulator triangulator = new ConformingDelaunayTriangulator(sites, 0.001);
        triangulator.setConstraints(Collections.EMPTY_LIST, new ArrayList(sites));
        triangulator.formInitialDelaunay();
        QuadEdgeSubdivision tin = triangulator.getSubdivision();
        Map<Polygon, Map<Integer, IsoLabel>> voronoiCells = ((List<Polygon>) tin.getVoronoiCellPolygons(geometryFactory))
                .stream()
                .collect(Collectors.toMap(p -> p, p -> isochrones.get((Coordinate) p.getUserData())));
        try {
            saveGeometries(((List<Geometry>) tin.getVoronoiCellPolygons(geometryFactory)), "voronoy");
        } catch (IOException e) {
            logger.warn("Cannot save Voronoy diagram", e);
        }
        Set<Warehouse<String>> firestations = voronoiCells.entrySet().stream()
                .map(e -> {
                    Warehouse<String> w = new Warehouse<>();
                    w.setLocation((Coordinate) e.getKey().getUserData());
                    w.setNeighborhood(e.getKey());
                    w.setReach(e.getValue());
                    w.setAssets(ImmutableMap.of("firetruck", 2D));
                    return w;
                })
                .collect(Collectors.toSet());
        GatherSupplies<String> gathering = new GatherSupplies<>(ImmutableMap.of("firetruck", 4D));
        List<IsoLabel> resultList = gathering.apply(firestations);
        logger.info("Preparing result for the graph of {} nodes", resultList.size());
        ObjectNode json = prepareResponse(resultList, resultStr, extendedHeader);
        sw.stop();
        logger.info("Request took {} seconds", sw.getSeconds());
        return Response.ok(json)
                .header("X-GH-Took", "" + sw.getMillis())
                .build();
    }

    private List<IsoLabel> getIsolabels(
            String vehicle,
            boolean reverseFlow,
            GHPoint point,
            long timeLimitInSeconds,
            double distanceInMeter,
            MultivaluedMap<String, String> hints
    ) {
        if (!encodingManager.hasEncoder(vehicle))
            throw new IllegalArgumentException("vehicle not supported:" + vehicle);
        FlagEncoder encoder = encodingManager.getEncoder(vehicle);
        EdgeFilter edgeFilter = DefaultEdgeFilter.allEdges(encoder);
        LocationIndex locationIndex = graphHopper.getLocationIndex();
        QueryResult qr = locationIndex.findClosest(point.lat, point.lon, edgeFilter);
        if (!qr.isValid())
            throw new IllegalArgumentException("Point not found:" + point);
        Graph graph = graphHopper.getGraphHopperStorage();
        QueryGraph queryGraph = new QueryGraph(graph);
        queryGraph.lookup(Collections.singletonList(qr));
        HintsMap hintsMap = new HintsMap();
        RouteResource.initHints(hintsMap, hints);
        Weighting weighting = graphHopper.createWeighting(hintsMap, encoder, graph);
        Isochrone isochrone = new Isochrone(queryGraph, weighting, reverseFlow);
        if (distanceInMeter > 0) {
            isochrone.setDistanceLimit(distanceInMeter);
        } else {
            isochrone.setTimeLimit(timeLimitInSeconds);
        }
        StopWatch sw = new StopWatch().start();
        List<IsoLabel> resultList = isochrone.search(qr.getClosestNode());
        sw.stop();
        logger.info("Spanning tree search took {} for ({}).", sw.getSeconds(), point);
        return resultList;
    }
    
    private ObjectNode prepareResponse(
            List<IsoLabel> resultList,
            String resultType,
            String extendedHeader
    ) {
        ObjectNode json = JsonNodeFactory.instance.objectNode();
        if ("polygon".equalsIgnoreCase(resultType)) {
            ArrayList<JsonFeature> features = new ArrayList<>();
            Map<Integer, Geometry> polygonShells = delaunayTriangulationIsolineBuilder.calcGeometries(resultList);
            for (Map.Entry<Integer, Geometry> polygonShell : polygonShells.entrySet()) {
                JsonFeature feature = new JsonFeature();
                HashMap<String, Object> properties = new HashMap<>();
                properties.put("time", polygonShell.getKey());
                feature.setProperties(properties);
                feature.setGeometry(polygonShell.getValue());
                features.add(feature);
            }
//            ObjectNode crs = json.putObject("crs");
//            crs.put("type", "name");
//            crs.putObject("properties").put("name", "EPSG:3857");
            json.put("type", "FeatureCollection");
            json.putPOJO("features", features);
        } else {
            Collection<String> header = new LinkedHashSet<>(Arrays.asList("longitude", "latitude", "time", "distance"));
            if (!Helper.isEmpty(extendedHeader))
                header.addAll(Arrays.asList(extendedHeader.split(",")));
            Map<Integer, IsoLabel> index = new HashMap<>(resultList.size());
            for (IsoLabel label : resultList) {
                index.put(label.adjNode, label);
            }
            List<List> items = new ArrayList(resultList.size());
            for (IsoLabel label : resultList) {
                List list = new ArrayList(header.size());
                for (String h : header) {
                    switch (h) {
                        case "distance":
                            list.add(label.distance);
                            break;
                        case "time":
                            list.add(label.time);
                            break;
                        case "node_id":
                            list.add(label.adjNode);
                            break;
                        case "edge_id":
                            list.add(label.edge);
                            break;
                        case "longitude":
                            list.add(label.adjCoordinate.x);
                            break;
                        case "latitude":
                            list.add(label.adjCoordinate.y);
                            break;
                        case "prev_longitude":
                            list.add(Optional.ofNullable((IsoLabel) label.parent).map(l -> l.adjCoordinate.x));
                            break;
                        case "prev_latitude":
                            list.add(Optional.ofNullable((IsoLabel) label.parent).map(l -> l.adjCoordinate.y));
                            break;
                        case "prev_node_id":
                            list.add(Optional.ofNullable(label.parent).map(l -> l.adjNode).orElse(0));
                            break;
                        case "prev_distance":
                            list.add(Optional.ofNullable((IsoLabel) label.parent).map(l -> l.distance).orElse(0D));
                            break;
                        case "prev_time":
                            list.add(Optional.ofNullable((IsoLabel) label.parent).map(l -> l.time).orElse(0L));
                            break;
                        default:
                            throw new IllegalArgumentException("Unknown property " + h);
                    }
                }
                items.add(list);
            }
            json.putPOJO("header", header);
            json.putPOJO("items", items);
        }
        return json;
    }
    
    private void saveGeometries(List<Geometry> lines, String name) throws IOException {
        File output = new File(name + ".json");
        if (output.exists()) {
            output.delete();
        }
        output.createNewFile();
        List<JsonFeature> features = lines.stream()
            .map(l -> {
                JsonFeature feature = new JsonFeature();
                feature.setGeometry(l);
                return feature;
            }).collect(Collectors.toList());
        ObjectMapper om = new ObjectMapper();
        om.registerModule(new JtsModule3D());
        ObjectNode json = JsonNodeFactory.instance.objectNode();
        json.put("type", "FeatureCollection");
        json.putPOJO("features", features);
        om.writeValue(output, json);
    }
}
