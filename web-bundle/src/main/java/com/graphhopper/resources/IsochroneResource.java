package com.graphhopper.resources;

import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.GraphHopper;
import com.graphhopper.isochrone.algorithm.DelaunayTriangulationIsolineBuilder;
import com.graphhopper.isochrone.algorithm.Isochrone;
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
import java.math.BigDecimal;
import org.locationtech.jts.geom.Coordinate;
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
import org.locationtech.jts.geom.Geometry;

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
            @QueryParam("distance_limit") @DefaultValue("-1") double distanceInMeter) {

        if (nBuckets < 1)
            throw new IllegalArgumentException("Number of buckets has to be positive");

        if (point == null)
            throw new IllegalArgumentException("point parameter cannot be null");

        StopWatch sw = new StopWatch().start();

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
        RouteResource.initHints(hintsMap, uriInfo.getQueryParameters());

        Weighting weighting = graphHopper.createWeighting(hintsMap, encoder, graph);
        Isochrone isochrone = new Isochrone(queryGraph, weighting, reverseFlow);

        if (distanceInMeter > 0) {
            isochrone.setDistanceLimit(distanceInMeter);
        } else {
            isochrone.setTimeLimit(timeLimitInSeconds);
        }

        if ("polygon".equalsIgnoreCase(resultStr)) {
            List<Isochrone.IsoLabelWithCoordinates> resultList = isochrone.search(qr.getClosestNode());
            if (isochrone.getVisitedNodes() > graphHopper.getMaxVisitedNodes() / 5) {
                throw new IllegalArgumentException("Server side reset: too many junction nodes would have to explored (" + isochrone.getVisitedNodes() + "). Let us know if you need this increased.");
            }
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
            ObjectNode json = JsonNodeFactory.instance.objectNode();
//            ObjectNode crs = json.putObject("crs");
//            crs.put("type", "name");
//            crs.putObject("properties").put("name", "EPSG:3857");
            json.put("type", "FeatureCollection");
            json.putPOJO("features", features);
            sw.stop();
            logger.info("took: " + sw.getSeconds() + ", visited nodes:" + isochrone.getVisitedNodes() + ", " + uriInfo.getQueryParameters());
            return Response.ok(json)
                    .header("X-GH-Took", "" + sw.getSeconds() * 1000)
                    .build();
        } else if ("pointlist".equalsIgnoreCase(resultStr)) {
            Collection<String> header = new LinkedHashSet<>(Arrays.asList("longitude", "latitude", "time", "distance"));
            if (!Helper.isEmpty(extendedHeader))
                header.addAll(Arrays.asList(extendedHeader.split(",")));
            List<Isochrone.IsoLabelWithCoordinates> resultList = isochrone.search(qr.getClosestNode());
            Map<Integer, Isochrone.IsoLabelWithCoordinates> index = new HashMap<>(resultList.size());
            for (Isochrone.IsoLabelWithCoordinates label : resultList) {
                index.put(label.adjNodeId, label);
            }
            List<List> items = new ArrayList(resultList.size());
            for (Isochrone.IsoLabelWithCoordinates label : resultList) {
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
                            list.add(label.adjNodeId);
                            break;
                        case "edge_id":
                            list.add(label.edgeId);
                            break;
                        case "longitude":
                            list.add(label.adjCoordinate.x);
                            break;
                        case "latitude":
                            list.add(label.adjCoordinate.y);
                            break;
                        case "prev_longitude":
                            list.add(label.baseCoordinate == null ? null : label.baseCoordinate.x);
                            break;
                        case "prev_latitude":
                            list.add(label.baseCoordinate == null ? null : label.baseCoordinate.y);
                            break;
                        case "prev_node_id":
                            list.add(label.baseNodeId);
                            break;
                        case "prev_distance":
                            list.add(Optional.ofNullable(index.get(label.baseNodeId)).map(i -> i.distance).orElse(0D));
                            break;
                        case "prev_time":
                            list.add(Optional.ofNullable(index.get(label.baseNodeId)).map(i -> i.time).orElse(0L));
                            break;
                        default:
                            throw new IllegalArgumentException("Unknown property " + h);
                    }
                }
                items.add(list);
            }
            ObjectNode json = JsonNodeFactory.instance.objectNode();
            json.putPOJO("header", header);
            json.putPOJO("items", items);
            sw.stop();
            logger.info("took: " + sw.getSeconds() + ", visited nodes:" + isochrone.getVisitedNodes() + ", " + uriInfo.getQueryParameters());
            return Response.fromResponse(jsonSuccessResponse(json, sw.getSeconds()))
                    .header("X-GH-Took", "" + sw.getSeconds() * 1000)
                    .build();

        } else {
            throw new IllegalArgumentException("type not supported:" + resultStr);
        }
    }

    private Response jsonSuccessResponse(ObjectNode json, float took) {
        // If you replace GraphHopper with your own brand name, this is fine.
        // Still it would be highly appreciated if you mention us in your about page!
        final ObjectNode info = json.putObject("info");
        info.putArray("copyrights")
                .add("GraphHopper")
                .add("OpenStreetMap contributors");
        info.put("took", Math.round(took * 1000));
        return Response.ok(json).build();
    }
}
