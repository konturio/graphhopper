/*
 *  Licensed to GraphHopper GmbH under one or more contributor
 *  license agreements. See the NOTICE file distributed with this work for
 *  additional information regarding copyright ownership.
 *
 *  GraphHopper GmbH licenses this file to you under the Apache License,
 *  Version 2.0 (the "License"); you may not use this file except in
 *  compliance with the License. You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package com.graphhopper.isochrone.algorithm;

import com.bedatadriven.jackson.datatype.jts.JtsModule3D;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.graphhopper.isochrone.model.IsoLabel;
import com.graphhopper.json.geo.JsonFeature;
import com.graphhopper.reader.dem.ElevationInterpolator;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import org.locationtech.jts.geom.*;
import org.locationtech.jts.triangulate.ConformingDelaunayTriangulator;
import org.locationtech.jts.triangulate.ConstraintVertex;
import org.locationtech.jts.triangulate.quadedge.QuadEdgeSubdivision;
import org.locationtech.jts.triangulate.quadedge.Vertex;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.locationtech.jts.operation.linemerge.LineMerger;
import org.locationtech.jts.triangulate.NonEncroachingSplitPointFinder;
import org.locationtech.jts.triangulate.Segment;
import org.locationtech.jts.triangulate.quadedge.QuadEdge;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;

/**
 * @author Peter Karich
 * @author Michael Zilske
 */
public class DelaunayTriangulationIsolineBuilder {

    private static final MathTransform transform;
    private static final MathTransform inverseTransform;
    private static final ElevationInterpolator Z_INTERPOLATOR = new ElevationInterpolator();
    
    private static NonEncroachingSplitPointFinder splitPointFinder = new NonEncroachingSplitPointFinder() {
        @Override
        public Coordinate findSplitPoint(Segment seg, Coordinate encroachPt) {
            Coordinate r = super.findSplitPoint(seg, encroachPt);
            Coordinate p1 = seg.getStart();
            Coordinate p2 = seg.getEnd();
            r.z = Z_INTERPOLATOR.calculateElevationBasedOnTwoPoints(r.y, r.x, p1.y, p1.x, p1.z, p2.y, p2.x, p2.z);
            return r;
        }
    };
    
    static {
        try {
            transform = CRS.findMathTransform(CRS.decode("EPSG:4236", true), CRS.decode("EPSG:3857", true));
            inverseTransform = transform.inverse();
        } catch (FactoryException | NoninvertibleTransformException e) {
            throw new RuntimeException(e);
        }
    }
    
    /**
     * @return a list of polygons wrapping the specified points
     */
    @SuppressWarnings("unchecked")
    public List<Coordinate[]> calcList(List<List<Coordinate>> pointLists, int maxIsolines) {

        if (maxIsolines > pointLists.size()) {
            throw new IllegalStateException("maxIsolines can only be smaller or equals to pointsList");
        }

        Collection<ConstraintVertex> sites = new ArrayList<>();
        for (int i = 0; i < pointLists.size(); i++) {
            List<Coordinate> level = pointLists.get(i);
            for (Coordinate coord : level) {
                ConstraintVertex site = new ConstraintVertex(coord);
                site.setZ((double) i);
                sites.add(site);
            }
        }
        ConformingDelaunayTriangulator conformingDelaunayTriangulator = new ConformingDelaunayTriangulator(sites, 0.0);
        conformingDelaunayTriangulator.setConstraints(new ArrayList(), new ArrayList());
        conformingDelaunayTriangulator.formInitialDelaunay();
        QuadEdgeSubdivision tin = conformingDelaunayTriangulator.getSubdivision();
        for (Vertex vertex : (Collection<Vertex>) tin.getVertices(true)) {
            if (tin.isFrameVertex(vertex)) {
                vertex.setZ(Double.MAX_VALUE);
            }
        }
        ArrayList<Coordinate[]> polygonShells = new ArrayList<>();
        ContourBuilder contourBuilder = new ContourBuilder(tin);
        // ignore the last isoline as it forms just the convex hull
        for (int i = 0; i < maxIsolines; i++) {
            MultiPolygon multiPolygon = contourBuilder.computeIsoline((double) i + 0.5);
            int maxPoints = 0;
            Polygon maxPolygon = null;
            for (int j = 0; j < multiPolygon.getNumGeometries(); j++) {
                Polygon polygon = (Polygon) multiPolygon.getGeometryN(j);
                if (polygon.getNumPoints() > maxPoints) {
                    maxPoints = polygon.getNumPoints();
                    maxPolygon = polygon;
                }
            }
            if (maxPolygon == null) {
                throw new IllegalStateException("no maximum polygon was found?");
            } else {
                polygonShells.add(maxPolygon.getExteriorRing().getCoordinates());
            }
        }
        return polygonShells;
    }
    
    @SuppressWarnings("unchecked")
    public Map<Integer, Geometry> calcGeometries(List<IsoLabel> pointList) {
        GeometryFactory gf = new GeometryFactory();
        long maxTime = 0;
        Map<Integer, Coordinate> index = new HashMap<>(pointList.size());
        Map<Coordinate, Double> zIndex = new HashMap<>(pointList.size());
        for (IsoLabel p : pointList) {
            try {
                long time = p.time / 1000; // convert time to seconds
                Coordinate c = JTS.transform(p.adjCoordinate, null, transform);
                c.z = time;
                index.put(p.adjNode, c);
                zIndex.put(c, (double) time);
                if (maxTime < time) {
                    maxTime = time;
                }
            } catch (TransformException e) {
                throw new RuntimeException(e);
            }
        }
        List<LineString> lines = new ArrayList<>(pointList.size());
        for (IsoLabel p : pointList) {
            if (p.parent != null && index.containsKey(p.parent.adjNode)) {
                Coordinate[] coordinates = new Coordinate[p.waypoints.size() + 2];
                coordinates[0] = index.get(p.parent.adjNode);
                try {
                    for (int i = 0; i < p.waypoints.size(); i++) {
                        coordinates[i + 1] = JTS.transform(new Coordinate(p.waypoints.getLon(i), p.waypoints.getLat(i)), null, transform);
                    }
                } catch (TransformException e) {
                    throw new RuntimeException(e);
                }
                coordinates[p.waypoints.size() + 1] = index.get(p.adjNode);
                lines.add(gf.createLineString(coordinates));
            }
        }
        try {
            saveLines(lines, "lines-source");
        } catch (IOException e) {
            System.out.println("Cannot save lines into a file");
        }
        Geometry multyLineString = gf.buildGeometry(lines);
        MultiLineString nodedLines = (MultiLineString) multyLineString.union(gf.createPoint(multyLineString.getCoordinate()));
        lines.clear();
        LineMerger lm = new LineMerger();
        lm.add(nodedLines);
        lines.addAll(lm.getMergedLineStrings());
        List<LineString> lines3d = new ArrayList<>(lines.size());
        for (LineString line : lines) {
            Coordinate[] coords = line.getCoordinates();
            for (Coordinate c : coords) {
                if (zIndex.containsKey(c)) {
                    c.setZ(zIndex.get(c));
                }
            }
            for (int j = 0; j < coords.length; j++) {
                Coordinate c = coords[j];
                if (Double.isNaN(c.z)) {
                    if (j == 0) {
                        double z = Double.NaN;
                        int k = j + 1;
                        while (Double.isNaN(z) && k < coords.length) {
                            z = coords[k++].z;
                        }
                        c.z = z;
                    } else if (j == coords.length - 1) {
                        double z = Double.NaN;
                        int k = j - 1;
                        while (Double.isNaN(z) && k > -1) {
                            z = coords[k--].z;
                        }
                        c.z = z;
                    } else {
                        Coordinate c1 = null;
                        for (int k = j + 1; k < coords.length; k ++) {
                            if (!Double.isNaN(coords[k].z)) {
                                c1 = coords[k]; 
                                break;
                            }
                        }
                        Coordinate c2 = null;
                        for (int k = j - 1; k > -1; k--) {
                            if (!Double.isNaN(coords[k].z)) {
                                c2 = coords[k];
                                break;
                            }
                        }
                        if (c1 != null && c2 != null) {
                            c.z = Z_INTERPOLATOR.calculateElevationBasedOnTwoPoints(c.y, c.x, c1.y, c1.x, c1.z, c2.y, c2.x, c2.z);
                        } else if (c1 != null) {
                            c.z = c1.z;
                        } else if (c2 != null) {
                            c.z = c2.z;
                        }
                    }
                }
                if (Double.isNaN(c.z)) {
                    System.out.println("Z is stil NaN for " + c + " in " + Arrays.toString(coords));
                    break;
                } else if (!zIndex.containsKey(c)) {
                    zIndex.put(c, c.z);
                }
            }
            lines3d.add(gf.createLineString(coords));
        }
        try {
            saveLines(lines3d, "lines-nodded");
        } catch (IOException | RuntimeException e) {
            System.out.println("Cannot save nodded lines into a file");
        }
        Set<ConstraintVertex> sites = new HashSet<>();
        List<Segment> segments = new ArrayList<>(pointList.size());
        for (LineString line : lines3d) {
            Coordinate[] coords = line.getCoordinates();
            for (int j = 0; j < coords.length; j++) {
                Coordinate c = coords[j];
                if (Double.isNaN(c.z) && zIndex.containsKey(c)) {
                    c.z = zIndex.get(c);
                }
                if (Double.isNaN(c.z)) {
                    System.out.println("Z is stil NaN after second pass for " + c + " in " + Arrays.toString(coords));
                }
                sites.add(new ConstraintVertex(c));
                if (j > 0) {
                    segments.add(new Segment(coords[j - 1], c));
                }
            }
            line.geometryChanged();
        }
        ConformingDelaunayTriangulator triangulator = new ConformingDelaunayTriangulator(sites, 0.00001); //XXX tune tolerance value to avoid ConstraintEnforcementException 
        triangulator.setSplitPointFinder(splitPointFinder);
        triangulator.setConstraints(segments, new ArrayList(sites));
        triangulator.formInitialDelaunay();
        triangulator.enforceConstraints();
        QuadEdgeSubdivision tin = triangulator.getSubdivision();
        for (QuadEdge edge : (Collection<QuadEdge>) tin.getEdges()) {
            Vertex vertex = edge.orig();
            if (Double.isNaN(vertex.getZ())) {
                if (tin.isFrameVertex(vertex)) {
                    vertex.setZ(maxTime);
                } else {
                    vertex.interpolateZValue(edge.dest(), edge.oPrev().dest(), edge.oNext().dest());
                }
            }
            vertex = edge.dest();
            if (Double.isNaN(vertex.getZ())) {
                if (tin.isFrameVertex(vertex)) {
                    vertex.setZ(maxTime);
                } else {
                    vertex.interpolateZValue(edge.orig(), edge.dPrev().orig(), edge.dNext().orig());
                }
            }
        }
        try {
//            saveAsObj(tin);
//            saveAsArray(tin);
            saveAsGeojson(tin);
        } catch (IOException | RuntimeException e) {
            System.out.println("Cannot save debugging file file");
        }
        ContourBuilder contourBuilder = new ContourBuilder(tin);
        // ignore the last isoline as it forms just the convex hull
        int thikness = 60; // one minute
        int maxIsolines = (int) (maxTime / thikness);
        if (maxIsolines == 0) {
            maxIsolines = 1;
        }
        Map<Integer, Geometry> result = new HashMap<>();
        for (int i = 1; i <= maxIsolines; i++) {
            MultiPolygon multiPolygon = contourBuilder.computeIsoline(i * thikness);
            try {
                result.put(i, JTS.transform(multiPolygon, inverseTransform));
            } catch (TransformException e) {
                System.out.println("Cannot transform resulting isochrone");
            }
        }
        return result;
    }
    
    private void saveLines(List<LineString> lines, String name) throws IOException {
        File output = new File(name + ".json");
        if (output.exists()) {
            output.delete();
        }
        output.createNewFile();
        GeometryFactory gf = new GeometryFactory();
        ArrayList<JsonFeature> features = new ArrayList<>();
        for (LineString l : lines) {
            JsonFeature feature = new JsonFeature();
            try {
                feature.setGeometry(JTS.transform(l, inverseTransform));
            } catch (TransformException e) {
                System.out.println("Cannot transform a line");
            }
            features.add(feature);
        }
        ObjectMapper om = new ObjectMapper();
        om.registerModule(new JtsModule3D());
        ObjectNode json = JsonNodeFactory.instance.objectNode();
//        ObjectNode crs = json.putObject("crs");
//        crs.put("type", "name");
//        crs.putObject("properties").put("name", "EPSG:3857");
        json.put("type", "FeatureCollection");
        json.putPOJO("features", features);
        om.writeValue(output, json);
    }
    
    private void saveAsObj(QuadEdgeSubdivision tin) throws IOException {
        String vertex = "v %f %f %f\n";
        String face = "f %d %d %d\n";
        String object = "o %s\n";
        int i = 1;
        Map<Vertex, Integer> index = new HashMap<>();
        File output = new File("tris.obj");
        if (output.exists()) {
            output.delete();
        }
        output.createNewFile();
        try (FileWriter fw = new FileWriter(output)) {
            for (Vertex v : (Collection<Vertex>) tin.getVertices(true)) {
                Coordinate c = JTS.transform(v.getCoordinate(), null, inverseTransform);
                c.setZ(v.getCoordinate().getZ());
                index.put(v, i++);
                fw.write(String.format(vertex, c.getX(), c.getY(), c.getZ()));
            }
            fw.flush();
            i = 0;
            for (Vertex[] v : (List<Vertex[]>) tin.getTriangleVertices(false)) {
                fw.write(String.format(face, index.get(v[0]), index.get(v[1]), index.get(v[2])));
            }
            fw.flush();
        } catch (TransformException e) {
            throw new RuntimeException(e);
        }
    }
    
    private void saveAsArray(QuadEdgeSubdivision tin) throws IOException {
        File output = new File("tris-array.json");
        if (output.exists()) {
            output.delete();
        }
        output.createNewFile();
        StringBuilder sb = new StringBuilder();
        try {
            for (Vertex[] triangle : (List<Vertex[]>) tin.getTriangleVertices(false)) {
                if (triangle[0].getZ() > 15 || triangle[1].getZ() > 15 || triangle[2].getZ() > 15) {
                    continue;
                }
                for (Vertex v : triangle) {
                    Coordinate c = JTS.transform(v.getCoordinate(), null, inverseTransform);
                    c.setZ(v.getCoordinate().getZ());
                    sb.append(String.format("[%f, %f, %f], ", c.getX(), c.getY(), c.getZ()));
                }
            }
        }catch (TransformException e) {
            throw new RuntimeException(e);
        }
        try (FileWriter fw = new FileWriter(output)) {
            fw.write('[');
            fw.write(sb.substring(0, sb.length() - 2));
            fw.write(']');
        }
    }
    
    private void saveAsGeojson(QuadEdgeSubdivision tin) throws IOException {
        File output = new File("tris.json");
        if (output.exists()) {
            output.delete();
        }
        output.createNewFile();
        GeometryFactory gf = new GeometryFactory();
        ArrayList<JsonFeature> features = new ArrayList<>();
        try {
            for (Vertex[] v : (List<Vertex[]>) tin.getTriangleVertices(false)) {
                JsonFeature feature = new JsonFeature();
                Coordinate[] coords = new Coordinate[4];
                coords[0] = JTS.transform(v[0].getCoordinate(), null, inverseTransform);
                coords[1] = JTS.transform(v[1].getCoordinate(), null, inverseTransform);
                coords[2] = JTS.transform(v[2].getCoordinate(), null, inverseTransform);
                coords[3] = coords[0];
                feature.setGeometry(gf.createPolygon(coords));
                features.add(feature);
            }
        }catch (TransformException e) {
            throw new RuntimeException(e);
        }
        ObjectMapper om = new ObjectMapper();
        om.registerModule(new JtsModule3D());
        ObjectNode json = JsonNodeFactory.instance.objectNode();
//        ObjectNode crs = json.putObject("crs");
//        crs.put("type", "name");
//        crs.putObject("properties").put("name", "EPSG:3857");
        json.put("type", "FeatureCollection");
        json.putPOJO("features", features);
        om.writeValue(output, json);
    }
    
}
