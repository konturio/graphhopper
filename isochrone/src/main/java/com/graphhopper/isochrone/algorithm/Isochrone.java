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

import com.carrotsearch.hppc.IntObjectHashMap;
import com.carrotsearch.hppc.procedures.IntObjectProcedure;
import com.graphhopper.coll.GHIntObjectHashMap;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.Graph;
import com.graphhopper.storage.NodeAccess;
import com.graphhopper.storage.SPTEntry;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;
import org.locationtech.jts.geom.Coordinate;

import java.util.*;

import static com.graphhopper.isochrone.algorithm.Isochrone.ExploreType.DISTANCE;
import static com.graphhopper.isochrone.algorithm.Isochrone.ExploreType.TIME;
import com.graphhopper.routing.util.DefaultEdgeFilter;

/**
 * @author Peter Karich
 */
public class Isochrone {

    enum ExploreType {TIME, DISTANCE}

    // TODO use same class as used in GTFS module?
    public class IsoLabel extends SPTEntry {

        IsoLabel(int edgeId, int adjNode, double weight, long time, double distance) {
            super(edgeId, adjNode, weight);
            this.time = time;
            this.distance = distance;
        }

        public Coordinate adjCoordinate;
        public long time;
        public double distance;

        @Override
        public String toString() {
            return super.toString() + ", time:" + time + ", distance:" + distance;
        }
    }

    private final Graph graph;
    private final Weighting weighting;
    private final boolean reverseFlow;
    private final EdgeExplorer explorer;
    private double limit = -1;
    private double finishLimit = -1;
    private ExploreType exploreType = TIME;

    public Isochrone(Graph g, Weighting weighting, boolean reverseFlow) {
        this.graph = g;
        this.weighting = weighting;
        this.reverseFlow = reverseFlow;
        if (reverseFlow) {
            explorer = graph.createEdgeExplorer(DefaultEdgeFilter.inEdges(weighting.getFlagEncoder()));
        } else {
            explorer = graph.createEdgeExplorer(DefaultEdgeFilter.outEdges(weighting.getFlagEncoder()));
        }
    }

    /**
     * Time limit in seconds
     */
    public void setTimeLimit(double limit) {
        exploreType = TIME;
        this.limit = limit * 1000;
        // we explore until all spt-entries are '>timeLimitInSeconds' 
        // and add some more into this bucket for car we need a bit more as 
        // we otherwise get artifacts for motorway endings
        this.finishLimit = this.limit + Math.max(this.limit * 0.14, 200_000);
    }

    /**
     * Distance limit in meter
     */
    public void setDistanceLimit(double limit) {
        exploreType = DISTANCE;
        this.limit = limit;
        this.finishLimit = limit + Math.max(limit * 0.14, 2_000);
    }

    /**
     * Calculates spanning tree from the node with specified id.
     *
     * @param from id of the node on the graph
     * @return spanning tree starting from the specified node
     */
    public List<IsoLabel> search(int from) {
        IntObjectHashMap<IsoLabel> fromMap = searchInternal(from);
        final List<IsoLabel> shortestPathEntries = new ArrayList<>(fromMap.size());
        final NodeAccess na = graph.getNodeAccess();
        fromMap.forEach(new IntObjectProcedure<IsoLabel>() {

            @Override
            public void apply(int nodeId, IsoLabel label) {
                double lat = na.getLatitude(nodeId);
                double lon = na.getLongitude(nodeId);
                label.adjCoordinate = new Coordinate(lon, lat);
                shortestPathEntries.add(label);
            }
        });
        return shortestPathEntries;
    }

    public List<Set<Integer>> search(int from, final int bucketCount) {
        IntObjectHashMap<IsoLabel> fromMap = searchInternal(from);
        final double bucketSize = limit / bucketCount;
        final List<Set<Integer>> list = new ArrayList<>(bucketCount);
        for (int i = 0; i < bucketCount; i++) {
            list.add(new HashSet<Integer>());
        }
        fromMap.forEach(new IntObjectProcedure<IsoLabel>() {

            @Override
            public void apply(int nodeId, IsoLabel label) {
                int bucketIndex = (int) (getExploreValue(label) / bucketSize);
                if (bucketIndex < 0) {
                    throw new IllegalArgumentException("edge cannot have negative explore value " + nodeId + ", " + label);
                } else if (bucketIndex == bucketCount) {
                    bucketIndex = bucketCount - 1;
                } else if (bucketIndex > bucketCount) {
                    return;
                }

                list.get(bucketIndex).add(nodeId);
            }
        });
        return list;
    }
    
    private IntObjectHashMap<IsoLabel> searchInternal(int from) {
        PriorityQueue<IsoLabel> fromHeap = new PriorityQueue<>(1000);
        IntObjectHashMap<IsoLabel> fromMap = new GHIntObjectHashMap<>(1000);
        IsoLabel currEdge = new IsoLabel(-1, from, 0, 0, 0);
        fromMap.put(from, currEdge);
        fromHeap.add(currEdge);
        while (!fromHeap.isEmpty() && !finished(currEdge)) {
            currEdge = fromHeap.poll();
            if (currEdge == null) {
                throw new AssertionError("Empty edge cannot happen");
            }
            int neighborNode = currEdge.adjNode;
            EdgeIterator iter = explorer.setBaseNode(neighborNode);
            while (iter.next()) {
                if (iter.getEdge() == currEdge.edge) {
                    continue;
                }
                double tmpWeight = weighting.calcWeight(iter, reverseFlow, currEdge.edge) + currEdge.weight;
                if (Double.isInfinite(tmpWeight)) continue;
                double tmpDistance = iter.getDistance() + currEdge.distance;
                long tmpTime = weighting.calcMillis(iter, reverseFlow, currEdge.edge) + currEdge.time;
                int tmpNode = iter.getAdjNode();
                IsoLabel nEdge = fromMap.get(tmpNode);
                if (nEdge == null) {
                    nEdge = new IsoLabel(iter.getEdge(), tmpNode, tmpWeight, tmpTime, tmpDistance);
                    nEdge.parent = currEdge;
                    fromMap.put(tmpNode, nEdge);
                    fromHeap.add(nEdge);
                } else if (nEdge.weight > tmpWeight) {
                    fromHeap.remove(nEdge);
                    nEdge.edge = iter.getEdge();
                    nEdge.weight = tmpWeight;
                    nEdge.distance = tmpDistance;
                    nEdge.time = tmpTime;
                    nEdge.parent = currEdge;
                    fromHeap.add(nEdge);
                }
            }
        }
        return fromMap;
    }
    
    private double getExploreValue(IsoLabel label) {
        return exploreType == TIME ? label.time : label.distance;
    }
    
    private boolean finished(IsoLabel currEdge) {
        return getExploreValue(currEdge) >= finishLimit;
    }
}
