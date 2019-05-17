package com.graphhopper.isochrone.model;

import com.graphhopper.storage.SPTEntry;
import com.graphhopper.util.PointList;
import org.locationtech.jts.geom.Coordinate;

/**
 * TODO use same class as used in GTFS module?
 */
public class IsoLabel extends SPTEntry {

    public Coordinate adjCoordinate;
    public long time;
    public double distance;
    public PointList waypoints;
    
    public IsoLabel(int edgeId, int adjNode, double weight, long time, double distance) {
        super(edgeId, adjNode, weight);
        this.time = time;
        this.distance = distance;
    }
    
    public IsoLabel(IsoLabel original) {
        super(original.edge, original.adjNode, original.weight);
        this.time = original.time;
        this.distance = original.distance;
        this.adjCoordinate = original.adjCoordinate;
        this.parent = original.parent;
        this.waypoints = original.waypoints;
    }

    @Override
    public String toString() {
        return super.toString() + ", time:" + time + ", distance:" + distance;
    }
}
