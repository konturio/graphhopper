package com.graphhopper.isochrone.model;

import java.util.Map;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Polygon;

/**
 * Represents a warehouse that contains supplies for certain neighborhood.
 *
 * @author Oleg Kurbatov &lt;o.v.kurbatov@gmail.com&gt;
 */
public class Warehouse<T> {
    
    private Coordinate location;
    
    private Polygon neighborhood;
    
    private Map<T, Double> assets;
    
    private Map<Integer, IsoLabel> reach;

    public Coordinate getLocation() {
        return location;
    }

    public void setLocation(Coordinate location) {
        this.location = location;
    }

    public Polygon getNeighborhood() {
        return neighborhood;
    }

    public void setNeighborhood(Polygon neighborhood) {
        this.neighborhood = neighborhood;
    }

    public Map<T, Double> getAssets() {
        return assets;
    }

    public void setAssets(Map<T, Double> assets) {
        this.assets = assets;
    }

    public Map<Integer, IsoLabel> getReach() {
        return reach;
    }

    public void setReach(Map<Integer, IsoLabel> reach) {
        this.reach = reach;
    }
    
}
