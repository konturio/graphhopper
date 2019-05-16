package com.graphhopper.isochrone.algorithm;

import com.graphhopper.isochrone.model.IsoLabel;
import com.graphhopper.isochrone.model.Warehouse;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;

/**
 * Calculates minimal time to gather required supplies at any point from specified set of warehouses.
 *
 * @author Oleg Kurbatov &lt;o.v.kurbatov@gmail.com&gt;
 */
public class GatherSupplies<T> implements Function<Set<Warehouse<T>>, List<IsoLabel>> {

    private final Map<T, Double> demand;

    private final GeometryFactory geometryFactory = new GeometryFactory();

    public GatherSupplies(Map<T, Double> demand) {
        this.demand = demand;
    }

    @Override
    public List<IsoLabel> apply(Set<Warehouse<T>> warehouses) {
        Map<Integer, Coordinate> nodeIndex = warehouses.stream()
                .flatMap(w -> w.getReach().values().stream())
                .collect(Collectors.toMap(e -> e.adjNode, e -> e.adjCoordinate, (a, b) -> a));
        Long maxTime = warehouses.stream()
                .flatMap(w -> w.getReach().values().stream())
                .map(r -> r.time)
                .max((a, b) -> Long.compare(a, b))
                .map(t -> t + 1)
                .orElse(Long.MAX_VALUE);
        Map<Integer, IsoLabel> nodes = nodeIndex.entrySet().parallelStream()
                .map(entry -> {
                    Integer pointId = entry.getKey();
                    Coordinate coordinate = entry.getValue();
                    Point point = geometryFactory.createPoint(coordinate);
                    return warehouses.stream()
                            .filter(w -> w.getNeighborhood().contains(point))
                            .filter(w -> w.getReach().containsKey(pointId))
                            .findFirst()
                            .map(domestic -> {
                                Deque<Warehouse<T>> suppliers = warehouses.stream()
                                        .filter(w -> w.getReach().containsKey(pointId))
                                        .sorted((a, b) -> Long.compare(a.getReach().get(pointId).time, b.getReach().get(pointId).time))
                                        .collect(Collectors.toCollection(() -> new LinkedList<>()));
                                Map<T, Double> request = new HashMap<>(demand);
                                Warehouse<T> lastSupplier = null;
                                while (!suppliers.isEmpty() && request.values().stream().mapToDouble(v -> v).sum() > 0) {
                                    lastSupplier = suppliers.poll();
                                    Map<T, Double> assets = lastSupplier.getAssets();
                                    request = request.entrySet().stream()
                                            .filter(e -> e.getValue() > 0)
                                            .collect(Collectors.toMap(e -> e.getKey(), e -> Math.max(0, e.getValue() - assets.getOrDefault(e.getKey(), 0D))));
                                }
                                long timeToSupply;
                                if (lastSupplier == null || request.values().stream().mapToDouble(v -> v).sum() > 0) {
                                    timeToSupply = maxTime;
                                } else {
                                    timeToSupply = lastSupplier.getReach().get(pointId).time;
                                }
                                IsoLabel result = new IsoLabel(domestic.getReach().get(pointId));
                                result.time = timeToSupply;
                                return result;
                            })
                            .orElse(null); // point is outside of neighborhoods
                })
                .filter(node -> node != null)
                .filter(node -> node.time < maxTime)
                .collect(Collectors.toMap(i -> i.adjNode, i -> i));
        nodes.values().stream()
                .filter(i -> i.parent != null)
                .forEach(i -> i.parent = nodes.get(i.parent.adjNode));
        return new ArrayList<>(nodes.values());
    }

}
