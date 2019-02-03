package com.yahoo.slykhachov.computationalgeometry.algos;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import com.yahoo.slykhachov.computationalgeometry.VoronoiDelauney;

public class ConvexHull {
	private ConvexHull() {}
	public static void buildConvexHull(List<VoronoiDelauney.Node> list) {	
		Optional<VoronoiDelauney.Node> opt = list.stream()
			.min((n1, n2) -> Double.compare(n1.getX(), n2.getX()));
		if (opt.isPresent()) {
			VoronoiDelauney.Node current = opt.get();
			Set<VoronoiDelauney.Node> result = new HashSet<>();
			result.add(current);
			SortedSet<Tuple<Double, VoronoiDelauney.Node>> collinearPoints = new TreeSet<>(
				(el1, el2) -> (int) (el1.getLeft() - el2.getLeft())
			);
			while (true) {
				VoronoiDelauney.Node nextTarget = list.get(0);
				for (int i = 0; i < list.size(); i++) {
					if (list.get(i) == current) {
						continue;
					}
					int val = crossProduct(current, nextTarget, list.get(i));
					if (val > 0) {
						nextTarget = list.get(i);
						collinearPoints = new TreeSet<Tuple<Double, VoronoiDelauney.Node>>(
							(el1, el2) -> (int) (el1.getLeft() - el2.getLeft())
						);
					} else {
						if (val == 0) {
							double distance1 = distance(current, nextTarget);
							double distance2 = distance(current, list.get(i));
							if (distance1 < distance2) {
								collinearPoints.add(
									new Tuple<Double, VoronoiDelauney.Node>(
										distance1,
										nextTarget
									)
								);
								nextTarget = list.get(i);
							} else {
								collinearPoints.add(
									new Tuple<Double, VoronoiDelauney.Node>(
										distance2,
										list.get(i)
									)
								);
							}
						}
					}
				}
				for (Tuple<Double, VoronoiDelauney.Node> tuple : collinearPoints) {
					VoronoiDelauney.Node n = tuple.getRight();
					if (!result.contains(n)) {
						result.add(n);
						current.addVertex(n);
						n.addVertex(current);
						current = n;
					}
				}
				if (nextTarget == opt.get()) {
					current.addVertex(nextTarget);
					nextTarget.addVertex(current);
					break;
				}
				result.add(nextTarget);
				current.addVertex(nextTarget);
				nextTarget.addVertex(current);
				current = nextTarget;
			}
		}		
	}
	private static double distance(VoronoiDelauney.Node n1, VoronoiDelauney.Node n2) {
		return Math.sqrt(Math.pow(n2.getX() - n1.getX(), 2) + Math.pow(n2.getY() - n1.getY(), 2));
	}
	private static int crossProduct(VoronoiDelauney.Node n1,
			VoronoiDelauney.Node n2, VoronoiDelauney.Node n3) {
		int x1 = n1.getX() - n2.getX();
		int x2 = n1.getX() - n3.getX();
		int y1 = n1.getY() - n2.getY();
		int y2 = n1.getY() - n3.getY();
		return x1 * y2 - x2 * y1;
	}
	private static class Tuple<A, B> {
		private A left;
		private B right;
		Tuple(A left, B right) {
			this.left = left;
			this.right = right;
		}
		A getLeft() {
			return this.left;
		}
		B getRight() {
			return this.right;
		}
	}
}
