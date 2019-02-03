package com.yahoo.slykhachov.computationalgeometry.algos;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.function.BiPredicate;
import java.util.stream.Stream;
import static java.util.stream.Collectors.toList;
import static com.yahoo.slykhachov.computationalgeometry.VoronoiDelauney.*;

public class Delauney {
	private Delauney() {}
	public static void buildDelauneyTriangulation(List<Node> list) {
		if (list.size() == 0) {
			return;
		}
		list.sort(
			(n1, n2) -> {
				if (n1.getX() - n2.getX() != 0) {
					return n1.getX() - n2.getX();
				} else {
					return n2.getY() - n1.getY();
				} 
		});
		buildDelauneyTriangulation(list.toArray(new Node[0]));
	}
	private static void buildDelauneyTriangulation(Node[] nodes) {	
		if (nodes.length > 3) {
			int mid = nodes.length / 2;
			Node[] leftTriangulation = new Node[mid];
			Node[] rightTriangulation = new Node[nodes.length - mid];
			System.arraycopy(nodes, 0, leftTriangulation, 0, mid);
			System.arraycopy(nodes, mid, rightTriangulation, 0, nodes.length - mid);
			buildDelauneyTriangulation(leftTriangulation);
			buildDelauneyTriangulation(rightTriangulation);
			merge(leftTriangulation, rightTriangulation);
		} else {
			switch (nodes.length) {
				case 2:
					nodes[0].addVertex(nodes[1]);
					nodes[1].addVertex(nodes[0]);
					break;
				case 3:
					nodes[0].addVertex(nodes[1]);
					nodes[0].addVertex(nodes[2]);
					nodes[1].addVertex(nodes[0]);
					nodes[1].addVertex(nodes[2]);
					nodes[2].addVertex(nodes[0]);
					nodes[2].addVertex(nodes[1]);
					break;
				default:
					throw new AssertionError();
			}
		}
	}
	private static void merge(Node[] leftTriangulation, Node[] rightTriangulation) {
		Node[] baseEdge = produceBaseEdge(leftTriangulation, rightTriangulation);
		while (true) {
			Node leftCandidate = findNewBaseEdgeEndPointCandidate(
				baseEdge[0],
				baseEdge,
				leftTriangulation,
				(edge, list) -> {
					Node leftBaseNode = edge[0];
					double baseVectorXComponent = edge[1].getX() - leftBaseNode.getX();
					double baseVectorYComponent = edge[1].getY() - leftBaseNode.getY();
					for (Node node : leftBaseNode.getListOfAdjacentVertices()) {
						if (crossProduct(
								baseVectorXComponent,
								baseVectorYComponent,
								node.getX() - leftBaseNode.getX(),
								node.getY() - leftBaseNode.getY()) < 0.0) {
							list.add(node);
						}
					}
				},
				(baseEdg, node) -> {
					Node leftBaseNode = baseEdg[0];
					double baseVectorXComponent = baseEdg[1].getX() - leftBaseNode.getX();
					double baseVectorYComponent = baseEdg[1].getY() - leftBaseNode.getY();
					double currentVectorXComponent = node.getX() - leftBaseNode.getX();
					double currentVectorYComponent = node.getY() - leftBaseNode.getY();
					return interiorAngle(
						baseVectorXComponent,
						baseVectorYComponent,
						currentVectorXComponent,
						currentVectorYComponent
					);
			});
			Node rightCandidate = findNewBaseEdgeEndPointCandidate(
				baseEdge[1],
				baseEdge,
				rightTriangulation,
				(edge, list) -> {
					Node rightBaseNode = edge[1];
					double baseVectorXComponent = edge[0].getX() - rightBaseNode.getX();
					double baseVectorYComponent = edge[0].getY() - rightBaseNode.getY();
					for (Node node : rightBaseNode.getListOfAdjacentVertices()) {
						if (crossProduct(
								baseVectorXComponent,
								baseVectorYComponent,
								node.getX() - rightBaseNode.getX(),
								node.getY() - rightBaseNode.getY()) > 0.0) {
							list.add(node);
						}
					}
				},
				(baseEdg, node) -> {
					Node rightBaseNode = baseEdg[1];
					double baseVectorXComponent = baseEdg[0].getX() - rightBaseNode.getX();
					double baseVectorYComponent = baseEdg[0].getY() - rightBaseNode.getY();
					double currentVectorXComponent = node.getX() - rightBaseNode.getX();
					double currentVectorYComponent = node.getY() - rightBaseNode.getY();
					return interiorAngle(
						baseVectorXComponent,
						baseVectorYComponent,
						currentVectorXComponent,
						currentVectorYComponent
					);
			});
			if ((rightCandidate == null) && (leftCandidate == null)) {
				return;
			} else {
				if ((rightCandidate != null) && (leftCandidate != null)) {
					Circle circle = Circle.createCircumCircleFromPoints(
							baseEdge[0].getX(),
							baseEdge[0].getY(),
							baseEdge[1].getX(),
							baseEdge[1].getY(),
							leftCandidate.getX(),
							leftCandidate.getY()
					);
					if (!circle.containsPoint(rightCandidate.getX(), rightCandidate.getY())) {
						leftCandidate.addVertex(baseEdge[1]);
						baseEdge[1].addVertex(leftCandidate);
						baseEdge[0] = leftCandidate;
					} else {
						rightCandidate.addVertex(baseEdge[0]);
						baseEdge[0].addVertex(rightCandidate);
						baseEdge[1] = rightCandidate;
					}
				} else {
					if ((rightCandidate != null) && (leftCandidate == null)) {
						rightCandidate.addVertex(baseEdge[0]);
						baseEdge[0].addVertex(rightCandidate);
						baseEdge[1] = rightCandidate;
					} else {
						leftCandidate.addVertex(baseEdge[1]);
						baseEdge[1].addVertex(leftCandidate);
						baseEdge[0] = leftCandidate;
					}
				}
			}
		}
	}
	private static Node findNewBaseEdgeEndPointCandidate(Node sideNode,
			Node[] baseEdge, 
			Node[] triangulation,
			BiConsumer<Node[], List<Node>> acceptor,
			BiFunction<Node[], Node, Double> function) {
		ArrayList<Node> candidateList = new ArrayList<>();
		acceptor.accept(baseEdge, candidateList);
		double minAngle;
		int currentIterationIndexOfMin;
		for (int i = 0, indexOfMin = 0; i < candidateList.size(); i++, indexOfMin++) {
			minAngle = Double.MAX_VALUE;
			currentIterationIndexOfMin = 0;
			for (int j = indexOfMin; j < candidateList.size(); j++) {
				Node tempNode = candidateList.get(j);
				double tempAngle = function.apply(baseEdge, tempNode);
				if (tempAngle < minAngle) {
					minAngle = tempAngle;
					currentIterationIndexOfMin = j;
				}
			}
			Node minAngleNode = candidateList.get(currentIterationIndexOfMin);
			candidateList.set(
				currentIterationIndexOfMin,
				candidateList.get(indexOfMin)
			);
			candidateList.set(
				indexOfMin,
				minAngleNode
			);
		}
		switch (candidateList.size()) {
			case 0:
				return null;
			case 1:
				return candidateList.get(0);
			default:
				for (int i = 0; i < candidateList.size(); i++) {
					if (i == candidateList.size() - 1) {
						return candidateList.get(i);
					}
					Node node = candidateList.get(i);
					Circle circumCircle = Circle.createCircumCircleFromPoints(
							baseEdge[0].getX(),
							baseEdge[0].getY(),
							baseEdge[1].getX(),
							baseEdge[1].getY(),
							node.getX(),
							node.getY()
					);
					Node nextCandidate = candidateList.get(i + 1);
					if (circumCircle.containsPoint(nextCandidate.getX(), nextCandidate.getY())) {
						sideNode.removeVertex(node);
						node.removeVertex(sideNode);
					} else {
						return node;
					}
				}
		}
		return null;
	}
	private static double crossProduct(double vX1, double vY1, double vX2, double vY2) {
	    return (vX1 * vY2) - (vX2 * vY1);
	}
	private static double interiorAngle(double vX1, double vY1, double vX2, double vY2) {
		double dotProduct = (vX1 * vX2) + (vY1 * vY2);
		double v1Magnitude = Math.sqrt(Math.pow(vX1, 2) + Math.pow(vY1, 2));
		double v2Magnitude = Math.sqrt(Math.pow(vX2, 2) + Math.pow(vY2, 2));
		return Math.toDegrees(Math.acos(dotProduct / (v1Magnitude * v2Magnitude)));
	}
	@FunctionalInterface
	interface QuadFunction<A, B, C, D> {
		D apply(A a, B b, C c);
	}
	private static Node[] produceBaseEdge(Node[] leftTriangulation,
			Node[] rightTriangulation) {
		List<Node> leftConvexHull = Stream.of(leftTriangulation)
			.map(n -> new Node(n.getX(), n.getY()))
			.collect(toList());
		List<Node> rightConvexHull = Stream.of(rightTriangulation)
			.map(n -> new Node(n.getX(), n.getY()))
			.collect(toList());	
		ConvexHull.buildConvexHull(leftConvexHull);
		ConvexHull.buildConvexHull(rightConvexHull);
		QuadFunction<Node, Node, BiPredicate<Node, Node>, Node> func1 = (baseEdgeNode, candidateNode, predicate) -> {	
			double angle1 = interiorAngle(
				(double) (candidateNode.getX() - baseEdgeNode.getX()),
				0.0,
				(double) (candidateNode.getX() - baseEdgeNode.getX()),
				(double) (candidateNode.getY() - baseEdgeNode.getY())
			);
			List<Node> list = candidateNode.getListOfAdjacentVertices();
			Node node2 = list.get(0);
			Node node3 = list.get(1);
			boolean b2 = predicate.test(node2, candidateNode);
			boolean b3 = predicate.test(node3, candidateNode);
			double angle2 = interiorAngle(
				(double) (node2.getX() - baseEdgeNode.getX()),
				0.0,
				(double) (node2.getX() - baseEdgeNode.getX()),
				(double) (node2.getY() - baseEdgeNode.getY())
			);
			double angle3 = interiorAngle(
				(double) (node3.getX() - baseEdgeNode.getX()),
				0.0,
				(double) (node3.getX() - baseEdgeNode.getX()),
				(double) (node3.getY() - baseEdgeNode.getY())
			);	
			if (b2 && b3) {
				if (angle2 < angle3) {
					if (angle2 < angle1) {
						return node2;
					} else {
						return null;
					}
				} else {
					if (angle3 < angle1) {
						return node3;
					} else {
						return null;
					}
				}
			} else {
				if (b2 && !b3) {
					return angle2 < angle1 ? node2 : null;
				} else {
					if (!b2 && b3) {
						return angle3 < angle1 ? node3 : null;
					} else {
						return null;
					}
				}
			}
		};
		Function<Node, Node> func2 = (n) -> {
			List<Node> list = n.getListOfAdjacentVertices();
			Node node1 = list.get(0);
			Node node2 = list.get(1);			
			boolean b1 = node1.getX() > n.getX();
			boolean b2 = node2.getX() > n.getX();
			if (b1 && b2) {
				return interiorAngle(
					(double) (node1.getX() - n.getX()),
					0.0,
					(double) (node1.getX() - n.getX()),
					(double) (node1.getY() - n.getY()))
					< interiorAngle(
						(double) (node2.getX() - n.getX()),
						0.0,
						(double) (node2.getX() - n.getX()),
						(double) (node2.getY() - n.getY()))
					? node1 : node2;
			} else {
				if (b1 && !b2) {
					return node1;
				} else {
					if (!b1 && b2) {
						return node2;
					} else {
						return null;
					}
				}
			}
		};
		Function<Node, Node> func3 = (n) -> {
			List<Node> list = n.getListOfAdjacentVertices();
			Node node1 = list.get(0);
			Node node2 = list.get(1);
			boolean b1 = node1.getX() < n.getX();
			boolean b2 = node2.getX() < n.getX();
			if (b1 && b2) {
				return interiorAngle(
					(double) (n.getX() - node1.getX()),
					0.0,
					(double) (n.getX() - node1.getX()),
					(double) (n.getY() - node1.getY()))
					< interiorAngle(
						(double) (n.getX() - node2.getX()),
						0.0,
						(double) (n.getX() - node2.getX()),
						(double) (n.getY() - node2.getY()))
					? node1 : node2;
			} else {
				if (b1 && !b2) {
					return node1;
				} else {
					if (!b1 && b2) {
						return node2;
					} else {
						return null;
					}
				}
			}
		};
		Node leftSideCandidate = leftConvexHull.stream()
			.min(
				(n1, n2) -> {
					if (Integer.compare(n1.getY(), n2.getY()) != 0) {
						return Integer.compare(n2.getY(), n1.getY());
					} else {
						return Integer.compare(n2.getX(), n1.getX());
					}
			}).get();
		Node rightSideCandidate = rightConvexHull.stream()
			.min(
				(n1, n2) -> {
					if (Integer.compare(n1.getY(), n2.getY()) != 0) {
						return Integer.compare(n2.getY(), n1.getY());
					} else {
						return Integer.compare(n1.getX(), n2.getX());
					}
			}).get();
		Node[] baseEdge = new Node[] {leftSideCandidate, rightSideCandidate};
		Node[] tempEdge;
		Node tempNode;
		if (Integer.compare(leftSideCandidate.getY(), rightSideCandidate.getY()) > 0) {//left node is lower than the right
			boolean continueOuterLoop;
			do {	
				tempNode = rightSideCandidate;
				do {
					if (rightConvexHull.size() == 2) {
						Node otherRight = baseEdge[1].getListOfAdjacentVertices().get(0);
						if (baseEdge[1].getX() < otherRight.getX()) {
							double angle1 = interiorAngle(
								(double) (otherRight.getX() - baseEdge[0].getX()),
								0.0,
								(double) (otherRight.getX() - baseEdge[0].getX()),
								(double) (baseEdge[0].getY() - otherRight.getY())
							);
							double angle2 = interiorAngle(
								(double) (otherRight.getX() - baseEdge[1].getX()),
								0.0,
								(double) (otherRight.getX() - baseEdge[1].getX()),
								(double) (baseEdge[1].getY() - otherRight.getY())
							);
							baseEdge[1] = angle1 > angle2 ? otherRight : baseEdge[1];
							break;
						} else {
							break;
						}
					}
					tempNode = func1.apply(
						baseEdge[0],
						tempNode,
						(n1 ,n2) -> n1.getX() > n2.getX()
					);
					if (tempNode != null) {
						tempEdge = new Node[] {baseEdge[0], tempNode};
						if (!doesEdgeIntersectTriangulation(tempEdge, rightConvexHull.toArray(new Node[0]))) {
							baseEdge[1] = tempNode;
						}
					}
				} while (tempNode != null);
				if (leftConvexHull.size() == 2) {
					Node otherLeft = baseEdge[0].getListOfAdjacentVertices().get(0);
					if (baseEdge[0].getX() < otherLeft.getX()
							&& otherLeft.getY() > baseEdge[1].getY()) {
						double angle1 = interiorAngle(
							(double) (otherLeft.getX() - baseEdge[1].getX()),
							0.0,
							(double) (otherLeft.getX() - baseEdge[1].getX()),
							(double) (otherLeft.getY() - baseEdge[1].getY())
						);
						double angle2 = interiorAngle(
							(double) (baseEdge[0].getX() - baseEdge[1].getX()),
							0.0,
							(double) (baseEdge[0].getX() - baseEdge[1].getX()),
							(double) (baseEdge[0].getY() - baseEdge[1].getY())
						);
						baseEdge[0] = angle1 > angle2 ? otherLeft : baseEdge[0];
					}
					continueOuterLoop = false;
				} else {
					if (doesEdgeIntersectTriangulation(baseEdge, leftConvexHull.toArray(new Node[0]))) {
						baseEdge[0] = func2.apply(baseEdge[0]);
						continueOuterLoop = true;
					} else {
						continueOuterLoop = false;
					}
				}
			} while (continueOuterLoop);
		} else {
			if (Integer.compare(leftSideCandidate.getY(), rightSideCandidate.getY()) < 0) {//right node is lower than the left
				boolean continueOuterLoop;
				do {	
					tempNode = leftSideCandidate;
					do {
						if (leftConvexHull.size() == 2) {
							Node otherLeft = baseEdge[0].getListOfAdjacentVertices().get(0);
							if (baseEdge[0].getX() > otherLeft.getX()) {
								double angle1 = interiorAngle(
									(double) (baseEdge[1].getX() - otherLeft.getX()),
									0.0,
									(double) (baseEdge[1].getX() - otherLeft.getX()),
									(double) (baseEdge[1].getY() - otherLeft.getY())
								);
								double angle2 = interiorAngle(
									(double) (baseEdge[0].getX() - otherLeft.getX()),
									0.0,
									(double) (baseEdge[0].getX() - otherLeft.getX()),
									(double) (baseEdge[0].getY() - otherLeft.getY())
								);
								baseEdge[0] = angle1 > angle2 ? otherLeft : baseEdge[0];
								break;
							} else {
								break;
							}
						}
						tempNode = func1.apply(
							baseEdge[1],
							tempNode,
							(n1, n2) -> n1.getX() < n2.getX()
						);
						if (tempNode != null) {
							tempEdge = new Node[] {tempNode, baseEdge[1]};
							if (!doesEdgeIntersectTriangulation(tempEdge, leftConvexHull.toArray(new Node[0]))) {
								baseEdge[0] = tempNode;
							}
						}
					} while (tempNode != null);
					if (rightConvexHull.size() == 2) {
						Node otherRight = baseEdge[1].getListOfAdjacentVertices().get(0);
						if (baseEdge[1].getX() > otherRight.getX()
								&& otherRight.getY() > baseEdge[0].getY()) {		
							double angle1 = interiorAngle(
								(double) (otherRight.getX() - baseEdge[0].getX()),
								0.0,
								(double) (otherRight.getX() - baseEdge[0].getX()),
								(double) (otherRight.getY() - baseEdge[0].getY())
							);
							double angle2 = interiorAngle(
								(double) (baseEdge[1].getX() - baseEdge[0].getX()),
								0.0,
								(double) (baseEdge[1].getX() - baseEdge[0].getX()),
								(double) (baseEdge[1].getY() - baseEdge[0].getY())
							);
							baseEdge[1] = angle1 > angle2 ? otherRight : baseEdge[1];
						}
						continueOuterLoop = false;
					} else {
						if (doesEdgeIntersectTriangulation(baseEdge, rightConvexHull.toArray(new Node[0]))) {	
							baseEdge[1] = func3.apply(baseEdge[1]);
							continueOuterLoop = true;
						} else {
							continueOuterLoop = false;
						}
					}
				} while (continueOuterLoop);
			}
		}
		Node left = extractOriginalNode(baseEdge[0], leftTriangulation);
		Node right = extractOriginalNode(baseEdge[1], rightTriangulation);
		left.addVertex(right);
		right.addVertex(left);
		return new Node[] {left, right};
	}
	private static Node extractOriginalNode(Node node, Node[] originalTriangulation) {
		for (Node original : originalTriangulation) {
			if (original.getX() == node.getX() && original.getY() == node.getY()) {
				return original;
			}
		}
		throw new NullPointerException();
	}
	private static boolean doesEdgeIntersectTriangulation(Node[] baseEdge,
			Node[] triangulation) {
		ArrayList<Node[]> edges = new ArrayList<>();
		for (Node node : triangulation) {
			node.setVisited(true);
			for (Node n : node.getListOfAdjacentVertices()) {
				if (!n.isVisited()) {
					edges.add(
						new Node[] {node, n}
					);
				}
			}
		}
		for (Node node : triangulation) {
			node.setVisited(false);
		}
		Line2D baseEdgeLine = new Line2D.Double(
			baseEdge[0].getX(),
			baseEdge[0].getY(),
			baseEdge[1].getX(),
			baseEdge[1].getY()
		);
		Line2D triangulationLine;
		for (Node[] edge : edges) {
			triangulationLine = new Line2D.Double(
				edge[0].getX(),
				edge[0].getY(),
				edge[1].getX(),
				edge[1].getY()
			);
			if (baseEdgeLine.intersectsLine(triangulationLine)) {	
				if (doLinesShareEndPoint(baseEdgeLine, triangulationLine)) {
					continue;
				} else {
					return true;
				}						
			}
		}
		return false;
	}
	private static boolean doLinesShareEndPoint(Line2D l1, Line2D l2) {
		return (l1.getP1().equals(l2.getP1()) || l1.getP1().equals(l2.getP2()))
			|| (l1.getP2().equals(l2.getP1()) || l1.getP2().equals(l2.getP2()));
	}
	private static class Circle {
		private double x;
		private double y;
		private double radius;
		private Circle(double x, double y, double radius) {
			this.x = x;
			this.y = y;
			this.radius = radius;
		}
		boolean containsPoint(double pointX, double pointY) {
			return Math.pow((pointX - this.x), 2) + Math.pow((pointY - this.y), 2) <= Math.pow(radius, 2);
		}
		static Circle createCircumCircleFromPoints(
    			double p1x,
    			double p1y,
    			double p2x,
    			double p2y,
    			double p3x,
    			double p3y) {	
    		double TOL = 0.0000001;
    		double offset = Math.pow(p2x, 2) + Math.pow(p2y, 2);
    		double bc = (Math.pow(p1x, 2) + Math.pow(p1y, 2) - offset ) / 2.0;
    		double cd = (offset - Math.pow(p3x, 2) - Math.pow(p3y, 2)) / 2.0;
    		double det = (p1x - p2x) * (p2y - p3y) - (p2x - p3x) * (p1y - p2y); 
    		if (Math.abs(det) < TOL) {
    			throw new IllegalArgumentException();
    		}
    		double idet = 1 / det;
    		double centerx = (bc * (p2y - p3y) - cd * (p1y - p2y)) * idet;
    		double centery = (cd * (p1x - p2x) - bc * (p2x - p3x)) * idet;
    		double radius = Math.sqrt(Math.pow(p2x - centerx, 2) + Math.pow(p2y - centery, 2));
    		return new Circle(centerx, centery, radius);
    	}
	}
}
