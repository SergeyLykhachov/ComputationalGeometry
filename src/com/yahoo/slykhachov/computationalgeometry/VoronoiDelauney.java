package com.yahoo.slykhachov.computationalgeometry;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.EventQueue;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Toolkit;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.Consumer;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JToolBar;
import com.yahoo.slykhachov.computationalgeometry.algos.ConvexHull;
import com.yahoo.slykhachov.computationalgeometry.algos.Delauney;

public class VoronoiDelauney {
    private MyPanel myPanel;
    public VoronoiDelauney() {
        JFrame jf = new JFrame("VoronoiDelauney");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setMyPanel(new MyPanel());
        jf.add(createToolBar(), BorderLayout.NORTH);
        jf.add(getMyPanel());
        jf.pack();
        jf.setVisible(true);
    }
    public MyPanel getMyPanel() {
    	return this.myPanel;
    }
    public void setMyPanel(MyPanel mp) {
    	this.myPanel = mp;
    }
    private static class MyPanel extends JPanel {
    	private static final long serialVersionUID = 1L;
		static final Dimension screenDimension = Toolkit.getDefaultToolkit().getScreenSize();
		private BufferedImage image;
		private Random random;
		private Consumer<Graphics2D> paintStrategy;
		private boolean isAllowedToBeDrawnUpon;
		private List<Node> nodeList;
		private Set<Node> nodeSet;
		MyPanel() {
			setPreferredSize(screenDimension);	
			setBackground(Color.WHITE);
			random = new Random();
			addMouseListener(
				new MouseAdapter() {
					@Override
					public void mouseClicked(MouseEvent me) {
						if (me.getButton() == MouseEvent.BUTTON1) {
							if (isAllowedToBeDrawUpon()) {
								Node n = new Node(me.getX(), me.getY());
								if (!nodeSet.contains(n)) {
									nodeSet.add(n);
									nodeList.add(n);
								}	
								repaint();
							}
						}	
					}
				}
			);
			isAllowedToBeDrawnUpon = true;
			paintStrategy = g -> paintPoints(g);
			nodeList = new ArrayList<>();
			nodeSet = new HashSet<>();
		}
		@Override
		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			Graphics2D g2d = (Graphics2D) g.create();
			paintStrategy.accept(g2d);
			g2d.dispose();
		} 
		private void paintPoints(Graphics2D g2d) {
			for (Node node : nodeList) {
				g2d.fill(new Ellipse2D.Double(node.getX() - 5, node.getY() - 5, 10, 10));
			}
		}
		private void paintPointGraph(Graphics2D g2d) {
			g2d.setStroke(new BasicStroke(2.0f));
			Line2D line;
			for (Node node : getNodeList()) {
				node.setVisited(true);
				for (Node n : node.getListOfAdjacentVertices()) {
					if (!n.isVisited()) {
						line = new Line2D.Double(
							node.getX(),
							node.getY(),
							n.getX(),
							n.getY()
						);
						g2d.draw(line);
					}
				}
				g2d.fill(new Ellipse2D.Double(node.getX() - 5, node.getY() - 5, 10, 10));
			}
		}
    	private void paintVoronoi(Graphics2D g2d) {
    		if (nodeList.size() > 0) {
    			image = new BufferedImage((int) screenDimension.getWidth(), (int) screenDimension.getHeight(), BufferedImage.TYPE_INT_RGB);
        		int px[] = new int[nodeList.size()];
        		int py[] = new int[nodeList.size()];
        		int colors[] = new int[nodeList.size()];
        		for (int i = 0; i < nodeList.size(); i++) {
        		    px[i] = nodeList.get(i).getX();
        		    py[i] = nodeList.get(i).getY();
        		    colors[i] = random.nextInt(16_777_215);
				}
        		for (int x = 0; x < screenDimension.getWidth(); x++) {
        		    for (int y = 0; y < screenDimension.getHeight(); y++) {
        		        int n = 0;
        		        for (int i = 0; i < nodeList.size(); i++) {
        		            if (distance(px[i], x, py[i], y) < distance(px[n], x, py[n], y)) {
        		                n = i;
        		            }
        		        }
        		        image.setRGB(x, y, colors[n]);
        		    }
        		}
        		Graphics2D ig2d = image.createGraphics();
        		ig2d.setColor(Color.BLACK);
        		for (int i = 0; i < nodeList.size(); i++) {
        		    ig2d.fill(new Ellipse2D.Double(px[i] - 5, py[i] - 5, 10, 10));
        		}
				g2d.drawRenderedImage(image, new AffineTransform());
			}
    	}
    	private static double distance(int x1, int x2, int y1, int y2) {
        	return Math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    	}
    	void setPaintStrategy(Consumer<Graphics2D> paintStrategy) {
    		this.paintStrategy = paintStrategy;
    	}
    	void setNodeList(List<Node> list) {
    		this.nodeList = list;
    	}
    	void setNodeSet(Set<Node> set) {
    		this.nodeSet = set;
    	}
    	List<Node> getNodeList() {
    		return this.nodeList;
    	}
    	void setIsAllowedToBeDrawUpon(boolean b) {
    		this.isAllowedToBeDrawnUpon = b;
    	}
    	boolean isAllowedToBeDrawUpon() {
    		return this.isAllowedToBeDrawnUpon;
    	}
    }
    private JToolBar createToolBar() {
		JToolBar jtb = new JToolBar();
		jtb.setFloatable(false);
		jtb.setLayout(new FlowLayout());
		jtb.setBackground(Color.GRAY);
		JButton convexHullButton = new JButton("ConvexHull");
		convexHullButton.addActionListener(
			ae -> {
				MyPanel panel = getMyPanel();
				List<Node> list = panel.getNodeList();
				if (list.size() == 0) {
					return;
				}
				list.forEach(n -> n.getListOfAdjacentVertices().clear());
				ConvexHull.buildConvexHull(list);
				list.forEach(n -> n.setVisited(false));
				panel.setPaintStrategy(
					panel::paintPointGraph
				);
				panel.setIsAllowedToBeDrawUpon(false);
				panel.repaint();
		});
		JButton delauneyButton = new JButton("Delauney");
		delauneyButton.addActionListener(
			ae -> {
				List<Node> nodeList = getMyPanel().getNodeList();
				if (nodeList.size() <= 1) {
					return;
				}
				nodeList.forEach(n -> n.getListOfAdjacentVertices().clear());
				Delauney.buildDelauneyTriangulation(
					getMyPanel().getNodeList()
				);
				nodeList.forEach(n -> n.setVisited(false));
				getMyPanel().setPaintStrategy(
					g -> getMyPanel().paintPointGraph(g)
				);
				getMyPanel().setIsAllowedToBeDrawUpon(false);
				getMyPanel().repaint();
		});
		JButton voronoiButton = new JButton("Voroni");
		voronoiButton.addActionListener(
			ae -> {
				List<Node> nodeList = getMyPanel().getNodeList();
				if (nodeList.size() == 0) {
					return;
				}
				getMyPanel().setPaintStrategy(
					g -> getMyPanel().paintVoronoi(g)
				);
				getMyPanel().setIsAllowedToBeDrawUpon(false);
				getMyPanel().repaint(); 
		});
		JButton allButton = new JButton("All");
		allButton.addActionListener(
			ae -> {
				MyPanel panel = getMyPanel();
				List<Node> nodeList = panel.getNodeList();
				if (nodeList.size() == 0) {
					return;
				}
				nodeList.forEach(n -> n.getListOfAdjacentVertices().clear());
				if (nodeList.size() > 1) {
					Delauney.buildDelauneyTriangulation(
						nodeList
					);
				}
				nodeList.forEach(n -> n.setVisited(false));
				Consumer<Graphics2D> painter = panel::paintVoronoi;
				panel.setPaintStrategy(
					painter.andThen(panel::paintPointGraph)
				);
				getMyPanel().setIsAllowedToBeDrawUpon(false);
				getMyPanel().repaint();
		});
		JButton flushButton = new JButton("Flush");
		flushButton.addActionListener(
			ae -> {
				MyPanel panel = getMyPanel();
				panel.setIsAllowedToBeDrawUpon(true);
				panel.setBackground(Color.WHITE);
				panel.setNodeList(new ArrayList<>());
				panel.setNodeSet(new HashSet<>());
				panel.setPaintStrategy(
					panel::paintPoints
				);
				panel.repaint();
		});
		jtb.add(convexHullButton);
		jtb.add(delauneyButton);
		jtb.add(voronoiButton);
		jtb.add(allButton);
		jtb.add(flushButton);
		return jtb;
	}
	public static class Node {
		private int x;
		private int y;
		private boolean visited;
		private ArrayList<Node> listOfAdjacentVertices;
		public Node(int x, int y) {
			this.x = x;
			this.y = y;
			this.visited = false;
			this.listOfAdjacentVertices = new ArrayList<>();
		}
		public int getX() {
			return this.x;
		}
		public int getY() {
			return this.y;
		}
		public boolean isVisited() {
			return this.visited;
		}
		public void setVisited(boolean b) {
			this.visited = b;
		}
		public void addVertex(Node node) {
			listOfAdjacentVertices.add(node);
		}
		public void removeVertex(Node node) {
			listOfAdjacentVertices.remove(node);
		}
		public List<Node> getListOfAdjacentVertices() {
			return this.listOfAdjacentVertices;
		}
		@Override
		public int hashCode() {
			int result = Integer.hashCode(this.x);
			result = 31 * result + Integer.hashCode(this.y);
			return result;
		}
		@Override
		public boolean equals(Object o) {
			if (o == this) {
				return true;
			}
			if (!(o instanceof Node)) {
            	return false;
            }
            Node n = (Node) o;
            return n.x == this.x && n.y == this.y;
		}
		@Override
		public String toString() {
			return "{X: " + x + ", Y: " + y + "}";
		}
	}
    public static void main(String[] args) throws Exception {
        EventQueue.invokeLater(VoronoiDelauney::new);
    }
}
