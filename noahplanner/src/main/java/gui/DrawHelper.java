package gui;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.HashMap;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.collision.AABB;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.arrays.IntArray;
import org.jbox2d.pooling.arrays.Vec2Array;

/**
 * {@link org.jbox2d.callbacks.DebugDraw} の実装
 */
public class DrawHelper {
	public static int circlePoints = 13;

	private final PhysicsPanel panel;
	private final IViewportTransform viewportTransform;
	private final ColorPool cpool = new ColorPool();
	private final boolean yFlip;

	private final Vec2 saxis = new Vec2();
	private final Vec2 temp = new Vec2();
	private final IntArray xIntsPool = new IntArray();
	private final IntArray yIntsPool = new IntArray();

	public DrawHelper(PhysicsPanel panel) {
		this.panel = panel;
		viewportTransform = panel.camera.getTransform();
		yFlip = panel.camera.transform.isYFlip();
	}

	private final Vec2Array vec2Array = new Vec2Array();

	public void drawCircle(Vec2 center, float radius, Color3f color) {
		Vec2[] vecs = vec2Array.get(circlePoints);
		generateCirle(center, radius, vecs, circlePoints);
		drawPolygon(vecs, circlePoints, color);
	}

	public void drawPoint(Vec2 argPoint, float argRadiusOnScreen, Color3f argColor) {
		getWorldToScreenToOut(argPoint, sp1);
		Graphics2D g = getGraphics();

		Color c = cpool.getColor(argColor.x, argColor.y, argColor.z);
		g.setColor(c);
		sp1.x -= argRadiusOnScreen;
		sp1.y -= argRadiusOnScreen;
		g.fillOval((int) sp1.x, (int) sp1.y, (int) argRadiusOnScreen * 2, (int) argRadiusOnScreen * 2);
	}

	private final Vec2 sp1 = new Vec2();
	private final Vec2 sp2 = new Vec2();

	public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
		getWorldToScreenToOut(p1, sp1);
		getWorldToScreenToOut(p2, sp2);

		Graphics2D g = getGraphics();
		Color c = cpool.getColor(color.x, color.y, color.z);
		g.setColor(c);

		g.drawLine((int) sp1.x, (int) sp1.y, (int) sp2.x, (int) sp2.y);
	}

	public void drawAABB(AABB argAABB, Color3f color) {
		Vec2 vecs[] = vec2Array.get(4);
		argAABB.getVertices(vecs);
		drawPolygon(vecs, 4, color);
	}

	public void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
		Vec2[] vecs = vec2Array.get(circlePoints);
		generateCirle(center, radius, vecs, circlePoints);
		drawSolidPolygon(vecs, circlePoints, color);
		if (axis != null) {
			saxis.set(axis).mulLocal(radius).addLocal(center);
			drawSegment(center, saxis, color);
		}
	}

	public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
		// inside
		Graphics2D g = getGraphics();
		int[] xInts = xIntsPool.get(vertexCount);
		int[] yInts = yIntsPool.get(vertexCount);

		for (int i = 0; i < vertexCount; i++) {
			getWorldToScreenToOut(vertices[i], temp);
			xInts[i] = (int) temp.x;
			yInts[i] = (int) temp.y;
		}

		Color c = cpool.getColor(color.x, color.y, color.z, .4f);
		g.setColor(c);
		g.fillPolygon(xInts, yInts, vertexCount);

		// outside
		drawPolygon(vertices, vertexCount, color);
	}

	public void drawString(float x, float y, String s, Color3f color) {
		Graphics2D g = getGraphics();
		if (g == null) {
			return;
		}
		Color c = cpool.getColor(color.x, color.y, color.z);
		g.setColor(c);
		g.drawString(s, x, y);
	}
	
	protected Graphics2D getGraphics() {
		return panel.getDBGraphics();
	}

	private final Vec2 temp2 = new Vec2();

	public void drawTransform(Transform xf) {
		Graphics2D g = getGraphics();
		getWorldToScreenToOut(xf.p, temp);
		temp2.setZero();
		float k_axisScale = 0.4f;

		Color c = cpool.getColor(1, 0, 0);
		g.setColor(c);

		temp2.x = xf.p.x + k_axisScale * xf.q.c;
		temp2.y = xf.p.y + k_axisScale * xf.q.s;
		getWorldToScreenToOut(temp2, temp2);
		g.drawLine((int) temp.x, (int) temp.y, (int) temp2.x, (int) temp2.y);

		c = cpool.getColor(0, 1, 0);
		g.setColor(c);
		temp2.x = xf.p.x + -k_axisScale * xf.q.s;
		temp2.y = xf.p.y + k_axisScale * xf.q.c;
		getWorldToScreenToOut(temp2, temp2);
		g.drawLine((int) temp.x, (int) temp.y, (int) temp2.x, (int) temp2.y);
	}
	
	private void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color){
		if(vertexCount == 1){
			drawSegment(vertices[0], vertices[0], color);
			return;
		}
		
		for(int i=0; i<vertexCount-1; i+=1){
			drawSegment(vertices[i], vertices[i+1], color);
		}
		
		if(vertexCount > 2){
			drawSegment(vertices[vertexCount-1], vertices[0], color);
		}
	}
	
	private void getWorldToScreenToOut(Vec2 argWorld, Vec2 argScreen) {
		viewportTransform.getWorldToScreen(argWorld, argScreen);
	}

	// CIRCLE GENERATOR

	private void generateCirle(Vec2 argCenter, float argRadius, Vec2[] argPoints, int argNumPoints) {
		float inc = MathUtils.TWOPI / argNumPoints;

		for (int i = 0; i < argNumPoints; i++) {
			argPoints[i].x = (argCenter.x + MathUtils.cos(i * inc) * argRadius);
			argPoints[i].y = (argCenter.y + MathUtils.sin(i * inc) * argRadius);
		}
	}

	/**
	 * Color のプール
	 */
	public class ColorPool {

		private HashMap<ColorKey, Color> colorMap = new HashMap<ColorKey, Color>();

		private final ColorKey queryKey = new ColorKey();

		public Color getColor(float r, float g, float b, float alpha) {

			queryKey.set(r, g, b, alpha);
			if (colorMap.containsKey(queryKey)) {
				return colorMap.get(queryKey);
			} else {
				Color c = new Color(r, g, b, alpha);
				ColorKey ck = new ColorKey();
				ck.set(r, g, b, alpha);
				colorMap.put(ck, c);
				return c;
			}
		}

		public Color getColor(float r, float g, float b) {
			return getColor(r, g, b, 1);
		}
	}

	class ColorKey {
		float r, g, b, a;

		public void set(float argR, float argG, float argB) {
			set(argR, argG, argB, 1);
		}

		public void set(float argR, float argG, float argB, float argA) {
			r = argR;
			g = argG;
			b = argB;
			a = argA;
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + Float.floatToIntBits(a);
			result = prime * result + Float.floatToIntBits(b);
			result = prime * result + Float.floatToIntBits(g);
			result = prime * result + Float.floatToIntBits(r);
			return result;
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			ColorKey other = (ColorKey) obj;
			if (Float.floatToIntBits(a) != Float.floatToIntBits(other.a))
				return false;
			if (Float.floatToIntBits(b) != Float.floatToIntBits(other.b))
				return false;
			if (Float.floatToIntBits(g) != Float.floatToIntBits(other.g))
				return false;
			if (Float.floatToIntBits(r) != Float.floatToIntBits(other.r))
				return false;
			return true;
		}
	}
}