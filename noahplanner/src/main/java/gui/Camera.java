package gui;

import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Vec2;

/**
 * カメラのインスタンス
 */
public class Camera {
	final OBBViewportTransform transform;
	final private Mat22 zoomInTransform;
	final private Mat22 zoomOutTransform;
	
	// 作業用インスタンス
	final private Vec2 worldDiff = new Vec2();
	final private Vec2 oldCenter = new Vec2();
	final private Vec2 newCenter = new Vec2();
	
	public Camera(float x, float y, float scale) {
		transform = new OBBViewportTransform();
		transform.setCamera(x, y, scale);
		zoomInTransform = Mat22.createScaleTransform(1.05f);
		zoomOutTransform = Mat22.createScaleTransform(0.95f);
	}
	
	public void setCenter(Vec2 center) {
		transform.setCenter(center);
	}
	
	public Vec2 getCenter() {
		return transform.getCenter();
	}
	
	public void zoomIn(Vec2 p) {
		transform.getScreenToWorld(p, oldCenter);
		transform.mulByTransform(zoomInTransform);
		transform.getScreenToWorld(p, newCenter);
		
		Vec2 worldDiff = oldCenter.subLocal(newCenter);
		moveWorld(worldDiff);
	}
	
	public void zoomOut(Vec2 p) {
		transform.getScreenToWorld(p, oldCenter);
		transform.mulByTransform(zoomOutTransform);
		transform.getScreenToWorld(p, newCenter);
		
		Vec2 worldDiff = oldCenter.subLocal(newCenter);
		moveWorld(worldDiff);
	}
	
	public void move(Vec2 screenDiff) {
		transform.getScreenVectorToWorld(screenDiff, worldDiff);
		moveWorld(worldDiff);
	}
	
	private void moveWorld(Vec2 worldDiff) {
//		if (!transform.isYFlip()) {
//		worldDiff.y = -worldDiff.y;
//	}
		transform.setCenter(transform.getCenter().addLocal(worldDiff));
	}
	
	public IViewportTransform getTransform() {
		return transform;
	}
}
