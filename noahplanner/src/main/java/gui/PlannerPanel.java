package gui;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.swing.JFrame;

import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Rot;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

public class PlannerPanel extends PhysicsPanel implements PlannerController {
	
	interface Settings {
		// テーブルの設定
		interface Table {
			// 表面の大きさ
			Vec2 surfaceSize = new Vec2(100.0f, 0.5f);
			Vec2 legLeftSize = new Vec2(0.5f, 20.0f);
			Vec2 legRightSize = new Vec2(0.5f, 20.0f);
			float density = 1.0f;
		}
		// 箱の設定
		interface Box {
			// 箱の大きさ
			Vec2 size = new Vec2(1.0f, 1.0f);
			// 箱の形状
			PolygonShape shape = new PolygonShape() {{
				setAsBox(size.x/2, size.y/2);
			}};
			// 箱の色
			Color3f color = new Color3f(0.55f, 0.27f, 0.07f);
			// 箱の名前の描画色
			Color nameColor = new Color(1.0f, 1.0f, 1.0f);
		}
		// ロボットの速度
		float RobotOperationSpeed = 20.0f;
		// ホームポジション
		Vec2 HomePosition = new Vec2(-Box.size.x*2.5f, -10.0f);
	}
	
	// テーブルのボディ
	private Body table;
	// テーブルポインタ
	private int tablePtr;
	// 操作用ロボット
	private Robot robot;
	// 箱とその名前の対応
	private Map<String,Box> boxMap = new HashMap<String,Box>();
	// 今持っているボックス
	private Box holdingBox = null;
	// 一番高い位置にあるボックスの Y
	private float highestBoxY = 0;
	// 現在の状態
	private ArrayList<String> states = new ArrayList<String>();
	// 状態の変化リスナー
	private StatesChangeListener statesChangeListener = null;
	// 状態の表示フラグ
	private boolean showStates = true; //FIXME
	// 描画前に world に対して行う操作のキュー
	private List<Runnable> manipulations = Collections.synchronizedList(new LinkedList<Runnable>());
	
	// 再生成しないための変数
	private int panelWidth, panelHeight;

	public PlannerPanel() {
		initialize();
		camera.setCenter(Settings.HomePosition.mul(0.5f));
	}

	@Override
	protected float getInitialZoom() {
		return 20.0f;
	}

	private void initialize() {
		addContactListener(statesWatcher);
		addComponentListener(new ComponentAdapter() {
			@Override
			public void componentResized(ComponentEvent e) {
				// update
				panelWidth = getWidth();
				panelHeight = getHeight();
			}
		});
		
		// ground を作る
		{
			BodyDef bd = new BodyDef();
			bd.position.x = 0;
			bd.position.y = Settings.Table.surfaceSize.y
					+MathUtils.max(Settings.Table.legLeftSize.y,Settings.Table.legRightSize.y);
			
			EdgeShape shape = new EdgeShape();
			shape.set(new Vec2(-500.0f, 0), new Vec2(500.0f, 0));
			
			Body ground = createBody(bd);
			Fixture surface = ground.createFixture(shape, 0);
			surface.setFriction(1.0f);
		}
		
		// table を作る
		table = null;
		{	
			BodyDef bd = new BodyDef();
			bd.position.x = 0;
			bd.position.y = Settings.Table.surfaceSize.y/2;
			bd.type = BodyType.DYNAMIC;
			table = createBody(bd);

			// --- 表面
			PolygonShape surfaceShape = new PolygonShape();
			surfaceShape.setAsBox(Settings.Table.surfaceSize.x/2, Settings.Table.surfaceSize.y/2);

			Fixture surface = table.createFixture(surfaceShape, Settings.Table.density);
			surface.setFriction(1.0f);
			
			// --- 足
			Transform xfLeft = new Transform();
			xfLeft.p.x = Settings.Table.surfaceSize.x*(-0.8f)/2;
			xfLeft.p.y = Settings.Table.surfaceSize.y/2+Settings.Table.legLeftSize.y/2;
			
			Vec2 vertices[] = new Vec2[4];
			vertices[0] = Transform.mul(xfLeft, new Vec2(-Settings.Table.legLeftSize.x/2, -Settings.Table.legLeftSize.y/2));
			vertices[1] = Transform.mul(xfLeft, new Vec2( Settings.Table.legLeftSize.x/2, -Settings.Table.legLeftSize.y/2));
			vertices[2] = Transform.mul(xfLeft, new Vec2( Settings.Table.legLeftSize.x/2,  Settings.Table.legLeftSize.y/2));
			vertices[3] = Transform.mul(xfLeft, new Vec2(-Settings.Table.legLeftSize.x/2,  Settings.Table.legLeftSize.y/2));
			PolygonShape legLeftShape = new PolygonShape();
			legLeftShape.set(vertices, 4);
			
			Fixture legLeft = table.createFixture(legLeftShape, Settings.Table.density);
			legLeft.setFriction(1.0f);

			Transform xfRight = new Transform();
			xfRight.p.x = Settings.Table.surfaceSize.x*(0.8f)/2;
			xfRight.p.y = Settings.Table.surfaceSize.y/2+Settings.Table.legRightSize.y/2;

			vertices[0] = Transform.mul(xfRight, new Vec2(-Settings.Table.legRightSize.x/2, -Settings.Table.legRightSize.y/2));
			vertices[1] = Transform.mul(xfRight, new Vec2( Settings.Table.legRightSize.x/2, -Settings.Table.legRightSize.y/2));
			vertices[2] = Transform.mul(xfRight, new Vec2( Settings.Table.legRightSize.x/2,  Settings.Table.legRightSize.y/2));
			vertices[3] = Transform.mul(xfRight, new Vec2(-Settings.Table.legRightSize.x/2,  Settings.Table.legRightSize.y/2));
			PolygonShape legRightShape = new PolygonShape();
			legRightShape.set(vertices, 4);
			
			Fixture legRight = table.createFixture(legRightShape, Settings.Table.density);
			legRight.setFriction(1.0f);
			
			// ----------
			
			UserData ud = new UserData();
			ud.color.set(0.80f, 0.52f, 0.25f);
			table.setUserData(ud);
		}
		tablePtr = 0;

		// Robot を作る
		robot = new Robot();
		
		holdingBox = null;
		states.clear();
	}
	
	private void updateStates() {
		synchronized (states) {
			states.clear();
			
			synchronized (boxMap) {
				Iterator<Box> it = boxMap.values().iterator();
				while (it.hasNext()) {
					Box box = it.next();
					Box boxOn = null;
					Box boxAbove = null;
					boolean onTable = false;
					
					ContactEdge edge = box.body.getContactList();
					while (edge != null) {
						final Contact contact = edge.contact;
						// この箱のボディ
						final Body bodyBox;
						// 接触しているオブジェクトのボディ
						final Body bodyObj;
						
						if (contact.getFixtureA().getBody() == box.body) {
							bodyBox = contact.getFixtureA().getBody();
							bodyObj = contact.getFixtureB().getBody();
						} else {
							bodyBox = contact.getFixtureB().getBody();
							bodyObj = contact.getFixtureA().getBody();
						}
						
						if (contact.isTouching()) {
							onTable = false;
							
							if (bodyBox.getWorldCenter().y < bodyObj.getWorldCenter().y) {
								if (bodyObj == table) {
									onTable = true;
								} else {
									String boxOnName = findBox(bodyObj);
									if (boxOnName != null) {
											boxOn = boxMap.get(boxOnName);
									}
								}
							} else if (bodyObj.getWorldCenter().y < bodyBox.getWorldCenter().y){
								String boxAboveName = findBox(bodyObj);
								if (boxAboveName != null) {
									boxAbove = boxMap.get(boxAboveName);
								}
							}
						}
						
						edge = edge.next;
					}
					
					if (boxOn != null) {
						states.add(box.name+" on "+boxOn.name);
					}
					if (boxAbove == null) {
						states.add("clear "+box.name);
					}
					if (onTable) {
						states.add("ontable "+box.name);
					}
				}
			}
			
			if (!robot.isGrabbing) {
				states.add("handEmpty");
			}
		}
		
		Collections.sort(states);
	}
	
	private void updateHighestBoxY(Box box) {
		if (box.body.getWorldCenter().y < highestBoxY) {
			highestBoxY = box.body.getWorldCenter().y;
			
			// 少し余裕を持ってホームポジションを highestBox より上の位置に変える
			Settings.HomePosition.y = highestBoxY-Settings.Box.size.y*2.0f;
		}
	}
	
	private int nextTable() {
		return ++tablePtr;
	}
	
	// --- interface implementation ---
	
	@Override
	public void initBoxes(List<String> states) throws InterruptedException {
		// ---
		class Processor {
			final HashMap<String,String> map;
			final ArrayList<String> keys;
			public Processor(List<String> states) {
				System.out.println(states);
				final Pattern p1 = Pattern.compile("ontable (.*)");
				final Pattern p2 = Pattern.compile("(.*) on (.*)");
				final HashMap<String,String> map = new HashMap<String,String>();
				final ArrayList<String> keys = new ArrayList<String>();
				for (String s : states) {
					Matcher m;
					
					m = p1.matcher(s);
					if (m.find()) {
						final String s1 = m.group(1);
						map.put(s1, null);
						continue;
					}
					
					m = p2.matcher(s);
					if (m.find()) {
						final String s1 = m.group(1);
						final String s2 = m.group(2);
						map.put(s1, s2);
						if (!map.containsKey(s2)) {
							map.put(s2, null);
						}
					}
				}
				keys.addAll(map.keySet());
				this.map = map;
				this.keys = keys;
			}
			public void process() {
				while (0 < keys.size()) {
					final String key = keys.get(0);
					process(key);
				}
			}
			private void process(String key) {
				if (!keys.contains(key)) return;
				
				final String on = map.get(key);
				if (on == null) {
					putBox(key, null);
				} else {
					process(on);
					putBox(key, on);
				}
				keys.remove(key);
			}
		}
		// ---

		final boolean animate = isAnimating();
		stopAnimating();
		
		clear();
		new Processor(states).process();
		
		if (animate) {
			startAnimating();
		}
	}

	@Override
	public void putBox(final String name, String on) {
		final Vec2 pos = new Vec2();
		final Box onBox = (on != null && !on.isEmpty()) ? boxMap.get(on) : null;
		
		if (onBox == null) {
			final int n = nextTable();
			pos.set(Settings.Box.size.x * 2.5f * n, -Settings.Box.size.y/2);
		} else {
			pos.set(onBox.body.getWorldCenter());
			pos.addLocal(0, -Settings.Box.size.y);
		}
		
		final Box box = boxMap.get(name);
		if (box == null) {
			final Runnable r = new Runnable() {
				@Override
				public void run() {
					final Box b = new Box(name, pos);
					boxMap.put(name, b);
					updateHighestBoxY(b);
				}
			};
			if (isAnimating()) {
				manipulations.add(r);
			} else {
				r.run();
			}
		} else {
			final Box fbox = box;
			final Runnable r = new Runnable() {
				@Override
				public void run() {
					fbox.body.setTransform(pos, 0);
					updateHighestBoxY(fbox);
				}
			};
			if (isAnimating()) {
				manipulations.add(r);
			} else {
				r.run();
			}
		}
	}
	
	@Override
	public void clear() throws InterruptedException {
		clearAll();
		
		final boolean animate = isAnimating();
		stopAnimating();
		
		synchronized (boxMap) {
			boxMap.clear();
		}
		
		initialize();
		
		if (animate) {
			startAnimating();
		}
	}
	
	@Override
	public void pickup(String target) throws InterruptedException {
		if (holdingBox != null) {
			System.out.println("Currently holding the box "+holdingBox.name+".");
			return;
		}
		
		// ピックアップする箱を取得する
		holdingBox = boxMap.get(target);
		if (holdingBox != null) {
			// 箱の中心位置
			final Vec2 pos = holdingBox.body.getWorldCenter();
			// ロボット手のひらの移動先
			final Vec2 posTo = new Vec2(pos).addLocal(0, -Settings.Box.size.y/2);
			
			// 他の箱にぶつけないように手のひらを移動する
			posTo.y = Settings.HomePosition.y;
			robot.moveTo(posTo);
			robot.moveTo(pos);
			robot.grab();
			posTo.y = Settings.HomePosition.y;
			robot.moveTo(posTo);
		}
	}

	@Override
	public void place(String to) throws InterruptedException {
		if (holdingBox == null) {
			System.out.println("Currently holding no box.");
			return;
		}
		
		final Vec2 pos = new Vec2();
		final Box onBox = (to != null && !to.isEmpty()) ? boxMap.get(to) : null;
		
		if (onBox == null) {
			final int n = nextTable();
			pos.set(Settings.Box.size.x * 2.5f * n, -Settings.Box.size.y);
		} else {
			pos.set(onBox.body.getWorldCenter());
			pos.addLocal(0, -Settings.Box.size.y*1.5f);
		}
		
		final Vec2 posTo = new Vec2(pos);
		
		// 他の箱にぶつけないように手のひらを移動する
		posTo.y = Settings.HomePosition.y;
		robot.moveTo(posTo);
		posTo.y = pos.y;
		robot.moveTo(posTo);
		robot.release();
		posTo.y = Settings.HomePosition.y;
		robot.moveTo(posTo);

		updateHighestBoxY(holdingBox);
		holdingBox = null;
	}
	
	public void showStates(boolean show) {
		showStates = show;
	}
	
	public List<String> getStates() {
		updateStates();
		
		ArrayList<String> out = new ArrayList<String>(states);
		return out;
	}
	
	@Override
	public void setStatesChangeListener(StatesChangeListener l) {
		statesChangeListener = l;
	}
	
	private final ContactAdapter statesWatcher = new ContactAdapter() {
		@Override
		public void beginContact(Contact contact) { fire(); }
		@Override
		public void endContact(Contact contact) { fire(); }
		
		private void fire() {
			updateStates();
			if (statesChangeListener != null) {
				statesChangeListener.onChangeStates((List<String>) states.clone());
			}
		}
	};
	
	private String findBox(Body body) {
		synchronized (boxMap) {
			Iterator<Entry<String, Box>> it = boxMap.entrySet().iterator();
			while (it.hasNext()) {
				Entry<String, Box> entry = it.next();
				if (entry.getValue().body == body) {
					return entry.getKey();
				}
			}
		}
		return null;
	}
	
	// --------------------------------
	
	@Override
	public boolean render() {
		manipulateWorld();
		return super.render();
	}

	private void manipulateWorld() {
		if (manipulations != null && !manipulations.isEmpty()) {
			synchronized (manipulations) {
				for (Iterator<Runnable> it = manipulations.iterator(); it.hasNext(); it.remove()) {
					it.next().run();
				}
			}
		}
	}
	
	@Override
	protected void drawDebugData(Graphics2D g) {
		super.drawDebugData(g);
		
		drawBoxNames(g);
		
		if (showStates) {
			drawStates(g);
		}
	}
	
	private void drawBoxNames(Graphics2D g) {
		final Font origFont = g.getFont();
		
		// box の名前を表示する
		synchronized (boxMap) {
			final Vec2 screenSize = new Vec2();
			final Vec2 screenCenter = new Vec2();
			camera.transform.getWorldVectorToScreen(Settings.Box.size, screenSize);
			g.setFont(getFont().deriveFont(screenSize.y));
			g.setColor(Settings.Box.nameColor);
			
			Iterator<Entry<String, Box>> it = boxMap.entrySet().iterator();
			while (it.hasNext()) {
				Box box = it.next().getValue();
				camera.transform.getWorldToScreen(box.body.getWorldCenter(), screenCenter);
				final int w = g.getFontMetrics().stringWidth(box.name);
				g.drawString(box.name, screenCenter.x - w/2.0f, screenCenter.y + screenSize.y/2);
			}
		}

		g.setFont(origFont);
	}
	
	private void drawStates(Graphics2D g) {
		final FontMetrics metrics = g.getFontMetrics();
		final int size = metrics.getHeight();
		final int height = panelHeight;
		int maxStringWidth = 0;
		synchronized (states) {
			int x = 0, y = size;
			for (Iterator<String> it = states.iterator(); it.hasNext();) {
				String state = it.next();
				final int stringWidth = metrics.stringWidth(state);
				if (maxStringWidth < stringWidth) maxStringWidth = stringWidth;
				g.drawString(state, x, y);
				y += size;
				if (height < y) {
					x += maxStringWidth+20;
					y = size;
					maxStringWidth = 0;
				}
			}
		}
	}
	
	/**
	 * 箱クラス
	 */
	private class Box {
		String name;
		Body body;
		
		public Box(String name, Vec2 pos) {
			this.name = name;
			initialize(pos);
		}
		
		private void initialize(Vec2 pos) {
			BodyDef bf = new BodyDef();
			bf.type = BodyType.DYNAMIC;
			bf.position.set(pos);
			body = createBody(bf);
			
			FixtureDef fd = new FixtureDef();
			fd.shape = Settings.Box.shape;
//			fd.density = 0.00000000000001f; // これを付けると回転するようになる
			fd.friction = .1f;
			body.createFixture(fd);
			
			UserData ud = new UserData();
			ud.color.set(Settings.Box.color);
			body.setUserData(ud);
		}
	}
	
	/**
	 * ロボットのクラス
	 */
	private class Robot {
		// ロボットの手のひらの Body
		Body palm;
		// 手のひらで物を持つために使う Joint
		Joint joint;
		// 今物を持っているかどうかを表すフラグ
		boolean isGrabbing;

		public Robot() {
			initialize();
		}
		
		private void initialize() {
			// Body
			{
				// TODO
			}
			
			// Hand
			{
				BodyDef bd = new BodyDef();
				bd.type = BodyType.KINEMATIC;
				bd.position.set(Settings.HomePosition);
				bd.angle = MathUtils.PI;

				Transform xf1 = new Transform();
				xf1.q.set(0.3524f * MathUtils.PI);
				Rot.mulToOut(xf1.q, new Vec2(1.0f, 0.0f), xf1.p);

			    Vec2[] vertices = new Vec2[3];

			    PolygonShape triangle1 = new PolygonShape();
			    vertices[0] = Transform.mul(xf1, new Vec2(-1.0f, 0.0f));
			    vertices[1] = Transform.mul(xf1, new Vec2(1.0f, 0.0f));
			    vertices[2] = Transform.mul(xf1, new Vec2(0.0f, 0.5f));
			    triangle1.set(vertices, 3);

			    Transform xf2 = new Transform();
			    xf2.q.set(-0.3524f * MathUtils.PI);
			    Rot.mulToOut(xf2.q, new Vec2(-1.0f, 0.0f), xf2.p);

			    PolygonShape triangle2 = new PolygonShape();
			    vertices[0] = Transform.mul(xf2, new Vec2(-1.0f, 0.0f));
			    vertices[1] = Transform.mul(xf2, new Vec2(1.0f, 0.0f));
			    vertices[2] = Transform.mul(xf2, new Vec2(0.0f, 0.5f));
			    triangle2.set(vertices, 3);
				
				palm = createBody(bd);
				palm.createFixture(triangle1, 0.2f);
				palm.createFixture(triangle2, 0.2f);
			}
			
			// Arm
			{
//				BodyDef bd = new BodyDef();
//				bd.type = BodyType.DYNAMIC;
//				bd.position.set(0, -8.0f);
//				
//				PolygonShape shape = new PolygonShape();
//				shape.set
//				
//				FixtureDef fd = new FixtureDef();
//				fd.shape = shape;
//				
//				Body arm = createBody(bd);
//				arm.createFixture(fd);
//				
//				RevoluteJointDef jd = new RevoluteJointDef();
//				jd.initialize(arm, palm, palm.getWorldCenter());
//				
//				createJoint(jd);
			}
		}
		
		/**
		 * 今接触している物を持つ
		 * @throws InterruptedException
		 */
		public void grab() throws InterruptedException {
			if (!isGrabbing) {
				// 接触している Body を探す
				ContactEdge it = palm.getContactList();
				while (it != null) {
					Contact contact = it.contact;
					
					// 接触していたら
					if (contact.isTouching()) {
						final Body bodyA = contact.getFixtureA().getBody();
						final Body bodyB = contact.getFixtureB().getBody();
						final Manifold manifold = contact.getManifold();
						
						final RevoluteJointDef jd = new RevoluteJointDef();
						jd.initialize(bodyA, bodyB, palm.getWorldPoint(manifold.localPoint));
						jd.collideConnected = true;
						
						// ワールド描画時を避けてワールドを操作
						final Object commandLock = new Object();
						manipulations.add(new Runnable() {
							@Override
							public void run() {
								// Body をつなげる Joint を追加する
								joint = createJoint(jd);
								synchronized (commandLock) {
									commandLock.notifyAll();
								}
							}
						});
						synchronized (commandLock) {
							commandLock.wait();
						}
						isGrabbing = true;
						
						return;
					}
					it = it.next;
				}
			}
		}
		
		/**
		 * 今持っている物を離す
		 * @throws InterruptedException
		 */
		public void release() throws InterruptedException {
			if (isGrabbing) {
				// ワールド描画時を避けてワールドを操作
				final Object commandLock = new Object();
				manipulations.add(new Runnable() {
					@Override
					public void run() {
						// Body をつなげる Joint を削除する
						destroyJoint(joint);
						synchronized (commandLock) {
							commandLock.notifyAll();
						}
					}
				});
				synchronized (commandLock) {
					commandLock.wait();
				}
				isGrabbing = false;
			}
		}
		
		/**
		 * 特定の位置に移動する
		 * @param to
		 * @throws InterruptedException
		 */
		public void moveTo(final Vec2 to) throws InterruptedException {
			final Thread t = new Thread(new Runnable() {
				@Override
				public void run() {
					try {
						Vec2 diff = to.sub(palm.getWorldCenter());
						while (0.001f < diff.length()) {
							palm.setLinearVelocity(diff.mulLocal(Settings.RobotOperationSpeed));
							Thread.sleep(1);
							diff = to.sub(palm.getWorldCenter());
						}
						palm.setTransform(to, palm.getAngle());
					} catch (InterruptedException e) {
					} finally {
						palm.setLinearVelocity(new Vec2(0, 0));
					}
				}
			});
			
			// 接触したら止まるための ContactListener
			final ContactListener stopper = new ContactAdapter() {
				@Override
				public void beginContact(Contact contact) {
					if (contact.getFixtureA().getBody() == palm ||
						contact.getFixtureB().getBody() == palm) {
						// 停止する
						t.interrupt();
					}
				}
			};
			
			addContactListener(stopper);
			
			t.start();
			t.join();

			removeContactListener(stopper);
		}
	}

	public static void main(String[] args) {
		JFrame f = new JFrame();
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setBounds(0, 0, 500, 300);
		final PlannerPanel p = new PlannerPanel();
		f.getContentPane().add(p);
		f.setVisible(true);
		
		new Thread(new Runnable() {
			@Override
			public void run() {
				while (true) {
					try { Thread.sleep(1000); } catch (Exception e) {}
					System.out.println(p.getStates());
				}
			}
		}).start();
		
		p.showStates(true);
		
		List<String> states = Arrays.asList(new String[] {
				"1 on 2",
				"2 on 3",
		});
		
		try {
			p.initBoxes(states);
			
//			Thread.sleep(500);
//			p.pickup("2");
//			Thread.sleep(500);
//			p.place("3");
//			Thread.sleep(500);
//			p.pickup("1");
//			Thread.sleep(500);
//			p.place("2");
			
//			Thread.sleep(1000);
//			p.clear();
//			
//			p.stopAnimating();
//			p.putBox("a", null);
//			p.startAnimating();
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}
}
