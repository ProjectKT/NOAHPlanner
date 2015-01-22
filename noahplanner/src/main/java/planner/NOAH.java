package planner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class NOAH {

	// メンバ

	// パラメーターを保存するクラス
	// MultiThreadで実装しようとした頃の名残
	NOAHParameters nPara;

	// 最初のプラン
	ArrayList<Node> plan = new ArrayList<Node>();
	int nodecount = 1;

	// 最終結果のプラン
	ArrayList<String> resultPlan = new ArrayList<String>();

	// JointSのリスト
	// JointSは前方が一個、後方が複数の結合部
	ArrayList<JointS> ss = new ArrayList<JointS>();

	// JointSのリスト
	// JointSは前方が一個、後方が複数の結合部
	ArrayList<JointJ> js = new ArrayList<JointJ>();

	/**
	 * A on Bが合ったとき underにB、overにAが入る
	 */
	ArrayList<String> under = new ArrayList<String>();
	ArrayList<String> over = new ArrayList<String>();

	// 存在しているオブジェクトのリスト
	ArrayList<String> objects = new ArrayList<String>();
	
	//クリアの数
	int clearNum = 0;

	public static void main(String args[]) {
		(new NOAH()).planning();
	}

	/**
	 * 初期状態とゴール状態が与えられない場合のコンストラクタ 初期状態とゴール状態はこちらが決めたものに勝手にセットされる
	 */
	public NOAH() {
		nPara = new NOAHParameters(initOperators(), initGoalState(),
				initCurrentState());
		setObjects();
	}

	/**
	 * 初期状態とゴール状態が与えられなる場合のコンストラクタ
	 * 
	 * @param goalState
	 *            　ゴール状態
	 * @param initialState
	 *            　初期状態
	 */
	public NOAH(ArrayList<String> goalState, ArrayList<String> initialState) {
		nPara = new NOAHParameters(initOperators(), goalState, initialState);
	}

	/**
	 * plannerのなごり現在全く使ってない
	 */
	private ArrayList<Operator> initOperators() {
		ArrayList<Operator> operators = new ArrayList<Operator>();

		// OPERATOR 1
		// / NAME
		String name1 = new String("Place ?x on ?y");
		// / IF
		ArrayList<String> ifList1 = new ArrayList<String>();
		ifList1.add(new String("clear ?y"));
		ifList1.add(new String("holding ?x"));
		// / ADD-LIST
		ArrayList<String> addList1 = new ArrayList<String>();
		addList1.add(new String("?x on ?y"));
		addList1.add(new String("clear ?x"));
		addList1.add(new String("handEmpty"));
		// / DELETE-LIST
		ArrayList<String> deleteList1 = new ArrayList<String>();
		deleteList1.add(new String("clear ?y"));
		deleteList1.add(new String("holding ?x"));
		Operator operator1 = new Operator(name1, ifList1, addList1, deleteList1);
		operators.add(operator1);

		// OPERATOR 2
		// / NAME
		String name2 = new String("remove ?x from on top ?y");
		// / IF
		List<String> ifList2 = new ArrayList<String>();
		ifList2.add(new String("?x on ?y"));
		ifList2.add(new String("clear ?x"));
		ifList2.add(new String("handEmpty"));
		// / ADD-LIST
		ArrayList<String> addList2 = new ArrayList<String>();
		addList2.add(new String("clear ?y"));
		addList2.add(new String("holding ?x"));
		// / DELETE-LIST
		ArrayList<String> deleteList2 = new ArrayList<String>();
		deleteList2.add(new String("?x on ?y"));
		deleteList2.add(new String("clear ?x"));
		deleteList2.add(new String("handEmpty"));
		Operator operator2 = new Operator(name2, ifList2, addList2, deleteList2);
		operators.add(operator2);

		// OPERATOR 3
		// / NAME
		String name3 = new String("pick up ?x from the table");
		// / IF
		ArrayList<String> ifList3 = new ArrayList<String>();
		ifList3.add(new String("ontable ?x"));
		ifList3.add(new String("clear ?x"));
		ifList3.add(new String("handEmpty"));
		// / ADD-LIST
		ArrayList<String> addList3 = new ArrayList<String>();
		addList3.add(new String("holding ?x"));
		// / DELETE-LIST
		ArrayList<String> deleteList3 = new ArrayList<String>();
		deleteList3.add(new String("ontable ?x"));
		deleteList3.add(new String("clear ?x"));
		deleteList3.add(new String("handEmpty"));
		Operator operator3 = new Operator(name3, ifList3, addList3, deleteList3);
		operators.add(operator3);

		// OPERATOR 4
		// / NAME
		String name4 = new String("put ?x down on the table");
		// / IF
		ArrayList<String> ifList4 = new ArrayList<String>();
		ifList4.add(new String("holding ?x"));
		// / ADD-LIST
		ArrayList<String> addList4 = new ArrayList<String>();
		addList4.add(new String("ontable ?x"));
		addList4.add(new String("clear ?x"));
		addList4.add(new String("handEmpty"));
		// / DELETE-LIST
		ArrayList<String> deleteList4 = new ArrayList<String>();
		deleteList4.add(new String("holding ?x"));
		Operator operator4 = new Operator(name4, ifList4, addList4, deleteList4);
		operators.add(operator4);

		return operators;
	}

	/**
	 * 初期状態がコンストラクタで与えられなかったときに使う初期状態
	 * 
	 * @return
	 */
	private ArrayList<String> initGoalState() {
		ArrayList<String> goalList = new ArrayList<String>();

		goalList.add("clear 2");
		goalList.add("clear 1");

		
		goalList.add("2 on 5");
		goalList.add("1 on 4");
		goalList.add("4 on 3");
		goalList.add("3 on 6");

/*
		goalList.add("clear A");
		goalList.add("A on 1");
		goalList.add("1 on 2");
		goalList.add("2 on 3");
		
		goalList.add("clear B");
		goalList.add("B on C");

		

		goalList.add("clear 4");
		goalList.add("4 on 5");
		goalList.add("5 on 6");
		
		goalList.add("clear 8");
		goalList.add("8 on 7");
		
		goalList.add("clear 9");
		goalList.add("ontable 9");

		goalList.add("clear 13");
		goalList.add("13 on 10");
		goalList.add("10 on 11");
		goalList.add("11 on 12");
		goalList.add("12 on 14");
		goalList.add("14 on 15");
		goalList.add("15 on 16");
		*/
		return goalList;
	}

	/**
	 * ゴール状態がコンストラクタで与えられなかった時用のゴール状態
	 * 
	 * @return
	 */
	private ArrayList<String> initCurrentState() {
		ArrayList<String> initialState = new ArrayList<String>();

		/*
		 * initialState.add("clear A"); initialState.add("clear D");
		 * 
		 * initialState.add("A on B"); initialState.add("B on C");
		 * initialState.add("D on E");
		 */

		
		initialState.add("clear 1");
		initialState.add("clear 4");

		initialState.add("1 on 2");
		initialState.add("2 on 3");
		initialState.add("4 on 5");
		initialState.add("5 on 6");

		/*
		initialState.add("clear 1");
		initialState.add("1 on 2");
		initialState.add("2 on 3");
		
		initialState.add("clear A");
		initialState.add("A on B");
		initialState.add("B on 4");
		initialState.add("4 on 5");
		initialState.add("5 on 6");
		
		initialState.add("clear C");
		initialState.add("C on 7");
		initialState.add("7 on 8");
		initialState.add("8 on 9");
		
		initialState.add("clear 13");
		initialState.add("13 on 10");
		initialState.add("10 on 11");
		initialState.add("11 on 12");
		initialState.add("12 on 14");
		initialState.add("14 on 15");
		initialState.add("15 on 16");
		 */
		return initialState;
	}

	/**
	 * GUI上でいじった初期状態を適用する
	 * 
	 * @param State
	 */
	public void setCurrentState(ArrayList<String> State) {
		nPara.setCurrentState(State);
	}

	/**
	 * GUI上でいじったゴール状態を適用する
	 * 
	 * @param GoalList
	 */
	public void setGoalState(ArrayList<String> GoalList) {
		nPara.setGoalState(GoalList);
	}

	/**
	 * GUI上で初期状態を取得する時用
	 * 
	 * @return
	 */
	public ArrayList<String> getCurrentState() {
		return nPara.getCurrentState();
	}

	/**
	 * GUI上でゴール状態を取得する時用
	 * 
	 * @return
	 */
	public ArrayList<String> getGoalState() {
		return nPara.getGoalState();
	}

	/**
	 * グローバル変数を初期化するメソッド
	 */
	private void initMember() {
		// 最初のプラン
		plan = new ArrayList<Node>();
		nodecount = 1;

		// 最終結果のプラン
		resultPlan = new ArrayList<String>();

		// JointSのリスト
		// JointSは前方が一個、後方が複数の結合部
		ss = new ArrayList<JointS>();

		// JointSのリスト
		// JointSは前方が一個、後方が複数の結合部
		js = new ArrayList<JointJ>();

		/**
		 * A on Bが合ったとき underにB、overにAが入る
		 */
		under = new ArrayList<String>();
		over = new ArrayList<String>();

		objects = new ArrayList<String>();
		setObjects();
		
		clearNum =0;
	}

	/**
	 * オブジェクトの登録
	 * 
	 */

	private void setObjects() {
		Pattern p1 = Pattern.compile("(.*) on (.*)");
		Pattern p2 = Pattern.compile("clear (.*)");
		for (String str : getCurrentState()) {
			Matcher m1 = p1.matcher(str);
			Matcher m2 = p2.matcher(str);

			if (m1.find()) {
				if (!objects.contains(m1.group(1))) {
					objects.add(m1.group(1));
				}
				if (!objects.contains(m1.group(2))) {
					objects.add(m1.group(2));
				}
			}

			if (m2.find()) {
				if (!objects.contains(m2.group(1))) {
					objects.add(m2.group(1));
				}
			}

		}
	}

	/**
	 * ゴール状態をプランに登録する 具体的には Goalに[A on B,B on C]があったら
	 * 
	 * | - A on B - | Start --- JointS --- ----JointJ --- Goal | - B on C - |
	 * 
	 * ができる
	 */
	private void initialPlanning() {
		ArrayList<String> goalState = nPara.getGoalState();

		// 先頭のJointSを作る
		// 先頭のJointSは前がStartノード
		JointS s = new JointS();
		s.changeForward("Start");

		// 末尾のJointJを作る
		// 末尾のJointJは後がGoalノード
		JointJ j = new JointJ();
		Node goal = new Node("Goal", 0, j, null);
		j.changeBack(goal);
		
		ArrayList<String> clearObjects = new ArrayList<String>();
		
		// ゴール状態の数分ループ
		Pattern p = Pattern.compile("(.*) on (.*)");
		Pattern p2 = Pattern.compile("ontable (.*)");
		Pattern p3 = Pattern.compile("clear (.*)");
		for (String str : goalState) {
			System.out.println(str);
			Matcher m = p.matcher(str);
			if (m.find()) {
				Node newNode = new Node(str, nodecount++, s, j);
				plan.add(newNode);
				s.addBack(newNode);
				j.addForward(newNode);
			}
			Matcher m3 = p3.matcher(str);
			if(m3.find()){
				clearObjects.add(m3.group(1));
			}
		}
		
		for(String str:goalState){
		Matcher m2 = p2.matcher(str);
		if(m2.find()){
			if(clearObjects.contains(m2.group(1))){
				Node newNode = new Node("clear "+m2.group(1), nodecount++, s, j);
				System.out.println("alone objects: "+newNode.getNodeName());
				plan.add(newNode);
				s.addBack(newNode);
				j.addForward(newNode);
			}
		}
		}
		ss.add(s);
		js.add(j);

	}


	/**
	 * ゴール状態のままでは不十分なのでプランを展開する 具体的には A on Bを
	 * 
	 * |- Clear A -| JointS--- ---JointJ --- Place A on B |- Clear B -|
	 * 
	 * に展開する
	 */
	private void expandPlan() {
		ArrayList<Node> newplan = new ArrayList<Node>();
		for (Node node : plan) {
			if (node.getNodeName().contains("on")) {
				JointS s = new JointS();
				s.changeForward(node.getForward());
				ss.add(s);
				JointJ j = new JointJ();
				js.add(j);

				ss.get(0).removeBack(node);
				ss.get(0).addBack(s);

				js.get(0).removeForward(node);

				Pattern p = Pattern.compile("(.*) on (.*)");
				Matcher m = p.matcher(node.getNodeName());
				if (m.find()) {
					over.add(m.group(1));
					under.add(m.group(2));
					Node newNode1 = new Node("clear " + m.group(1),
							nodecount++, s, j);
					s.addBack(newNode1);
					j.addForward(newNode1);
					Node newNode2 = new Node("clear " + m.group(2),
							nodecount++, s, j);
					s.addBack(newNode2);
					j.addForward(newNode2);
					Node newNode3 = new Node("Place " + m.group(1) + " on "
							+ m.group(2), nodecount++, j, node.getBack());
					j.changeBack(newNode3);
					js.get(0).addForward(newNode3);

					newplan.add(newNode1);
					newplan.add(newNode2);
					newplan.add(newNode3);
				}
			}else{
				System.out.println("ontalbe       "+node);
				newplan.add(node);
				clearNum++;
			}
		}
		plan.clear();
		plan = newplan;
	}

	// tableから一番近いonを行う際のPrace情報
	ArrayList<Node> headNodes = new ArrayList<Node>();

	/**
	 * 一回目の順序付けを行うメソッド
	 */
	private void checkInterference() {
		System.out.println(under);
		// 一番後ろのJointJをとってくる
		JointJ j = js.get(0);
		
		//最後のJの前に直接つながるクリアの数
		
		ArrayList<Node> list = j.getForward();
		ArrayList<Node> subPlan = new ArrayList<Node>();
		Pattern p = Pattern.compile("Place (.*) on (.*)");
		while ((subPlan.size() + clearNum) != list.size()) {
			System.out.println("subPlanSize "+subPlan.size()+" jF ");
		//	System.out.println("subPlan" + subPlan.size() + "list"
			//		+ list.size());
			for (Node node : list) {
				Matcher m = p.matcher(node.getNodeName());
				if (m.find()) {
					if (!under.contains(m.group(1))) {
						under.remove(m.group(2));

						if (!subPlan.contains(node)) {
							subPlan.add(0, node);
						}
					}
				
				}
			}

		}

		headNodes.addAll(subPlan);

		for (int i = 0; i < subPlan.size(); i++) {
			Matcher m2 = p.matcher(subPlan.get(i).getNodeName());
			if (m2.find()) {
				for (int i2 = i + 1; i2 < subPlan.size(); i2++) {
					Matcher m3 = p.matcher(subPlan.get(i2).getNodeName());
					if (m3.find()) {
						System.out.println(m2.group(1) + " " + m3.group(2));
						if (m2.group(1).equals(m3.group(2))) {
							System.out.println(subPlan.get(i).getNodeName()
									+ "  " + subPlan.get(i2).getNodeName());
							subPlan.get(i).changeBack(
									subPlan.get(i2).getForward());
							((JointJ) subPlan.get(i2).getForward())
									.addForward(subPlan.get(i));
							j.removeForward(subPlan.get(i));
							headNodes.remove(subPlan.get(i2));
							break;
						}
					}

				}

			}

		}

	}

	/**
	 * 二回目の順序づけで使うメソッド
	 */
	private void checkIF() {

		headNodes.clear();
		Pattern p = Pattern.compile("Place (.*) on (.*)");
		Pattern p2 = Pattern.compile("remove (.*) from on top (.*)");

		ArrayList<JointJ> deleteList = new ArrayList<JointJ>();
		for (int i = 1; i < js.size(); i++) {

			boolean stackFlag = false;
			JointJ j = js.get(i);
			ArrayList<Node> forward = j.getForward();
			ArrayList<Node> subPlan = new ArrayList<Node>();

			for (Node node : forward) {
				Matcher m = p.matcher(node.getNodeName());
				if (m.find()) {
					subPlan.add(node);
					stackFlag = true;
				} else {
					subPlan.add(0, node);
				}
			}

			if (stackFlag) {
				System.out.println("subPlan" + subPlan + "\n\n");
				Object joint = subPlan.get(subPlan.size() - 1);
				while (joint instanceof Node) {

					Node next = (Node) joint;
					joint = next.getForward();

				}

				if (joint instanceof JointJ) {
					System.out.println("succes");
					JointJ target = (JointJ) joint;

					for (int k = 0; k < (subPlan.size() - 1); k++) {
						System.out.println(subPlan.get(k));
						Node jointNode = subPlan.get(k);

						jointNode.changeBack(target);
						target.addForward(jointNode);
					}

					Node stackNode = subPlan.get(subPlan.size() - 1);
					Node jointStack = ((JointJ) stackNode.getBack()).getBack();
					stackNode.changeBack(jointStack);
					jointStack.changeForward(stackNode);

					deleteList.add(j);
				}
			}

		}

		for (JointJ j : deleteList) {
			js.remove(j);
		}

		JointS s = ss.get(0);
		ArrayList<JointS> slist = new ArrayList<JointS>();
		for (Object obj : s.getBack()) {
			if (obj instanceof JointS) {
				slist.add((JointS) obj);
			}
		}

		for (JointS s2 : slist) {
			for (Object obj : s2.getBack()) {
				s.addBack(obj);
				((Node) obj).changeForward(s);
			}
			s.removeBack(s2);
			ss.remove(s2);

		}
	}

	/**
	 * 一回目の冗長削除で使うメソッド
	 */
	private void checkLengthy() {

		for (Node node : headNodes) {
			ArrayList<String> clearList = new ArrayList<String>();

			System.out.println("\ncheck " + node.getNodeName() + " do \n");

			do {
				System.out.println("node : " + node);

				System.out.println("clearList : " + clearList);

				Node clearNode = ((JointJ) node.getForward()).getForward().get(
						1);
				Node extraNode = ((JointJ) node.getForward()).getForward().get(
						0);
				System.out.println("clearNode? :" + clearNode);
				if (clearList.contains(clearNode.getNodeName())) {
					System.out.println("in  clearNode");
					((JointJ) clearNode.getBack()).removeForward(clearNode);
					Object x = clearNode.getForward();
					if (x instanceof JointS) {
						System.out.println("success JointS\n" + x);
						JointS jointS = (JointS) x;

						jointS.removeBack(clearNode);
						plan.remove(clearNode);

						((JointS) jointS.getForward()).addBack(extraNode);
						((JointS) jointS.getForward()).removeBack(jointS);

						extraNode.changeForward(((JointS) jointS.getForward()));

						ss.remove(jointS);
					} else {

						System.out.println("not JointS \n" + x);
					}
				}
				clearList.add(((JointJ) node.getForward()).getForward().get(0)
						.getNodeName());
				clearList.add(((JointJ) node.getForward()).getForward().get(1)
						.getNodeName());
				System.out.println(node.getBack().getClass().toString());
				if (((JointJ) node.getBack()).equals(js.get(0))) {
					System.out.println("goalNode :" + node.getBack());
					System.out.println(node.getBack().toString());
					break;
				} else {
					node = ((JointJ) node.getBack()).getBack();
				}

				System.out.println("");
			} while (true);
		}
		System.out.println("finish checklengthy");

	}

	/**
	 * 二回目以降の冗長削除で使うメソッド
	 */
	private void checkLen() {

		ArrayList<Node> clearList = new ArrayList<Node>();
		Pattern p = Pattern.compile("clear (.*)");
		for (int i = 1; i < js.size(); i++) {
			JointJ j = js.get(i);

			HashMap<String, Integer> data = new HashMap<String, Integer>();
			HashMap<String, Node> nodeData = new HashMap<String, Node>();
			for (Node node : j.getForward()) {
				int count = 1;

				Matcher m = p.matcher(node.getNodeName());
				while (!m.find()) {
					count++;
					node = ((Node) node.getForward());

					m = p.matcher(node.getNodeName());
				}

				if (data.containsKey(node.getNodeName())) {
					if (data.get(node.getNodeName()) > count) {
						clearList.add(node);
					} else {
						clearList.add(nodeData.get(node.getNodeName()));
						data.remove(node.getNodeName());
						data.put(node.getNodeName(), count);
						nodeData.remove(node.getNodeName());
						nodeData.put(node.getNodeName(), node);
					}
				} else {
					data.put(node.getNodeName(), count);
					nodeData.put(node.getNodeName(), node);
				}
			}
		}

		for (Node node : clearList) {
			JointS s = ss.get(0);
			s.removeBack(node);
			while (true) {
				if (node.getBack() instanceof Node) {
					plan.remove(node);
					node = (Node) node.getBack();
				} else {
					plan.remove(node);
					((JointJ) node.getBack()).removeForward(node);
					break;
				}
			}

		}
	}

	/**
	 * 初回から最後まで詳細化はこのメソッドで行う
	 */
	private void refinement() {

		JointS startNode = ss.get(0);
		ArrayList<String> currentState = nPara.getCurrentState();
		ArrayList<Node> deleteList = new ArrayList<Node>();
		ArrayList<Node> addList = new ArrayList<Node>();
		for (Object obj : startNode.getBack()) {

			if (obj instanceof Node) {
				if (!currentState.contains(((Node) obj).getNodeName())) {
					Pattern p1 = Pattern.compile("clear (.*)");
					Matcher m1 = p1.matcher(((Node) obj).getNodeName());

					if (m1.find()) {

						for (String str : currentState) {
							Pattern p2 = Pattern.compile("(.*) on (.*)");
							Matcher m2 = p2.matcher(str);
							if (m2.find()) {
								if (m2.group(2).equals(m1.group(1))) {
									Node newNode1 = new Node("clear "
											+ m2.group(1), nodecount++,
											((Node) obj).getForward(), null);
									Node newNode2 = new Node("remove "
											+ m2.group(1) + " from on top "
											+ m2.group(2), nodecount++, null,
											((Node) obj).getBack());

									newNode1.changeBack(newNode2);
									newNode2.changeForward(newNode1);

									plan.remove(obj);
									deleteList.add((Node) obj);

									Object joint = ((Node) obj).getBack();

									if (joint instanceof JointJ) {
										((JointJ) ((Node) obj).getBack())
												.removeForward(obj);
										addList.add(newNode1);
										((JointJ) ((Node) obj).getBack())
												.addForward(newNode2);
									} else {
										addList.add(newNode1);
										((Node) ((Node) obj).getBack())
												.changeForward(newNode2);

									}

									plan.add(newNode1);
									plan.add(newNode2);

								}
							}
						}
					}
				}
			}
		}

		for (Node delete : deleteList) {
			startNode.removeBack(delete);
		}

		for (Node add : addList) {
			startNode.addBack(add);
		}
	}

	/**
	 * 順序付け、冗長削除、詳細化を繰り返しを行うことで Startの後ろのJointSとGoalの前のJointJの間に各作りたい山を作る系列が残るので
	 * 最後にそれらを一本道に変換する
	 * 
	 * @return
	 */
	private ArrayList<String> lastOrder() {

		ArrayList<Node> preList = new ArrayList<Node>();
		ArrayList<Node> orderList = new ArrayList<Node>();
		ArrayList<String> orderString = new ArrayList<String>();
		Pattern p1 = Pattern.compile("clear (.*)");
		Pattern p2 = Pattern.compile("Place (.*) on (.*)");
		Pattern p3 = Pattern.compile("remove (.*) from on top (.*)");

		for (Object obj : ss.get(0).getBack()) {
			if (obj instanceof Node) {
				Node node = ((Node) obj);
				Matcher m1 = p1.matcher(node.getNodeName());
				if (m1.find()) {
					Object next = node.getBack();

					if (next instanceof Node) {
						preList.add(((Node) next));
					} else {
						if (next instanceof JointJ) {
							JointJ j = (JointJ) next;

							if (j.getForward().size() > 1) {
								j.removeForward(node);
							} else {
								preList.add(j.getBack());
							}
						}
					}
				}

			}
		}

		System.out.println("check1");

		while (true) {
			System.out.println("pre" + preList);
			System.out.println("order" + orderList);
			Boolean addFrag = false;
			ArrayList<String> overList = new ArrayList<String>();
			ArrayList<String> underList = new ArrayList<String>();
			// stackが入ってる
			ArrayList<Node> nodeList = new ArrayList<Node>();
			Node deleteNode = null;
			Node deleteNode2 = null;
			// stackの値とってくる
			for (Node node : preList) {
				Matcher stackMat = p2.matcher(node.getNodeName());

				if (stackMat.find()) {
					overList.add(stackMat.group(1));
					underList.add(stackMat.group(2));
					nodeList.add(node);
				}
			}
			// stackの下の値から優先度の高いunstackの決定
			for (Node node : preList) {
				Matcher unMat = p3.matcher(node.getNodeName());

				if (unMat.find()) {
					if (underList.contains(unMat.group(1))) {
						orderList.add(node);
						orderString.add(node.getNodeName());

						deleteNode = node;
						addFrag = true;
						break;
					}
				}
			}

			// stackの上の値から優先度の高いunstackの決定+ stack
			if (!addFrag) {
				for (Node node : preList) {
					Matcher unMat = p3.matcher(node.getNodeName());

					if (unMat.find()) {
						if (overList.contains(unMat.group(1))) {
							orderList.add(node);
							orderString.add(node.getNodeName());

							deleteNode = node;
							addFrag = true;

							deleteNode2 = nodeList.get(overList.indexOf(unMat
									.group(1)));

							break;
						}
					}
				}
			}

			// stackの決定もしくはunstackのみの場合の処理
			if (!addFrag) {
				if (nodeList.size() > 0) {
					deleteNode = nodeList.get(0);
					orderList.add(deleteNode);
					orderString.add(deleteNode.getNodeName());
				} else {
					// unstackのみの場合

					break;

				}
			}

			preList.remove(deleteNode);
			Object x = deleteNode.getBack();

			Node nextNode = null;

			while (true) {
				if (x instanceof Node) {
					nextNode = (Node) x;

					if (orderString.contains(nextNode.getNodeName())) {
						x = nextNode.getBack();
					} else {

						preList.add(nextNode);
						break;
					}
				} else {
					if (x instanceof JointJ) {
						JointJ j = (JointJ) x;

						if (j.getForward().size() == 1) {
							if (!j.getBack().getNodeName().equals("Goal")) {
								preList.add(j.getBack());
							}
							break;
						} else {
							j.removeForward(nextNode);
							break;
						}
						// break;
					}
				}
			}

			if (deleteNode2 != null) {
				orderList.add(deleteNode2);
				orderString.add(deleteNode2.getNodeName());

				preList.remove(deleteNode2);

				Object y = deleteNode2.getBack();

				if (y instanceof Node) {
					preList.add((Node) y);

				}

			}

			if (preList.size() == 0) {
				break;
			}

		}

		System.out.println("第一完了" + orderList);

		// 第二段階
		// 先頭のunstackの冗長削除

		HashMap<String, Node> NodeMap = new HashMap<String, Node>();
		HashMap<Node, Node> NextMap = new HashMap<Node, Node>();
		HashMap<String, Integer> LengthMap = new HashMap<String, Integer>();
		ArrayList<Node> delList = new ArrayList<Node>();
		ArrayList<Node> delList2 = new ArrayList<Node>();
		for (Object obj : ss.get(0).getBack()) {
			Node node = ((Node) obj);
			Node next = node;
			Integer count = 0;
			while (true) {
				if (next.getBack() instanceof Node) {
					next = (Node) next.getBack();
					count++;
				} else {
					System.out.println("node " + node + ", count " + count);
					if (count != 0) {
						if (NodeMap.containsKey(node.getNodeName())) {
							System.out.println("!!");
							if (LengthMap.get(node.getNodeName()) < count) {
								delList.add(NodeMap.get(node.getNodeName()));
								delList2.add(NextMap.get(NodeMap.get(node
										.getNodeName())));
								NodeMap.remove(node.getNodeName());
								LengthMap.remove(node.getNodeName());
								NextMap.remove(node);

								NodeMap.put(node.getNodeName(), node);
								LengthMap.put(node.getNodeName(), count);
								NextMap.put(node, next);
							} else {
								delList.add(node);
								delList2.add(next);
							}
						} else {
							NodeMap.put(node.getNodeName(), node);
							LengthMap.put(node.getNodeName(), count);
							NextMap.put(node, next);
						}
					}
					break;

				}
			}
		}
		System.out.println("pre :" + preList);

		System.out.println("delList1 : " + delList);

		for (Node node : delList) {
			ss.get(0).removeBack(node);
			if (node.getBack() instanceof Node) {
				preList.remove(node.getBack());
			} else {
				System.out.println("node.getBack :" + node.getBack());
				JointJ j = ((JointJ) node.getBack());
				if (j.getForward().size() == 1) {
					if (!preList.contains(j.getBack())) {
						preList.add(j.getBack());
						System.out.println("del node1: " + node);
						System.out.println("add preList1: " + j.getBack());
					}
				} else {
					j.removeForward(node);
				}
			}

		}
		System.out.println("delList2 : " + delList2);
		for (Node node : delList2) {
			JointJ j = (JointJ) node.getBack();
			if (j.getForward().size() == 1) {
				if (!preList.contains(j.getBack())) {
					preList.add(j.getBack());
					System.out.println("del node2: " + node);
					System.out.println("add preList2: " + j.getBack());
				}

				//
				// ここでは不要になったJがでるが消すかどうか審議中
				//
				// j.removeForward(node);
			} else {
				j.removeForward(node);
			}

		}

		// 残っているJを探す
		if (preList.size() > 0) {
			ArrayList<JointJ> jList = new ArrayList<JointJ>();

			for (Node node : preList) {
				Matcher unSt = p3.matcher(node.getNodeName());

				if (unSt.find()) {
					Object z = node.getBack();
					while (true) {
						if (!(z instanceof JointJ)) {
							z = ((Node) z).getBack();
						} else {
							if (!jList.contains(z)) {
								jList.add((JointJ) z);
							}
							break;
						}

					}
				}
			}

			System.out.println("bug 前　pre  :" + preList);
			System.out.println("orderList :" + orderList);
			// 元からある山の処理を消す

			//
			//
			// 　　バグ？　　あり
			//
			// 消える物がきえない
			//
			// 　
			//
			//
			//
			//
			ArrayList<JointJ> delJ = new ArrayList<JointJ>();
			for (JointJ j : jList) {
				boolean unMatHit = false;
				Matcher stackMap = p2.matcher(j.getBack().getNodeName());
				Matcher unMap;
				Node change = null;
				if (stackMap.find()) {
					for (Node node : j.getForward()) {
						unMap = p3.matcher(node.getNodeName());
						if (unMap.find()) {
							unMatHit = true;
							if (stackMap.group(1).equals(unMap.group(1))
									&& stackMap.group(2).equals(unMap.group(2))) {
								change = node;
							}
						}
					}
					System.out.println("Stack = ? :" + j.getBack());
					System.out.println("change = ?  :" + change);
					if (change != null) {
						System.out.println("success?");
						while (true) {
							Boolean onlyStack = false;
							// スタックだけが残る場合を考えて、Unstack側が空になったとき、フラグを立てておく
							System.out.println("change  消すUN  :" + change);
							System.out.println("change  消すST  :" + j.getBack());
							System.out.println("消すUNの前のもの :"
									+ change.getForward());

							System.out.println("change  消すST 後  :"
									+ j.getBack().getBack());
							Node last = change;
							j.removeForward(change);
							// bug
							if (!(change.getForward() instanceof JointS)) {
								change = (Node) change.getForward();
								if (orderString.contains(change.getNodeName())) {
									preList.remove(last);
									onlyStack = true;
								} else {
									Matcher clearMat = p1.matcher(change
											.getNodeName());

									if (clearMat.find()) {
										preList.remove(last);
										onlyStack = true;
									} else {
										j.addForward(change);
									}
								}

							} else {
								onlyStack = true;
								preList.remove(last);
							}

							Object obj = j.getBack().getBack();
							System.out.println(obj.getClass().toString());
							if (obj instanceof Node) {
								// System.out.println("success");
								if (onlyStack) {
									preList.add((Node) obj);
									System.out.println("delBack "+j.getBack()+" add preList "+obj);
									j.changeBack((Node)obj);
									break;
								} else {
									System.out.println("changeBack "+j.getBack()+" to "+obj);
									j.changeBack((Node) obj);
								}
							} else {
								if (obj instanceof JointJ) {
									// ここのifにて、stackだけが残る場合を考えなければならない

									JointJ tail = (JointJ) obj;

									tail.removeForward(j.getBack());
									Matcher mat = p3.matcher(change
											.getNodeName());
									if (mat.find()) {
										System.out.println("UNSTACKのみ残る"
												+ change);
										for (Node node : j.getForward()) {
											tail.addForward(node);
											node.changeBack(tail);

											delJ.add(j);
										}
									} else {
										delJ.add(j);
									}

									break;
								}
							}

							stackMap = p2.matcher(j.getBack().getNodeName());
							unMap = p3.matcher(change.getNodeName());
							if (stackMap.find() && unMap.find()) {
								if (stackMap.group(1).equals(unMap.group(1))
										&& stackMap.group(2).equals(
												unMap.group(2))) {

								} else {
									break;
								}
							}
						}
					} else {
						// unstackに該当するものが無いとき、
						// stackは見つかったけどunstackが見つかってない時
						
						
						if(unMatHit){
							
						}else{
							System.out.println("if no-unstack  then add :"
									+ j.getBack());
							boolean exist = false;
							for (Node node : preList) {
								String pre = node.getNodeName();
								if (pre.equals(j.getBack().getNodeName())) {
									exist = true;
								}
							}
							if (!exist) {
								System.out.println("    add preList" + j.getBack());
								preList.add(j.getBack());
							}
						}
					}

				}
			}

			System.out.println("jList before remove" + jList);

			for (JointJ del : delJ) {
				System.out.println("消すJの後 :" + del.getBack().getNodeName());
				jList.remove(del);
			}

			jList.remove(js.get(0));
			System.out.println("check");
			System.out.println("pre  :" + preList);
			System.out.println("orderStr" + orderString);

			System.out.println("jList :" + jList);
			for(JointJ j : jList){
				System.out.println("j.back  :"+j.getBack()+"\nj.foward :"+j.getForward());
			}

			// 残ったものの順序決定
			ArrayList<ArrayList<Node>> tList = new ArrayList<ArrayList<Node>>();
			System.out.println("jlistsize" + jList.size());
			for (JointJ j : jList) {
				System.out.println("j forward: " + j.getForward());
				ArrayList<Node> temp = new ArrayList<Node>();
				Node next = j.getBack();
				while (true) {
					temp.add(next);

					if (next.getBack() instanceof Node) {
						next = (Node) next.getBack();
					} else {
						break;
					}
				}
				for (int k = 0; k < j.getForward().size(); k++) { // jの前の数
					Node unStack = j.getForward().get(k);
					while (true) {

						Matcher unMat = p3.matcher(unStack.getNodeName());

						if (unMat.find()) {
							int t;
							for (t = 0; t < temp.size(); t++) {
								Matcher stackMat = p2.matcher(temp.get(t)
										.getNodeName());

								if (stackMat.find()) {
									if (unMat.group(1)
											.equals(stackMat.group(1))) {
										temp.add(t, unStack);
										break;
									} else if (unMat.group(2).equals(
											stackMat.group(1))) {
										temp.add(t, unStack);
										// fix me?
										break;

									} else if ((!unMat.group(1).equals(
											stackMat.group(2)))
											&& (!unMat.group(2).equals(
													stackMat.group(2)))) {

										// tugi
									} else {
										temp.add(0, unStack);
										break;
									}
								} else {
									if (unStack.getBack().equals(temp.get(t))) {
										temp.add(t, unStack);
										break;
									} else {

										// 自分と関係ないunStack
									}
								}
							}

							// 何ともマッチしなかった
							if (t == temp.size()) {
								temp.add(0, unStack);
							}

						}

						if (!preList.contains(unStack)) {
							if (unStack.getForward() instanceof JointS) {
								preList.remove(unStack);
								System.out.println("remove 1 " + unStack);
								break;
							} else {
								unStack = ((Node) unStack.getForward());
							}
						} else {
							Matcher unSt = p3.matcher(unStack.getNodeName());
							if (unSt.find()) {
								preList.remove(unStack);
							}
							System.out.println("remove 2 " + unStack);
							break;
						}

					}


				}
				System.out.println("add temp " + temp);

				tList.add(temp);
				System.out.println("temp" + temp);

			}

			System.out.println("preList" + preList);
			if (preList.size() > 0) {
				for (Node node : preList) {
					
					Boolean checkt = false;
					for(ArrayList<Node> t:tList){
						if(t.contains(node)){
							checkt = true;
						}
					}
					if(checkt){
						continue;
					}
					
					ArrayList<Node> temp = new ArrayList<Node>();
					temp.add(node);

					Node next = node;

					while (true) {
						if (next.getBack() instanceof Node) {
							next = ((Node) next.getBack());
							temp.add(next);

						} else {
							break;
						}

					}

					System.out.println("temp at 1 :" + temp);
					tList.add(temp);
				}
			}

			System.out.println("tList :" + tList);
			System.out.println("orderList :" + orderList);

			ArrayList<Node> nodes = new ArrayList<Node>();
			ArrayList<String> words = new ArrayList<String>();

			for (String str : nPara.getCurrentState()) {
				Matcher clearWord = p1.matcher(str);
				if (clearWord.find()) {
					words.add(clearWord.group(1));
				}
			}

			// 先頭の初期化
			for (ArrayList<Node> list : tList) {
				nodes.add(list.get(0));
			}
			while (true) {
				// System.out.println("orderList"+orderList);
				// System.out.println("orderString"+orderString);
				ArrayList<Node> noUse = new ArrayList<Node>();
				//先頭でないノードのリスト
				for (ArrayList<Node> list : tList) {
					noUse.addAll(list);
					noUse.remove(list.get(0));
					// System.out.println(list);
				}
				// System.out.println("pre"+preList);
				// System.out.println("words"+words);
				// System.out.println("nodes"+nodes);
				ArrayList<String> over = new ArrayList<String>();
				ArrayList<String> under = new ArrayList<String>();
				ArrayList<Node> stacks = new ArrayList<Node>();
				ArrayList<Node> unstacks = new ArrayList<Node>();

				ArrayList<Integer> clearNodeIndexList = new ArrayList<Integer>();
				// clearNodeが一個でもあるかのフラグ
				boolean clearflag = false;
				// over,under
				for (Node node : nodes) {
					Matcher stack = p2.matcher(node.getNodeName());

					if (stack.find()) {
						over.add(stack.group(1));
						under.add(stack.group(2));

						stacks.add(node);
					} else {
						unstacks.add(node);
					}
				}

				// System.out.println(over);
				// System.out.println(under);

				for (int k = 0; k < stacks.size(); k++) {
					if (words.contains(over.get(k))
							&& words.contains(under.get(k))) {
						clearNodeIndexList.add(k);
						clearflag = true;
					}
				}
				// ここまでで下準備

				// clearがあった時
				if (clearflag) {
					Boolean addcheck = false;
					for (Integer clearNodeIndex : clearNodeIndexList) {
						
						boolean noUseclearNode = false;
						for(Node noUseNode : noUse){
							Matcher stackMat = p2.matcher(noUseNode.getNodeName());
							Matcher unMat = p3.matcher(stacks.get(clearNodeIndex).getNodeName());
							
							if(stackMat.find()){
								if(unMat.find()){
									if(stackMat.group(1).equals(unMat.group(1))){
										noUseclearNode = true;
										break;
									}
								}
							}							
						}
						
						if(noUseclearNode){
							continue;
						}
						
						
						System.out.println("clear   "
								+ stacks.get(clearNodeIndex));
						boolean flag1 = true;

						// 下のものが何かの上にあるとき
						for (Node node : unstacks) {
							Matcher unstack = p3.matcher(node.getNodeName());
							if (unstack.find()) {
								if (unstack.group(1).equals(
										under.get(clearNodeIndex))) {
									orderList.add(node);
									orderString.add(node.getNodeName());
									words.add(unstack.group(2));
									addcheck = true;
									break;
								}
							}

						}
						for (Node node : unstacks) {
							Matcher unstack = p3.matcher(node.getNodeName());

							if (unstack.find()) {
								if (unstack.group(1).equals(
										over.get(clearNodeIndex))) {

									System.out.println("clear   type1");
									System.out.println("clear   " + node);
									orderList.add(node);
									orderString.add(node.getNodeName());
									// unstackの下をclearにする
									words.add(unstack.group(2));
									orderList.add(stacks.get(clearNodeIndex));
									orderString.add(stacks.get(clearNodeIndex)
											.getNodeName());
									// 下側のモノのclearは消す
									words.remove(under.get(clearNodeIndex));
									addcheck = true;
									flag1 = false;
									break;
								}
							}
						}
						// 床においてあるものを載せるとき
						if (flag1) {
							System.out.println("clear   type2");
							orderList.add(stacks.get(clearNodeIndex));
							orderString.add(stacks.get(clearNodeIndex)
									.getNodeName());
							// 下側のモノのclearは消す
							words.remove(under.get(clearNodeIndex));
							addcheck = true;
							break;
						}
						
						if(addcheck){
							break;
						}
					}
				} else {
					// クリアがなかったとき

					if (stacks.size() > 0 && unstacks.size() > 0) {
						// stackとunstackが共存するとき

						// stackの下(甲)をclearにする
						boolean okflag1 = false;
						for (String underS : under) {

							if (okflag1) {
								break;
							}
							if (!words.contains(underS)) {

								for (ArrayList<Node> list : tList) {
									// 前方が求める甲であるunstackになるまで遡る
									// 途中でstackが出るかListがなくなれば諦める
									ArrayList<String> addClear = new ArrayList<String>();
									if (unstacks.contains(list.get(0))) {
										for (int i = 0; i < list.size(); i++) {

											Matcher unstackMat = p3
													.matcher(list.get(i)
															.getNodeName());

											if (unstackMat.find()) {

												addClear.add(unstackMat
														.group(2));

												if (unstackMat.group(1).equals(
														underS)) {
													// 見つかった場合

													for (int j = 0; j < i + 1; j++) {
														orderList.add(list
																.get(j));
														orderString.add(list
																.get(j)
																.getNodeName());

													}
													// clearの処理
													for (String add : addClear) {
														words.add(add);
													}
													okflag1 = true;
												}
											} else {
												break;
											}
										}
									}
								}
							} else {
								// すでにクリアで何かの上にあるとき
								for (Node unstack : unstacks) {
									Matcher unSt = p3.matcher(unstack
											.getNodeName());
									if (unSt.find()) {
										if (unSt.group(1).equals(underS)) {
											orderList.add(unstack);
											orderString.add(unstack
													.getNodeName());
											okflag1 = true;
											words.add(unSt.group(2));
											break;
										}
									}
								}
							}
						}

						// stackの上(乙)をclearにする
						boolean okflag2 = false;
						if (!okflag1) {
							for (String overS : over) {
								if (okflag2) {
									break;
								}
								if (!words.contains(overS)) {
									for (ArrayList<Node> list : tList) {
										ArrayList<String> addClear = new ArrayList<String>();
										if (unstacks.contains(list.get(0))) {
											for (int i = 0; i < list.size(); i++) {
												Matcher unstackMat = p3
														.matcher(list.get(i)
																.getNodeName());

												if (unstackMat.find()) {
													// System.out.println(list.get(i));
													addClear.add(unstackMat
															.group(2));

													if (unstackMat.group(2)
															.equals(overS)) {
														// 見つかった場合
														System.out
																.println("in5");
														for (int j = 0; j < i + 1; j++) {
															orderList.add(list
																	.get(j));
															orderString
																	.add(list
																			.get(j)
																			.getNodeName());

														}
														// clearの処理
														for (String add : addClear) {
															words.add(add);
														}
														okflag2 = true;
													}
												}
											}
										}
									}
								}
							}
						}

					} else if (stacks.size() == 0) {
						// unstackだけのとき
						// ここが重要かもしれない？

						ArrayList<String> stackList = new ArrayList<String>();
						HashMap<String, Integer> sLen = new HashMap<String, Integer>();

						for (int h = 0; h < tList.size(); h++) {
							stackList.add(null);
							ArrayList<Node> list = tList.get(h);

							for (int k = 0; k < list.size(); k++) {
								Node temp = list.get(k);
								Matcher sMat = p2.matcher(temp.getNodeName());

								if (sMat.find()) {
									stackList.remove(h);
									stackList.add(h, sMat.group(1));
									sLen.put(sMat.group(1), k);
									break;
								}

							}
						}

						int action = -1;
						for (int k = 0; k < stackList.size(); k++) {
							if (stackList.get(k) != null) {
								String stack = stackList.get(k);

								Matcher sMat = p2.matcher(stack);

								if (sMat.find()) {
									for (int l = 0; l < nodes.size(); l++) {
										Node node = nodes.get(l);

										Matcher unMat = p3.matcher(node
												.getNodeName());

										if (unMat.find()) {
											if (unMat.group(1).equals(
													sMat.group(1))) {
												if (action == -1) {
													action = k;
												} else {
													if (sLen.get(stackList
															.get(action)) > sLen
															.get(stack)) {
														action = k;
													}
												}
											}
										}
									}
								} else {

								}
							}
						}

						if (action != -1) {
							orderList.add(nodes.get(action));
							orderString.add(nodes.get(action).getNodeName());

							Matcher clearMat = p3.matcher(nodes.get(action)
									.getNodeName());
							if (clearMat.find()) {
								words.add(clearMat.group(2));
							}

						} else {
							for (Node node : nodes) {
								orderList.add(node);
								orderString.add(node.getNodeName());

								Matcher clearMat = p3.matcher(node
										.getNodeName());
								if (clearMat.find()) {
									words.add(clearMat.group(2));
								}
							}
						}

					} else {
						// stackだけの時
						// おそらくこのバターンはありえない
						// これになる前にクリア条件をみたすはず

					}
				}
				nodes.clear();
				ArrayList<ArrayList<Node>> delArray = new ArrayList<ArrayList<Node>>();
				for (ArrayList<Node> list : tList) {
					while (true) {
						if (list.size() > 0) {
							if (orderList.contains(list.get(0))) {
								list.remove(0);
							} else {
								nodes.add(list.get(0));
								break;
							}
						} else {
							delArray.add(list);
							break;
						}

					}
				}

				for (ArrayList<Node> list : delArray) {
					tList.remove(list);
				}

				if (tList.size() == 0) {
					break;
				}
			}

		}
		System.out.println("lastOrder" + orderList);
		System.out.println(orderString);

		return orderString;
	}

	/**
	 * できたプランを今回の課題の形式に落としこむ
	 * 
	 * @param lastOrder
	 */

	private ArrayList<String> planEmbossing(ArrayList<String> lastOrder) {
		ArrayList<String> finalPlan = new ArrayList<String>();

		Pattern sP = Pattern.compile("Place (.*) on (.*)");
		Pattern rP = Pattern.compile("remove (.*) from on top (.*)");

		String lastStr1 = null;
		String lastStr2 = null;
		String lastType = null;
		for (String str : lastOrder) {
			Matcher sMat = sP.matcher(str);
			Matcher rMat = rP.matcher(str);

			if (sMat.find()) {
				if (lastType == null) {
					finalPlan.add("pick up " + sMat.group(1)
							+ " from the table");
					finalPlan.add(str);
				} else {
					if (lastType.equals("unstack")) {
						if (lastStr1.equals(sMat.group(1))) {
							finalPlan.add(str);
						} else {
							finalPlan.add("put " + lastStr1
									+ " down on the table");
							finalPlan.add("pick up " + sMat.group(1)
									+ " from the table");
							finalPlan.add(str);
						}
					} else {
						finalPlan.add("pick up " + sMat.group(1)
								+ " from the table");
						finalPlan.add(str);
					}
				}

				lastType = "stack";
				lastStr1 = sMat.group(1);
				lastStr2 = sMat.group(2);
			}

			if (rMat.find()) {
				if (lastType != null) {
					if (lastType.equals("unstack")) {
						finalPlan.add("put " + lastStr1 + " down on the table");
					}
				}
				finalPlan.add(str);

				lastType = "unstack";
				lastStr1 = rMat.group(1);
				lastStr2 = rMat.group(2);
			}

		}

		return finalPlan;
	}

	/**
	 * plannigを行うメソッド これを呼ぶ前に初期状態をセットするメソッドsetCurrentStateと
	 * ゴール状態をセットするメソッドsetGoalStateを使う必要あり 結果のArrayListはgetResultで得る
	 */
	public void planning() {
		// グローバル変数の初期化
		initMember();

		// ゴールのリストをプランに登録
		initialPlanning();

		System.out.println("initialState" + getCurrentState());
		System.out.println("goalState" + getGoalState());
		// プランを展開
		expandPlan();

		System.out.println("initialState \n");
		printState();

		// 一回目の順序づけ
		checkInterference();
		System.out.println("順序付け終了");
		printState();
		checkLengthy();
		System.out.println("冗長削除終了");
		printState();
		System.out.println("詳細化終了");
		refinement();

		System.out.println("\nAct1 result \n");
		printState();

		// 二回目の順序づけ
		System.out.println("順序付け終了2");
		checkIF();

		printState();
		ArrayList<Node> lastPlan = new ArrayList<Node>();
		lastPlan.addAll(plan);
		// 以下二回目以降の冗長削除、詳細化繰り返し
		while (true) {
			checkLen();
			System.out.println("冗長削除終了2+");
			refinement();
			System.out.println("詳細化終了2+");
			if (lastPlan.equals(plan)) {
				break;
			} else {
				lastPlan.clear();
				lastPlan.addAll(plan);
			}
			System.out.println("\n loop");
			printState();
		}

		int count = 0;
		System.out.println("lastOrder前");
		ArrayList<String> lastOrder = lastOrder();
		System.out.println("lastOrder終了");
		System.out.print(lastOrder);
		ArrayList<String> finalPlan = planEmbossing(lastOrder);

		System.out.println("\nfinalPlan!!!");
		for (String str : finalPlan) {
			System.out.println("count " + (count++) + " : " + str);

		}

		resultPlan = finalPlan;
	}

	/**
	 * 結果を返すメソッド
	 * 
	 * @return
	 */
	public ArrayList<String> getResult() {
		return resultPlan;
	}

	public ArrayList<String> getObjects() {
		return objects;
	}

	// 途中経過を確認していたメソッド
	private void printState() {
		for (JointS joint : ss) {

			System.out.println("center :" + joint);

			System.out.println("forward : " + joint.getForward());

			Iterator it = joint.getBack().iterator();
			while (it.hasNext()) {

				Object obj = it.next();
				System.out.println("back : " + obj);

			}
			System.out.println("");
		}

		for (JointJ joint2 : js) {

			System.out.println("joint2 " + joint2);

			Iterator it = joint2.getForward().iterator();
			while (it.hasNext()) {

				Object obj = it.next();

				System.out.println("forward : " + obj);

			}

			System.out.println("back : " + joint2.getBack());

			System.out.println("");
		}

		for (Node node : plan) {
			System.out
					.println(node.getNodeName() + "  " + node.getNodeNumber());
			System.out.println("forward " + node.getForward() + "\nback "
					+ node.getBack() + "\n");
		}
	}

}
