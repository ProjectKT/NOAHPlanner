package gui;

import gui.PlannerController.StatesChangeListener;

import java.awt.BorderLayout;
import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButtonMenuItem;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextArea;
import javax.swing.border.BevelBorder;
import javax.swing.border.EtchedBorder;

import planner.NOAH;
import component.DataFilter;

public class SampleGUI extends JFrame implements ActionListener {

	//
	JPanel panel = new JPanel();
	CardLayout layout = new CardLayout();
	JMenuItem mntmOpenFile;
	JMenuItem mntmAdd;
	// Label
	JLabel stepLabel;
	// 物体を表示するパネル
	PlannerPanel plannerPanel;
	// 初期状態
	PlannerPanel startPanel;
	// 終了状態
	PlannerPanel goalPanel;
	// Noah
	NOAH noah;
	// PlannerStepExecutor
	final PlannerStepExecutor plannerStepExecutor = new PlannerStepExecutor();

	ArrayList<String> startList = new ArrayList<String>();
	ArrayList<String> goalList = new ArrayList<String>();
	ArrayList<String> objects = new ArrayList<String>();
	ArrayList<String> ansList = new ArrayList<String>();

	JTabbedPane tab1 = new JTabbedPane();
	JTabbedPane tab2 = new JTabbedPane();
	JTextArea txtStart = new JTextArea("");// 初期状態のエリア
	JTextArea txtGoal = new JTextArea("");// 目標状態のエリア
	JTextArea txtAnswer = new JTextArea("");// 解答のエリア

	File currentFile = null;

	// コンストラクタ
	public SampleGUI() {
		initialize();

		setVisible(true);

		// 初期状態・終了状態を設定
		initPlanner();

		// TODO Planner の出力を元に PlannerPanel のコマンドを呼ぶ操作パネル、レイアウトの作成
		// 1. 初期状態、目標状態を入れるための入力コンポーネントを用意する
		// 2. 入力された内容を元に Planner を動かし、プランをもらう
		// 3. 受け取ったプランを元に、 PlannerPanel の操作メソッドを呼び、操作する

	}

	private void loadInitialFile() {
		currentFile = new File("demoData2.data");

		// currentFile = new File(".//..//..//..//..//demoData2.data");
		System.out.println(currentFile);
	}

	// 初期化
	private void initialize() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(10, 10, 550, 600);
		setTitle("SampleGUI");
		getContentPane().setLayout(new BorderLayout(0, 0));

		JMenuBar menuBar = new JMenuBar();
		getContentPane().add(menuBar, BorderLayout.NORTH);

		JMenu mnFile = new JMenu("File");
		mnFile.setBackground(new Color(192, 192, 192));
		mnFile.setForeground(Color.RED);
		menuBar.add(mnFile);

		mntmOpenFile = new JMenuItem("Open File");
		mntmOpenFile.addActionListener(this);
		mntmOpenFile.setActionCommand("Open File");
		mnFile.add(mntmOpenFile);

		JMenu mnNewMenu_1 = new JMenu("State");
		menuBar.add(mnNewMenu_1);

		JRadioButtonMenuItem rdbtnmntmNewRadioItem_2 = new JRadioButtonMenuItem(
				"Graphics");
		rdbtnmntmNewRadioItem_2.setSelected(true);
		mnNewMenu_1.add(rdbtnmntmNewRadioItem_2);
		rdbtnmntmNewRadioItem_2.addActionListener(this);
		rdbtnmntmNewRadioItem_2.setActionCommand("graphics");

		JRadioButtonMenuItem rdbtnmntmNewRadioItem_3 = new JRadioButtonMenuItem(
				"Text");
		mnNewMenu_1.add(rdbtnmntmNewRadioItem_3);
		rdbtnmntmNewRadioItem_3.addActionListener(this);
		rdbtnmntmNewRadioItem_3.setActionCommand("text");

		JMenu mnNewMenu_2 = new JMenu("Add");
		
		mntmAdd = new JMenuItem("Add");
		mntmAdd.addActionListener(this);
		mntmAdd.setActionCommand("Add");
		mnNewMenu_2.add(mntmAdd);
		menuBar.add(mnNewMenu_2);

		ButtonGroup group2 = new ButtonGroup();
		group2.add(rdbtnmntmNewRadioItem_2);
		group2.add(rdbtnmntmNewRadioItem_3);

		// --- Card1: グラフィックのタブ
		JPanel card1 = new JPanel();
		card1.setLayout(new BorderLayout());

		// - Page 1: 初期状態と目標状態を決めるページ
		JPanel page1 = new JPanel(new BorderLayout());
		page1.addComponentListener(new ComponentAdapter() {
			@Override
			public void componentShown(ComponentEvent e) {
				try {
					startPanel.startAnimating();
					goalPanel.startAnimating();
				} catch (Exception e0) {
					e0.printStackTrace();
				}
			}

			@Override
			public void componentHidden(ComponentEvent e) {
				try {
					startPanel.stopAnimating();
					goalPanel.stopAnimating();
				} catch (Exception e0) {
					e0.printStackTrace();
				}
			}
		});

		JPanel page1_panels = new JPanel(new GridLayout(2, 1));
		page1.add(BorderLayout.CENTER, page1_panels);

		startPanel = new PlannerPanel();
		// startPanel.enableScrollScreen(false);
		startPanel.setStatesChangeListener(new StatesChangeListener() {
			@Override
			public void onChangeStates(List<String> states) {
				startList.clear();
				startList.addAll(states);
			}
		});

		JPanel start = new JPanel();
		start.setLayout(new BorderLayout());
		start.add(BorderLayout.NORTH, new JLabel("初期状態"));
		start.add(BorderLayout.CENTER, startPanel);
		page1_panels.add(start);

		goalPanel = new PlannerPanel();
		// goalPanel.enableScrollScreen(false);
		goalPanel.setStatesChangeListener(new StatesChangeListener() {
			@Override
			public void onChangeStates(List<String> states) {
				goalList.clear();
				goalList.addAll(states);
			}
		});

		JPanel goal = new JPanel();
		goal.setLayout(new BorderLayout());
		goal.add(BorderLayout.NORTH, new JLabel("目標状態"));
		goal.add(BorderLayout.CENTER, goalPanel);
		page1_panels.add(goal);

		JPanel ctrl = new JPanel();
		page1.add(BorderLayout.SOUTH, ctrl);

		JButton okButton = new JButton("plan");
		okButton.addActionListener(this);
		okButton.setActionCommand("plan");
		ctrl.add(okButton);

		tab1.add("select", page1);

		// - Page 2: 実行結果を表示するページ
		JPanel page2 = new JPanel(new BorderLayout());
		page2.addComponentListener(new ComponentAdapter() {
			@Override
			public void componentShown(ComponentEvent e) {
				try {
					plannerPanel.startAnimating();
				} catch (Exception e0) {
					e0.printStackTrace();
				}
			}

			@Override
			public void componentHidden(ComponentEvent e) {
				try {
					plannerPanel.stopAnimating();
				} catch (Exception e0) {
					e0.printStackTrace();
				}
			}
		});

		JPanel page2_panel = new JPanel(new BorderLayout());
		stepLabel = new JLabel();
		plannerStepExecutor.updateStepLabel();
		page2_panel.add(BorderLayout.NORTH, stepLabel);
		plannerPanel = new PlannerPanel();
		plannerPanel.showStates(true);
		page2_panel.add(BorderLayout.CENTER, plannerPanel);
		page2.add(BorderLayout.CENTER, page2_panel);

		JButton playButton = new JButton("Play");// 初期状態を表示するボタン
		playButton.addActionListener(plannerStepExecutor);
		playButton.setActionCommand("play");
		JButton pauseButton = new JButton("Pause");// 初期状態を表示するボタン
		pauseButton.addActionListener(plannerStepExecutor);
		pauseButton.setActionCommand("pause");
		JButton startButton = new JButton("初期状態へ");// 初期状態を表示するボタン
		startButton.addActionListener(plannerStepExecutor);
		startButton.setActionCommand("start");
		JButton nextButton = new JButton("次へ");// 次の状態に進むボタン
		nextButton.addActionListener(plannerStepExecutor);
		nextButton.setActionCommand("next");
		JButton goalButton = new JButton("目標状態へ");// 目標状態を表示するボタン
		goalButton.addActionListener(plannerStepExecutor);
		goalButton.setActionCommand("goal");
		JPanel btnPanel = new JPanel();
		btnPanel.add(playButton);
		btnPanel.add(pauseButton);
		btnPanel.add(startButton);
		btnPanel.add(nextButton);
		btnPanel.add(goalButton);

		page2.add(BorderLayout.SOUTH, btnPanel);
		tab1.add("answer", page2);

		card1.add(tab1);

		// --- Card2: テキストで表示する
		JPanel card2 = new JPanel();
		card2.setLayout(new GridLayout());

		EtchedBorder border = new EtchedBorder();

		//

		JPanel sPanel = new JPanel();// 初期状態等の変更ページ
		sPanel.setLayout(new GridLayout());
		JPanel aPanel = new JPanel();// 解答ページ
		aPanel.setLayout(new BorderLayout());

		tab2.add("select", sPanel);
		tab2.add("answer", aPanel);

		JPanel startPanel = new JPanel();// 初期状態エリア用のパネル
		JScrollPane scroll1 = new JScrollPane(txtStart);
		startPanel.setLayout(new BorderLayout());
		startPanel.add(new JLabel("start"), BorderLayout.NORTH);
		startPanel.add(scroll1, BorderLayout.CENTER);
		startPanel.setBorder(border);

		JPanel goalPanel = new JPanel();// 目標状態エリア用のパネル
		JScrollPane scroll2 = new JScrollPane(txtGoal);
		goalPanel.setLayout(new BorderLayout());
		goalPanel.add(new JLabel("goal"), BorderLayout.NORTH);
		goalPanel.add(scroll2, BorderLayout.CENTER);
		goalPanel.setBorder(border);

		JPanel buttonPanel = new JPanel();// ボタン用のパネル
		// buttonPanel.setLayout(new FlowLayout());
		JButton OK = new JButton("ok");
		OK.addActionListener(this);
		OK.setActionCommand("OK");
		JButton RESET = new JButton("reset");
		RESET.addActionListener(this);
		RESET.setActionCommand("RESET");
		JButton GtoS = new JButton("GtoS");
		GtoS.addActionListener(this);
		GtoS.setActionCommand("GtoS");

		// ---
		JPanel bP1 = new JPanel();
		JPanel bP2 = new JPanel();
		JPanel bP3 = new JPanel();
		JPanel bP4 = new JPanel();
		JPanel bP5 = new JPanel();
		JPanel bP6 = new JPanel();

		bP3.add(RESET);
		bP4.add(GtoS);
		bP5.add(OK);

		bP1.setLayout(new GridLayout(3, 1));
		bP1.add(bP3);
		bP1.add(bP4);
		bP1.add(bP5);
		bP1.setBorder(border);

		buttonPanel.add(bP1);
		buttonPanel.add(bP2);
		buttonPanel.add(bP6);
		buttonPanel.setLayout(new GridLayout(3, 1));
		// ---

		// buttonPanel.add(RESET);
		// buttonPanel.add(GtoS);
		// buttonPanel.add(OK);
		// buttonPanel.setBorder(border);

		JPanel answerPanel = new JPanel();// 目標状態エリア用のパネル
		JScrollPane scroll3 = new JScrollPane(txtAnswer);
		answerPanel.setLayout(new BorderLayout());
		answerPanel.add(scroll3, BorderLayout.CENTER);
		answerPanel.setBorder(border);

		sPanel.add(startPanel);
		sPanel.add(goalPanel);
		sPanel.add(buttonPanel);
		aPanel.add(answerPanel);

		card2.add(tab2);

		//
		panel.setLayout(layout);
		panel.add(card1);
		panel.add(card2);

		getContentPane().add(panel, BorderLayout.CENTER);

		BevelBorder border4 = new BevelBorder(BevelBorder.RAISED);
		BevelBorder border8 = new BevelBorder(BevelBorder.RAISED);
		BevelBorder border9 = new BevelBorder(BevelBorder.RAISED);
		// ---

	}

	private void initPlanner() {
		// NOAHを準備
		noah = new NOAH();

		// 初期状態の準備
		final ArrayList<String> startList = new ArrayList<String>();
		for (String start : noah.getCurrentState()) {
			startList.add(start);
		}
		Collections.sort(startList);
		initPlannerPanel(startList, startPanel);
		this.startList.clear();
		this.startList.addAll(startList);

		// 目標状態の準備
		final ArrayList<String> goalList = new ArrayList<String>();
		for (String goal : noah.getGoalState()) {
			goalList.add(goal);
		}
		Collections.sort(goalList);
		initPlannerPanel(goalList, goalPanel);
		this.goalList.clear();
		this.goalList.addAll(goalList);

		loadInitialFile();

		try {
			loadData();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		System.out.println("startList" + startList);
		System.out.println("goalList" + goalList);
		objects = noah.getObjects();
	}

	/**
	 * PlannerPanel の状態を初期化する
	 * 
	 * @param states
	 *            初期化する状態
	 * @param panel
	 *            パネル
	 * @throws InterruptedException
	 */
	private synchronized void initPlannerPanel(ArrayList<String> states,
			PlannerPanel panel) {
		try {
			panel.initBoxes(states);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	private void textsToStates() {
		startList.clear();
		String strs1[] = txtStart.getText().split("\n");
		for (int i = 0; i < strs1.length; i++) {
			startList.add(strs1[i]);
		}
		goalList.clear();
		String strs2[] = txtGoal.getText().split("\n");
		for (int i = 0; i < strs2.length; i++) {
			goalList.add(strs2[i]);
		}
	}

	private void plan() {
		noah.setCurrentState(startList);
		noah.setGoalState(goalList);
		noah.planning();
		ansList = noah.getResult();
	}

	private String nth(int i) {
		switch (i % 10) {
		case 1:
			return String.valueOf(i) + "st";
		case 2:
			return String.valueOf(i) + "nd";
		case 3:
			return String.valueOf(i) + "rd";
		default:
			return String.valueOf(i) + "th";
		}
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		String cmd = e.getActionCommand();
		if (cmd == "graphics") {
			layout.first(panel);
		} else if (cmd.equals("plan")) {
			plan();
			plannerStepExecutor.initialize();
			tab1.setSelectedIndex(1);

			for (String op : ansList) {
				System.out.println("ans --- " + op);
			}
		} else if (cmd.equals("OK")) {
			textsToStates();
			plan();
			for (int i = 0; i < ansList.size(); i++)
				txtAnswer.append(ansList.get(i) + "\n");
			tab2.setSelectedIndex(1);

		} else if (cmd.equals("RESET")) {
			txtStart.setText("");
			txtGoal.setText("");
			txtAnswer.setText("");

		} else if (cmd.equals("GtoS")) {
			txtStart.setText(txtGoal.getText());
			txtGoal.setText("");
		} else if (cmd.equals("text")) {
			layout.last(panel);
		} else if (cmd.equals("Open File")) {

			System.out.println("!!");
			JFileChooser fileChooser = new JFileChooser();
			fileChooser.addChoosableFileFilter(new DataFilter());
			// fileChooser.setCurrentDirectory(currentDirectory);
			fileChooser.setDialogTitle("OpenFile");
			int selected = fileChooser
					.showOpenDialog((Component) e.getSource());
			if (selected == JFileChooser.APPROVE_OPTION) {
				File file = fileChooser.getSelectedFile();
				System.out.println(file.getPath());
				// loadFile(file);
				currentFile = file;
				try {
					loadData();
				} catch (IOException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
			}

		}else if(cmd.equals("Add")){
			 String value = JOptionPane.showInputDialog(this, "ブロックの名前は？");
			 if(objects.contains(value)){
				 
			 }else{
				 objects.add("value");
				 startPanel.putBox(value, null);
				 goalPanel.putBox(value, null);
			 }
		}

	}

	private void loadData() throws IOException {
		StringBuilder sb = new StringBuilder();
		BufferedReader reader = null;
		reader = new BufferedReader(new FileReader(currentFile));
		Boolean currentGoalMidFlag = true;
		ArrayList<String> startList = new ArrayList<String>();
		ArrayList<String> goalList = new ArrayList<String>();
		try {
			String s;
			while ((s = reader.readLine()) != null) {
				System.out.println(s);
				if (s.isEmpty()) {
					currentGoalMidFlag = false;
				}
				if (currentGoalMidFlag) {
					startList.add(s);
				} else {
					goalList.add(s);
				}
			}
		} finally {
			if (reader != null) {
				reader.close();
			}
		}
		// 初期状態の準備
		Collections.sort(startList);
		initPlannerPanel(startList, startPanel);
		this.startList.clear();
		this.startList.addAll(startList);

		// 目標状態の準備
		Collections.sort(goalList);
		initPlannerPanel(goalList, goalPanel);
		this.goalList.clear();
		this.goalList.addAll(goalList);

		System.out.println("startList" + startList);
		System.out.println("goalList" + goalList);
		objects = noah.getObjects();
	}

	private class PlannerStepExecutor implements ActionListener, Runnable {
		final Pattern p1 = Pattern.compile("pick up (.*) from the table");
		final Pattern p2 = Pattern.compile("remove (.*) from (.*)");
		final Pattern p3 = Pattern.compile("put (.*) down on the table");
		final Pattern p4 = Pattern.compile("Place (.*) on (.*)");
		// JFileChooser fileChooser = new JFileChooser();
		int ptr = 0;
		boolean loop = false;
		Thread th;

		@Override
		public void actionPerformed(ActionEvent e) {
			String cmd = e.getActionCommand();

			if (cmd.equals("play")) {
				start();
			} else if (cmd.equals("pause")) {
				try {
					stop();
				} catch (Exception e0) {
					e0.printStackTrace();
				}
			} else if (cmd.equals("start")) {
				initPlannerPanel(startList, plannerPanel);
				ptr = 0;
			} else if (cmd.equals("goal")) {
				initPlannerPanel(goalList, plannerPanel);
				ptr = ansList.size() - 1;
			} else if (cmd.equals("next")) {
				if (ptr < ansList.size()) {
					String op = ansList.get(ptr);
					ptr++;
					try {
						execute(op);
					} catch (InterruptedException e0) {
						e0.printStackTrace();
					}
				}
			}
		}

		private void loadFile(File file) {
			// TODO Auto-generated method stub

		}

		public void initialize() {
			try {
				stop();
			} catch (InterruptedException e) {
			}
			ptr = 0;
			loop = false;
			initPlannerPanel(startList, plannerPanel);
			updateStepLabel();
		}

		public void updateStepLabel() {
			if (ptr == 0) {
				stepLabel.setText("初期状態");
			} else if (ptr == ansList.size()) {
				stepLabel.setText("目標状態");
			} else {
				stepLabel.setText(nth(ptr) + " process");
			}
		}

		private void execute(String op) throws InterruptedException {
			Matcher m;

			System.out.println("executing: " + op);

			// ラベル更新
			updateStepLabel();

			// pickup (.*) from the table
			m = p1.matcher(op);
			if (m.find()) {
				String name = m.group(1);
				plannerPanel.pickup(name);
				return;
			}

			// remove (.*) from (.*)
			m = p2.matcher(op);
			if (m.find()) {
				String name = m.group(1);
				plannerPanel.pickup(name);
				return;
			}

			// put (.*) down on the table
			m = p3.matcher(op);
			if (m.find()) {
				plannerPanel.place(null);
				return;
			}

			// Place (.*) on (.*)
			m = p4.matcher(op);
			if (m.find()) {
				String name = m.group(2);
				plannerPanel.place(name);
				return;
			}

		}

		private void start() {
			if (th == null) {
				loop = true;
				th = new Thread(this);
				th.start();
			}
		}

		private void stop() throws InterruptedException {
			if (th != null) {
				loop = false;
				th.interrupt();
				th.join();
				th = null;
			}
		}

		@Override
		public void run() {
			String op = null;
			try {
				while (loop) {
					synchronized (ansList) {
						op = (0 <= ptr && ptr < ansList.size()) ? ansList
								.get(ptr) : null;
					}
					if (op == null) {
						break;
					}
					ptr++;

					execute(op);
				}
			} catch (InterruptedException e) {
				System.out.println("PlannerStepExecutor: thread interrupted.");
			}
			th = null;
		}
	};

	public static void main(String[] args) {
		//

		SampleGUI gui = new SampleGUI();
		gui.setVisible(true);
	}

}
