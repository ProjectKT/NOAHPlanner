package planner;

import java.util.ArrayList;

//NOAHがもつパラメータ、複数のスレッドで共有する
public class NOAHParameters {
	//オペレーターを格納
	private ArrayList<Operator> operators;
	//現在状態
	private ArrayList<String> currentState;
	//目標状態
	private ArrayList<String> goalState;
	
	NOAHParameters(ArrayList<Operator> operators,ArrayList<String> goalState, ArrayList<String> currentState){
		this.operators = operators;
		this.currentState = currentState;
		this.goalState  = goalState;
	}
	
	/**
	 * 現在状態をセット
	 * @param State
	 */
	public void setCurrentState(ArrayList<String> State){
		currentState.clear();
		for(String str: State){
			currentState.add(str);
		}
	}
	
	/**
	 * 目標状態をセット
	 * @param GoalList
	 */
	public void setGoalState(ArrayList<String> GoalList){
		goalState.clear();
		for(String str : GoalList){
			goalState.add(str);
		}
	}
	
	public ArrayList<String> getCurrentState(){
		return currentState;
	}
	
	public ArrayList<String> getGoalState(){
		return goalState;
	}
	
}
