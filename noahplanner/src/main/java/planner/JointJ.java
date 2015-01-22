package planner;

import java.util.ArrayList;
import java.util.Iterator;

public class JointJ {
	private ArrayList<Node> forwards = new ArrayList<Node>();
	private Node back;
	
	public void addForward(Node forward){
		forwards.add(forward);
	}
	
	public void changeBack(Node back){
		this.back = back;
	}
	
	public void removeForward(Object forward){
		forwards.remove(forward);
	}
	
	public ArrayList<Node> getForward(){
		return forwards;
	}
	
	public Node getBack(){
		return back;
	}
}
