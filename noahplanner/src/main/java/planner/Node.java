package planner;

public class Node {
	//ノードの内容
	private String node;
	//ノードの番号
	private int number;
	//ノードがどこにつながっているか（前）
	private Object forward;
	//ノードがどこにつながっているか（後）
	private Object back;
	
	Node(String node, int num, Object forward, Object back){
		this.node = node;
		this.number = num;
		this.forward = forward;
		this.back = back;
	}
	
	public void changeForward(Object forward){
		this.forward = forward;
	}
	
	public void changeBack(Object back){
		this.back = back;
	}
	
	public String getNodeName(){
		return node;
	}
	
	public int getNodeNumber(){
		return number;
	}
	
	public Object getForward(){
		return forward;
	}
	
	public Object getBack(){
		return back;
	}
	
	public String toString(){

		return node;
	}
	
}
