package planner;

import java.util.ArrayList;
import java.util.Iterator;

public class JointS {
	private Object forward;
	private ArrayList<Object> backs = new ArrayList<Object>();
	
	public void changeForward(Object forward){
		this.forward = forward;
	}
	
	public void addBack(Object back){
		backs.add(back);
	}

	public void removeBack(Object back){
		backs.remove(back);
	}
	
	
	public Object getForward(){
		return forward;
	}
	
	public ArrayList<Object> getBack(){
		return backs;
	}
}
