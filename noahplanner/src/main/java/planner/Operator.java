package planner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.List;
import java.util.StringTokenizer;

/**
 * オペレータ
 */
public class Operator {
	public String name;
	List<String> ifList;
	List<String> addList;
	List<String> deleteList;

	public Operator(String theName, List<String> theIfList, List<String> theAddList, List<String> theDeleteList) {
		name = theName;
		ifList = theIfList;
		addList = theAddList;
		deleteList = theDeleteList;
	}

	public List<String> getAddList() {
		return addList;
	}

	public List<String> getDeleteList() {
		return deleteList;
	}

	public List<String> getIfList() {
		return ifList;
	}

	public String toString() {
		String result = "NAME: " + name + "\n" + "IF :" + ifList + "\n"
				+ "ADD:" + addList + "\n" + "DELETE:" + deleteList;
		return result;
	}

	public List<String> applyState(List<String> theState) {
		for (int i = 0; i < addList.size(); i++) {
			theState.add(addList.get(i));
		}
		for (int i = 0; i < deleteList.size(); i++) {
			theState.add(deleteList.get(i));
		}
		return theState;
	}

	public Operator getRenamedOperator(int uniqueNum) {
		List<String> vars = new ArrayList<String>();
		// IfListの変数を集める
		for (int i = 0; i < ifList.size(); i++) {
			String anIf = (String) ifList.get(i);
			vars = getVars(anIf, vars);
		}
		// addListの変数を集める
		for (int i = 0; i < addList.size(); i++) {
			String anAdd = (String) addList.get(i);
			vars = getVars(anAdd, vars);
		}
		// deleteListの変数を集める
		for (int i = 0; i < deleteList.size(); i++) {
			String aDelete = (String) deleteList.get(i);
			vars = getVars(aDelete, vars);
		}
		Hashtable renamedVarsTable = makeRenamedVarsTable(vars, uniqueNum);

		// 新しいIfListを作る
		List<String> newIfList = new ArrayList<String>();
		for (int i = 0; i < ifList.size(); i++) {
			String newAnIf = renameVars((String) ifList.get(i), renamedVarsTable);
			newIfList.add(newAnIf);
		}
		// 新しいaddListを作る
		List<String> newAddList = new ArrayList<String>();
		for (int i = 0; i < addList.size(); i++) {
			String newAnAdd = renameVars((String) addList.get(i), renamedVarsTable);
			newAddList.add(newAnAdd);
		}
		// 新しいdeleteListを作る
		List<String> newDeleteList = new ArrayList<String>();
		for (int i = 0; i < deleteList.size(); i++) {
			String newADelete = renameVars((String) deleteList.get(i), renamedVarsTable);
			newDeleteList.add(newADelete);
		}
		// 新しいnameを作る
		String newName = renameVars(name, renamedVarsTable);

		return new Operator(newName, newIfList, newAddList, newDeleteList);
	}

	private List<String> getVars(String thePattern, List<String> vars) {
		StringTokenizer st = new StringTokenizer(thePattern);
		for (int i = 0; i < st.countTokens();) {
			String tmp = st.nextToken();
			if (var(tmp)) {
				vars.add(tmp);
			}
		}
		return vars;
	}

	private Hashtable makeRenamedVarsTable(List<String> vars, int uniqueNum) {
		Hashtable result = new Hashtable();
		for (int i = 0; i < vars.size(); i++) {
			String newVar = (String) vars.get(i) + uniqueNum;
			result.put((String) vars.get(i), newVar);
		}
		return result;
	}

	private String renameVars(String thePattern, Hashtable renamedVarsTable) {
		String result = new String();
		StringTokenizer st = new StringTokenizer(thePattern);
		for (int i = 0; i < st.countTokens();) {
			String tmp = st.nextToken();
			if (var(tmp)) {
				result = result + " " + (String) renamedVarsTable.get(tmp);
			} else {
				result = result + " " + tmp;
			}
		}
		return result.trim();
	}

	public Operator instantiate(HashMap<String,String> theBinding) {
		// name を具体化
		String newName = instantiateString(name, theBinding);
		// ifList を具体化
		List<String> newIfList = new ArrayList<String>();
		for (int i = 0; i < ifList.size(); i++) {
			String newIf = instantiateString((String) ifList.get(i), theBinding);
			newIfList.add(newIf);
		}
		// addList を具体化
		List<String> newAddList = new ArrayList<String>();
		for (int i = 0; i < addList.size(); i++) {
			String newAdd = instantiateString((String) addList.get(i), theBinding);
			newAddList.add(newAdd);
		}
		// deleteListを具体化
		List<String> newDeleteList = new ArrayList<String>();
		for (int i = 0; i < deleteList.size(); i++) {
			String newDelete = instantiateString((String) deleteList.get(i), theBinding);
			newDeleteList.add(newDelete);
		}
		return new Operator(newName, newIfList, newAddList, newDeleteList);
	}

	private String instantiateString(String thePattern, HashMap<String,String> theBinding) {
		String result = new String();
		StringTokenizer st = new StringTokenizer(thePattern);
		for (int i = 0; i < st.countTokens();) {
			String tmp = st.nextToken();
			if (var(tmp)) {
				String newString = (String) theBinding.get(tmp);
				if (newString == null) {
					result = result + " " + tmp;
				} else {
					result = result + " " + newString;
				}
			} else {
				result = result + " " + tmp;
			}
		}
		return result.trim();
	}

	private boolean var(String str1) {
		// 先頭が ? なら変数
		return str1.startsWith("?");
	}
}
