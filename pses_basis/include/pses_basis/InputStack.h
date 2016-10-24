#ifndef InputStack_H
#define InputStack_H

#include <string>
#include <list>

class InputStack{
	public:
	//InputStack();
	InputStack(unsigned int maxSize):maxSize(maxSize){};

	inline void push(std::string& data){
		if(isFull()){
			stack.pop_front();
		}
		stack.push_back(data);
	}

	inline void pop(std::string& out){
		if(!isEmpty()){
			out = stack.front();
			stack.pop_front();
		}else{
			out = "";
		}
		
	}

	inline bool isFull(){
		return maxSize==stack.size();
	}

	inline bool isEmpty(){
		return stack.size()==0;
	}

	inline int size(){
		return stack.size();
	}

	private:
	unsigned int maxSize;
	std::list<std::string> stack;
};

#endif