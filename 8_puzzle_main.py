import numpy as np
from algos import Node
from tkinter import *
import cv2
from PIL import ImageTk, Image
import time
import random

def solveFull():
    global solveAll
    solveAll=1
    nextState()
        
def clearAll():
    global pointer, screen1
    pointer=0
    screen1.destroy()
    main_screen()
    
def nextState():
    global pointer, path, screen1, btnNext
    pointer=pointer+1
    if pointer==len(path):
        #btnNext.pack_forget()
        Label(screen1,text="Goal State Reached",font=("calibri",18),height=2,width=20,bg="grey",fg='#e9d8a6').place(x=550/4,y=650,anchor=CENTER)
        Button(screen1,text="Go To Menu",font=("calibri",18),height=1,width=12,bg="#0a9396",fg='#e9d8a6',\
           command=clearAll).place(x=3*550/4,y=650,anchor=CENTER)
    else:
        showOnScreen(path[pointer])
        
        
def showOnScreen(state):
    global tkimgs, screen1,pointer, path
    startx=45
    starty=145
    size=150
    for i in range(0,3):
        for j in range(0,3):
            piece=tkimgs[state[i][j]]
            Label(screen1,image=piece).place(x=startx+(size+5)*i,y=starty+(size+5)*j,anchor=NW)
    global solveAll, btnNext, step_text, step_info, factor, time
    time=round(time,5)
    if pointer!=len(path)-1:
        Label(screen1,text=f'{step_text} {abs(step_info[pointer+1]-factor)}',font=("calibri",18),height=1,width=25,fg="#0a9396")\
                                      .place(x=550/2,y=710,anchor=CENTER)
    Label(screen1,text=f'Total Steps: {len(path)}     Time Taken:{time}',font=("calibri",18),height=1,width=40,fg="#0a9396")\
                                      .place(x=550/2,y=750,anchor=CENTER)
    if solveAll==1:
            n_time=800
            if len(path)>40:
                n_time=300
            screen1.after(n_time,nextState)
    btnNext=Button(screen1,text="NEXT STATE",font=("calibri",18),height=1,width=12,bg="#005f73",fg='#e9d8a6',\
           command=nextState).place(x=3*550/4,y=650,anchor=CENTER)
    Button(screen1,text="SOLVE FULL",font=("calibri",18),height=1,width=12,bg="#0a9396",fg='#e9d8a6',\
           command=solveFull).place(x=550/4,y=650,anchor=CENTER)
    screen1.mainloop()
    
def visualize(path):
    global screen1, width1, height1, pointer, solveAll, run_times
    solveAll=0
    if path==False:
        Label(screen1,text="This problem tree is too deep for this algorithm.",fg='red',width=width1,height="2",font=("calibri",15)).pack(pady=20)
        Button(screen1,text="Go Back To Menu",font=("calibri",22),height=1,width=18,bg="#001219",fg='#e9d8a6',\
           command=clearAll).place(x=550/2,y=350,anchor=CENTER)
        
    else:
        Label(screen1,text="Keep clicking to go to next step",bg="#001219",fg='#94d2bd',width=width1,height="1",font=("calibri",15)).pack(pady=5)
        if run_times==0:
            cv2.imshow("Goal State",img1)
            run_times=1
        global tkimgs, pieces
        count=0
        for i in range(0,3):
            for j in range(0,3):
                piece=pieces[i][j]
                print(type(piece))
                im = Image.fromarray(piece)
                imgtk = ImageTk.PhotoImage(image=im)
                if count!=8:
                    tkimgs[count+1]=imgtk
                    count+=1
                else:
                    tkimgs[0]=imgtk
        showOnScreen(path[pointer])
        screen1.mainloop()
        

def show_compare(path,data,time,step_counts,al):
    global screen1, width1, options
    Label(screen1,text=f'{options[al]}',bg="#001219",fg='#94d2bd',width=width1,height="1",font=("calibri",15)).pack()
    if path:
        time=round(time,5)
        Label(screen1,text=f'Steps:{len(path)}   Time taken:{time}    Nodes Visited: {step_counts}',bg="#E9D8A6",fg='#001219',width=width1,height="1",font=("calibri",15)).pack()
    else:
        Label(screen1,text="Problem tree too deep for this algo",bg="#E9D8A6",fg='#001219',width=width1,height="1",font=("calibri",15)).pack()
    return 
        
def compare_all(goal_state,root_node):
    global screen1, width1
    Label(screen1,text="",bg="#E9D8A6",fg='#001219',width=width1,height="1",font=("calibri",15)).pack()
    path,data,time,step_counts=root_node.breadth_first_search(goal_state)
    done=show_compare(path,data,time,step_counts,0)
    path,data,time,step_counts=root_node.depth_first_search(goal_state)
    done=show_compare(path,data,time,step_counts,1)
    path,data,time,step_counts=root_node.best_first_search(goal_state)
    done=show_compare(path,data,time,step_counts,2)
    path,data,time,step_counts=root_node.uniform_cost_search(goal_state)
    done=show_compare(path,data,time,step_counts,3)
    path,data,time,step_counts=root_node.iterative_deepening_DFS(goal_state)
    done=show_compare(path,data,time,step_counts,4)
    path,data,time,step_counts=root_node.a_star_search(goal_state,"num_misplaced")
    done=show_compare(path,data,time,step_counts,5)
    path,data,time,step_counts=root_node.a_star_search(goal_state,"manhattan")
    done=show_compare(path,data,time,step_counts,6)
    path,data,time,step_counts=root_node.a_star_search(goal_state,"fair_manhattan")
    done=show_compare(path,data,time,step_counts,7)
    Button(screen1,text="Go To Menu",font=("calibri",18),height=1,width=12,bg="#0a9396",fg='#e9d8a6',\
           command=clearAll).place(x=3*550/4,y=650,anchor=CENTER)
    screen1.mainloop()
    
    
def segregate(initial_state,algo):
    print(algo)
    goal_state = np.array([1,2,3,4,5,6,7,8,0]).reshape(3,3)
    print(goal_state)
    root_node = Node(state=initial_state,parent=None,action=None,depth=0,step_cost=0,path_cost=0,heuristic_cost=0)
    ["Breadth First Search","Depth First Search","Best First Search","Uniform Cost Search","Iterative Deepening DFS",\
         "A* Search (Misplaced)","A* Search (Manhattan)","A* Search (Fair-Manhattan)","Compare All Algorithms",""]
    global path, data, step_text, step_info, factor, time 
    if algo== 0:
        path,data,time,step_counts=root_node.breadth_first_search(goal_state)
        if path!=False:
            step_text="depth in tree:"
            step_info=data[0]
            factor=0
    elif algo== 1:
        path,data,time,step_counts=root_node.depth_first_search(goal_state)
        if path!=False:
            step_text="depth in tree:"
            step_info=data[0]
            factor=0
    elif algo== 2:
        path,data,time,step_counts=root_node.best_first_search(goal_state)
        if path!=False:
            step_text="Cost:"
            step_info=data[1]
            factor=0
    elif algo==3:
        path,data,time,step_counts=root_node.uniform_cost_search(goal_state)
        if path!=False:
            step_text="Path Cost:"
            step_info=data[2]
            factor=max(step_info)
    elif algo== 4:
        path,data,time,step_counts=root_node.iterative_deepening_DFS(goal_state)
        if path!=False:
            step_text="depth in tree:"
            step_info=data[0]
            factor=0
    elif algo== 5:
        path,data,time,step_counts=root_node.a_star_search(goal_state,"num_misplaced")
        if path!=False:
            step_text="Misplaced Cost:"
            step_info=data[1]
            factor=0
    elif algo== 6:
        path,data,time,step_counts=root_node.a_star_search(goal_state,"manhattan")
        if path!=False:
            step_text="Manhattan Cost:"
            step_info=data[1]
            factor=min(step_info)
    elif algo== 7:
        path,data,time,step_counts=root_node.a_star_search(goal_state,"fair_manhattan")
        if path!=False:
            step_text="Fair-Manhattan Cost:"
            step_info=data[1]
            factor=min(step_info)
    elif algo==8:
        compare_all(goal_state,root_node)
    #path,data,time,step_counts=root_node.iterative_deepening_DFS(goal_state)
    #path,data,time,step_counts=root_node.a_star_search(goal_state,"num_misplaced")
    print(data,time)
    global pointer
    ponter=0
    visualize(path)
    
def getOpts(): 
        global screen, clicked, Lb1, case, algo
        for i in Lb1.curselection():
            algo=(Lb1.get(i))

            
        case=clicked.get()
        init={"TEST":test,"EASY":easy,"MEDIUM":medium,"HARD":hard,"RANDOM":rand}
        initial_state=init[case]
        algorithm=options.index(algo)
        print (initial_state,'\n')
        screen.destroy()
        global screen1, width1, height1
        screen1=Tk()
        screen1.title("Visualizer")
        width1=550
        height1= 800
        screen1.geometry(f'{width1}x{height1}')
        Label(screen1,text="8-Puzzle Solving Algorithm Visualizer",bg="#005f73",fg='#e9d8a6',width=width1,height="2",font=("calibri",20)).pack()
        Label(screen1,text=f'{case} case for {algo}',bg="#94d2bd",fg='#001219',width=width1,height="1",font=("calibri",15)).pack()
        segregate(initial_state,algorithm)
        screen1.mainloop()

def main_screen():
    global screen
    screen=Tk()
    screen.title("8-PUZZLE")
    width=500
    height=500
    screen.geometry(f'{width}x{height}')
    
    
    Label(screen,text="8-Puzzle Solving Algorithm Visualizer",bg="#005f73",fg='#e9d8a6',width=width,height="2",font=("calibri",20)).pack()
    Label(screen,text="Choose An Algorithm",bg="#001219",fg='#94d2bd',width=width,height="1",font=("calibri",15)).pack()
    global clicked, Lb1, case, algo
    Lb1=Listbox(screen,bg="#e9d8a6",fg="#001219",width=50,height=len(options),font=("calibri",15),highlightbackground="#0a9396",highlightthickness=2,selectbackground="#0a9396")
    for option in options:
        Lb1.insert(END,option)
    Lb1.pack()

    Label(screen,text="Choose Difficulty of Sample",bg="#001219",fg='#94d2bd',width=width,height="1",font=("calibri",15)).pack()
    clicked=StringVar()
    clicked.set(difficulty[1])
    drop=OptionMenu(screen,clicked,*difficulty)
    drop.config(bg="#e9d8a6",fg="#001219",width=50,font=("calibri",15),highlightbackground="#0a9396",highlightthickness=2)
    drop.pack()

    bookbutton=Button(screen,text="VISUALIZE",font=("calibri",22),height=2,width=12,bg="#0a9396",fg='#e9d8a6',command=getOpts).pack(pady=10)


    screen.mainloop()


    
img1=cv2.imread("ironman.png")
img1=img1[75:525,175:625]
img1=cv2.resize(img1,(450,450))
blue,green,red = cv2.split(img1)
img = cv2.merge((red,green,blue))
global pieces
pieces=[]
tkimgs={}
for i in range(0,3):
    row=[]
    for j in range(0,3):
        piece=img[150*j:150*(j+1),150*i:150*(i+1)]
        if i==2 and j==2:
            piece=np.zeros((150,150,3),dtype="uint8")    
        row.append(piece)
    pieces.append(row)
    
run_times=0
pointer=0
btnNext=None
test = np.array([1,2,3,4,0,6,7,5,8]).reshape(3,3)
easy = np.array([2,0,3,1,4,6,7,5,8]).reshape(3,3)
medium = np.array([2,3,6,1,4,8,7,5,0]).reshape(3,3)
hard = np.array([3,4,6,2,7,8,1,0,5]).reshape(3,3)
rand=[1,2,3,4,0,6,7,5,8]
np.random.shuffle(rand)
rand=np.array(rand).reshape(3,3)
print(rand)

print(hard)
difficulty=["TEST","EASY","MEDIUM","HARD"]
#difficulty=["TEST","EASY","MEDIUM","HARD","RANDOM"]
options=["Breadth First Search","Depth First Search","Best First Search","Uniform Cost Search","Iterative Deepening DFS",\
         "A* Search (Misplaced)","A* Search (Manhattan)","A* Search (Fair-Manhattan)","Compare All Algorithms"]


main_screen()


