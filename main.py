
import pygame
import sys, time, random, heapq, math

#LAYOUT
WIN_W, WIN_H   = 1280, 720
SIDE_W         = 300
TOPBAR_H       = 46
STATUSBAR_H    = 34
SIDE_X         = WIN_W - SIDE_W
GRID_X, GRID_Y = 0, TOPBAR_H
GRID_W         = WIN_W - SIDE_W      # 980
GRID_H         = WIN_H - TOPBAR_H - STATUSBAR_H  # 640
GRID_PAD       = 18
FPS            = 60
SI             = SIDE_X + 10         # sidebar content left
SW             = SIDE_W - 20         # sidebar content width

#PALETTE 
BG        = ( 16,  18,  28)
TOPBAR_BG = (  9,  10,  18)
SIDE_BG   = ( 19,  21,  32)
SEC_BG    = ( 27,  30,  44)
SEC_HD    = ( 36,  40,  58)
GRID_BG   = ( 22,  25,  36)
DIV       = ( 40,  44,  64)
BOR       = ( 50,  56,  80)

cW   = (235, 238, 248)
cGR  = (120, 128, 152)
cDIM = ( 68,  74, 100)
cTV  = (148, 194, 255)
cTG  = ( 72, 218, 130)
cTO  = (255, 162,  44)
cTR  = (228,  74,  74)
cTY  = (255, 215,  50)   # yellow hint

# cell colours
cE  = ( 38,  42,  58)
cWL = ( 11,  12,  17)
cST = ( 44, 204,  92)
cGL = (212,  52,  52)
cAG = (255, 155,  26)
cPT = ( 26, 196, 172)
cVI = ( 48, 112, 205)
cFR = (248, 206,  38)

# button colours
BB = ( 50,  96, 185); BH = ( 70, 128, 225)
GB = ( 32, 174, 100); GH = ( 48, 208, 124)
DB = ( 36,  40,  57); DH = ( 52,  57,  78)
AC = ( 32, 174, 100)

#  HEURISTICS 
def manhattan(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
def euclidean(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])

#  ALGORITHMS 
def run_astar(grid, start, goal, hfn):
    gc={start:0}; par={start:None}
    pq=[(hfn(start,goal),0,start)]
    closed=set(); exp=0; ctr=0
    vis=set(); fro={start}
    while pq:
        _,_,cur=heapq.heappop(pq)
        if cur in closed: continue
        closed.add(cur); vis.add(cur); fro.discard(cur); exp+=1
        if cur==goal:
            path,n=[],goal
            while n: path.append(n); n=par[n]
            return list(reversed(path)),exp,vis,fro
        for dr,dc in((-1,0),(1,0),(0,-1),(0,1)):
            nb=(cur[0]+dr,cur[1]+dc)
            if 0<=nb[0]<grid.rows and 0<=nb[1]<grid.cols and grid.cells[nb[0]][nb[1]]!=1:
                ng=gc[cur]+1
                if nb not in gc or ng<gc[nb]:
                    gc[nb]=ng; par[nb]=cur; ctr+=1
                    heapq.heappush(pq,(ng+hfn(nb,goal),ctr,nb)); fro.add(nb)
    return None,exp,vis,fro

def run_gbfs(grid, start, goal, hfn):
    pq=[(hfn(start,goal),0,start,[start])]
    visited={start}; exp=0; ctr=0
    vis=set(); fro={start}
    while pq:
        _,_,cur,path=heapq.heappop(pq)
        fro.discard(cur); vis.add(cur); exp+=1
        if cur==goal: return path,exp,vis,fro
        for dr,dc in((-1,0),(1,0),(0,-1),(0,1)):
            nb=(cur[0]+dr,cur[1]+dc)
            if 0<=nb[0]<grid.rows and 0<=nb[1]<grid.cols \
               and grid.cells[nb[0]][nb[1]]!=1 and nb not in visited:
                visited.add(nb); ctr+=1
                heapq.heappush(pq,(hfn(nb,goal),ctr,nb,path+[nb])); fro.add(nb)
    return None,exp,vis,fro

# GRID MODEL 
class Grid:
    def __init__(self, rows, cols, start, goal, density=0.28):
        self.rows=rows; self.cols=cols
        self.start=start; self.goal=goal
        self.cells=[[0]*cols for _ in range(rows)]
        self.randomize(density)

    def randomize(self, d):
        self.cells=[[0]*self.cols for _ in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                if (r,c) not in (self.start, self.goal) and random.random()<d:
                    self.cells[r][c]=1

    def set_wall(self, r, c, v):
        """Place or erase a wall. Start/goal are always protected."""
        if (r,c) not in (self.start, self.goal):
            self.cells[r][c]=v

    def move_start(self, r, c):
        """Move the start node; clears wall there if any."""
        if (r,c) != self.goal:
            self.cells[self.start[0]][self.start[1]] = 0  # clear old
            self.start=(r,c)
            self.cells[r][c]=0   # clear new position too

    def move_goal(self, r, c):
        """Move the goal node; clears wall there if any."""
        if (r,c) != self.start:
            self.cells[self.goal[0]][self.goal[1]] = 0
            self.goal=(r,c)
            self.cells[r][c]=0

    def spawn_obstacle(self, safe):
        pool=[(r,c) for r in range(self.rows) for c in range(self.cols)
              if self.cells[r][c]==0 and (r,c) not in safe
              and (r,c) not in (self.start, self.goal)]
        if pool:
            r,c=random.choice(pool); self.cells[r][c]=1; return(r,c)
        return None

#  WIDGETS 
class Btn:
    def __init__(self,rect,label,base=BB,hov=BH,act=AC):
        self.rect=pygame.Rect(rect); self.label=label
        self.base=base; self.hov=hov; self.act=act; self.active=False
    def draw(self,surf,mouse,font):
        bg=self.act if self.active else(self.hov if self.rect.collidepoint(mouse) else self.base)
        pygame.draw.rect(surf,bg,self.rect,border_radius=6)
        pygame.draw.rect(surf,BOR,self.rect,1,border_radius=6)
        t=font.render(self.label,True,cW); surf.blit(t,t.get_rect(center=self.rect.center))
    def hit(self,pos): return self.rect.collidepoint(pos)


class Stepper:
    """  [−]  value  [+]  with a text label drawn to its left """
    def __init__(self, val, lo, hi, x, y, w=180, h=24, fmt=str):
        self.val=val; self.lo=lo; self.hi=hi; self.fmt=fmt
        bw=22
        self.rd=pygame.Rect(x,     y,bw,h)
        self.ri=pygame.Rect(x+w-bw,y,bw,h)
        self.rm=pygame.Rect(x+bw+2,y,w-bw*2-4,h)

    def draw(self, surf, mouse, fv):
        for r,sym in [(self.rd,"−"),(self.ri,"+")]:
            bg=BH if r.collidepoint(mouse) else DB
            pygame.draw.rect(surf,bg,r,border_radius=4)
            pygame.draw.rect(surf,BOR,r,1,border_radius=4)
            s=fv.render(sym,True,cW); surf.blit(s,s.get_rect(center=r.center))
        pygame.draw.rect(surf,SEC_BG,self.rm,border_radius=4)
        pygame.draw.rect(surf,DIV,   self.rm,1,border_radius=4)
        v=fv.render(self.fmt(self.val),True,cTV)
        surf.blit(v,v.get_rect(center=self.rm.center))

    def click(self,pos,step=1):
        if self.rd.collidepoint(pos): self.val=max(self.lo,self.val-step); return True
        if self.ri.collidepoint(pos): self.val=min(self.hi,self.val+step); return True
        return False

#  SECTION BOX 
def draw_sec(surf,x,y,w,h,title,f_sec):
    r=pygame.Rect(x,y,w,h)
    pygame.draw.rect(surf,SEC_BG,r,border_radius=7)
    pygame.draw.rect(surf,BOR,   r,1,border_radius=7)
    hb=pygame.Rect(x+1,y+1,w-2,22)
    pygame.draw.rect(surf,SEC_HD,hb,border_radius=6)
    pygame.draw.rect(surf,SEC_HD,pygame.Rect(x+1,y+12,w-2,11))
    surf.blit(f_sec.render(title,True,cGR),(x+9,y+4))

#CELL GEOMETRY 
def cell_geo(rows,cols):
    aw=GRID_W-2*GRID_PAD; ah=GRID_H-2*GRID_PAD
    cs=max(6,min(aw//cols,ah//rows))
    cx=GRID_X+(GRID_W-cs*cols)//2
    cy=GRID_Y+(GRID_H-cs*rows)//2
    return cs,cx,cy

# MAIN 
def main():
    pygame.init()
    screen=pygame.display.set_mode((WIN_W,WIN_H))
    pygame.display.set_caption("Informed Path Finder")
    clock=pygame.time.Clock()

    FT   = pygame.font.SysFont("Segoe UI",17,bold=True)
    FSB  = pygame.font.SysFont("Segoe UI",11)
    FSC  = pygame.font.SysFont("Segoe UI",10,bold=True)
    FB   = pygame.font.SysFont("Segoe UI",13,bold=True)
    FL   = pygame.font.SysFont("Segoe UI",11)
    FV   = pygame.font.SysFont("Segoe UI",11,bold=True)
    FSM  = pygame.font.SysFont("Segoe UI",10)
    FST  = pygame.font.SysFont("Segoe UI",12)
    FSTP = pygame.font.SysFont("Segoe UI",12,bold=True)

    # mutable app state 
    A = {
        "rows":20, "cols":30, "density":28,   # density in % (integer 1-60)
        "start":(0,0), "goal":(19,29),
        "grid":None, "cell":0,"cx":0,"cy":0,
        "alg":"astar", "hname":"manhattan",
        "dyn":False,
        "vis":set(),"fro":set(),
        "fp":[],"fp_set":set(),
        "agent":None,"astep":0,"moving":False,
        "nexp":0,"pcost":0,"ms":0.0,
        "status":"Press  ▶  Run Search  to begin",
        "scol":cGR,"atimer":0,
        "place_mode":"wall",   # "wall" | "start" | "goal"
    }

    def rebuild():
        r,c=A["rows"],A["cols"]
        # clamp start/goal into new grid bounds
        A["start"]=(min(A["start"][0],r-1), min(A["start"][1],c-1))
        A["goal"] =(min(A["goal"][0], r-1), min(A["goal"][1], c-1))
        if A["start"]==A["goal"]:
            A["goal"]=(r-1,c-1)
            if A["start"]==A["goal"]: A["start"]=(0,0)
        A["grid"]=Grid(r,c,A["start"],A["goal"],A["density"]/100)
        A["cell"],A["cx"],A["cy"]=cell_geo(r,c)

    def clear_path():
        A.update({"vis":set(),"fro":set(),"fp":[],"fp_set":set(),
                  "agent":None,"astep":0,"moving":False})

    def do_search(start=None):
        src=start or A["grid"].start
        hfn=manhattan if A["hname"]=="manhattan" else euclidean
        fn =run_astar if A["alg"]=="astar" else run_gbfs
        t0 =time.perf_counter()
        path,exp,vis,fro=fn(A["grid"],src,A["grid"].goal,hfn)
        A["ms"]=(time.perf_counter()-t0)*1000
        A["vis"]=vis; A["fro"]=fro; A["nexp"]+=exp
        if path:
            pre=A["fp"][:A["astep"]] if A["astep"] else []
            A["fp"]=pre+path; A["fp_set"]=set(A["fp"])
            A["agent"]=src; A["astep"]=len(pre)
            A["moving"]=True; A["pcost"]=len(path)-1
            A["status"]=(f"Path found — cost {A['pcost']}   "
                         f"expanded {A['nexp']}   {A['ms']:.2f} ms")
            A["scol"]=cTG
        else:
            A["moving"]=False
            A["status"]="No path found — try removing some walls"
            A["scol"]=cTR



    def sec_r(y,h): return (SIDE_X+6, y, SIDE_W-12, h)

    yA=TOPBAR_H+4; hA=82
    yH=yA+hA+3;   hH=76
    yS=yH+hH+3;   hS=106
    yC=yS+hS+3;   hC=110
    yM=yC+hC+3;   hMt=140
    yL=yM+hMt+3;  hL=88

    bw=SIDE_W-24; BH2=30; BHS=26

    def mb(y,lbl,base=BB,hov=BH,act=AC):
        return Btn((SI,y,bw,BH2),lbl,base,hov,act)
    def mb2(x,y,w,lbl,base=DB,hov=DH):
        return Btn((x,y,w,BHS),lbl,base,hov,base)

    # algorithm
    b_as=mb(yA+26,"A*  Search");        b_as.active=True
    b_gb=mb(yA+60,"Greedy Best-First")
    # heuristic
    b_mn=mb(yH+26,"Manhattan Distance"); b_mn.active=True
    b_eu=mb(yH+56,"Euclidean Distance")
    # grid-size steppers
    stp_row=Stepper(A["rows"],5,40,  SI+50,yS+28,w=bw-50,h=24)
    stp_col=Stepper(A["cols"],5,60,  SI+50,yS+56,w=bw-50,h=24)
    stp_den=Stepper(A["density"],5,60, SI+50,yS+82,w=bw-50,h=24,fmt=lambda v:f"{v}%")
    # controls
    b_rn=mb(yC+26,"▶   Run Search",GB,GH,GH)
    b_dy=mb(yC+60,"Dynamic Mode: OFF")
    hw2=(bw-6)//2
    b_nw=mb2(SI,      yC+94,hw2,"↺  New Grid")
    b_cl=mb2(SI+hw2+6,yC+94,hw2,"✕  Clear")

    # place-mode buttons (inside Grid Size section, row 4)
    # mode: wall / start / goal
    bpm_w=mb2(SI,        yS+82+28, (bw-12)//3,   "✏  Wall",  DB, DH)
    bpm_s=mb2(SI+(bw-12)//3+6,  yS+82+28, (bw-12)//3, "S  Start", DB, DH)
    bpm_g=mb2(SI+((bw-12)//3+6)*2, yS+82+28, (bw-12)//3, "G  Goal",  DB, DH)
    bpm_w.active=True   # default

    rebuild()

    # ── draw helpers ─────────────────────────────────────
    def draw_topbar():
        pygame.draw.rect(screen,TOPBAR_BG,(0,0,WIN_W,TOPBAR_H))
        pygame.draw.line(screen,BOR,(0,TOPBAR_H),(WIN_W,TOPBAR_H),1)
        screen.blit(FT.render("Informed Path Finder",True,cW),(16,8))
        
        info=FSB.render(
            f"Grid: {A['rows']}×{A['cols']}   Density: {A['density']}%   "
            f"Cell: {A['cell']}px   Alg: {'A*' if A['alg']=='astar' else 'GBFS'}   "
            f"Heuristic: {A['hname'].capitalize()}",
            True,cDIM)
        screen.blit(info,(SIDE_X-info.get_width()-14,18))

    def draw_statusbar():
        y=WIN_H-STATUSBAR_H
        pygame.draw.rect(screen,TOPBAR_BG,(0,y,WIN_W,STATUSBAR_H))
        pygame.draw.line(screen,BOR,(0,y),(WIN_W,y),1)
        screen.blit(FST.render(A["status"],True,A["scol"]),(16,y+10))

        mode_hints = {
            "wall":  "LClick: place wall   RClick: erase   S+Click: set Start   G+Click: set Goal",
            "start": "Click anywhere on grid to place Start node   (press W to switch back)",
            "goal":  "Click anywhere on grid to place Goal node    (press W to switch back)",
        }
        h=FSM.render(mode_hints[A["place_mode"]],True,cDIM)
        screen.blit(h,(WIN_W-h.get_width()-14,y+11))

    def draw_grid_area():
        pygame.draw.rect(screen,GRID_BG,(GRID_X,GRID_Y,GRID_W,GRID_H))
        pygame.draw.line(screen,BOR,(SIDE_X,GRID_Y),(SIDE_X,WIN_H-STATUSBAR_H),1)
        cs=A["cell"]; cx=A["cx"]; cy=A["cy"]; g=A["grid"]
        for r in range(A["rows"]):
            for c in range(A["cols"]):
                pos=(r,c); v=g.cells[r][c]
                if   pos==g.start:          col=cST
                elif pos==g.goal:           col=cGL
                elif v==1:                  col=cWL
                elif A["agent"]==pos:       col=cAG
                elif pos in A["fp_set"]:    col=cPT
                elif pos in A["vis"]:       col=cVI
                elif pos in A["fro"]:       col=cFR
                else:                       col=cE
                gap=1 if cs>=9 else 0
                br =2 if cs>=13 else 0
                pygame.draw.rect(screen,col,
                    (cx+c*cs+gap,cy+r*cs+gap,cs-gap*2,cs-gap*2),border_radius=br)

        # placement-mode overlay hint on grid
        if A["place_mode"]=="start":
            pygame.draw.rect(screen,cST,(GRID_X,GRID_Y,GRID_W,GRID_H),3)
            t=FB.render("Click to place  START",True,cST)
            screen.blit(t,(GRID_X+(GRID_W-t.get_width())//2, GRID_Y+8))
        elif A["place_mode"]=="goal":
            pygame.draw.rect(screen,cGL,(GRID_X,GRID_Y,GRID_W,GRID_H),3)
            t=FB.render("Click to place  GOAL",True,cGL)
            screen.blit(t,(GRID_X+(GRID_W-t.get_width())//2, GRID_Y+8))

    def draw_sidebar():
        pygame.draw.rect(screen,SIDE_BG,(SIDE_X,0,SIDE_W,WIN_H))
        pygame.draw.rect(screen,TOPBAR_BG,(SIDE_X,0,SIDE_W,TOPBAR_H))
        pygame.draw.line(screen,BOR,(SIDE_X,TOPBAR_H),(SIDE_X+SIDE_W,TOPBAR_H),1)
        pygame.draw.line(screen,BOR,(SIDE_X,0),(SIDE_X,WIN_H),1)
        screen.blit(FB.render("Pathfinding Agent",True,cW),(SIDE_X+12,15))
        mouse=pygame.mouse.get_pos()

        # ── ALGORITHM ────────────────────────────────────
        draw_sec(screen,*sec_r(yA,hA),"ALGORITHM",FSC)
        b_as.draw(screen,mouse,FB); b_gb.draw(screen,mouse,FB)

        # ── HEURISTIC ────────────────────────────────────
        draw_sec(screen,*sec_r(yH,hH),"HEURISTIC",FSC)
        b_mn.draw(screen,mouse,FB); b_eu.draw(screen,mouse,FB)

        # ── GRID SETTINGS ────────────────────────────────
        draw_sec(screen,*sec_r(yS,hS),"GRID SETTINGS",FSC)
        for lbl,sy in [("Rows",yS+30),("Cols",yS+58),("Density",yS+84)]:
            screen.blit(FL.render(lbl,True,cGR),(SI,sy+3))
        stp_row.draw(screen,mouse,FSTP)
        stp_col.draw(screen,mouse,FSTP)
        stp_den.draw(screen,mouse,FSTP)
        # place-mode sub-row
        screen.blit(FL.render("Place",True,cGR),(SI,yS+84+32))
        bpm_w.draw(screen,mouse,FL); bpm_s.draw(screen,mouse,FL); bpm_g.draw(screen,mouse,FL)

        # ── CONTROLS ─────────────────────────────────────
        draw_sec(screen,*sec_r(yC,hC),"CONTROLS",FSC)
        b_rn.draw(screen,mouse,FB); b_dy.draw(screen,mouse,FB)
        b_nw.draw(screen,mouse,FB); b_cl.draw(screen,mouse,FB)

        # ── METRICS ──────────────────────────────────────
        draw_sec(screen,*sec_r(yM,hMt),"METRICS",FSC)
        met=[
            ("Nodes Expanded", str(A["nexp"]),                            cTV),
            ("Path Cost",      str(A["pcost"]),                           cTV),
            ("Exec Time",      f"{A['ms']:.2f} ms",                       cTV),
            ("Algorithm",      "A* Search" if A["alg"]=="astar" else "GBFS", cW),
            ("Heuristic",      A["hname"].capitalize(),                   cW),
            ("Dynamic",        "ON" if A["dyn"] else "OFF",
                               cTG if A["dyn"] else cGR),
            ("Grid",           f"{A['rows']} × {A['cols']}",              cDIM),
            ("Density",        f"{A['density']}%",                        cDIM),
            ("Cell Size",      f"{A['cell']} px",                         cDIM),
        ]
        my=yM+26
        for lbl,val,vc in met:
            ls=FL.render(lbl,True,cGR)
            vs=FV.render(val,True,vc)
            screen.blit(ls,(SI,my))
            screen.blit(vs,(SI+bw-vs.get_width(),my))
            pygame.draw.line(screen,DIV,(SI,my+13),(SI+bw,my+13),1)
            my+=14

        # ── LEGEND ───────────────────────────────────────
        draw_sec(screen,*sec_r(yL,hL),"LEGEND",FSC)
        legend=[(cST,"Start"),(cGL,"Goal"),(cAG,"Agent"),(cPT,"Path"),
                (cVI,"Visited"),(cFR,"Frontier"),(cWL,"Wall"),(cE,"Empty")]
        cw2=bw//2
        ly0=yL+24
        for i,(col,lbl) in enumerate(legend):
            lx=SI+(i%2)*cw2
            ly2=ly0+(i//2)*16
            pygame.draw.rect(screen,col,(lx,ly2+1,11,11),border_radius=2)
            pygame.draw.rect(screen,BOR,(lx,ly2+1,11,11),1,border_radius=2)
            screen.blit(FSM.render(lbl,True,cW),(lx+15,ly2+1))

    # ── main loop ────────────────────────────────────────
    STEP_MS=100
    while True:
        dt=clock.tick(FPS)
        for ev in pygame.event.get():
            if ev.type==pygame.QUIT: pygame.quit(); sys.exit()

            elif ev.type==pygame.MOUSEBUTTONDOWN:
                mx,my2=ev.pos
                keys=pygame.key.get_pressed()

                # ── grid click ───────────────────────────
                if GRID_X<=mx<SIDE_X and GRID_Y<=my2<WIN_H-STATUSBAR_H:
                    cs=A["cell"]; cx=A["cx"]; cy=A["cy"]
                    c2=(mx-cx)//cs; r2=(my2-cy)//cs
                    if 0<=r2<A["rows"] and 0<=c2<A["cols"]:
                        g=A["grid"]
                        mode=A["place_mode"]
                        if mode=="start":
                            g.move_start(r2,c2)
                            A["start"]=(r2,c2)
                            A["place_mode"]="wall"
                            bpm_w.active=True; bpm_s.active=False; bpm_g.active=False
                            clear_path()
                            A["status"]="Start moved — press ▶ Run Search"; A["scol"]=cTY
                        elif mode=="goal":
                            g.move_goal(r2,c2)
                            A["goal"]=(r2,c2)
                            A["place_mode"]="wall"
                            bpm_w.active=True; bpm_s.active=False; bpm_g.active=False
                            clear_path()
                            A["status"]="Goal moved — press ▶ Run Search"; A["scol"]=cTY
                        else:
                            # keyboard shortcuts: hold S or G while clicking
                            if keys[pygame.K_s]:
                                g.move_start(r2,c2); A["start"]=(r2,c2)
                                clear_path()
                                A["status"]="Start moved — press ▶ Run Search"; A["scol"]=cTY
                            elif keys[pygame.K_g]:
                                g.move_goal(r2,c2); A["goal"]=(r2,c2)
                                clear_path()
                                A["status"]="Goal moved — press ▶ Run Search"; A["scol"]=cTY
                            else:
                                g.set_wall(r2,c2,1 if ev.button==1 else 0)
                                clear_path()
                                A["status"]="Grid edited — press ▶ Run Search"; A["scol"]=cGR

                # ── sidebar click ────────────────────────
                elif mx>=SIDE_X:
                    if   b_as.hit(ev.pos): A["alg"]="astar"; b_as.active=True;  b_gb.active=False
                    elif b_gb.hit(ev.pos): A["alg"]="gbfs";  b_gb.active=True;  b_as.active=False
                    elif b_mn.hit(ev.pos): A["hname"]="manhattan"; b_mn.active=True; b_eu.active=False
                    elif b_eu.hit(ev.pos): A["hname"]="euclidean"; b_eu.active=True; b_mn.active=False
                    elif b_rn.hit(ev.pos):
                        A["nexp"]=0; A["pcost"]=0; A["ms"]=0.0; clear_path(); do_search()
                    elif b_dy.hit(ev.pos):
                        A["dyn"]=not A["dyn"]
                        b_dy.label=f"Dynamic Mode: {'ON' if A['dyn'] else 'OFF'}"
                        b_dy.active=A["dyn"]
                    elif b_nw.hit(ev.pos):
                        A["rows"]=stp_row.val; A["cols"]=stp_col.val; A["density"]=stp_den.val
                        A["nexp"]=0; A["pcost"]=0; A["ms"]=0.0; clear_path(); rebuild()
                        A["status"]="New grid — press ▶ Run Search"; A["scol"]=cGR
                    elif b_cl.hit(ev.pos):
                        clear_path(); A["nexp"]=0; A["pcost"]=0
                        A["status"]="Path cleared"; A["scol"]=cGR
                    # place-mode buttons
                    elif bpm_w.hit(ev.pos):
                        A["place_mode"]="wall";  bpm_w.active=True;  bpm_s.active=False; bpm_g.active=False
                    elif bpm_s.hit(ev.pos):
                        A["place_mode"]="start"; bpm_s.active=True;  bpm_w.active=False; bpm_g.active=False
                        A["status"]="Click on the grid to place the Start node"; A["scol"]=cST
                    elif bpm_g.hit(ev.pos):
                        A["place_mode"]="goal";  bpm_g.active=True;  bpm_w.active=False; bpm_s.active=False
                        A["status"]="Click on the grid to place the Goal node"; A["scol"]=cGL
                    else:
                        # steppers
                        stp_row.click(ev.pos); stp_col.click(ev.pos); stp_den.click(ev.pos)

            elif ev.type==pygame.KEYDOWN:
                if ev.key==pygame.K_r:
                    A["nexp"]=0; A["pcost"]=0; A["ms"]=0.0; clear_path(); do_search()
                elif ev.key==pygame.K_SPACE:
                    A["dyn"]=not A["dyn"]
                    b_dy.label=f"Dynamic Mode: {'ON' if A['dyn'] else 'OFF'}"
                    b_dy.active=A["dyn"]
                elif ev.key==pygame.K_w:
                    A["place_mode"]="wall"
                    bpm_w.active=True; bpm_s.active=False; bpm_g.active=False

        # agent movement
        if A["moving"] and A["fp"]:
            A["atimer"]+=dt
            if A["atimer"]>=STEP_MS:
                A["atimer"]=0
                if A["astep"]<len(A["fp"])-1:
                    A["astep"]+=1; A["agent"]=A["fp"][A["astep"]]
                    if A["dyn"] and random.random()<0.07:
                        safe=set(A["fp"])
                        obs=A["grid"].spawn_obstacle(safe)
                        if obs and obs in A["fp"][A["astep"]:]:
                            A["status"]=f"Obstacle at {obs}! Re-planning…"; A["scol"]=cTO
                            do_search(start=A["agent"])
                else:
                    A["moving"]=False
                    A["status"]=(f"✔  Goal reached!   cost={A['pcost']}   "
                                 f"expanded={A['nexp']}   {A['ms']:.1f} ms")
                    A["scol"]=cTG

        screen.fill(BG)
        draw_topbar()
        draw_grid_area()
        draw_sidebar()
        draw_statusbar()
        pygame.display.flip()

if __name__=="__main__":
    main()
