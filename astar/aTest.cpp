#include <iostream>
#include <vector>
#include <stack>
#include <utility>
#include <set>
#include <math.h>
#include <random>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#define maxR 9
#define maxC 10

//pair<int,int>
//pair<double,pair<int,int>>
struct cell{
    int parI;
    int parJ;
    double f;
    double c;
    double h;
};

bool validCheck(int row,int col){
    return (row>=0&&row<maxR&&col>=0&&col<maxC);

}
bool blockCheck(int grid[][maxC],int row, int col){
    if(grid[row][col]==1){
        return 1;
    } else {
        return 0;
    }

}
bool destCheck(int row,int col,std::pair<int,int> dest){
    if(row==dest.first&&col==dest.second){
        return 1;
    } else {
        return 0;
    }

}

//Euclidean heuristics
double calcH(int row,int col,std::pair<int,int> d){
    return sqrt(pow(row-d.first,2)+pow(col-d.second,2));
}

void pathDetails(cell cellD[][maxC],std::pair <int,int> d){
    printf("Path:");
    int row=d.first;
    int col=d.second;
   
    
    std::stack<std::pair<int,int>> Path;
    while(!(cellD[row][col].parI==row&&cellD[row][col].parJ==col)){
        Path.push(std::make_pair(row,col));
        int tR=cellD[row][col].parI;
        int tC=cellD[row][col].parJ;
        row=tR;
        col=tC;

    }
    Path.push(std::make_pair(row,col));
    
    while(!Path.empty()){
        std::pair<int,int> p=Path.top();
        Path.pop();
        printf("->(%d %d)",p.first,p.second);
        
    }
    
}
void aStar(int grid[][maxC],std::pair <int,int> s,std::pair<int,int> t,cell cellD[][maxC]){
    bool close[maxR][maxC];
    memset(close,false,sizeof(close));
    int i,j;
    
    for(i=0;i<maxR;i++){
        for(j=0;j<maxC;j++){
            cellD[i][j].f=FLT_MAX;
            cellD[i][j].c=FLT_MAX;
            cellD[i][j].h=FLT_MAX;
            cellD[i][j].parI=-1;
            cellD[i][j].parJ=-1;
        }
    }
    
    i=s.first;
    j=s.second;
    cellD[i][j].f=0;
    cellD[i][j].c=0;
    cellD[i][j].h=0;
    cellD[i][j].parI=i;
    cellD[i][j].parJ=j;

    std::set<std::pair<double,std::pair<int,int>>> open;
    open.insert(std::make_pair(0,std::make_pair(i,j)));

    bool destC=false;
    while(!open.empty()){
        std::pair<double,std::pair<int,int>> p=*open.begin();
        open.erase(open.begin());

        i=p.second.first;
        j=p.second.second;
        close[i][j]=true;

      /*
         Generating all the 8 successor of this cell
 
             N.W   N   N.E
               \   |   /
                \  |  /
             W----Cell----E
                 / | \
               /   |  \
            S.W    S   S.E
 
         Cell-->Popped Cell (i, j)
         N -->  North       (i-1, j)
         S -->  South       (i+1, j)
         E -->  East        (i, j+1)
         W -->  West           (i, j-1)
         N.E--> North-East  (i-1, j+1)
         N.W--> North-West  (i-1, j-1)
         S.E--> South-East  (i+1, j+1)
         S.W--> South-West  (i+1, j-1)*/
         double nC,nH,nF;
         if(validCheck(i-1,j)==true){
            if(destCheck(i-1,j,t)==true){
                cellD[i-1][j].parI=i;
                cellD[i-1][j].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
                
            }else if(close[i-1][j]==false&&blockCheck(grid,i-1,j)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i-1,j,t);
                nF=nH+nC;
                if(cellD[i-1][j].f==FLT_MAX||cellD[i-1][j].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i-1,j)));
                    cellD[i - 1][j].f = nF;
                    cellD[i - 1][j].c = nC;
                    cellD[i - 1][j].h = nH;
                    cellD[i - 1][j].parI = i;
                    cellD[i - 1][j].parJ = j;
                }
            }
         }  
        if(validCheck(i+1,j)==true){
            if(destCheck(i+1,j,t)==true){
                cellD[i+1][j].parI=i;
                cellD[i+1][j].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
            }else if(close[i+1][j]==false&&blockCheck(grid,i+1,j)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i+1,j,t);
                nF=nH+nC;
                if(cellD[i+1][j].f==FLT_MAX||cellD[i+1][j].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i+1,j)));
                    cellD[i + 1][j].f = nF;
                    cellD[i + 1][j].c = nC;
                    cellD[i + 1][j].h = nH;
                    cellD[i + 1][j].parI = i;
                    cellD[i + 1][j].parJ = j;
                }
            }
         }  
         if(validCheck(i,j+1)==true){
            if(destCheck(i,j+1,t)==true){
                cellD[i][j+1].parI=i;
                cellD[i][j+1].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
            }else if(close[i][j+1]==false&&blockCheck(grid,i,j+1)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i,j+1,t);
                nF=nH+nC;
                if(cellD[i][j+1].f==FLT_MAX||cellD[i][j+1].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i,j+1)));
                    cellD[i][j+1].f = nF;
                    cellD[i][j+1].c = nC;
                    cellD[i][j+1].h = nH;
                    cellD[i][j+1].parI = i;
                    cellD[i ][j+1].parJ = j;
                }
            }
         }  
         if(validCheck(i,j-1)==true){
            if(destCheck(i,j-1,t)==true){
                cellD[i][j-1].parI=i;
                cellD[i][j-1].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
            }else if(close[i][j-1]==false&&blockCheck(grid,i,j-1)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i,j-1,t);
                nF=nH+nC;
                if(cellD[i][j-1].f==FLT_MAX||cellD[i][j-1].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i,j-1)));
                    cellD[i][j-1].f = nF;
                    cellD[i][j-1].c = nC;
                    cellD[i][j-1].h = nH;
                    cellD[i][j-1].parI = i;
                    cellD[i ][j-1].parJ = j;
                }
            }
         }  
         if(validCheck(i+1,j+1)==true){
            if(destCheck(i+1,j+1,t)==true){
                cellD[i+1][j+1].parI=i;
                cellD[i+1][j+1].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
            }else if(close[i+1][j+1]==false&&blockCheck(grid,i+1,j+1)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i+1,j+1,t);
                nF=nH+nC;
                if(cellD[i+1][j+1].f==FLT_MAX||cellD[i+1][j+1].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i+1,j+1)));
                    cellD[i+1][j+1].f = nF;
                    cellD[i+1][j+1].c = nC;
                    cellD[i+1][j+1].h = nH;
                    cellD[i+1][j+1].parI = i;
                    cellD[i +1][j+1].parJ = j;
                }
            }
         }  
         if(validCheck(i-1,j+1)==true){
            if(destCheck(i-1,j+1,t)==true){
                cellD[i-1][j+1].parI=i;
                cellD[i-1][j+1].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
            }else if(close[i-1][j+1]==false&&blockCheck(grid,i-1,j+1)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i-1,j+1,t);
                nF=nH+nC;
                if(cellD[i-1][j+1].f==FLT_MAX||cellD[i-1][j+1].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i-1,j+1)));
                    cellD[i-1][j+1].f = nF;
                    cellD[i-1][j+1].c = nC;
                    cellD[i-1][j+1].h = nH;
                    cellD[i-1][j+1].parI = i;
                    cellD[i -1][j+1].parJ = j;
                }
            }
         }  
         if(validCheck(i+1,j-1)==true){
            if(destCheck(i+1,j-1,t)==true){
                cellD[i+1][j-1].parI=i;
                cellD[i+1][j-1].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
            }else if(close[i+1][j-1]==false&&blockCheck(grid,i+1,j-1)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i+1,j-1,t);
                nF=nH+nC;
                if(cellD[i+1][j-1].f==FLT_MAX||cellD[i+1][j-1].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i+1,j-1)));
                    cellD[i+1][j-1].f = nF;
                    cellD[i+1][j-1].c = nC;
                    cellD[i+1][j-1].h = nH;
                    cellD[i+1][j-1].parI = i;
                    cellD[i +1][j-1].parJ = j;
                }
            }
         }  
         if(validCheck(i+1,j-1)==true){
            if(destCheck(i+1,j-1,t)==true){
                cellD[i+1][j-1].parI=i;
                cellD[i+1][j-1].parJ=j;
                printf("Target found\n");
                pathDetails(cellD,t);
                destC=true;
                break;
            }else if(close[i-1][j-1]==false&&blockCheck(grid,i-1,j-1)==true){
                nC=cellD[i][j].c+1;
                nH=calcH(i-1,j-1,t);
                nF=nH+nC;
                if(cellD[i-1][j-1].f==FLT_MAX||cellD[i-1][j-1].f>nF){
                    open.insert(std::make_pair(nF,std::make_pair(i-1,j-1)));
                    cellD[i-1][j-1].f = nF;
                    cellD[i-1][j-1].c = nC;
                    cellD[i-1][j-1].h = nH;
                    cellD[i-1][j-1].parI = i;
                    cellD[i -1][j-1].parJ = j;
                }
            }
         }  
        
         //repeat but change i and j to the correct axis
    }
   // printf("Path Created\n");
 
}
void printGrid(int grid[][maxC]){
    printf("\n");
    for(int i=0;i<maxR;i++){
        for(int j=0;j<maxC;j++){
            if(grid[i][j]==0){
                printf("x");
            } else {
                printf("o");
            }
       
        }
        printf("\n");
    }
}
void printGridPath(int grid[][maxC],std::pair<int,int> s,std::pair<int,int>t,cell cellD[][maxC]){
    printf("\n");
    for(int i=0;i<maxR;i++){
        for(int j=0;j<maxC;j++){
            if(grid[i][j]==0){
                printf("x");
            }else if(i==t.first&&j==t.second){
                printf("T");
            } else if(i==s.first&&j==s.second){
                printf("S");
            }else if(cellD[i][j].parI!=-1&&cellD[i][j].parJ!=-1){
                printf("-");
            }else{
                printf("o");
            }
       
        }
        printf("\n");
    }
}

void gridToBinary(nav_msgs::OccupancyGrid &oG,int std::vector<std::vector<int>> &nA){
    nA.resize(oG.info.height,std::vector<int>(oG.info.width));
    for(int i=0;i<oG.info.height;i++){
        for(int j=0;j<oG.info.width;j++){
            if (oG.data[i*oG.info.width+j]>50){
                nA[i][j]=0;
            } else {
                nA[i][j]=1;
            }
        }
    }
}
int main(){
    
    cell cellD[maxR][maxC];
    cell cellD2[maxR][maxC];
    std::random_device ran;
    std::mt19937 gen(ran());
    std::uniform_int_distribution<> dis(0,1);
    int grid[maxR][maxC]={{1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
            { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
            { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }};
    int grid2[maxR][maxC];
    for(int i =0;i<maxR;i++){
        for(int j=0;j<maxC;j++){
            grid2[i][j]=dis(gen);
        }
    }       
            
            std::uniform_int_distribution<> disRow(0,maxR-1);
            std::uniform_int_distribution<>disCol(0,maxC-1);
            
            std::pair<int,int>src2=std::make_pair(disRow(gen),disCol(gen));
            std::pair<int,int> tar2=std::make_pair(disRow(gen),disCol(gen));
            if(blockCheck(grid2,src2.first,src2.second)==0){
              src2=std::make_pair(disRow(gen),disCol(gen));  
            } else if ((blockCheck(grid2,tar2.first,tar2.second)==0)){
              tar2=std::make_pair(disRow(gen),disCol(gen));
            }
            std::pair<int,int> src=std::make_pair(0,0);
            std::pair<int,int> tar=std::make_pair(7,8);
            
            printf("Target: %d %d",tar2.first,tar2.second);
            printf("Source: %d %d",src2.first,src2.second);
            aStar(grid,src,tar,cellD);
            aStar(grid2,src2,tar2,cellD2);
            printGrid(grid2);
            printGridPath(grid2,src2,tar2,cellD2);
            
}