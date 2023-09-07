/***********************************************************************  
*                                INCLUDES                              *
************************************************************************/
#include <bits/stdc++.h>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

/***********************************************************************
*                                REDEFINES                             *
************************************************************************/
typedef pair<int, int> IntPair;
typedef pair<double, pair<int, int> > IntPairWeight;

/***********************************************************************
*                              NODE STRUCTURE                          *
************************************************************************/

// Relevant information about each location inside the map
struct Node 
{
    // row and column of the less cost parent
    int parent_col = -1, parent_row = -1;
    // TotalWeight = CurrentWeight + PreviousWeight + Heuristic
    float TotalWeight = FLT_MAX, PreviousWeight = FLT_MAX, Heuristic = FLT_MAX;
    // Original position inside vector <int> Map
    int OriginalIndex;
    bool IsObstacle;
};

/***********************************************************************
*                                FUNCTIONS                              *
************************************************************************/
// Check if a location is inside Map Dimensions
bool IsInsideMap (int row, int col, int Height, int Width)
{
    return (row >= 0) && (row < Height) && (col >= 0) && (col < Width);
}

// Initialiasing the closed list with false values
bool** InitializeClosedList(IntPair MapDimensions)
{
    int col, row, oIndex = 0, iniTimes = 0;
    // array of columns
    bool** closedList = new bool*[MapDimensions.first]; 

    for (row = 0; row < MapDimensions.second; row++)
    {        
        for (col = 0; col < MapDimensions.first; col++) 
        {
            //Initialising each colum, only once
            if(iniTimes<MapDimensions.first)
            {
                closedList[col] = new bool[MapDimensions.second];
                iniTimes++;
            }

            closedList[col][row] = false;

        }
    }
    return closedList;
}

// Initialize map details
Node** InitializeMapDetails(IntPair Start, IntPair MapDimensions,const vector <int>& Map)
{
    int col, row, oIndex = 0, iniTimes = 0;
    // array of columns
    Node** mapDetails = new Node*[MapDimensions.first]; 

    for (row = 0; row < MapDimensions.second; row++)
    {        
        for (col = 0; col < MapDimensions.first; col++) 
        {
            //Initialising each colum, only once
            if(iniTimes<MapDimensions.first)
            {
                mapDetails[col] = new Node[MapDimensions.second];
                iniTimes++;
            }

            mapDetails[col][row].OriginalIndex = oIndex;
            mapDetails[col][row].IsObstacle = !Map[oIndex];
            // For the rest of properties we use the default values in the struct declaration

            oIndex++;
        }
    }

    // Initialising Start node details
    col = Start.first, row = Start.second;
    mapDetails[col][row].TotalWeight = 0.0;
    mapDetails[col][row].PreviousWeight = 0.0;
    mapDetails[col][row].Heuristic = 0.0;
    mapDetails[col][row].parent_col = col;
    mapDetails[col][row].parent_row = row;

    return mapDetails;
}

// Tracing the path from Target point to Start point ( tracing backwards)
void SetOutMap (vector<int>& OutPath, Node** MapDetails, IntPair Target)
{
    int col = Target.first;
    int row = Target.second;
    
    // The Start position is the only node thas its parent has the same i,j components
    while (!(MapDetails[col][row].parent_col == col && MapDetails[col][row].parent_row == row)) 
    {
        OutPath.push_back(MapDetails[col][row].OriginalIndex);
        int temp_row = MapDetails[col][row].parent_row;
        int temp_col = MapDetails[col][row].parent_col;
        row = temp_row;
        col = temp_col;
    }

    int i = 0;
    // 
    while (i < OutPath.size() / 2) 
    {
        iter_swap(OutPath.begin()+i, OutPath.end()-(i+1));
        i++;
    }

}

// Calculates Manhattan Distance heuristic
float GetManhattanDistance(int Row, int Col, IntPair Target)
{
    return abs(Col - Target.first) + abs(Row - Target.second);
}

// Executes each A* iteration. Returns true if target point is reached.
bool ExecutesAStarIteration(int SuccesorCol, int SuccesorRow, int ParentCol, int ParentRow,
                            IntPair Target, Node** MapDetails, set<IntPairWeight>* openList,
                            bool** closedList, IntPair MapDimensions)
{
    // To save the new cost of the 4 successors
    float newPrevWeight, newHeuristic, newTotalWeight;

    // Ignoring invalid positions
    if (IsInsideMap(SuccesorRow, SuccesorCol, MapDimensions.second, MapDimensions.first))
    {

        Node* successorNode = &MapDetails[SuccesorCol][SuccesorRow];

        // If this succesor is the target point, the algorithm ends
        if (SuccesorCol == Target.first && SuccesorRow == Target.second) 
        {
            // Set the Parent of the destination cell
            successorNode->parent_col = ParentCol;
            successorNode->parent_row = ParentRow;
            return true;
        }
        else
        {            
            // If the successor isn't inside the closed list and isn't an obstacle, we continue
            if (!closedList[SuccesorCol][SuccesorRow] && !successorNode->IsObstacle) 
            {              
                newPrevWeight = MapDetails[ParentCol][ParentRow].PreviousWeight + 1.0f; //Default weight of each node;
                newHeuristic = GetManhattanDistance(SuccesorRow, SuccesorCol, Target);
                newTotalWeight = newPrevWeight + newHeuristic;

                // If the succesor doesn't have a TotalWeight it means it has never been inside the open list, so we add it.
                // OR
                // If the succesor TotalWeight is bigger than the new one, we add it to the list again
                if (successorNode->TotalWeight == FLT_MAX || successorNode->TotalWeight > newTotalWeight) 
                {
                    openList->insert(make_pair(newTotalWeight, make_pair(SuccesorCol, SuccesorRow)));

                    // Update the details of this cell
                    successorNode->TotalWeight = newTotalWeight;
                    successorNode->PreviousWeight = newPrevWeight;
                    successorNode->Heuristic = newHeuristic;
                    successorNode->parent_col = ParentCol;
                    successorNode->parent_row = ParentRow;
                }
            }
        }    
    }

    return false;
}

/***********************************************************************
*                              FINDPATH                                *
*               This function implements the A* algorithm              *
************************************************************************/
bool FindPath(pair<int, int> Start,
              pair<int, int> Target,
              const vector<int>& Map,
              pair<int, int> MapDimensions,
              vector<int>& OutPath)
{  
    // This scenario is not included in the input restrictions
    if(Start == Target)
    {
        return true;
    }
    
    // Save a boolean per map node. True means that node is inside the closed list. 
    bool** closedList = InitializeClosedList(MapDimensions);

    // This 2D array contains the structure where the algorithm saves heuristic calculations and the less cost parent node 
    Node** mapDetails = InitializeMapDetails(Start, MapDimensions, Map);

    // Aux index 
    int col = Start.first;
    int row = Start.second;

    // In this set we save the possible candidates for the next iteration, sorted by ascending TotalWeight 
    // In this case, TotalWeight is represented by the first element in the set (the float number)
    // NOTE: A set structure naturally orders its elements after every insertion.
    set<IntPairWeight>* openList = new set<IntPairWeight>; 
    
    // Put the Start node as the first element 
    openList->insert(make_pair(0.0f, make_pair(col, row)));


    // The A* algorithm starts 
    while (!openList->empty()) 
    {
        // Each iteration, we get the best candidate in the open list, it means, the one with the lowest weight (f)
        IntPairWeight bestCandidate = *openList->begin();
 
        // Remove this candidate from the open list
        openList->erase(openList->begin());
 
        // Add the picked candidate to the closed list
        col = bestCandidate.second.first;
        row = bestCandidate.second.second;
        closedList[col][row] = true;

        // Trying to get the four successors of this candidate
        // North:                                                                 
        if(ExecutesAStarIteration(col,row+1,col,row,Target,mapDetails,openList,closedList,MapDimensions) 
        // East:
        || ExecutesAStarIteration(col+1,row,col,row,Target,mapDetails,openList,closedList,MapDimensions) 
        // West:
        || ExecutesAStarIteration(col-1,row,col,row,Target,mapDetails,openList,closedList,MapDimensions) 
        // South:
        || ExecutesAStarIteration(col,row-1,col,row,Target,mapDetails,openList,closedList,MapDimensions))
        {
            // The target point has been reached. Setting the required solution.
            SetOutMap(OutPath,mapDetails,Target);

            return true;
        }

    }

    // No more candidates to put in openlist. The A* algorithm ends without finding a solution.
    return false;

}


bool IsGraphicModeActive = 1;

template <typename T>
string NumberToString ( T Number )
{
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

enum AnsiColourCodes  { Red = 31, Green = 32, Yellow = 33, Blue = 34};

string PrintColouredString(string str, AnsiColourCodes ColourCode)
{
   return "\033["+ NumberToString(ColourCode)+ "m"+str+"\033[0m";
}

// Print two dimensional int array, colouring in red the algorithm solution if any
void PrintTwoDimIntArray(int** IntArray, int Height, int Width, const vector<int>* OutMap = NULL, IntPair* Start = NULL, IntPair* Target = NULL)
{
   if(!IsGraphicModeActive) return;

   int OutIndex = 0;
   for(int row = 0; row < Height; row++)
   {
        for(int col = 0; col < Width; col++)
        {
            // First position in each row
            if(col == 0)
                cout << "| ";

            //Printing Start position in blue
            if(Start && Start->first == col && Start->second == row)
            {
                cout << PrintColouredString(NumberToString(IntArray[col][row]),Blue) << " ";
            }
            else
            //Printing End position in blue
            if(Target && Target->first == col && Target->second == row)
            {
                cout << PrintColouredString(NumberToString(IntArray[col][row]),Green) << " ";
            }
            else
            // If this position is on the final solution: print in red
            if(OutMap && find(OutMap->begin(), OutMap->end(), OutIndex) != OutMap->end())//(&OutMap[OutIndex] != NULL)
            {
                cout << PrintColouredString(NumberToString(IntArray[col][row]),Red) << " ";
            }
            else
            {
                cout << (IntArray[col][row]) << " ";
            }
            OutIndex++;

            // Last position in each row
            if (col == Width-1)
            {
                cout << "|" << endl;
            }
                
        }
   }
   cout << endl;
}
int parseLine(char* line){
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}
int getValue(){ //Note: this value is in KB!
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

// Transforms int vector into a two dimensional int array, starting at (0,0)
int ** IntVectorTo2DimArray(vector<int> IntVector, int Height, int Width)
{
    // aux
    int row = 0, col = 0;
    int vIndex = 0;
    int iniTimes = 0;

    //declaring columns pointers
    int ** result = new int*[Width];

    for(int row = 0; row < Height; row++)
    {
        for(int col = 0; col < Width; col++)
        {
            //Initialising each colum, only once
            if(iniTimes<Width)
            {
                result[col] = new int[Height];
                iniTimes++;
            }
                
            result[col][row] = IntVector[vIndex];
            vIndex++;
        }
    }
        
    return result;
}


/*********************************************************************** 
*                                MAIN                                  *
************************************************************************/
int main() {

    // FindPath Inputs
    IntPair Start;
    IntPair Target;
    IntPair MapDimensions;
    vector<int> Map;
    vector<int> OutPath;

    // Auxiliary vars.
    string strmap;
    int** TwoDimMap;

   ifstream indata; // indata is like cin
   int num,num2,num3,num4,num5,num6; // variable for input value
   string map,file;

   cin >> file;
   indata.open(file.c_str()); // opens the file
   if(!indata) 
   { // file couldn't be opened
      cerr << "Error: file could not be opened" << endl;
      return 0;;
   }

        indata >> num >> num2 >> strmap >> num3 >> num4 >> num5 >> num6;

        MapDimensions.first = num;
        MapDimensions.second = num2;   
        Start.first = num3;
        Start.second = num4;
        Target.first = num5;
        Target.second = num6;
    
        // Transforming read string to vector<int>.
        for(int i=0;i<strmap.size();i++)
        {   
            //Non numeric chars will be ignored. 
            if(isdigit(strmap[i]))  
            {
                // Any num different than 0 or 1 will be casted to 0 or 1.
                Map.push_back((bool)(strmap[i]-'0')); 
            }
                
        }

    
    // Printing the input map
    if(IsGraphicModeActive)
    {
        TwoDimMap = IntVectorTo2DimArray(Map, MapDimensions.second, MapDimensions.first);
        cout << "\n--- \e[1mTHIS IS YOUR MAP\e[0m ---\n";
        PrintTwoDimIntArray(TwoDimMap, MapDimensions.second, MapDimensions.first);
    }
    cout << "Calling FindPath...";
    //Calling the A* algorithm implementation
    if(FindPath(Start, Target, Map, MapDimensions, OutPath))
    {
        cout << "SUCCESS: Path finded! \n";
        cout <<"OutPath: {";
        for(int i = 0; i< OutPath.size(); i++)
        {    
            if(i<OutPath.size()-1)        
                cout <<OutPath[i] << ", ";
            else
                cout <<OutPath[i];
        }
        cout << "}\n";
cout << "\nRAM: " << getValue()/1000.0f;
        // Printing the solution map
        if(IsGraphicModeActive)
        {
            cout << "\n--- \e[1mSHORTEST PATH\e[0m ---\n";
            PrintTwoDimIntArray(TwoDimMap, MapDimensions.second, MapDimensions.first,&OutPath,&Start,&Target);
        }
    }
    else
    {
         cout << "FAILURE: There is no path \n";
    }

    return 0;
}