----------------------------------------------------------------------------------------------------------------------------------------------------------

This C++ code implements a A* algorithm in 2D maps. Diagonal movement is ignored.

----------------------------------------------------------------------------------------------------------------------------------------------------------

2D maps are matrices of size [M,N] filled with binary numbers. 

    -----> 0 represents an obstacle. 
    -----> 1 represents a valid position.

2D Map Entry Format:

Columns and rows     ->   16 10
2D Map              ->   1101101011100110011011011011000000100011000110111111111000110011001010101001110001101010010010111000011111101000101110100111111011100010010010110000001111001101
Starting position   ->   0 0
Final position      ->   15 9

----------------------------------------------------------------------------------------------------------------------------------------------------------
In this directory, you can find a file called InputExample. You can use it to test the code and then make your own entry files.


You can try really big 2D maps with millions of rows or columns. The file InputExampleLong is an example of a big 2D Map.

----------------------------------------------------------------------------------------------------------------------------------------------------------

Command to compile in a Linux console:
g++ Pathfinder.cpp -o pfexec -std=c++11


Then we call the executable (it is included in the directory file just in case)
./pfexec

----------------------------------------------------------------------------------------------------------------------------------------------------------
The program would wait until you write the name of the file that contains the data entry. For example, in the first execution, you can write InputExample. 
----------------------------------------------------------------------------------------------------------------------------------------------------------

There is a global bool called IsGraphicModeActive. You can disable it if you don't want to see any graphic representation. For big maps it's really slow.
