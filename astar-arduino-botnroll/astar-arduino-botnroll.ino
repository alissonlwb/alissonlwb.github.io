/*************************************************
 * Public Constants
 *************************************************/

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

#include <iomanip>
#include <math.h>
#include <ctime>
#include <string>
#include <QueueArray.h>

#include <BnrOneA.h>   // Bot'n Roll ONE A library
#include <SPI.h>       // SPI communication library required by BnrOne.cpp
BnrOneA one;           // declaration of object variable to control the Bot'n Roll ONE A
//constants definition
#define SSPIN  2       // Slave Select (SS) pin for SPI communication

using namespace std;

const int IDIM = 6; // horizontal size of the squares
const int JDIM = 6;
const int NDIR = 4; // number of possible directions to go at any position

// if NDIR = 4
const int iDir[NDIR] = {1, 0, -1, 0};
const int jDir[NDIR] = {0, 1, 0, -1};

// if NDIR = 8
//const int iDir[NDIR] = {1, 1, 0, -1, -1, -1, 0, 1};
//const int jDir[NDIR] = {0, 1, 1, 1, 0, -1, -1, -1};

int squares[IDIM][JDIM];

// list of closed (check-out) nodes
int closedNodes[IDIM][JDIM];

// list of open (not-yet-checked-out) nodes
int openNodes[IDIM][JDIM];

// map of directions (0: East, 1: North, 2: West, 3: South) 
int dirMap[IDIM][JDIM];

struct Location
{
    int row, col;

    Location()
    {
        row = col = 0;
    };

    Location(int r, int c)
    {
        row = r;
        col = c;
    };
};

class Node
{
    // current position
    int rPos;
    int cPos;

    // total distance already travelled to reach the node
    int GValue;

    // FValue = GValue + remaining distance estimate
    int FValue;  // smaller FValue gets priority

    public:
        Node(const Location &loc, int g, int f) 
            {rPos = loc.row; cPos = loc.col; GValue = g; FValue = f;}
    
		Location getLocation() const {return Location(rPos,cPos);}
        int getGValue() const {return GValue;}
        int getFValue() const {return FValue;}

        void calculateFValue(const Location& locDest)
        {
             FValue = GValue + getHValue(locDest) * 10; 
        }

        
        void updateGValue(const int & i) // i: direction
        {
            GValue += (NDIR == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
        }
        
        // Estimation function for the remaining distance to the goal.
        const int & getHValue(const Location& locDest) const
        {
            static int rd, cd, d;
            rd = locDest.row - rPos;
            cd = locDest.col - cPos;         

            // Euclidian Distance
            // d = static_cast<int>(sqrt((double)(rd*rd+cd*cd)));

            // Manhattan distance
            d = abs(rd) + abs(cd);
            
            // Chebyshev distance
            //d = max(abs(rd), abs(cd));

            return(d);
        }

	// Determine FValue (in the priority queue)
	friend bool operator<(const Node & a, const Node & b)
	{
	        //MUDEI ISSO DAQUIIIII!!!!!!!!!!!!!!!!!
                // < signal makes the right path
		return a.getFValue() < b.getFValue();
	}
};

String pathFind(struct Location &locStart, struct Location &locFinish){

   // list of open (not-yet-checked-out) nodes
   QueueArray <Node> q[2];
   //static priority_queue<Node> q[2]; 

    // q index
    static int qi; 

    static Node* pNode1;
    static Node* pNode2;
    static int i, j, row, col, iNext, jNext;
    static char c;
    qi = 0;

    // reset the Node lists (0 = ".")
    for(j = 0; j < JDIM; j++) {
        for(i = 0; i < IDIM; i++) {
            closedNodes[i][j] = 0;
            openNodes[i][j] = 0;
        }
    }

    // create the start node and push into list of open nodes
    pNode1 = new Node(locStart, 0, 0); 
    pNode1->calculateFValue(locFinish);
    q[qi].push(*pNode1);
  
    // A* search
    while(!q[qi].isEmpty()) {
        // get the current node w/ the lowest FValue
        // from the list of open nodes
        pNode1 = new Node( q[qi].front().getLocation(), 
                     q[qi].front().getGValue(), q[qi].front().getFValue());

        row = (pNode1->getLocation()).row; 
	    col = pNode1->getLocation().col;
	    
            //cout << "row, col=" << row << "," << col << endl;
            //Serial.print("row, col=");
            //Serial.print(row);
            //Serial.print(",");
            //Serial.println(col);
            
          
	    // remove the node from the open list
        q[qi].pop(); 
        openNodes[row][col] = 0;

        // mark it on the closed nodes list
        closedNodes[row][col] = 1;

        // stop searching when the goal state is reached
        if(row == locFinish.row && col == locFinish.col) {
		// drawing direction map
		//cout << endl;
                Serial.println("");

		for(j = JDIM - 1; j >= 0; j--) {
			for(i = 0; i < IDIM; i++) {
				//cout << dirMap[i][j];
                                //Serial.print(dirMap[i][j]);
			}
			//cout << endl;
                        Serial.println("");


		}
		//cout << endl;
                Serial.println("");


	        // generate the path from finish to start from dirMap
            String path = "";
            while(!(row == locStart.row && col == locStart.col)) {
                j = dirMap[row][col];
                c = '0' + (j + NDIR/2) % NDIR;
                path = c + path;
                row += iDir[j];
                col += jDir[j];
            }

            // garbage collection
            delete pNode1;

            // empty the leftover nodes
            while(!q[qi].isEmpty()) q[qi].pop();           
            return path;
        }

        // generate moves in all possible directions
        for(i = 0; i < NDIR; i++) {
            iNext = row + iDir[i]; 
	        jNext = col + jDir[i];

	        // if not wall (obstacle) nor in the closed list
            if(!(iNext < 0 || iNext > IDIM - 1 || jNext < 0 || jNext > JDIM - 1 || 
			squares[iNext][jNext] == 1 || closedNodes[iNext][jNext] == 1)) {
               
		        // generate a child node
                pNode2 = new Node( Location(iNext, jNext), pNode1->getGValue(), pNode1->getFValue());
                pNode2->updateGValue(i);
                pNode2->calculateFValue(locFinish);

                // if it is not in the open list then add into that
                if(openNodes[iNext][jNext] == 0) {
                    openNodes[iNext][jNext] = pNode2->getFValue();
                    q[qi].push(*pNode2);
                    // mark its parent node direction
                    dirMap[iNext][jNext] = (i + NDIR/2) % NDIR;
                }

		        // already in the open list
                else if(openNodes[iNext][jNext] > pNode2->getFValue()) {
                    // update the FValue info
                    openNodes[iNext][jNext] = pNode2->getFValue();

                    // update the parent direction info,  mark its parent node direction
                    dirMap[iNext][jNext] = (i + NDIR/2) % NDIR;

                    // replace the node by emptying one q to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(q[qi].front().getLocation().row == iNext && 
                        q[qi].front().getLocation().col == jNext)) {                
                        q[1 - qi].push(q[qi].front());
                        q[qi].pop();       
                    }

				// remove the wanted node
                    q[qi].pop(); 
                    
                    // empty the larger size q to the smaller one
                    if(q[qi].count() > q[1 - qi].count()) qi = 1 - qi;
                    while(!q[qi].isEmpty()) {                
                        q[1 - qi].push(q[qi].front());
                        q[qi].pop();       
                    }
                    qi = 1 - qi;

				// add the better node instead
                    q[qi].push(*pNode2); 
                }
                else delete pNode2; 
            }
        }
        delete pNode1; 
    }
    // no path found
  return "";
}

//BUZZER
int melody[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4};
int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };

void buzz(){
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++)
  {
    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];
    tone(9, melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(9);
  }
}

void setup() {                
  Serial.begin(57600);
  Serial.println("LOl");
  one.spiConnect(SSPIN);   // starts the SPI communication module
  one.stop();              // stops motors
  delay(1000);             // waits 1 second
}

void loop() {
   // create empty squares
    for(int j = 0; j < JDIM; j++) {
        for(int i = 0; i < IDIM; i++) squares[i][j] = 0;
    }

    // make wall
    //squares[4][2] = 1;
    //squares[4][3] = 1;
    //squares[4][4] = 1;
    squares[0][5] = 1;
    squares[1][0] = 1;
    squares[1][1] = 1;
    squares[1][5] = 1;
    squares[2][3] = 1;
    squares[3][1] = 1;
    squares[3][2] = 1;
    squares[3][5] = 1;
    squares[4][1] = 1;
    squares[4][4] = 1;
    squares[5][1] = 1;

    // starting and ending positions
    int iStart = 0,jStart = 0;
    int iEnd = 5,jEnd = 5;    

    Serial.print("Grid Size (IDIM,JDIM):");
    Serial.print(IDIM);
    Serial.print(",");
    Serial.println(JDIM);
    //cout << "Grid Size (IDIM,JDIM): "<< IDIM<< "," << JDIM << endl;
    
    Serial.print("Start");
    Serial.print(iStart);
    Serial.print(",");
    Serial.println(jStart);
    //cout << "Start: " << iStart<<","<< jStart << endl;
    
    Serial.print("Finish");
    Serial.print(iEnd);
    Serial.print(",");
    Serial.println(jEnd);  
    //cout << "Finish: " << iEnd<<","<< jEnd << endl;
 
    //clock_t start = clock();

    // get the path
    Location starttt = Location(iStart, jStart);
    Location enddd = Location(iEnd, jEnd);
    String path = pathFind(starttt, enddd);
    Serial.println(path);

    //clock_t end = clock();
    //double time = double(end - start);
    //cout << "Time (ms): "<< time << endl;
    //cout << "path: " << path << endl;

    // follow the path on the squares and display it 
    if(path.length() > 0) {
        char c;
	int m,n;
        int i = iStart;
        int j = jStart;
        squares[i][j] = 2;
        for(m = 0; m < path.length(); m++) {
            //c = path.at(m);
            c = path.charAt(m);
            n = atoi(&c); 
            i = i + iDir[n];
            j = j + jDir[n];
            squares[i][j] = 3;
        }
        squares[i][j] = 4;
        
        
        // display the squares with the path
        for(j = JDIM - 1; j >= 0; j--) {
            for(i = 0; i < IDIM; i++) {
                if(squares[i][j] == 0)
                    //cout << ".";
                    Serial.print(".");  
                else if(squares[i][j] == 1)
                    //cout << "O"; //obstacle
                    Serial.print("0");  
                else if(squares[i][j] == 2)
                    //cout << "I"; //Initial
                    Serial.print("I");  

                else if(squares[i][j] == 3)
                   // cout << "P"; //path
                    Serial.print("P");  
                else if(squares[i][j] == 4)
                    //cout << "F"; //final
                    Serial.print("F");  

	    }
                Serial.println("");  
                //cout << endl;
                
        }
        /*
        //r-turn
        delay(500);
        one.lcd2("  Rotate Right ");
        one.move(25,-25);         // Rotate Right
        delay(500);
        one.move(25,25);
        
        //l-turn
        delay(500);
        one.lcd2("  Rotate Left ");
        one.move(-25,25);         // Rotate Left
        delay(500);
        one.move(25,25);
        */
        
        // run the path (0: East, 1: North, 2: West, 3: South)
        // CHANGE FOR LOOP FOR STRING LENGTH
        for(int i = 0; i < 12 ; i++) {
         
         Serial.print("path[i] = ");
         Serial.println(path[i]);
            
            if(path[i] == '0'){
              
                if(path[i-1] == '0'){
                  one.move(25,25);          // Forward
                  delay(600);              // wait 1 second
                }
                
                //r-turn
                else if(path[i-1] == '1'){
                  delay(600);
                  one.move(70,30);         // Rotate Right
                  delay(600);
                  one.stop();               // Stop
                }
                
                //l-turn
                else if(path[i-1] == '3'){
                  delay(600);
                  one.move(30,70);         // Rotate Left
                  delay(600);
                  one.stop();               // Stop
                } 
                
            }
            
            else if(path[i] == '1'){
                Serial.println("Forward");
                one.lcd2("  Forward ");
                one.move(25,25);          // Forward
                delay(600);              // wait 1 second
                one.lcd2("     Stop   ");
                one.stop();               // Stop
                delay(600);
            }
            
            else if(path[i] == '2'){
                
                if(path[i-1] == '2'){
                  one.move(25,25);          // Forward
                  delay(600);              // wait 1 second
                }
                
                //l-turn
                else if(path[i-1] == '1'){
                  delay(600);
                  one.move(30,70);         // Rotate Left
                  delay(600);
                  one.stop();               // Stop
                }
                
                //r-turn
                else if(path[i-1] == '3'){
                  delay(600);
                  one.move(70,30);         // Rotate Right
                  delay(600);
                  one.stop();               // Stop
                } 
                
            }
            else if(path[i] == '3'){
              
              //r-turn
              if(path[i-1] == '0'){
                delay(600);
                one.move(70,30);         // Rotate Right
                delay(600);
                one.stop();               // Stop
              }
              
              //l-turn
              else if(path[i-1] == '2'){
                delay(600);
                one.move(30,70);         // Rotate Left
                delay(600);
                one.stop();               // Stop
              }
              
              else if(path[i-1] == '3'){
                delay(600);
                one.move(25,25);         // Forward
                delay(600);
                one.stop();               // Stop
              }

        }
    
    }
    
    /*
    
    //CURVA CORRETA
    else if(pbutton == 3){
      delay(500);
      one.move(30,70);         // Rotate Left
      delay(600);
      one.stop();               // Stop
    }
    
    */
    
    //buzz();
    Serial.println("saiu");     
    //return(0);
    // wait for a second  
  }
}
