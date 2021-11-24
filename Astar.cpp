#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <assert.h>
#include <memory.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#define FREEGLUT_STATIC
#define _LIB
#define FREEGLUT_LIB_PRAGMAS 1
#include <freeglut/include/GL/freeglut.h>

/*
* Santeri Karhu
implementation of A* on pixelmap. change heuristic on row 142 in setH() function of class Node to change heuristic :)
the program executes 10 times in a row, and then shows you the costs of each route
*/
namespace
{
	//forward declarations
	class Node;
	bool checkIfClosed(Node N);
	std::vector<Node> removeNode(std::vector<Node> vector, Node N);

	uint8_t* getPixel(int x, int y, uint8_t* data, int width)
	{
		return &data[3 * (y * width + x)];
	}
	void setPixel(int x, int y, uint8_t* data, int width, uint8_t r, uint8_t b, uint8_t g)
	{
		uint8_t* pixel = getPixel(x, y, data, width);
		pixel[0] = b;
		pixel[0] = g;
		pixel[0] = r;
	}

	//node class for better code readability in doPathFinding function
	class Node
	{
		friend Node operator<(Node& node1, Node& node2)
		{
			if (node1.getF() < node2.getF())
			{
				return node1;
			}
			else
			{
				return node2;
			}
		}
	public:

		Node() {}
		Node(int x, int y, uint8_t* data, int width)
		{
			this->parent = __nullptr;
			pixel = getPixel(x, y, data, width);
			location[0] = x;
			location[1] = y;
			this->setColor(pixel[0], pixel[1], pixel[2]);
			f = 0;
			g = 0;
			h = 0;
		}
		std::string getColor()
		{
			if (color[0] == 255)
				return "blue";
			else if (color[1] == 255)
				return "green";
			else if (color[2] == 255)
				return "red";
		}
		void setColor(uint8_t r, uint8_t g, uint8_t b)
		{
			color[0] = b;
			color[1] = g;	//color var
			color[2] = r;

			pixel[0] = b;
			pixel[1] = g;	//pixel color
			pixel[2] = r;
		}
		bool getIsWall()
		{
			if (this->getColor() == "green")
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		int getLocation(int axel)
		{
			return location[axel];
		}
		void setParent(Node* newParent)
		{
			this->parent = newParent;
		}
		Node* getParent()
		{
			return this->parent;
		}
		int getF()
		{
			return this->f;
		}
		void setF()
		{
			this->f = g + h;
			//modifying to f = g+h*k where k is 0,2,5,10,20
			//k at 0 makes the route pretty much follow walls
			//higher k makes this process longer and the route is longer as a result
			//higher k = larger lists of nodes

		}
		int getG()
		{
			if (this->parent == __nullptr)
				return 0;
			else
				return g;
		}
		void setG(int newG)
		{
			g = newG;
		}
		int getH()
		{
			return this->h;
		}
		void setH()
		{
			double distance = 0;
			double x = this->location[0];
			double y = this->location[1];

			double dist = pow((x - 126), 2) + pow((y - 1), 2);
			distance = sqrt(dist);
			this->h = distance; //uncomment to enable heuristic
			//this->h = 0;		  //comment to enable heuristic
		}

	private:
		int f;
		int g;
		int h;
		uint8_t* pixel;
		int color[3];
		int location[2];
		Node* parent;
	};

	//global variables
	bool first = true;
	bool donezo = false;
	bool done = false;
	std::vector<Node*> openNodes;
	std::vector<Node*> closedNodes;
	Node* N;
	Node* endNode;
	bool waitoneLoop = true;
	bool allDone;
	std::vector<int> totalcosts;
	//sub-programs

	//check if N is already in closedNodes
	bool checkIfClosed(Node* N)
	{
		bool isClosed = false;
		for (size_t i = 0; i < closedNodes.size(); i++)
		{
			if (N->getLocation(0) == closedNodes[i]->getLocation(0) && N->getLocation(1) == closedNodes[i]->getLocation(1))
			{
				isClosed = true;
				return true;
			}
		}
		if (!isClosed)
			return false;
	}
	int checkIfOpen(Node* N)
	{
		bool isOpen = false;
		for (size_t i = 0; i < openNodes.size(); i++)
		{
			if (N->getLocation(0) == openNodes[i]->getLocation(0) && N->getLocation(1) == openNodes[i]->getLocation(1))
			{
				isOpen = true;
				return i;
			}
		}
		if (!isOpen)
			return -1;
	}
	//remove node N from vector if found Node with same location 
	std::vector<Node*> removeNode(std::vector<Node*> vector, Node* N)
	{
		bool erased = false;
		for (size_t i = 0; i < vector.size(); i++)
		{
			if (vector[i]->getLocation(0) == N->getLocation(0) && vector[i]->getLocation(1) == N->getLocation(1))
			{
				vector.erase(vector.begin() + i);
				erased = true;
			}
		}
		if (!erased)
			printf("\t couldnt find node to remove\n");

		return vector;

	}
	//push back newNode if vector doesn't contain it
	std::vector<Node*> pushBackNode(std::vector<Node*> vector, Node* newNode)
	{
		bool found = false;

		for (size_t i = 0; i < vector.size(); i++)
		{
			//onko vektorissa jo sama node
			if (vector[i]->getLocation(0) == newNode->getLocation(0) && vector[i]->getLocation(1) == newNode->getLocation(1))
				found = true;
		}
		if (!found)
			vector.push_back(newNode);

		return vector;
	}
	//method for adding adjacent nodes to a vector
	std::vector<Node*> addJacent(std::vector<Node*> vector, Node* node, uint8_t* outputData, int width)
	{
		std::vector<Node*> children;
		std::vector<Node*> kids;

		kids.push_back(new Node(node->getLocation(0) + 1, node->getLocation(1), outputData, width)); //right
		kids.push_back(new Node(node->getLocation(0), node->getLocation(1) - 1, outputData, width)); //bottom
		kids.push_back(new Node(node->getLocation(0) - 1, node->getLocation(1), outputData, width)); //left
		kids.push_back(new Node(node->getLocation(0), node->getLocation(1) + 1, outputData, width)); //top

		kids.push_back(new Node(node->getLocation(0) + 1, node->getLocation(1) + 1, outputData, width));//topright
		kids.push_back(new Node(node->getLocation(0) + 1, node->getLocation(1) - 1, outputData, width));//bottomright
		kids.push_back(new Node(node->getLocation(0) - 1, node->getLocation(1) - 1, outputData, width));//bottomleft
		kids.push_back(new Node(node->getLocation(0) - 1, node->getLocation(1) + 1, outputData, width));//topleft
		
		//tarkista onko nodet seiniä tai onko opennodesissa jo näitä nodeja 
		for (size_t i = 0; i < kids.size(); i++)
		{
			if (!kids[i]->getIsWall() && !checkIfClosed(kids[i]))
				children = pushBackNode(children, kids[i]);
			else
			{
				kids.erase(kids.begin() + i);
			}
		}
		//asetetaan parent, f, g, h arvot
		for (int i = 0; i < children.size(); i++)
		{
			children[i]->setParent(node);
			children[i]->setG((node->getG() + 1));
			children[i]->setH();
			children[i]->setF();
		}

		int isThere = -1;
		//check if in openNodes, if true, check if g is smaller. if true, add to openNodes.
		for (size_t i = 0; i < children.size(); i++)
		{
			isThere = checkIfOpen(children[i]);
			if (isThere != -1)
			{
				if (children[i]->getG() < vector[isThere]->getG())
				{
					vector[isThere]->setParent(node);
					//vector.push_back(children[i]);
					children = removeNode(children, children[i]);
				}
			}
		}

		//add color to rest of kids and add them to openNodes
		for (size_t i = 0; i < children.size(); i++)
		{
			children[i]->setColor(0, 0, 255);
			vector = pushBackNode(vector, children[i]);
		}

		//clear lists
		for (size_t i = 0; i < kids.size(); i++)
		{
			kids.erase(kids.begin() + i);
		}
		for (size_t i = 0; i < children.size(); i++)
		{
			children.erase(children.begin() + i);
		}
		return vector;
	}
	//get lowest cost node in nodes
	Node* getLowestCostNode(std::vector<Node*> nodes)
	{
		std::sort(nodes.begin(), nodes.end(), [](Node* node1, Node* node2)
			{
				return node1->getF() < node2->getF();

			});
		Node* lowest = nodes.front();
		return lowest;
	}
	void paintRoute(Node* node, uint8_t* outputData, int width)
	{
		donezo = false;
		while (!donezo)
		{
			node->setColor(255, 0, 0);
			node = node->getParent();
			if (node->getParent() == __nullptr)
				donezo = true;
		}
		printf("route painting done\n");
	}

	void doPathFinding(const uint8_t* inputData, int width, int height, uint8_t* outputData, int startX, int startY, int endX, int endY)
	{
		if (first)
		{
			done = false;
			//tehdään tutkittava node N
			N = new Node(startX, startY, outputData, width);
			endNode = new Node(endX, endY, outputData, width);
			N->setColor(0, 0, 255);
			endNode->setColor(255, 0, 0);
			N->setG(0);
			N->setH();
			N->setF();
			openNodes.push_back(N);
		}

		if (!done && !first && !openNodes.empty())
		{
			//get lowest cost node and switch it to closedlist
			N = getLowestCostNode(openNodes);
			openNodes = removeNode(openNodes, N);
			closedNodes = pushBackNode(closedNodes, N);

			//add 8 adjacent squares if they are not walls or are not already in openNodes
			openNodes = addJacent(openNodes, N, outputData, width);

			//set colors for N and open and closednodes
			N->setColor(255, 0, 0);
			for (size_t i = 0; i < closedNodes.size(); i++)
			{
				closedNodes[i]->setColor(0, 0, 255);
			}
			for (size_t i = 0; i < openNodes.size(); i++)
			{
				openNodes[i]->setColor(255, 255, 0);
			}
		
			// tarkista onko valmista
			for (int i = 0; i < openNodes.size(); i++)
			{
				if (openNodes[i]->getLocation(0) == endX && openNodes[i]->getLocation(1) == endY)
				{
					N = openNodes[i];
					totalcosts.push_back(N->getF());
					done = true;
				}
			}
		}

		//actions to take when done. still todo : draw the optimal route to the map from closedNodes
		if (done)
		{
			if (waitoneLoop) 
			{
			paintRoute(N, outputData, width);
			}
			else
			{
				for (size_t i = 0; i < openNodes.size(); i++)
				{
				printf("OPENNODES %d\tNcost = %d\tNloc = x:%d, y:%d\n", i, openNodes[i]->getF(), openNodes[i]->getLocation(0), openNodes[i]->getLocation(1));
				}
				for (size_t i = 0; i < openNodes.size(); i++)
				{
				printf("CLOSEDNODES %d\tNcost = %d\tNloc = x:%d, y:%d\n", i, closedNodes[i]->getF(), closedNodes[i]->getLocation(0), closedNodes[i]->getLocation(1));
				}

				std::cout << "loop done\n";
				Sleep(INFINITE); //windows.h Sleep for ms
				//sleep(5) //unistd sleep if on linux
				for (size_t i = 0; i < closedNodes.size(); i++)
				{
					closedNodes[i]->setColor(255,255,255);
					delete closedNodes[i];
					closedNodes.erase(closedNodes.begin() + i);
				}
				for (size_t i = 0; i < openNodes.size(); i++)
				{
					openNodes[i]->setColor(255,255,255);
					delete openNodes[i];
					openNodes.erase(openNodes.begin() + i);
				}
				allDone = true;
			}
				
			waitoneLoop = false;
		}
		if (first)
			first = false;
	}
}

namespace
{
	// Quick and dirty function for reading bmp-files to opengl textures.
	GLuint loadBMPTexture(const char* fileName, int* w, int* h, uint8_t** data)
	{
		assert(w != 0);
		assert(h != 0);
		assert(data != 0);
		FILE* file;
		if ((file = fopen(fileName, "rb")) == NULL)
			return 0;
		fseek(file, 18, SEEK_SET);

		int width = 0;
		fread(&width, 2, 1, file);
		fseek(file, 2, SEEK_CUR);
		int height = 0;
		fread(&height, 2, 1, file);
		printf("Image \"%s\" (%dx%d)\n", fileName, width, height);

		*data = new uint8_t[3 * width * height];
		assert(data != 0);
		fseek(file, 30, SEEK_CUR);
		fread(*data, 3, width * height, file);
		fclose(file);

		GLuint  texId;
		glGenTextures(1, &texId);
		glBindTexture(GL_TEXTURE_2D, texId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, *data);
		glBindTexture(GL_TEXTURE_2D, 0);
		if (w) *w = width;
		if (h) *h = height;
		return texId;
	}

	// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
	// Global variables
	uint8_t* pixelStart;
	uint8_t* pixelEnd;
	// OpenGL texture ids for rendering.
	GLuint inputTexture = 0;
	GLuint outputTexture = 0;
	// Input and output data in pixels. outputData is updated to outputTexture each frame
	uint8_t* inputData = 0;
	uint8_t* outputData = 0;
	// width and height of the input and output datas
	int width = 0;
	int height = 0;
	// start and end position for path finding. These are found automatically from input file.
	int startX = -1;
	int startY = -1;
	int endX = -1;
	int endY = -1;

	// Initialization
	bool init()
	{
		glMatrixMode(GL_PROJECTION);	
		glOrtho(0, 512 + 4, 256 + 2, 0, -1, 1);

		// Load input file, choises: input1-5 and inputOriginal
		inputTexture = loadBMPTexture("inputOriginal.bmp", &width, &height, &inputData);
		if (0 == inputTexture)
		{
			printf("Error! Cannot open file: \"input.bmp\"\n");
			return false;
		}

		// Make outputTexture
		glGenTextures(1, &outputTexture);
		glBindTexture(GL_TEXTURE_2D, outputTexture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glBindTexture(GL_TEXTURE_2D, 0);

		// Copy inputData also to outputData
		outputData = new uint8_t[3 * width * height];
		memcpy(outputData, inputData, 3 * width * height);

		/*srand(time(0));
		int randX;
		int randY;
		//set random start and end
		randX = rand() % 126 + 1;
		randY = rand() % 126 + 1;
		pixelStart = &inputData[3 * (randY * width + randX)];

		pixelStart[0] = 255;
		pixelStart[1] = 0;
		pixelStart[2] = 0;

		randX = rand() % 126 + 1;
		randY = rand() % 126 + 1;

		pixelEnd = &inputData[3 * (randY * width + randX)];

		pixelEnd[0] = 0;
		pixelEnd[1] = 0;
		pixelEnd[2] = 255;*/

		// find start and end
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				uint8_t* pix = &inputData[3 * (y * width + x)]; // get pixel
				uint8_t r = pix[0];
				uint8_t g = pix[1];
				uint8_t b = pix[2];
				if (255 == r && g == 0 && b == 0) // Red?
				{
					// Start
					startX = x;
					startY = y;
					printf("Start position: <%d,%d>\n", x, y);
				}
				if (255 == b && r == 0 && g == 0) // Blue?
				{
					// End
					endX = x;
					endY = y;
					printf("End position: <%d,%d>\n", x, y);
				}
			}
		}

		if (startX < 0 || startY < 0)
		{
			printf("Error! Start position not found\n");
			return false;
		}

		if (endX < 0 || endY < 0)
		{
			printf("Error! End position not found\n");
			return false;
		}

		return true;
	}
	bool reinit()
	{
		N = 0;
		endNode = 0;
		openNodes.clear();
		closedNodes.clear();
	
		pixelStart[0] = 0;
		pixelStart[1] = 0;
		pixelStart[2] = 0;

		pixelEnd[0] = 0;
		pixelEnd[1] = 0;
		pixelEnd[2] = 0;

		// Load input filec
		inputTexture = loadBMPTexture("input4.bmp", &width, &height, &inputData);
		if (0 == inputTexture)
		{
			printf("Error! Cannot open file: \"input.bmp\"\n");
			return false;
		}
	
		// Make outputTexture
		glGenTextures(1, &outputTexture);
		glBindTexture(GL_TEXTURE_2D, outputTexture);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glBindTexture(GL_TEXTURE_2D, 0);

		// Copy inputData also to outputData
		outputData = new uint8_t[3 * width * height];
		memcpy(outputData, inputData, 3 * width * height);

		int randX;
		int randY;
		
		//set random start and end
		randX = rand() % 126 + 1;
		randY = rand() % 126 + 1;
		pixelStart = &inputData[3 * (randY * width + randX)];

		pixelStart[0] = 255;
		pixelStart[1] = 0;
		pixelStart[2] = 0;

		randX = rand() % 126 + 1;
		randY = rand() % 126 + 1;

		pixelEnd = &inputData[3 * (randY * width + randX)];

		pixelEnd[0] = 0;
		pixelEnd[1] = 0;
		pixelEnd[2] = 255;

		// find start and end
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				uint8_t* pix = &inputData[3 * (y * width + x)]; // get pixel
				uint8_t r = pix[0];
				uint8_t g = pix[1];
				uint8_t b = pix[2];
				if (255 == r && g == 0 && b == 0) // Red?
				{
					// Start
					startX = x;
					startY = y;
					printf("Start position: <%d,%d>\n", x, y);
				}
				if (255 == b && r == 0 && g == 0) // Blue?
				{
					// End
					endX = x;
					endY = y;
					printf("End position: <%d,%d>\n", x, y);
				}
			}
		}

		if (startX < 0 || startY < 0)
		{
			printf("Error! Start position not found\n");
			return false;
		}

		if (endX < 0 || endY < 0)
		{
			printf("Error! End position not found\n");
			return false;
		}

		return true;
	}
	// Draw/Render
	void draw()
	{
		/*if (allDone)
		{
			done = false;
			donezo = false;
			first = true;
			allDone = false;
			waitoneLoop = true;

			if (!reinit())
				printf("reinit failed!\n");
		}*/
		doPathFinding(inputData, width, height, outputData, startX , startY, endX, endY);	

	 /*if (totalcosts.size() == 10)
		{*/
			for (int i = 0; i < totalcosts.size(); i++)
			{
				printf("cost of lap %d: %d\n", i, totalcosts[i]);
			}
			/*exit(1);
		}*/

		// Copy outputData to outputTexture
		glBindTexture(GL_TEXTURE_2D, outputTexture);
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, outputData);
		glBindTexture(GL_TEXTURE_2D, 0);

		glClear(GL_COLOR_BUFFER_BIT);

		// Draw output texture to right half of the screen
		glPushMatrix();
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, outputTexture);
		glBegin(GL_QUADS);
		glTexCoord2d(0, 1); glVertex2d(2 + 256, 1);
		glTexCoord2d(0, 0); glVertex2d(2 + 256, 1 + 256);
		glTexCoord2d(1, 0); glVertex2d(2 + 512, 1 + 256);
		glTexCoord2d(1, 1); glVertex2d(2 + 512, 1);
		glEnd();
		glDisable(GL_TEXTURE_2D);
		glPopMatrix();
		glutSwapBuffers();
		glutPostRedisplay();
	}
} // end - anonymous namespace

// Main
int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(2 * (512 + 4), 2 * (256 + 2));
	glutCreateWindow("Pathfinding Demo");
	glutDisplayFunc(draw);
	if (!init()) return -1;
	glutMainLoop();
	delete inputData;
	delete outputData;
	return 0;
}