
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "GLFW\glfw3.h"
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "GLFW/glfw3dll.lib")

#include <glm\glm.hpp>

#include <Windows.h>

#include "fluidQ.h"

using namespace std;
using namespace glm;

const float imageWidth = 600,
		imageHeight = 600;

float PI = 3.14159;

const int mapW = 128,
	mapH = 128;

const float dt = 1.f / 60.f;
const float rho = 0.1;

float r[mapW][mapH];
float p[mapW][mapH];

GLFWwindow* window;

enum cellType
{
	WATER, AIR, SOLID
};
cellType type[mapW][mapH];

struct particle
{	
	vec2 pos;
	int type;

	particle(vec2 p, int t = 0)
	{
		pos = p;
		type = t;
	}
};

vector<particle> parts;

fluidQ* ink;
fluidQ* u;
fluidQ* v;

float nrand()
{
	return (float)rand() / RAND_MAX;
}

void setupParticles()
{
	for (int x = 0; x < mapW + 1; ++x)
	{
		for (int y = 0; y < mapH + 1; ++y)
		{
			u->set(x, y, 0);
			v->set(x, y, 0);
		}
	}
}

void enforceBoundary()
{
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			if (type[x][y] != SOLID)
				continue;

			u->at(x, y) = 0;
			u->at(x + 1, y) = 0;

			v->at(x, y) = 0;
			v->at(x, y + 1) = 0;
		}
	}
	
	for (int x = 0; x < mapW + 1; ++x)
	{
		v->set(x, 0, 0);
		v->set(x, v->h - 1, 0);
	}
	for (int y = 0; y < mapH + 1; ++y)
	{
		u->set(0, y, 0);
		u->set(u->w - 1, y, 0);
	}
}
void createWalls()
{
	for (int y = 0; y < mapH; ++y)
	{
		type[20][y] = SOLID;
		type[mapW - 10][y] = SOLID;
	}
	for (int i = 0; i <= 64; ++i)
	{
		type[i][64 - i] = SOLID;
		if (i < 64) type[i][64 - i - 1] = SOLID;
	}

	type[64][80] = SOLID;
	//cellType[63][79] = 1;
	//cellType[65][79] = 1;

	/*for (int x = 0; x < mapW; ++x)
	{
		type[x][0] = SOLID;
		type[0][x] = SOLID;
		type[x][mapH - 1] = SOLID;
		type[mapW - 1][x] = SOLID;
	}*/
}

void computeR()
{
	float scale = rho / dt;

	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			r[x][y] = scale * (u->at(x + 1, y) - u->at(x, y) +
				v->at(x, y + 1) - v->at(x, y));
		}
	}
}
void project()
{
	float scale = 1;
	int N = 600;

	float maxDelta = 0;
	for (int iter = 0; iter < N; ++iter)
	{
		maxDelta = 0;

		for (int y = 0; y < mapH; ++y)
		{
			for (int x = 0; x < mapW; ++x)
			{
				if (type[x][y] == AIR)
					continue;
				
				float sigma = 0;
				float n = 0;

				if (x > 0) {
					if (type[x - 1][y] != SOLID)
					{
						if (type[x - 1][y] == WATER)
							sigma += 1 * p[x - 1][y];
						++n;
					}
				}
				if (y > 0) {
					if (type[x][y - 1] != SOLID)
					{
						if (type[x][y - 1] == WATER)
							sigma += 1 * p[x][y - 1];
						++n;
					}
				}
				if (x < mapW - 1) {
					if (type[x + 1][y] != SOLID)
					{
						if (type[x + 1][y] == WATER)
							sigma += 1 * p[x + 1][y];
						++n;
					}
				}
				if (y < mapH - 1) {
					if (type[x][y + 1] != SOLID)
					{
						if (type[x][y + 1] == WATER)
							sigma += 1 * p[x][y + 1];
						++n;
					}
				}

				float newP = (r[x][y] - sigma) / -n;
				maxDelta = max(maxDelta, fabs(p[x][y] - newP));

				p[x][y] = newP;
			}
		}

		//enforceBoundary();

		if (maxDelta < 0.001)
		{
			//printf("maxdelta good enough after %d\n", iter);
			return;
		}
	}
	//printf("maximum change was %f \n", maxDelta);
}
void applyPressure()
{
	float scale = dt / (rho);

	for (int y = 0;  y < mapH; y++)
	{
		for (int x = 0; x < mapW; x++)
		{	
			if (type[x][y] != WATER)
				continue;
			u->at(x, y) -= scale * p[x][y];
			u->at(x + 1, y) += scale * p[x][y];
			v->at(x, y) -= scale * p[x][y];
			v->at(x, y + 1) += scale * p[x][y];
		}
	}
}
/*void applyPressure()
{
	float scale = dt / rho;

	for (int y = 0; y < mapH; y++)
	{
		for (int x = 0; x < mapW; x++)
		{
			int idx = y * mapW + x;
			u->at(x, y) -= scale * pressure(idx);
			u->at(x + 1, y) += scale * pressure(idx);
			v->at(x, y) -= scale * pressure(idx);
			v->at(x, y + 1) += scale * pressure(idx);
		}
	}
}*/

void applyExternal()
{
	static int iter = 0;
	static int amount = 0;
	for (int x = 59; x <= 69; ++x)
	{
		for (int y = 100; y < 112; ++y)
		{
			if (iter > 1200)
				continue;
			ink->at(x, y) = 3;
			//v->at(x, y) -= 0.6;
			//v->at(x, y) = 0;
			//u->at(x, y) = -8;
			//if (iter % 4 == 0)
			if (type[x][y] != WATER)
			{
				parts.push_back(particle(vec2(x + nrand(), y + nrand())));
				parts.push_back(particle(vec2(x - 10.f + nrand(), y + nrand()), 1));
				//parts.push_back(particle(vec2(x + 0.5, y + 0.5)));
				++amount;
			}
		}
	}
	++iter;
	printf("Amount released: %d\n", amount);

	for (int y = 0; y < mapH + 1; ++y)
		for (int x = 0; x < mapW; ++x)
			v->at(x, y) -= 9 * dt;
}

void updateCellType()
{
	for (int y = 1; y < mapH - 1; ++y)
	{
		for (int x = 1; x < mapW - 1; ++x)
		{
			if (type[x][y] == AIR &&
				type[x - 1][y] == WATER &&
				type[x + 1][y] == WATER &&
				type[x][y - 1] == WATER &&
				type[x][y + 1] == WATER)
				parts.push_back(particle(vec2(x + nrand(), y + nrand())));
		}
	}
	
	int amountNow = 0;
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			if (type[x][y] == WATER)
				++amountNow;
			if (type[x][y] != SOLID)
				type[x][y] = AIR;
		}
	}
	printf("amount now: %d\n", amountNow);

	for (particle& p : parts)
	{
		RK2Integrator(&p.pos.x, &p.pos.y, dt, u, v);
		
		if (p.pos.x < 0)
			p.pos.x = 0;
		if (p.pos.y < 0)
			p.pos.y = 0;
		if (p.pos.x > mapW - 0.01)
			p.pos.x = mapW - 0.01;
		if (p.pos.y > mapH - 0.01)
			p.pos.y = mapH - 0.01;

		if (type[(int)(p.pos.x)][(int)(p.pos.y)] != SOLID)
			type[(int)(p.pos.x)][(int)(p.pos.y)] = WATER;
	}
}

void extrapolate()
{
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			if (type[x][y] == WATER)
				continue;

			if (y > 0)
			{
				if (type[x][y - 1] == WATER)
				{
					v->at(x, y + 1) = v->at(x, y);
				}
			}
			if (y < mapH - 1)
			{
				if (type[x][y + 1] == WATER)
				{
					v->at(x, y) = v->at(x, y + 1);
				}
			}
			if (x > 0)
			{
				if (type[x - 1][y] == WATER)
				{
					u->at(x + 1, y) = u->at(x, y);
				}
			}
			if (x < mapW - 1)
			{
				if (type[x + 1][y] == WATER)
				{
					u->at(x, y) = u->at(x + 1, y);
				}
			}
			
		}
	}
}

void update()
{
	applyExternal();
	
	computeR();
	project();
	applyPressure();

	enforceBoundary();

	u->advect(dt, u, v);
	v->advect(dt, u, v);

	u->flip();
	v->flip();

	updateCellType();
	extrapolate();

	//printf("%d\n", (int)parts.size());

	// diagnostics
	{
		double rx, ry;
		glfwGetCursorPos(window, &rx, &ry);
		rx /= imageWidth / mapW; ry /= imageHeight / mapH;
		ry = mapH - ry;

		//printf("%i, %i \n", (int)rx, (int)ry);

		if (0 <= rx && rx < mapW &&
			0 <= ry && ry < mapH)
		{
			printf("-----------------------------\n");
			printf("D: %f\n", ink->at(rx, ry));
			printf("V: %f, %f\n", u->lerp(rx, ry), v->lerp(rx, ry));
			printf("P: %f\n", p[(int)rx][(int)ry]);
			printf("-----------------------------\n\n");
		}

		/*if (0 <= rx && rx < cellW &&
			0 <= ry && ry < cellH)
		{
			if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
			{
				ink->set(rx, ry, ink->at(rx, ry) + 1);
				//vel[(int)rx][(int)ry] += p;
			}
		}*/
	}
}

void draw()
{
	glViewport(0, 0, imageWidth, imageHeight);
	glClear(GL_COLOR_BUFFER_BIT);

	glLoadIdentity();
	glTranslatef(-1, -1, 0);
	glScalef(2, 2, 1);
	glScalef(1.f / mapW, 1.f / mapH, 1);

	glBegin(GL_QUADS);
	{
		for (int x = 0; x < mapW; ++x)
		{
			for (int y = 0; y < mapH; ++y)
			{
				float f = ink->at(x, y) / (3);
				glColor3f(0, 0, 0);
				if (type[x][y] == AIR)
					glColor3f(0, 0, 1);
				if (type[x][y] == SOLID)
					glColor3f(0, 1, 0);

				float xVel = u->lerp(x + 0.5, y + 0.5);
				float yVel = v->lerp(x + 0.5, y + 0.5);
				xVel = xVel / (abs(xVel) + 1) / 2;
				yVel = yVel / (abs(yVel) + 1) / 2;
				//glColor3f(0, xVel + 0.5, yVel + 0.5);

				glVertex2f(x, y);
				glVertex2f(x + 1, y);
				glVertex2f(x + 1, y + 1);
				glVertex2f(x, y + 1);
			}
		}
	}
	glEnd();

	glBegin(GL_POINTS);
	{
		for (particle p : parts)
		{
			if (p.type == 0)
				glColor3f(1, 0, 0);
			if (p.type == 1)
				glColor3f(0, 1, 0);
			glVertex2f(p.pos.x, p.pos.y);
		}
	}
	glEnd();

	glBegin(GL_LINES);
	{
		for (int x = 0; x < mapW; x += 5)
		{
			for (int y = 0; y < mapH; y += 5)
			{
				if (type[x][y] != WATER)
					continue;

				float xVel = u->at(x, y), 
					yVel = v->at(x, y);
				glColor4f(0, 1, 0, 0.25f);
				glVertex2f(x, y);
				glColor4f(0, 1, 0, 0.5f);
				glVertex2f(x + xVel, y + yVel);
			}
		}
	}
	glEnd();
}

int main()
{
	srand(time(0));
	
	if (!glfwInit())
	{
		printf("couldn't initialize GLFW");
		return 0;
	}

	// no window hints. don't really care

	window = glfwCreateWindow(imageWidth, imageHeight, "lulz", NULL, NULL);
	if (!window)
	{
		printf("failed to open glfw window");
		return 0;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);

	ink = new fluidQ(mapW, mapH, 0.5, 0.5, 1);
	u = new fluidQ(mapW + 1, mapH + 1, 0.0, 0.5, 1);
	v = new fluidQ(mapW + 1, mapH + 1, 0.5, 0.0, 1);
	setupParticles();

	memset(p, 0, mapW * mapH * sizeof(float));
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			type[x][y] = AIR;
		}
	}
	createWalls();

	// main loop
	auto currentTime = chrono::high_resolution_clock::now();
	float accumulator = 0;
	int iter = 0;
	while (!glfwWindowShouldClose(window))
	{
		auto newTime = chrono::high_resolution_clock::now();
		float frameTime = chrono::duration_cast<chrono::milliseconds>(newTime - currentTime).count();

		if (frameTime >= dt)
		{
			update();

			++iter;
			//printf("%d\n", iter);

			currentTime = newTime;
		}
		
		draw();

		//Sleep(500);
		
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	printf("%d", window);
	glfwDestroyWindow(window);
	glfwTerminate();

	delete ink;
	delete u;
	delete v;

	return 0;
}