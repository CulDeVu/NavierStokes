
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
#undef max
#undef min

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
	vec2 vel;

	particle(vec2 p, vec2 v)
	{
		pos = p;
		vel = v;
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
			if (y < mapH)
				u->set(x, y, 0);
			if (x < mapW)
				v->set(x, y, 0);
		}
	}

	for (int x = 25; x < 64; ++x)
	{
		for (int y = mapH - 80; y < mapH - 20; ++y)
		{
			float q = 2;
			for (int i = 0; i < q; ++i)
			{
				for (int j = 0; j < q; ++j)
				{
					float xpos = x + i / q + nrand() / q;
					float ypos = y + j / q + nrand() / q;
					parts.push_back(particle(vec2(xpos, ypos), vec2(0, 0)));
				}
			}
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

		enforceBoundary();

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

int numPartsInCell[mapW][mapH];
void applyExternal()
{
	/*static int iter = 0;
	static int amount = 0;
	for (int x = 59; x <= 69; ++x)
	{
		//for (int y = 100; y < 112; ++y)
		int y = 100;
		{
			if (iter > 1200)
				continue;

			if (type[x][y] != WATER)
			{
				float q = 2;
				for (int i = 0; i < q; ++i)
				{
					for (int j = 0; j < q; ++j)
					{
						float xpos = x + i / q + nrand() / q;
						float ypos = y + j / q + nrand() / q;
						parts.push_back(particle(vec2(xpos, ypos), vec2(0, 0)));
					}
				}

				++amount;
			}
		}
	}
	++iter;
	printf("Amount released: %d\n", amount);*/

	for (particle& p : parts)
		p.vel.y -= 9 * dt;
}
void updateCellType()
{
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			if (type[x][y] == SOLID)
				continue;
			type[x][y] = AIR;
		}
	}

	for (particle& p : parts)
	{
		int x = p.pos.x;
		int y = p.pos.y;

		if (type[x][y] == SOLID)
			continue;
		type[x][y] = WATER;

		numPartsInCell[x][y] += 1;
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
			if (type[x][y] == SOLID)
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
void updateParticles()
{
	for (particle& p : parts)
	{
		p.pos += p.vel * dt;

		if (p.pos.x < 0)
			p.pos.x = 0;
		if (p.pos.y < 0)
			p.pos.y = 0;
		if (p.pos.x > mapW - 0.01)
			p.pos.x = mapW - 0.01;
		if (p.pos.y > mapH - 0.01)
			p.pos.y = mapH - 0.01;
	}

	float maxNum = 0;
	vec2 maxCellIndex = vec2();
	float minNum = 80000;
	vec2 minCellIndex = vec2();
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			if (numPartsInCell[x][y] > maxNum)
			{
				maxNum = numPartsInCell[x][y];
			}
		}
	}
}

void particlesToGrid()
{
	float u_weights[mapW + 1][mapH];
	float v_weights[mapW][mapH + 1];
	for (int y = 0; y < mapH + 1; ++y)
	{
		for (int x = 0; x < mapW + 1; ++x)
		{
			if (y < mapH)
				u_weights[x][y] = 0;
			if (x < mapW)
				v_weights[x][y] = 0;
		}
	}

	u->clear();
	v->clear();

	for (particle& p : parts)
	{
		int ix = p.pos.x;
		int iy = p.pos.y;

		// u
		{
			float tx = p.pos.x - ix;
			float ty = max(0.f, min(mapH - 0.5f, p.pos.y - iy - 0.5f));
			
			u->at(ix, iy) += p.vel.x * (1 - tx) * (1 - ty);
			u->at(ix + 1, iy) += p.vel.x * tx * (1 - ty);
			u->at(ix, iy + 1) += p.vel.x * (1 - tx) * ty;
			u->at(ix + 1, iy + 1) += p.vel.x * tx * ty;

			u_weights[ix][iy] += (1 - tx) * (1 - ty);
			u_weights[ix + 1][iy] += tx * (1 - ty);
			u_weights[ix][iy + 1] += (1 - tx) * ty;
			u_weights[ix + 1][iy + 1] += tx * ty;
		}
		
		// v
		{
			float tx = max(0.f, min(mapW - 0.5f, p.pos.x - ix - 0.5f));
			float ty = p.pos.y - iy;

			v->at(ix, iy) += p.vel.y * (1 - tx) * (1 - ty);
			v->at(ix + 1, iy) += p.vel.y * tx * (1 - ty);
			v->at(ix, iy + 1) += p.vel.y * (1 - tx) * ty;
			v->at(ix + 1, iy + 1) += p.vel.y * tx * ty;

			v_weights[ix][iy] += (1 - tx) * (1 - ty);
			v_weights[ix + 1][iy] += tx * (1 - ty);
			v_weights[ix][iy + 1] += (1 - tx) * ty;
			v_weights[ix + 1][iy + 1] += tx * ty;
		}
	}

	for (int y = 0; y < mapH + 1; ++y)
	{
		for (int x = 0; x < mapW + 1; ++x)
		{
			if (y < mapH)
				u->at(x, y) /= max(1.f, u_weights[x][y]);
			if (x < mapW)
				v->at(x, y) /= max(1.f, v_weights[x][y]);
		}
	}
}
void gridToParticles()
{
	for (particle& p : parts)
	{
		int ix = p.pos.x;
		int iy = p.pos.y;
		float tx = p.pos.x - ix;
		float ty = p.pos.y - iy;

		// u
		{
			float tx = p.pos.x - ix;
			float ty = max(0.f, min(mapH - 0.5f, p.pos.y - iy - 0.5f));

			p.vel.x += u->diffAt(ix, iy) * (1 - tx) * (1 - ty);
			p.vel.x += u->diffAt(ix + 1, iy) * tx * (1 - ty);
			p.vel.x += u->diffAt(ix, iy + 1) * (1 - tx) * ty;
			p.vel.x += u->diffAt(ix + 1, iy + 1) * tx * ty;
		}

		// v
		{
			float tx = max(0.f, min(mapW - 0.5f, p.pos.x - ix - 0.5f));
			float ty = p.pos.y - iy;

			p.vel.y += v->diffAt(ix, iy) * (1 - tx) * (1 - ty);
			p.vel.y += v->diffAt(ix + 1, iy) * tx * (1 - ty);
			p.vel.y += v->diffAt(ix, iy + 1) * (1 - tx) * ty;
			p.vel.y += v->diffAt(ix + 1, iy + 1) * tx * ty;
		}

		/*p.vel.x += u->diffAt(ix, iy) * (1 - tx);
		p.vel.x += u->diffAt(ix + 1, iy) * tx;

		p.vel.y += v->diffAt(ix, iy) * (1 - ty);
		p.vel.y += v->diffAt(ix, iy + 1) * ty;*/

		/*u->at(ix, iy) += p.vel.x * (1 - tx);
		u->at(ix + 1, iy) += p.vel.x * tx;
		v->at(ix, iy) += p.vel.y * (1 - ty);
		v->at(ix, iy + 1) += p.vel.y * ty;*/
	}
}

void update()
{
	applyExternal();
	extrapolate();
	updateParticles();
	updateCellType();

	particlesToGrid();

	u->copySrcToOld();
	v->copySrcToOld();

	enforceBoundary();

	computeR();
	project();
	applyPressure();

	enforceBoundary();

	gridToParticles();

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

	/*glBegin(GL_QUADS);
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
	glEnd();*/

	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	{
		for (particle p : parts)
		{
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