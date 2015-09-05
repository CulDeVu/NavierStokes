
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

struct particle
{	
	vec2 pos;

	particle(vec2 p)
	{
		pos = p;
	}
};

vector<particle> parts;

fluidQ* ink;
fluidQ* u;
fluidQ* v;
fluidQ* levelSet;

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
	/*for (int y = 0; y < mapH; ++y)
	{
		type[20][y] = SOLID;
		type[mapW - 10][y] = SOLID;
	}
	for (int i = 0; i <= 64; ++i)
	{
		type[i][64 - i] = SOLID;
	}

	type[64][80] = SOLID;
	//cellType[63][79] = 1;
	//cellType[65][79] = 1;*/
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
				if (levelSet->at(x, y) > 0)
					continue;
				
				float sigma = 0;
				float n = 0;

				if (x > 0) {
					if (levelSet->at(x - 1, y) < 0)
						sigma += 1 * p[x -1][y];
					++n;
				}
				if (y > 0) {
					if (levelSet->at(x, y - 1) < 0)
						sigma += 1 * p[x][y - 1];
					++n;
				}
				if (x < mapW - 1) {
					if (levelSet->at(x + 1, y) < 0)
						sigma += 1 * p[x + 1][y];
					++n;
				}
				if (y < mapH - 1) {
					if (levelSet->at(x, y + 1) < 0)
						sigma += 1 * p[x][y + 1];
					++n;
				}

				float newP = (r[x][y] - sigma) / -n;
				maxDelta = max(maxDelta, fabs(p[x][y] - newP));

				p[x][y] = newP;
			}
		}

		if (maxDelta < 0.01)
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
			if (levelSet->at(x, y) > 0)
				continue;
			u->at(x, y) -= scale * p[x][y];
			u->at(x + 1, y) += scale * p[x][y];
			v->at(x, y) -= scale * p[x][y];
			v->at(x, y + 1) += scale * p[x][y];
		}
	}
}

vector<vec2> markers;
void recomputeLevelSet()
{
	markers.clear();
	for (int y = 0; y < mapH; y++)
	{
		for (int x = 0; x < mapW; x++)
		{
			float cur = levelSet->at(x, y);

			if (cur == 0)
			{
				markers.push_back(vec2(x, y));
			}
			else if (cur < 0 && (
				levelSet->at(x + 1, y) > 0 ||
				levelSet->at(x - 1, y) > 0 ||
				levelSet->at(x, y - 1) > 0 ||
				levelSet->at(x, y + 1) > 0))
			{
				markers.push_back(vec2(x, y));
			}
			else
			{
				levelSet->at(x, y) = 5 * cur / abs(cur);
			}
		}
	}

	for (int y = 0; y < mapH; y++)
	{
		for (int x = 0; x < mapW; x++)
		{
			bool nope = false;
			for (int i = 0; i < markers.size(); ++i)
				if ((int)(markers[i].x + 0.5) == x && (int)(markers[i].y + 0.5) == y)
					nope = true;
			if (nope)
				continue;

			float bestScore = 5;
			vec2 best = vec2();
			for (int i = 0; i < markers.size(); ++i)
			{
				int dx = abs(markers[i].x - x);
				int dy = abs(markers[i].y - y);

				if (dx + dy < 5 * 2)
					continue;

				float len = sqrt(dx * dx + dy * dy);
				if (len < bestScore)
				{
					best = vec2(x, y);
					bestScore = len;
				}
			}

			float cur = levelSet->at(x, y);
			float bestVal = levelSet->at(best.x, best.y);
			levelSet->at(x, y) = bestScore * cur / abs(cur) + bestVal;
		}
	}
}

void extrapolate()
{
	for (int y = 0; y < mapH; y++)
	{
		for (int x = 0; x < mapW; x++)
		{
			if (levelSet->at(x, y) <= 0)
				continue;

			if (y < mapH - 1)
			{
				if (levelSet->at(x, y + 1) < 0)
					v->at(x, y) = v->at(x, y + 1);
			}
			if (y > 0)
			{
				if (levelSet->at(x, y - 1) < 0)
					v->at(x, y + 1) = v->at(x, y);
			}

			if (x > 0)
			{
				if (levelSet->at(x - 1, y) < 0)
					u->at(x + 1, y) = u->at(x, y);
			}
			if (x < mapW - 1)
			{
				if (levelSet->at(x + 1, y) < 0)
					u->at(x, y) = u->at(x + 1, y);
			}
		}
	}
}

void applyExternal()
{
	static int iter = 0;
	for (int y = 0; y < mapH; ++y)
	{
		for (int x = 0; x < mapW; ++x)
		{
			if (iter > 0)
				continue;
			vec2 o = vec2(64, 100);
			levelSet->at(x, y) = min(levelSet->at(x, y), length(vec2(x, y) - o) - 10);

			if (levelSet->at(x, y) < 0)
			{
				u->at(x, y) = -8;
			}
		}
	}
	++iter;

	for (int y = 0; y < mapH + 1; ++y)
		for (int x = 0; x < mapW; ++x)
			v->at(x, y) -= 9 * dt;
}

void update()
{
	recomputeLevelSet();
	extrapolate();
	
	applyExternal();

	levelSet->advect(dt, u, v);
	levelSet->flip();
	
	computeR();
	project();
	applyPressure();

	enforceBoundary();

	u->advect(dt, u, v);
	v->advect(dt, u, v);

	u->flip();
	v->flip();

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
			printf("L: %f\n", levelSet->at(rx, ry));
			printf("-----------------------------\n\n");
		}

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
				float f = levelSet->at(x, y) / 5;
				glColor3f(f, f, f);
				if (levelSet->at(x, y) < 0)
					glColor3f(0, 0, 1);
				glVertex2f(x, y);
				glVertex2f(x + 1, y);
				glVertex2f(x + 1, y + 1);
				glVertex2f(x, y + 1);
			}
		}
	}
	glEnd();

	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	{
		for (particle p : parts)
		{
			glVertex2f(p.pos.x, p.pos.y);
		}
	}
	glEnd();

	glColor3f(1, 0, 0);
	glBegin(GL_POINTS);
	{
		for (vec2 m : markers)
		{
			glVertex2f(m.x + 0.5f, m.y + 0.5f);
		}
	}
	glEnd();

	glBegin(GL_LINES);
	{
		for (int x = 0; x < mapW; x += 5)
		{
			for (int y = 0; y < mapH; y += 5)
			{
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
	levelSet = new fluidQ(mapW, mapH, 0.5, 0.5, 1);
	setupParticles();

	for (int x = 0; x < mapW; ++x)
	{
		for (int y = 0; y < mapH; ++y)
		{
			levelSet->at(x, y) = 8000;
		}
	}

	memset(p, 0, mapW * mapH * sizeof(float));

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

		Sleep(500);
		
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