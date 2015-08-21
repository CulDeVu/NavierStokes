
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <vector>

#include "GLFW\glfw3.h"
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "GLFW/glfw3dll.lib")

#include <Windows.h>

#include <glm\glm.hpp>

using namespace std;
using namespace glm;

const int imageWidth = 256,
		imageHeight = 256;

float PI = 3.14159;

struct cell
{
	float amount;
	vec2 vel;
};

const int cellW = 100,
	cellH = 100;
float density[cellW][cellH];
vec2 vel[cellW][cellH];
GLFWwindow* window;

float nrand()
{
	return (float)rand() / RAND_MAX;
}

void setupParticles()
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			density[x][y] = (float)y / cellH / 2;

			float vx = x - 20;
			float vy = y - 20;
			vel[x][y] = vec2(-vy, vx) / 20.f;
		}
	}
}

template <typename T>
void diffuse(T density[cellW][cellH], T density_next[cellW][cellH])
{
	float diff = 1;
	float a = 1.f / 60.f * diff;

	for (int k = 0; k < 20; ++k)
	{
		for (int x = 0; x < cellW; ++x)
		{
			for (int y = 0; y < cellH; ++y)
			{
				if (x == 0 || x == cellW - 1 || y == 0 || y == cellH - 1)
				{
					density_next[x][y] = density[x][y];
					continue;
				}
				
				density_next[x][y] = (density[x][y] +
					a * (density[x - 1][y] +
					density[x + 1][y] +
					density[x][y - 1] +
					density[x][y + 1])) / (1 + 4 * a);
			}
		}
	}
}

template <typename T>
void advect(T density[cellW][cellH], T density_next[cellW][cellH], vec2 vel[cellW][cellH])
{
	/*int i, j, i0, j0, i1, j1;
	float x, y, s0, t0, s1, t1, dt0;
	dt0 = 1.f / 60.f;
	for (i = 1; i <= cellW; i++) {
		for (j = 1; j <= cellH; j++) {
			x = i - dt0*vel[i][j].x; y = j - dt0*vel[i][j].y;
			if (x<0.5) x = 0.5; if (x>cellW + 0.5) x = cellW + 0.5; i0 = (int)x; i1 = i0 + 1;
			if (y<0.5) y = 0.5; if (y>cellH + 0.5) y = cellH + 0.5; j0 = (int)y; j1 = j0 + 1;
			s1 = x - i0; s0 = 1 - s1; t1 = y - j0; t0 = 1 - t1;
			density_next[i][j] = s0*(t0*density[i0][j0] + t1*density[i0][j1]) +
				s1*(t0*density[i1][j0] + t1*density[i1][j1]);
		}
	}*/

	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			vec2 advectTarget = vec2(x, y) - vel[x][y] / 60.f;

			float tu = advectTarget.x - floor(advectTarget.x);
			float tv = advectTarget.y - floor(advectTarget.y);
			int xt = floor(advectTarget.x);
			int yt = floor(advectTarget.y);

			float ll = (1 - tu) * (1 - tv);
			float lh = (1 - tu) * tv;
			float hl = tu * (1 - tv);
			float hh = tu * tv;

			if (xt < 0 || xt + 1 >= cellW || yt < 0 || yt + 1 >= cellH)
			{
				density_next[x][y] = density[x][y];
				continue;
			}

			density_next[x][y] = ll * density[xt][yt];
			density_next[x][y] += hl * density[xt + 1][yt];
			density_next[x][y] += lh * density[xt][yt + 1];
			density_next[x][y] += hh * density[xt + 1][yt + 1];
		}
	}
}

template <typename T>
void copy(T dst[cellW][cellH], T src[cellW][cellH])
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			dst[x][y] = src[x][y];
		}
	}
}

void update()
{
	vec2 F_grav = vec2(0, -1);

	float density_next[cellW][cellH];
	vec2 vel_next[cellW][cellH];
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			density_next[x][y] = 0;
			vel[x][y] += F_grav / 60.f / max(0.01f, density[x][y]);
		}
	}

	// density step

	// add sources
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
	{
		double rx, ry;
		glfwGetCursorPos(window, &rx, &ry);
		printf("%i, %i \n", (int)rx, (int)ry);
		rx /= 600.f / cellW; ry /= 600.f / cellH;
		ry = cellH - ry;
		density[(int)rx][(int)ry] += 60 / 60.f;
	}
	copy(density_next, density); diffuse(density, density_next);
	copy(density_next, density); advect(density, density_next, vel);

	// velocity step

	//copy(vel_next, vel); diffuse(vel, vel_next);
	copy(vel_next, vel); advect(vel, vel_next, vel);

	//copy(vel_next, vel);
	copy(density, density_next);
	copy(vel, vel_next);

	/*for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			if (y + 1 < cellH)
				vel[x][y].y += max(0.f, density[x][y] - density[x][y + 1]) / 2;
			if (y - 1 >= 0)
				vel[x][y].y -= max(0.f, density[x][y] - density[x][y - 1]) / 2;

			if (x + 1 < cellW)
				vel[x][y].x += max(0.f, density[x][y] - density[x + 1][y]) / 2;
			if (x - 1 >= 0)
				vel[x][y].x -= max(0.f, density[x][y] - density[x - 1][y]) / 2;
		}
	}

	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			vec2 advectTarget = vec2(x, y) + vel[x][y] / 60.f;
			
			float tu = advectTarget.x - floor(advectTarget.x);
			float tv = advectTarget.y - floor(advectTarget.y);
			int xt = floor(advectTarget.x);
			int yt = floor(advectTarget.y);

			float f = density[x][y];
			vec2 v = vel[x][y];
			float ll = (1 - tu) * (1 - tv);
			float lh = (1 - tu) * tv;
			float hl = tu * (1 - tv);
			float hh = tu * tv;

			if (xt < 0 || xt + 1 >= cellW || yt < 0 || yt + 1 >= cellH)
			{
				density_next[x][y] += density[x][y];
				continue;
			}

			density_next[xt][yt] += ll * f;
			vel_next[xt][yt] += ll * v;

			density_next[xt + 1][yt] += hl * f;
			vel_next[xt + 1][yt] += hl * v;

			density_next[xt][yt + 1] += lh * f;
			vel_next[xt][yt + 1] += lh * v;

			density_next[xt + 1][yt + 1] += hh * f;
			vel_next[xt + 1][yt + 1] += hh * v;
		}
	}*/

	/*for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			density[x][y] = density_next[x][y];
			//vel[x][y] = vel_next[x][y];
		}
	}*/
}

void draw()
{
	glViewport(0, 0, 600, 600);
	glClear(GL_COLOR_BUFFER_BIT);

	glLoadIdentity();
	glTranslatef(-1, -1, 0);
	glScalef(2, 2, 1);
	glScalef(1.f / cellW, 1.f / cellH, 1);

	glBegin(GL_QUADS);
	{
		for (int x = 0; x < cellW; ++x)
		{
			for (int y = 0; y < cellH; ++y)
			{
				float f = density[x][y] / (density[x][y] + 1);
				if (density[x][y] >= 0 && density[x][y] < 1000)
					glColor3f(f, f, f);
				if (density[x][y] < 0)
					glColor3f(0, 1, 0);
				if (density[x][y] >= 1000)
					glColor3f(1, 0, 0);
				glVertex2f(x, y);
				glVertex2f(x + 1, y);
				glVertex2f(x + 1, y + 1);
				glVertex2f(x, y + 1);
			}
		}

	}
	glEnd();

	glBegin(GL_LINES);
	{
		for (int x = 0; x < cellW; x += 5)
		{
			for (int y = 0; y < cellH; y += 5)
			{
				vec2 v = vel[x][y];
				glColor4f(1, 0, 0, 0.25f);
				glVertex2f(x + 0.5, y + 0.5);
				glColor3f(1, 0, 0);
				glVertex2f(x + 0.5 + v.x, y + 0.5 + v.y);
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

	window = glfwCreateWindow(600, 600, "lulz", NULL, NULL);
	if (!window)
	{
		printf("failed to open glfw window");
		return 0;
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);

	setupParticles();
	// main loop
	while (!glfwWindowShouldClose(window))
	{
		update();
		
		draw();

		//Sleep(500);
		
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}