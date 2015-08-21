
#include <chrono>
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

const int cellW = 100,
	cellH = 100;
const float dt = 1.f / 60.f;

float ink[cellW][cellH];
float ink_next[cellW][cellH];

vec2 vel[cellW][cellH];
vec2 vel_next[cellW][cellH];

float pressure[cellW][cellH];
GLFWwindow* window;

float nrand()
{
	return (float)rand() / RAND_MAX;
}

vec2 gradient(int x, int y, float g[cellW][cellH])
{
	return vec2((g[x + 1][y] - g[x - 1][y]) / 2.f, (g[x][y + 1] - g[x][y - 1]) / 2.f);
}
float divergence(int x, int y, vec2 g[cellW][cellH])
{
	return (g[x + 1][y].x - g[x - 1][y].x) / 2.f + (g[x][y + 1].y - g[x][y - 1].y) / 2.f;
}

void setupParticles()
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			//ink[x][y] = 4 * (float)y / cellH;

			vec2 v1 = vec2(x - 30, y - 30) / 10.f;
			vec2 v2 = vec2(x - 70, y - 70) / 5.f;
			float t1 = length(v1);
			float t2 = length(v2);
			float t = t1 / (t1 + t2);
			
			//vel[x][y] = vec2(-v1.y, v1.x) * (1 - t) + vec2(v2.y, -v2.x) * t;
			//vel[x][y] = vec2(-v1.x + 2 * v1.y, -v1.y - 2 * v1.x);
			/*vel[x][y] = vec2(
				v1.y - (v1.x*v1.x*v1.x + v1.x),
				-v1.x);*/
			vel[x][y] = vec2(-v1.y, v1.x);
			//vel[x][y] = vec2();
			//vel[x][y] = vec2(-y, x) / 10.f;
		}
	}

	for (int i = 0; i < cellH; i += 4)
		ink[50][i] = 100;
}

template <typename T>
void copy(T src[cellW][cellH], T dst[cellW][cellH])
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			dst[x][y] = src[x][y];
			src[x][y] = T();
		}
	}
}

float clampAbove(float x)
{
	return x;
}
template <typename T>
void advectEulerian(T q[cellW][cellH], T q_next[cellW][cellH])
{
	for (int y = 0; y < cellH; ++y)
	{
		for (int x = 0; x < cellW; ++x)
		{
			if (x - 1 < 0 || x + 1 > cellW - 1 || y < 0 || y + 1 > cellH - 1)
				continue;

			q_next[x][y] = q[x][y];

			T dq_dx = (q[x + 1][y] - q[x - 1][y]) / 2.f;
			T dq_dy = (q[x][y + 1] - q[x][y - 1]) / 2.f;
			
			T u_dot_del_q = vel[x][y].x * dq_dx + vel[x][y].y * dq_dy;

			T dq_dt = -u_dot_del_q;
			q_next[x][y] += dq_dt * dt;
			//q_next[x][y] = clamp(q_next[x][y], -10.f, 10000000.f);
		}
	}
}

template <typename T>
void advectForwardLangrangian(T q[cellW][cellH], T q_next[cellW][cellH])
{
	for (int y = 0; y < cellH; ++y)
	{
		for (int x = 0; x < cellW; ++x)
		{
			vec2 advectTarget = vec2(x, y) + vel[x][y] / 60.f;

			float tu = advectTarget.x - floor(advectTarget.x);
			float tv = advectTarget.y - floor(advectTarget.y);
			int xt = floor(advectTarget.x);
			int yt = floor(advectTarget.y);

			float ll = (1 - tu) * (1 - tv);
			float lh = (1 - tu) * tv;
			float hl = tu * (1 - tv);
			float hh = tu * tv;

			if (xt < 0 || xt + 1 > cellW - 1 || yt < 0 || yt + 1 > cellH - 1)
			{
				continue;
			}

			q_next[xt][yt] += ll * q[x][y];
			q_next[xt + 1][yt] += hl * q[x][y];
			q_next[xt][yt + 1] += lh * q[x][y];
			q_next[xt + 1][yt + 1] += hh * q[x][y];
		}
	}
}

template <typename T>
void advectBackwardsLangrangian(T q[cellW][cellH], T q_next[cellW][cellH])
{
	for (int y = 0; y < cellH; ++y)
	{
		for (int x = 0; x < cellW; ++x)
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

			if (xt < 0 || xt + 1 > cellW - 1 || yt < 0 || yt + 1 > cellH - 1)
			{
				//q_next[x][y] += q[x][y];
				q_next[x][y] = T();
				continue;
				//xt = min(cellW - 2, max(xt, 0));
				//yt = min(cellH - 2, max(yt, 0));
			}

			q_next[x][y] += ll * q[xt][yt];
			q_next[x][y] += hl * q[xt + 1][yt];
			q_next[x][y] += lh * q[xt][yt + 1];
			q_next[x][y] += hh * q[xt + 1][yt + 1];
		}
	}
}

void enforceBoundary()
{
	for (int x = 0; x < cellW; ++x)
	{
		vel[x][0].y = 0;
		vel[x][cellH - 1].y = 0;
	}

	for (int y = 0; y < cellH; ++y)
	{
		vel[0][y].x = 0;
		vel[cellW - 1][y].x = 0;
	}
}

vec2 prevPos = vec2();
void update()
{
	vec2 F_grav = vec2(0, -1);

	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			ink_next[x][y] = 0;
			vel_next[x][y] = vec2();

			//vel[x][y] += F_grav / 60.f;
		}
	}

	advectForwardLangrangian(ink, ink_next);
	//for (int x = 0; x < cellW; ++x) for (int y = 0; y < cellH; ++y) ink_next[x][y] = max(0.f, ink_next[x][y]);
	//advectEulerian(vel, vel_next);
	copy(vel, vel_next);

	enforceBoundary();

	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			ink[x][y] = ink_next[x][y];
			vel[x][y] = vel_next[x][y];
		}
	}

	// diagnostics
	{
		double rx, ry;
		glfwGetCursorPos(window, &rx, &ry);
		rx /= 600.f / cellW; ry /= 600.f / cellH;
		ry = cellH - ry;

		printf("%i, %i \n", (int)rx, (int)ry);

		/*if (0 <= rx && rx < cellW &&
			0 <= ry && ry < cellH)
		{
			printf("-----------------------------\n");
			printf("D: %f\n", ink[(int)rx][(int)ry]);
			printf("V: %f, %f\n", vel[(int)rx][(int)ry].x, vel[(int)rx][(int)ry].y);
			printf("P: %f\n", pressure[(int)rx][(int)ry]);
			printf("-----------------------------\n\n");
		}*/

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
		{
			vec2 p = (vec2(rx, ry) - prevPos) * 10.f;
			ink[(int)rx][(int)ry] += 1;
			//vel[(int)rx][(int)ry] += p;
		}

		prevPos = vec2(rx, ry);
	}
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
				float f = ink[x][y] / (max(0.f, ink[x][y]) + 1);
				if (ink[x][y] >= 0)
					glColor3f(f, f, f);
				/*if (ink[x][y] < 0)
					glColor3f(0, 1, 0);
				if (ink[x][y] >= 1000)
					glColor3f(1, 0, 0);*/
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
	auto currentTime = chrono::high_resolution_clock::now();
	float accumulator = 0;
	int iter = 0;
	while (!glfwWindowShouldClose(window))
	{
		auto newTime = chrono::high_resolution_clock::now();
		float frameTime = chrono::duration_cast<chrono::milliseconds>(newTime - currentTime).count();

		if (frameTime >= dt && iter < 3000)
		{
			update();

			if (2000 < iter && iter < 4000)
				ink[65][20] += 1;

			++iter;
			//printf("iter %d\n", iter);

			currentTime = newTime;
		}
		
		draw();

		//Sleep(500);
		
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}