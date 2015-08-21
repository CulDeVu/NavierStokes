
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
			density[x][y] = 4 * (float)y / cellH;

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

/*void enforceIncompressability(vec2 cur[cellW][cellH], vec2 next[cellW][cellH])
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{

		}
	}
}*/

/*void navier(float cur[cellW][cellH], float next[cellW][cellH], vec2 vel[cellW][cellH])
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			float dDensity_dt = 0;
			vec2 curVel = vel[x][y];

			if (x - 2 < 0 || x + 2 >= cellW || y - 2 < 0 || y + 2 >= cellH)
			{
				next[x][y] = cur[x][y];
				continue;
			}

			vec2 del_density = vec2((cur[x + 1][y] - cur[x - 1][y]) / 2,
				(cur[x][y + 1] - cur[x][y - 1]) / 2);
			float advection = curVel.x * del_density.x + curVel.y * del_density.y;
			dDensity_dt -= advection;

			float laplacian_density = (cur[x + 1][y] - 2.f * cur[x][y] + cur[x - 1][y]) + (cur[x][y + 1] - 2.f * cur[x][y] + cur[x][y - 1]);
			float viscosity = 0.1;
			dDensity_dt += viscosity * laplacian_density;

			next[x][y] = cur[x][y] + dDensity_dt / 60.f;
			if (next[x][y] < 0)
				next[x][y] = 0;
		}
	}
}*/

vec2 getPressureTerm(int x, int y, float viscosity)
{
	vec2 del_pressure;

	float dp_dx = 0;
	float dp_dy = 0;

	if (y == cellH - 1)
		dp_dy = (0 - density[x][y - 1]) / 2.f;
	else if (y == 0)
		dp_dy = (density[x][y + 1] - density[x][y]) / 1.f;
	else
		dp_dy = (density[x][y + 1] - density[x][y - 1]) / 2.f;

	if (x == cellW - 1)
		dp_dx = (0 - density[x - 1][y]) / 2.f;
	else if (x == 0)
		dp_dx = (density[x + 1][y] - density[x][y]) / 1.f;
	else
		dp_dx = (density[x + 1][y] - density[x - 1][y]) / 2.f;

	del_pressure = vec2(dp_dx, dp_dy);
	return -del_pressure / viscosity;
}

void navier(float density[cellW][cellH], vec2 vel[cellW][cellH], vec2 vel_next[cellW][cellH])
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			vec2 dDensity_dt = vec2();
			vec2 curVel = vel[x][y];

			if (x - 1 < 0 || x + 1 >= cellW || y - 1 < 0 || y + 1 >= cellH)
			{
				vel_next[x][y] = vel[x][y];
				continue;
			}

			vec2 del_density_x = (vel[x + 1][y] - vel[x - 1][y]) / 1.f;
			vec2 del_density_y = (vel[x][y + 1] - vel[x][y - 1]) / 1.f;
			vec2 advection = curVel.x * del_density_x + curVel.y * del_density_y;
			dDensity_dt -= advection;

			dDensity_dt += getPressureTerm(x, y, 0.1);

			vec2 laplacian_density = (vel[x + 1][y] - 2.f * vel[x][y] + vel[x - 1][y]) + (vel[x][y + 1] - 2.f * vel[x][y] + vel[x][y - 1]);
			float viscosity = 5.f;
			dDensity_dt += viscosity * laplacian_density;

			vel_next[x][y] = vel[x][y] + dDensity_dt / 60.f;
			if (length(vel_next[x][y]) > 10)
				vel_next[x][y] = normalize(vel_next[x][y]) * 10.f;
		}
	}
}

void enforceIncompressibility(vec2 cur[cellW][cellH])
{
	for (int i = 0; i < 4; ++i)
	{
		for (int x = 0; x < cellW; ++x)
		{
			for (int y = 0; y < cellH; ++y)
			{
				if (x - 1 < 0 || x + 1 >= cellW || y - 1 < 0 || y + 1 >= cellH)
				{
					continue;
				}

				vec2 del_density_x = (cur[x + 1][y] - cur[x - 1][y]) / 2.f;
				vec2 del_density_y = (cur[x][y + 1] - cur[x][y - 1]) / 2.f;
				vec2 divergence = del_density_x + del_density_y;

				if (divergence.x != 0)
				{
					cur[x + 1][y].x -= divergence.x / 4;
					cur[x - 1][y].x += divergence.x / 4;
				}
				if (divergence.y != 0)
				{
					cur[x][y + 1].y -= divergence.y / 4;
					cur[x][y - 1].y += divergence.y / 4;
				}
			}
		}
	}
}

void updateDensityField(float cur[cellW][cellH], float next[cellW][cellH], vec2 vel[cellW][cellH], vec2 vel_next[cellW][cellH])
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			vec2 advectTarget = vec2(x, y) + vel[x][y] / 60.f;

			float tu = advectTarget.x - (int)(advectTarget.x);
			float tv = advectTarget.y - (int)(advectTarget.y);
			int xt = floor(advectTarget.x);
			int yt = floor(advectTarget.y);

			float ll = (1 - tu) * (1 - tv);
			float lh = (1 - tu) * tv;
			float hl = tu * (1 - tv);
			float hh = tu * tv;

			if (xt < 0 || xt + 1 >= cellW || yt < 0 || yt + 1 >= cellH)
			{
				next[x][y] = cur[x][y];
				continue;
			}

			if (cur[x][y] > 0)
			{
				/*vel_next[xt][yt] = (ll * vel[x][y] * cur[x][y] + vel_next[xt][yt] * next[xt][yt]) / (ll * cur[x][y] + next[xt][yt]);
				vel_next[xt + 1][yt] = (hl * vel[x][y] * cur[x][y] + vel_next[xt + 1][yt] * next[xt + 1][yt]) / (hl * cur[x][y] + next[xt + 1][yt]);
				vel_next[xt][yt + 1] = (lh * vel[x][y] * cur[x][y] + vel_next[xt][yt + 1] * next[xt][yt + 1]) / (lh * cur[x][y] + next[xt][yt + 1]);
				vel_next[xt + 1][yt + 1] = (hh * vel[x][y] * cur[x][y] + vel_next[xt + 1][yt + 1] * next[xt + 1][yt + 1]) / (hh * cur[x][y] + next[xt + 1][yt + 1]);*/

				next[xt][yt] += ll * cur[x][y];
				next[xt + 1][yt] += hl * cur[x][y];
				next[xt][yt + 1] += lh * cur[x][y];
				next[xt + 1][yt + 1] += hh * cur[x][y];
			}
			//vel_next[xt][yt] += ll * vel[x][y];
			//vel_next[xt + 1][yt] += hl * vel[x][y];
			//vel_next[xt][yt + 1] += lh * vel[x][y];
			//vel_next[xt + 1][yt + 1] += hh * vel[x][y];

		}
	}
}

void copy(vec2 src[cellW][cellH], vec2 dst[cellW][cellH])
{
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			dst[x][y] = src[x][y];
			src[x][y] = vec2();
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

	float density_next[cellW][cellH];
	vec2 vel_next[cellW][cellH];
	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			density_next[x][y] = 0;
			vel_next[x][y] = vec2();
			//vel[x][y] += F_grav / 20.f;
			//vel_next[x][y] = vel[x][y];
		}
	}

	// add sources
	{
		double rx, ry;
		glfwGetCursorPos(window, &rx, &ry);
		rx /= 600.f / cellW; ry /= 600.f / cellH;
		ry = cellH - ry;

		printf("%i, %i \n", (int)rx, (int)ry);

		if (0 <= rx && rx < cellW &&
			0 <= ry && ry < cellH)
		{
			printf("-----------------------------\n");
			printf("D: %f\n", density[(int)rx][(int)ry]);
			printf("V: %f, %f\n", vel[(int)rx][(int)ry].x, vel[(int)rx][(int)ry].y);
			vec2 pressure = getPressureTerm(rx, ry, 0.1);
			printf("Pressure: %f, %f\n", pressure.x, pressure.y);
			printf("-----------------------------\n\n");
		}

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT))
		{
			vec2 p = (vec2(rx, ry) - prevPos) * 50.f;
			vel[(int)rx][(int)ry] += p;
		}

		prevPos = vec2(rx, ry);
			//density[(int)rx][(int)ry] += 60 / 60.f;
	}

	//navier(density, density_next, vel);
	navier(density, vel, vel_next);
	//copy(vel_next, vel);
	enforceIncompressibility(vel_next);
	updateDensityField(density, density_next, vel, vel_next);

	for (int x = 0; x < cellW; ++x)
	{
		for (int y = 0; y < cellH; ++y)
		{
			density[x][y] = density_next[x][y];
			vel[x][y] = vel_next[x][y];
		}
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