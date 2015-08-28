
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

using namespace std;

const int imageWidth = 128,
		imageHeight = 128;

float PI = 3.14159;

const int mapW = 128,
	mapH = 128;

const int cellW = 128,
	cellH = 128;
const float dt = 1.f / 60.f;
const float rho = 0.1;  

float pressure[cellW][cellH];
GLFWwindow* window;

int cellType[mapW][mapH];

class fluidQ
{
	float* src;
	float* dst;

	float ox, oy;
	float delta_x;

	float lerp(float a, float b, float t)
	{
		return a * (1 - t) + b * t;
	}

	float cerp(float a, float b, float c, float d, float t)
	{
		float t_2 = t * t;
		float t_3 = t_2 * t;

		float minV = min(min(min(a, b), b), d);
		float maxV = max(max(max(a, b), b), d);

		float ret =
			a*(0.0 - 0.5*t + 1.0*t_2 - 0.5*t_3) +
			b*(1.0 + 0.0*t - 2.5*t_2 + 1.5*t_3) +
			c*(0.0 + 0.5*t + 2.0*t_2 - 1.5*t_3) +
			d*(0.0 + 0.0*t - 0.5*t_2 + 0.5*t_3);

		return min(maxV, max(minV, ret));
	}

	void euler(float* x, float* y, float delta_t, fluidQ* u, fluidQ* v)
	{
		float uVel = u->lerp(*x, *y) / delta_x;
		float vVel = v->lerp(*x, *y) / delta_x;

		*x -= uVel * delta_t;
		*y -= vVel * delta_t;
	}

	void RK3(float* x, float* y, float delta_t, fluidQ* u, fluidQ* v)
	{
		float u1 = u->lerp(*x, *y) / delta_x;
		float v1 = v->lerp(*x, *y) / delta_x;

		float x2 = *x - u1 * delta_t / 2;
		float y2 = *y - v1 * delta_t / 2;

		float u2 = u->lerp(x2, y2) / delta_x;
		float v2 = v->lerp(x2, y2) / delta_x;

		float x3 = *x - u2 * delta_t * 3 / 4.f;
		float y3 = *y - v2 * delta_t * 3 / 4.f;

		float u3 = u->lerp(x3, y3);
		float v3 = v->lerp(x3, y3);

		*x -= (2 * u1 + 3 * u2 + 4 * u3) * delta_t / 9;
		*y -= (2 * v1 + 3 * v2 + 4 * v3) * delta_t / 9;
	}

public:
	int w, h;

	fluidQ(int width, int height, float oX, float oY, float delta_X)
	{
		w = width;
		h = height;
		ox = oX;
		oy = oY;
		delta_x = delta_X;
		
		src = new float[w * h];
		dst = new float[w * h];

		memset(src, 0, w * h * sizeof(float));
	}

	~fluidQ()
	{
		delete[] src;
		delete[] dst;
	}

	void flip()
	{
		float* lulz = src;
		src = dst;
		dst = lulz;
	}

	float at(int x, int y) const
	{
		return src[y * w + x];
	}
	float &at(int x, int y)
	{
		return src[y * w + x];
	}

	float lerp(float x, float y)
	{
		x = min(max(x - ox, 0.f), w - 1.001f);
		y = min(max(y - oy, 0.f), h - 1.001f);
		int ix = (int)x;
		int iy = (int)y;

		x -= ix;
		y -= iy;

		float x00 = at(ix + 0, iy + 0);
		float x10 = at(ix + 1, iy + 0);
		float x01 = at(ix + 0, iy + 1);
		float x11 = at(ix + 1, iy + 1);

		return lerp(lerp(x00, x10, x), lerp(x01, x11, x), y);
	}

	float cerp(float x, float y)
	{
		x = min(max(x - ox, 0.f), w - 1.001f);
		y = min(max(y - oy, 0.f), h - 1.001f);
		int ix = (int)x;
		int iy = (int)y;

		x -= ix;
		y -= iy;

		int x0 = max(ix - 1, 0), x1 = ix, x2 = ix + 1, x3 = min(ix + 2, w - 1);
		int y0 = max(iy - 1, 0), y1 = iy, y2 = iy + 1, y3 = min(iy + 2, h - 1);

		double q0 = cerp(at(x0, y0), at(x1, y0), at(x2, y0), at(x3, y0), x);
		double q1 = cerp(at(x0, y1), at(x1, y1), at(x2, y1), at(x3, y1), x);
		double q2 = cerp(at(x0, y2), at(x1, y2), at(x2, y2), at(x3, y2), x);
		double q3 = cerp(at(x0, y3), at(x1, y3), at(x2, y3), at(x3, y3), x);

		return cerp(q0, q1, q2, q3, y);
	}

	void advect(float delta_t, fluidQ* u, fluidQ* v)
	{
		for (int iy = 0; iy < h; ++iy)
		{
			for (int ix = 0; ix < w; ++ix)
			{
				float x = ix + ox;
				float y = iy + oy;

				RK3(&x, &y, delta_t, u, v);

				dst[iy * w + ix] = cerp(x, y);
			}
		}
	}

	void set(float x, float y, float v)
	{
		int ix = x;
		int iy = y;

		src[iy * w + ix] = v;
	}
};

fluidQ* ink;
fluidQ* u;
fluidQ* v;

float nrand()
{
	return (float)rand() / RAND_MAX;
}

void setupParticles()
{
	for (int x = 0; x < cellW + 1; ++x)
	{
		for (int y = 0; y < cellH + 1; ++y)
		{
			u->set(x, y, 0);
			v->set(x, y, 0);
		}
	}
}

void enforceBoundary()
{
	for (int y = 0; y < cellH; ++y)
	{
		for (int x = 0; x < cellW; ++x)
		{
			if (cellType[x][y] == 1)
			{
				u->at(x, y) = 0;
				u->at(x + 1, y) = 0;
				v->at(x, y) = 0;
				v->at(x, y + 1) = 0;

			}
		}
	}
	
	for (int x = 0; x < cellW + 1; ++x)
	{
		v->set(x, 0, 0);
		v->set(x, v->h - 1, 0);
	}
	for (int y = 0; y < cellH + 1; ++y)
	{
		u->set(0, y, 0);
		u->set(u->w - 1, y, 0);
	}
}
void createWalls()
{
	for (int y = 0; y < cellH; ++y)
	{
		cellType[20][y] = 1;
		cellType[mapW - 10][y] = 1;
	}
	for (int i = 0; i <= 64; ++i)
	{
		cellType[i][64 - i] = 1;
	}

	cellType[64][80] = 1;
}

float r[128][128];
float p[128][128];
void buildR()
{
	float scale = 1;

	for (int y = 0; y < 128; ++y)
	{
		for (int x = 0; x < 128; ++x)
		{
			r[x][y] = -scale * (u->at(x + 1, y) - u->at(x, y) +
				v->at(x, y + 1) - v->at(x, y));
		}
	}
}
void project()
{
	float scale = dt / rho;
	int N = 600;

	float maxDelta = 0;
	for (int iter = 0; iter < N; ++iter)
	{
		maxDelta = 0;

		for (int y = 0; y < 128; ++y)
		{
			for (int x = 0; x < 128; ++x)
			{
				float diag = 0.001;
				float offDiag = 0.001;

				if (x > 0) {
					diag += scale;
					offDiag -= scale * p[x - 1][y];
				}
				if (y > 0) {
					diag += scale;
					offDiag -= scale * p[x][y - 1];
				}
				if (x < 128 - 1) {
					diag += scale;
					offDiag -= scale * p[x + 1][y];
				}
				if (y < 128 - 1) {
					diag += scale;
					offDiag -= scale * p[x][y + 1];
				}

				float newP = (r[x][y] - offDiag) / diag;

				maxDelta = max(maxDelta, fabs(p[x][y] - newP));

				p[x][y] = newP;
			}
		}

		if (maxDelta < 0.01)
		{
			printf("maxdelta good enough\n");
			return;
		}
	}
	printf("maximum change was %f \n", maxDelta);
}
void applyPressure()
{
	double scale = dt / (rho);

	for (int y = 0;  y < 128; y++)
	{
		for (int x = 0; x < 128; x++)
		{
			u->at(x, y) -= scale * p[x][y];
			u->at(x + 1, y) += scale * p[x][y];
			v->at(x, y) -= scale * p[x][y];
			v->at(x, y + 1) += scale * p[x][y];
		}
	}

	/*for (int y = 0; y < 128; y++)
		u->at(0, y) = u->at(128, y) = 0.0;
	for (int x = 0; x < 128; x++)
		v->at(x, 0) = v->at(x, 128) = 0.0;*/
}

void applyExternal()
{
	for (int x = 59; x <= 69; ++x)
	{
		for (int y = 100; y < 112; ++y)
		{
			ink->set(x, y, 3);
			u->set(x, y, 0);
			v->set(x, y, -20);
		}
	}
}

void update()
{
	applyExternal();
	
	buildR();
	project();
	applyPressure();
	enforceBoundary();

	u->advect(dt, u, v);
	v->advect(dt, u, v);

	ink->advect(dt, u, v);

	ink->flip();
	u->flip();
	v->flip();

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
			printf("D: %f\n", ink->at(rx, ry));
			printf("V: %f, %f\n", u->lerp(rx, ry), v->lerp(rx, ry));
			printf("P: %f\n", pressure[(int)rx][(int)ry]);
			printf("-----------------------------\n\n");
		}*/

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
				if (cellType[x][y] == 1)
				{
					glColor3f(0, 1, 0);
					glVertex2f(x, y);
					glVertex2f(x + 1, y);
					glVertex2f(x + 1, y + 1);
					glVertex2f(x, y + 1);
					continue;
				}
				float f = ink->at(x, y) / (3);
				if (ink->lerp(x, y) >= 0)
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
		for (int x = 0; x < cellW + 1; x += 5)
		{
			for (int y = 0; y < cellH + 1; y += 5)
			{
				float xVel = u->at(x, y), 
					yVel = v->at(x, y);
				glColor4f(1, 0, 0, 0.25f);
				glVertex2f(x, y);
				glColor3f(1, 0, 0);
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

	ink = new fluidQ(128, 128, 0.5, 0.5, 1);
	u = new fluidQ(128 + 1, 128 + 1, 0.0, 0.5, 1);
	v = new fluidQ(128 + 1, 128 + 1, 0.5, 0.0, 1);
	setupParticles();

	memset(p, 0, 128 * 128 * sizeof(float));
	memset(cellType, 0, mapW * mapH * sizeof(int));
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