#ifndef BOIDS_H
#define BOIDS_H

#include <iostream>
#include <vector>
#include <cmath>
#include <tuple>

#include <SDL2/SDL.h>

// Window constants
int const WIDTH = 800;
int const HEIGHT = 600;

// Boids constants
const float RAD_STEP = 0.01;
const float DIST_THRESHOLD = 50;
const float MAX_SPEED = 3;
const float CENTERING_FACTOR = 0.005; // How much the boid will turn towards the barycenter
const float AVOID_FACTOR = 0.001; // How much the boid will turn away from another boid
const float ALIGNEMENT_FACTOR = 0.01; // How much the boid will align with other boids
const float TURN_FACTOR = 0.05; // How much the boid will turn away from the screen edges
const float BOIDS_SIZE = 5;

const float OBSTACLES_SIZE = 10;
const float SELECT_DISTANCE = 10;


template <typename T>
T distance(T x1, T y1, T x2, T y2) {
	T dx = x1 - x2;
	T dy = y1 - y2;
	return std::sqrt(dx*dx + dy*dy);
}

template <typename ColorType>
class Object;
using SDLColorType = std::tuple<Uint8, Uint8, Uint8, Uint8>;

struct global_t {
	SDL_Window * window = NULL;
	SDL_Renderer * renderer = NULL;

	// Objects
	std::vector<Object<SDLColorType>*> sceneObjects;

	// Running
	bool running = true;
};

global_t w;


template <typename ColorType>
class Object {
	protected:
		int scale;
		ColorType color;
	public:
		bool selected;
		bool obstacle;
		float x;
		float y;
		Object (float x, float y, int scale, ColorType color, bool obstacle) 
			: x(x), y(y), scale(scale), color(color), obstacle(obstacle), selected(false) {}
		virtual int draw() = 0;
		virtual void update() = 0;
};

class Obstacles : public Object<SDLColorType> {
	public:
		Obstacles (float x, float y, int scale, SDLColorType color) : Object(x,y,scale,color,true) {}

		int draw() override {
			// Draws the obstacle to the screen
			Uint8 r, g, b, a;
			std::tie(r,g,b,a) = !selected ? color : std::make_tuple<Uint8, Uint8, Uint8, Uint8>(255,255,0,255);

			SDL_Rect rect = {(int)(x-scale/2), (int)(y-scale/2), scale, scale};
			SDL_SetRenderDrawColor(w.renderer, r, g, b, a);
			return SDL_RenderFillRect(w.renderer, &rect);
		}

		void update() override {}
};

class Boids : public Object<SDLColorType> {
	private:
		float vx; // Velocity in x direction
    	float vy; // Velocity in y direction
		// x,y coordinates that are inherited represent the middle of the base line of the triangle
	public:
		Boids (float x, float y, int scale, SDLColorType color, float vx, float vy) 
		: Object(x,y,scale,color,false), vx(vx), vy(vy) {}

		int draw() override {
			// Draws the boid to the screen

			Uint8 r, g, b, a;
			std::tie(r,g,b,a) = !selected ? color : std::make_tuple<Uint8, Uint8, Uint8, Uint8>(0,255,255,255);

			float direction = std::atan2(vy, vx);
			float orthogonal = direction + M_PI / 2;
			if (orthogonal > 2*M_PI) orthogonal -= 2*M_PI;

			float delta_x = scale / 2 * std::cosf(orthogonal); // l * cos(orthogonal)
       		float delta_y = scale / 2 * std::sinf(orthogonal); // l * sin(orthogonal)
			float delta_x_normal = 1.5 * scale * std::cosf(direction);
			float delta_y_normal = 1.5 * scale * std::sinf(direction);

			const std::vector< SDL_Vertex > verts =
				{
					SDL_Vertex {SDL_FPoint{x-delta_x, y-delta_y}, SDL_Color{r,g,b,a}, SDL_FPoint{ 0 }},
					SDL_Vertex {SDL_FPoint{x+delta_x, y+delta_y}, SDL_Color{r,g,b,a}, SDL_FPoint{ 0 }},
					SDL_Vertex {SDL_FPoint{x+delta_x_normal, y+delta_y_normal}, SDL_Color{r,g,b,a}, SDL_FPoint{ 0 }}
				};
			
			return SDL_RenderGeometry(w.renderer, nullptr, verts.data(), verts.size(), nullptr, 0);
		}

		void turn_towards(float dir_x, float dir_y) {
			// Turns the boid progressively towards a direction
			float target = std::atan2(dir_y, dir_x);
			float direction = std::atan2(vy, vx);
			float diff = target - direction;
			if (fabs(diff) < RAD_STEP) return; // If close enough, it will stop jittering

			if (diff > M_PI) diff -= 2*M_PI;
			if (diff < -M_PI) diff += 2*M_PI;
			if (diff > 0) {
				direction += RAD_STEP;
			} else {
				direction -= RAD_STEP;
			}

			float norm = distance(vx,vy,0.f,0.f);
			vx = std::cos(direction)*norm;
			vy = std::sin(direction)*norm;
		}
		void update() override {
			float force_x = 0, force_y = 0;

			// Barycenter calculation
			float bary_x = 0;
			float bary_y = 0;
			int total_N = 0;
			// Alignement calculation
			float xvel_avg = 0;
			float yvel_avg = 0;
			int N = 0;

			for (Object * obj : w.sceneObjects){
				if (obj->obstacle) {
					// Turn away from obstacle
					float dist = distance(x,y,obj->x,obj->y);
					if (dist < DIST_THRESHOLD){
						force_x -= (obj->x - x) * AVOID_FACTOR;
						force_y -= (obj->y - y) * AVOID_FACTOR;
					}
					continue;
				};

				bary_x += obj->x;
				bary_y += obj->y;
				total_N++;
				if (obj == this) continue;

				float dist = distance(x,y,obj->x,obj->y);
				// If the boid is too close to another boid, it will turn away
				if (dist < DIST_THRESHOLD){
					force_x -= (obj->x - x) * AVOID_FACTOR;
					force_y -= (obj->y - y) * AVOID_FACTOR;
				}
				// Else it will attempt to align
				else {
					xvel_avg += dynamic_cast<Boids*>(obj)->vx;
					yvel_avg += dynamic_cast<Boids*>(obj)->vy;
					N++;		
				}
			}
			// Barycenter calculation
			bary_x /= total_N;
			bary_y /= total_N;
			// Alignement calculation
			if (N > 0){
				xvel_avg /= N;
				yvel_avg /= N;
				force_x += (xvel_avg - vx) * ALIGNEMENT_FACTOR;
				force_y += (yvel_avg - vy) * ALIGNEMENT_FACTOR;
			}

			// Turn towards the barycenter
			// If there is only 1 boid, it does not make sense to turn towards the barycenter
			if (N > 1){ 
				
				float centerVector_x = bary_x - x;
				float centerVector_y = bary_y - y;
				float norm = std::sqrt(centerVector_x*centerVector_x + centerVector_y*centerVector_y);
				centerVector_x /= norm;
				centerVector_y /= norm;

				force_x += centerVector_x * CENTERING_FACTOR;
				force_y += centerVector_y * CENTERING_FACTOR;
			}

			// Boids will avoid the screen edges with a 50px margin
			if (x < 100){
				force_x += TURN_FACTOR;
			}
			if (x > WIDTH - 100){
				force_x -= TURN_FACTOR;
			}
			if (y < 100){
				force_y += TURN_FACTOR;
			}
			if (y > HEIGHT - 100){
				force_y -= TURN_FACTOR;
			}


			// Updates the boid's position based on its velocity
			float dt = 1;
			vx += force_x * dt;
			vy += force_y * dt;
			float norm = distance(0.f,0.f,vx,vy);
			if (norm > MAX_SPEED){
				vx = vx / norm * MAX_SPEED;
				vy = vy / norm * MAX_SPEED;
			}

			turn_towards(force_x, force_y);

        	x += vx;
        	y += vy;
		}
};

#endif