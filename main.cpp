#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include <tuple>

#include <SDL2/SDL.h>

int const WIDTH = 800;
int const HEIGHT = 600;

// Boids constants
const float RAD_STEP = 0.01;
const float DIST_THRESHOLD = 50;
const float MAX_SPEED = 3;
float CENTERING_FACTOR = 0.005; // How much the boid will turn towards the barycenter
float AVOID_FACTOR = 0.001; // How much the boid will turn away from another boid


class Object; // declaration

struct global_t {
	SDL_Window * window = NULL;
	SDL_Renderer * renderer = NULL;

	// Objects
	std::vector<Object*> sceneObjects;

	// random
	std::random_device rd;
	std::default_random_engine eng;
	std::uniform_real_distribution<float> rand;

};

global_t w;


class Object {
	protected:
		int scale;
		std::tuple<Uint8, Uint8, Uint8, Uint8> color;
	public:
		bool obstacle;
		float x;
		float y;
		Object (float x, float y, int scale, std::tuple<Uint8, Uint8, Uint8, Uint8> color, bool obstacle) 
			: x(x), y(y), scale(scale), color(color), obstacle(obstacle) {}
		virtual int draw() = 0;
		virtual void update() = 0;
};

class Boids : public Object {
	private:
		float vx; // Velocity in x direction
    	float vy; // Velocity in y direction
		// x,y coordinates that are inherited represent the middle of the base line of the triangle
	public:
		Boids (float x, float y, int scale, std::tuple<Uint8, Uint8, Uint8, Uint8> color, float vx, float vy) 
		: Object(x,y,scale,color, false), vx(vx), vy(vy) {}

		int draw() override {
			// Draws the boid to the screen

			Uint8 r, g, b, a;
			std::tie(r,g,b,a) = color;

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

			float norm = std::sqrt(vx*vx + vy*vy);
			vx = std::cos(direction)*norm;
			vy = std::sin(direction)*norm;
		}
		void update() override {
			float force_x = 0, force_y = 0;

			// If the boid is too close to another boid, it will turn away
			float bary_x = 0;
			float bary_y = 0;
			int N = 0;
			for (Object* obj : w.sceneObjects){
				if (obj->obstacle) continue;
				bary_x += obj->x;
				bary_y += obj->y;
				N++;
				if (obj == this) continue;

				float dx = obj->x - x;
				float dy = obj->y - y;
				float dist = std::sqrt(dx*dx + dy*dy);
				if (dist < DIST_THRESHOLD){
					force_x -= dx * AVOID_FACTOR;
					force_y -= dy * AVOID_FACTOR;
				}
			}
			bary_x /= N;
			bary_y /= N;

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

			// Updates the boid's position based on its velocity
			
			float dt = 1;
			vx += force_x * dt;
			vy += force_y * dt;
			if (std::sqrt(vx*vx + vy*vy) > MAX_SPEED){
				float norm = std::sqrt(vx*vx + vy*vy);
				vx = vx / norm * MAX_SPEED;
				vy = vy / norm * MAX_SPEED;
			}

			// printf("Force : %f %f, Speed : %f %f, Speed norm : %f\n", force_x, force_y, vx, vy, std::sqrt(vx*vx + vy*vy));
			turn_towards(force_x, force_y);

        	x += vx;
        	y += vy;

		}
};

void do_render() {
	// Reset to black screen
	SDL_SetRenderDrawColor(w.renderer, 0u, 0u, 0u, SDL_ALPHA_OPAQUE);
	SDL_RenderClear(w.renderer);
	
	// Render the scene
	for (Object* x : w.sceneObjects){
		x->draw();
	}

	SDL_RenderPresent(w.renderer);
}

void do_update() {
	// Update the barycenter
	float sum_x = 0;
	float sum_y = 0;

	// Update the scene
	for (Object* x : w.sceneObjects){
		x->update();
	}
}

void init_scene(){
	// auto color = std::make_tuple<Uint8, Uint8, Uint8, Uint8>(255,0,0,255);
	// Boids * b1 =  new Boids(WIDTH/2, HEIGHT/2, 25, color, 1., 0.);
	// w.sceneObjects.push_back(b1);
}

int main(int argc, char ** argv)
{
	std::srand(time(NULL));

	int status;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS | SDL_INIT_TIMER) != 0) {
		return 1;
	}

	w.window = SDL_CreateWindow("BOIDS",
			SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
			WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
	if (not w.window) {
		return 1;
	}

	// get the default renderer
	w.renderer = SDL_CreateRenderer(w.window, -1, 0);
	if (not w.renderer) {
		return 1;
	}

	init_scene();
	bool end = false;
	while (not end) {
		SDL_Event event;
		if (SDL_WaitEventTimeout(&event, 20)) {
			switch (event.type) {
			case SDL_WINDOWEVENT:
				switch (event.window.event) {
					case SDL_WINDOWEVENT_CLOSE:
						end = true;
						break;
					default:
						break;
				}
				break;
			case SDL_KEYDOWN:
				if (event.key.keysym.sym == SDLK_ESCAPE) {
					end = true;
				}
				break;
			case SDL_KEYUP:
				break;
			case SDL_MOUSEBUTTONDOWN:					
				// Add a boid on left click where the mouse is
				if (event.button.button == SDL_BUTTON_LEFT){
					auto color = std::make_tuple<Uint8, Uint8, Uint8, Uint8>(255,0,0,255);
					Boids * b1 =  new Boids(event.button.x, event.button.y, 10, color, 1., 0.);
					w.sceneObjects.push_back(b1);
				}
				// If right click all boids are deleted
				if (event.button.button == SDL_BUTTON_RIGHT){
					for (Object* x : w.sceneObjects){
						delete x;
					}
					w.sceneObjects.clear();
				}
				break;
			}
		} else {
			// Got time out or error
			char const * e = SDL_GetError();
			if (e != NULL) {
				if (strlen(e) != 0) {
					// Print and Ignore errors (MacOS has error)
					std::cout << "ERROR: " << e << std::endl;
					SDL_ClearError();
				}
			}

			do_update();
			do_render();
		}
	}

	SDL_DestroyRenderer(w.renderer);
	SDL_DestroyWindow(w.window);
	SDL_CloseAudio();
	SDL_Quit();
	return 0;
}
